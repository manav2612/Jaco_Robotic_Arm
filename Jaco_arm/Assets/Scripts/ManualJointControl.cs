using System;
using System.Collections;
using UnityEngine;

/// <summary>
/// Cartesian control of the Jaco gripper tip. Arrows translate in world XZ, W/S
/// change height, and A/D rotate the base counterclockwise/clockwise from above.
/// T exchanges arrow control for tool-local roll (R/G), pitch (P/H), yaw (Y/L).
/// The pose IK holds orientation during translation and position during rotation.
/// Left-click drag also translates the tip in a fixed world plane (mouse X/Y map onto
/// that plane's two axes; Tab cycles XZ -> XY -> YZ), and right-click drag moves world Y.
/// </summary>
public class ManualJointControl : MonoBehaviour
{
    const int k_NumRobotJoints = 6;

    enum DragPlane { XZ, XY, YZ }

    // This j2n6s200 pose puts the tip at (0, 0.4, -0.4), with the elbow raised
    // and the URDF tool approach axis (+Y after import) pointing down.
    static readonly float[] k_StartAngles = { -7.66934f, 195.41690f, 99.62644f, -31.45060f, 101.46753f, -129.11994f };

    [SerializeField] GameObject m_j2n6s200;
    public GameObject j2n6s200 { get => m_j2n6s200; set => m_j2n6s200 = value; }
    [SerializeField] bool m_ManualModeEnabled = true;
    public bool ManualModeEnabled { get => m_ManualModeEnabled; set => m_ManualModeEnabled = value; }

    [SerializeField] float m_MoveSpeed = 0.3f;
    [SerializeField] float m_SpeedScale = 3f;
    [SerializeField] float m_MouseSensitivity = 0.0015f; // meters of goal movement per mouse-axis delta unit
    [SerializeField] DragPlane m_DragPlane = DragPlane.XZ;
    [SerializeField] float m_GripperSpeed = 60f;
    [SerializeField] float m_ScrollGripperSensitivity = 40f;
    [SerializeField] float m_WristJogSpeed = 30f;
    [SerializeField] float m_BaseJogSpeed = 30f;
    [SerializeField] float m_DampingLambda = 0.03f;
    [SerializeField] float m_MaxStepMeters = 0.015f;
    [SerializeField] float m_ReachSafetyMargin = 0.95f;
    [SerializeField] float m_MaxTargetLeadDegrees = 20f;
    [SerializeField] float m_ArmDriveStiffness = 10000f;
    [SerializeField] float m_ArmDriveDamping = 1000f;
    [SerializeField] float m_GripperDriveStiffness = 200f;
    [SerializeField] float m_GripperDriveDamping = 20f;
    // Grip squeeze is stiffness * (target - actual), so a target that keeps running past contact
    // crushes or ejects the object. Cap how far the finger target may lead the measured angle.
    [SerializeField] float m_GripperMaxTargetLeadDegrees = 5f;
    // Hard ceiling on finger drive torque; prevents crushing and violent depenetration.
    [SerializeField] float m_GripperForceLimit = 10f;
    // Grasp realism: contact quality for the fingers and the objects they hold.
    [SerializeField] float m_FingerFriction = 1f;
    [SerializeField] int m_SolverIterations = 32;
    [SerializeField] int m_SolverVelocityIterations = 8;
    [SerializeField] float m_PhysicsTimestep = 0.01f; // seconds; 0 leaves the project's fixed timestep alone
    // Grasp assist: when both fingers touch the same rigidbody it is pinned to the wrist with a
    // breakable fixed joint (friction alone lets flat-faced fingers squeeze objects out). Opening
    // the gripper (Z / scroll) releases it.
    [SerializeField] bool m_GraspAssist = true;
    [SerializeField] float m_GraspHoldDelay = 0.15f;  // seconds both fingers must touch before pinning
    [SerializeField] float m_GraspBreakForce = 300f;
    [SerializeField] float m_GraspBreakTorque = 30f;
    // Stand cans/bottles upright on Play (long axis -> world up, same table spot).
    [SerializeField] bool m_StandObjectsUpright = true;
    [SerializeField] string[] m_UprightNameContains = { "coca_cola", "pepsi", "bottle" };
    [SerializeField] float m_UnreachableErrorThreshold = 0.05f;
    [SerializeField] float m_UnreachableWindowSeconds = 1.5f;
    [SerializeField] float m_UnreachableMinImprovementFrac = 0.15f;

    // Express angular error in comparable units to position error in the pose solve.
    const float k_OrientationWeight = 0.25f;
    const float k_MaxJointSpeed = 60f;
    const float k_MaxGoalLeadMeters = 0.08f;
    const float k_MaxGoalLeadDegrees = 20f;

    ArticulationBody[] m_ArmJoints;
    ArticulationBody m_RightGripper, m_LeftGripper;
    Transform m_EndEffector;
    readonly bool[] m_JointLocked = new bool[k_NumRobotJoints];
    readonly Vector3[] m_LinearColumns = new Vector3[k_NumRobotJoints];
    readonly Vector3[] m_AngularColumns = new Vector3[k_NumRobotJoints];
    readonly double[,] m_NormalSystem = new double[k_NumRobotJoints, k_NumRobotJoints + 1];
    Vector3 m_DesiredPosition, m_ArmBasePosition, m_PendingMouseDelta;
    Quaternion m_DesiredRotation;
    float m_MaxReachMeters, m_PendingScroll;
    bool m_Ready, m_OrientationMode;
    readonly System.Collections.Generic.List<FingerContactProbe> m_Probes = new System.Collections.Generic.List<FingerContactProbe>();
    FixedJoint m_GraspJoint;
    Rigidbody m_HeldBody;
    float m_BothTouchSince = -1f, m_NoRegrabUntil;
    string m_GraspStatus = "armed";
    ControlInput m_Input;
    string m_DebugStatus = "Setting arched start pose";
    float m_WindowStartTime = -1f, m_WindowStartErrorMag;
    bool m_TargetUnreachable;

    struct ControlInput
    {
        public Vector3 translation;
        public Vector3 toolRotation;
        public float baseTurn;
        public float gripper;
    }

    IEnumerator Start()
    {
        if (m_j2n6s200 == null)
        {
            Debug.LogError("Manual control needs the j2n6s200 robot reference.", this);
            enabled = false;
            yield break;
        }

        m_ArmJoints = new ArticulationBody[k_NumRobotJoints];
        var linkName = string.Empty;
        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            linkName += SourceDestinationPublisher.LinkNames[i];
            var link = m_j2n6s200.transform.Find(linkName);
            m_ArmJoints[i] = link == null ? null : link.GetComponent<ArticulationBody>();
            if (m_ArmJoints[i] == null)
            {
                Debug.LogError($"Missing Jaco articulation: {linkName}", this);
                enabled = false;
                yield break;
            }
        }

        m_EndEffector = m_j2n6s200.transform.Find(linkName + "/j2n6s200_end_effector");
        var right = m_j2n6s200.transform.Find(linkName + "/j2n6s200_link_finger_1");
        var left = m_j2n6s200.transform.Find(linkName + "/j2n6s200_link_finger_2");
        m_RightGripper = right == null ? null : right.GetComponent<ArticulationBody>();
        m_LeftGripper = left == null ? null : left.GetComponent<ArticulationBody>();
        if (m_EndEffector == null || m_RightGripper == null || m_LeftGripper == null)
        {
            Debug.LogError("Jaco tool frame or finger articulations are missing.", this);
            enabled = false;
            yield break;
        }

        if (!m_ManualModeEnabled)
            yield break;

        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            var joint = m_ArmJoints[i];
            var angle = ClampToLimits(joint, k_StartAngles[i]);
            joint.jointPosition = new ArticulationReducedSpace(angle * Mathf.Deg2Rad);
            joint.jointVelocity = new ArticulationReducedSpace(0f);
            ConfigureDrive(joint, m_ArmDriveStiffness, m_ArmDriveDamping, angle);
        }
        ConfigureDrive(m_RightGripper, m_GripperDriveStiffness, m_GripperDriveDamping, ActualAngle(m_RightGripper), m_GripperForceLimit);
        ConfigureDrive(m_LeftGripper, m_GripperDriveStiffness, m_GripperDriveDamping, ActualAngle(m_LeftGripper), m_GripperForceLimit);

        if (m_StandObjectsUpright)
            StandObjectsUpright();
        ApplyGraspPhysics();

        // PhysX updates the link transforms from the reduced coordinates at its next step.
        yield return new WaitForFixedUpdate();
        m_DesiredPosition = m_EndEffector.position;
        m_DesiredRotation = m_EndEffector.rotation;
        m_ArmBasePosition = JointPivot(m_ArmJoints[0]);
        var previous = m_ArmBasePosition;
        for (var i = 1; i < k_NumRobotJoints; i++)
        {
            var pivot = JointPivot(m_ArmJoints[i]);
            m_MaxReachMeters += Vector3.Distance(previous, pivot);
            previous = pivot;
        }
        m_MaxReachMeters += Vector3.Distance(previous, m_EndEffector.position);
        m_Ready = true;
    }

    // Rotates each matching scene object so its longest axis points along world up, then
    // re-seats it on the surface it was resting on (same footprint centre, same lowest point).
    void StandObjectsUpright()
    {
        foreach (var root in UnityEngine.SceneManagement.SceneManager.GetActiveScene().GetRootGameObjects())
        {
            if (!NameMatches(root.name))
                continue;
            var renderers = root.GetComponentsInChildren<Renderer>();
            if (renderers.Length == 0)
                continue;

            var before = CombinedBounds(renderers);

            // Long axis of the object's own shape, measured with the rotation reset.
            var originalRotation = root.transform.rotation;
            root.transform.rotation = Quaternion.identity;
            var size = CombinedBounds(renderers).size;
            var axis = Vector3.up;
            var longest = size.y;
            var second = Mathf.Max(size.x, size.z);
            if (size.x >= size.y && size.x >= size.z) { axis = Vector3.right; longest = size.x; second = Mathf.Max(size.y, size.z); }
            else if (size.z >= size.y && size.z >= size.x) { axis = Vector3.forward; longest = size.z; second = Mathf.Max(size.x, size.y); }
            if (longest < second * 1.2f)
            {
                root.transform.rotation = originalRotation; // roughly cubic: no clear long axis
                continue;
            }

            root.transform.rotation = Quaternion.FromToRotation(axis, Vector3.up);
            // The long axis only fixes up-vs-down ambiguity; a bottle must have its narrow neck on top.
            if (NeckPointsDown(root))
                root.transform.RotateAround(CombinedBounds(renderers).center, Vector3.right, 180f);
            var after = CombinedBounds(renderers);
            root.transform.position += new Vector3(before.center.x - after.center.x,
                before.min.y - after.min.y + 0.002f, before.center.z - after.center.z);

            foreach (var body in root.GetComponentsInChildren<Rigidbody>())
            {
                body.linearVelocity = Vector3.zero;
                body.angularVelocity = Vector3.zero;
            }
        }
        Physics.SyncTransforms();
    }

    // True when the top 15% of the object's height is wider than the bottom 15% (neck is down).
    // Symmetric objects such as cans come out within the tolerance and are left alone.
    static bool NeckPointsDown(GameObject root)
    {
        var verts = new System.Collections.Generic.List<Vector3>();
        foreach (var filter in root.GetComponentsInChildren<MeshFilter>())
        {
            var mesh = filter.sharedMesh;
            if (mesh == null || !mesh.isReadable)
                continue;
            foreach (var v in mesh.vertices)
                verts.Add(filter.transform.TransformPoint(v));
        }
        if (verts.Count < 16)
            return false;

        float minY = float.MaxValue, maxY = float.MinValue;
        var centre = Vector2.zero;
        foreach (var v in verts)
        {
            minY = Mathf.Min(minY, v.y);
            maxY = Mathf.Max(maxY, v.y);
            centre += new Vector2(v.x, v.z);
        }
        centre /= verts.Count;

        var band = (maxY - minY) * 0.15f;
        float topSum = 0f, bottomSum = 0f;
        int topCount = 0, bottomCount = 0;
        foreach (var v in verts)
        {
            var radius = (new Vector2(v.x, v.z) - centre).magnitude;
            if (v.y >= maxY - band) { topSum += radius; topCount++; }
            else if (v.y <= minY + band) { bottomSum += radius; bottomCount++; }
        }
        if (topCount == 0 || bottomCount == 0)
            return false;
        return topSum / topCount > 1.15f * (bottomSum / bottomCount);
    }

    bool NameMatches(string objectName)
    {
        foreach (var part in m_UprightNameContains)
            if (!string.IsNullOrEmpty(part) && objectName.IndexOf(part, StringComparison.OrdinalIgnoreCase) >= 0)
                return true;
        return false;
    }

    static Bounds CombinedBounds(Renderer[] renderers)
    {
        var bounds = renderers[0].bounds;
        for (var i = 1; i < renderers.Length; i++)
            bounds.Encapsulate(renderers[i].bounds);
        return bounds;
    }

    // Runtime contact-quality tuning, applied here so it does not depend on scene/prefab data:
    // grippy fingertips (Maximum combine wins over the object's own material), more solver
    // iterations for stable articulation contacts, and continuous collision detection on
    // the graspable objects so fast closing does not tunnel into a finger.
    void ApplyGraspPhysics()
    {
        if (m_PhysicsTimestep > 0f)
            Time.fixedDeltaTime = m_PhysicsTimestep;

        var fingerMaterial = new PhysicsMaterial("JacoFingerGrip")
        {
            staticFriction = m_FingerFriction,
            dynamicFriction = m_FingerFriction,
            frictionCombine = PhysicsMaterialCombine.Maximum,
            bounciness = 0f,
            bounceCombine = PhysicsMaterialCombine.Minimum
        };
        foreach (var finger in new[] { m_RightGripper, m_LeftGripper })
        {
            foreach (var collider in finger.GetComponentsInChildren<Collider>())
            {
                collider.sharedMaterial = fingerMaterial;
                var probe = collider.gameObject.GetComponent<FingerContactProbe>();
                if (probe == null)
                    probe = collider.gameObject.AddComponent<FingerContactProbe>();
                probe.Label = $"{finger.name}/{collider.name}";
                probe.Finger = finger;
                m_Probes.Add(probe);
            }
            finger.solverIterations = m_SolverIterations * 2;
            finger.solverVelocityIterations = m_SolverVelocityIterations * 2;
        }

        foreach (var joint in m_ArmJoints)
        {
            joint.solverIterations = m_SolverIterations;
            joint.solverVelocityIterations = m_SolverVelocityIterations;
        }

        try
        {
            foreach (var target in GameObject.FindGameObjectsWithTag("Target"))
            {
                if (!target.TryGetComponent<Rigidbody>(out var body))
                    continue;
                body.collisionDetectionMode = CollisionDetectionMode.ContinuousDynamic;
                body.interpolation = RigidbodyInterpolation.Interpolate;
                body.solverIterations = m_SolverIterations;
                body.solverVelocityIterations = m_SolverVelocityIterations;
            }
        }
        catch (UnityException)
        {
            // "Target" tag is not defined in this project; nothing to tune.
        }
    }

    static void ConfigureDrive(ArticulationBody joint, float stiffness, float damping, float angle, float forceLimit = -1f)
    {
        var drive = joint.xDrive;
        if (forceLimit > 0f)
            drive.forceLimit = forceLimit;
        drive.stiffness = stiffness;
        drive.damping = damping;
        drive.target = angle;
        drive.targetVelocity = 0f;
        joint.xDrive = drive;
    }

    static float ActualAngle(ArticulationBody joint) => joint.jointPosition[0] * Mathf.Rad2Deg;
    static Vector3 JointAxis(ArticulationBody joint) => joint.transform.TransformDirection(joint.anchorRotation * Vector3.right).normalized;
    static Vector3 JointPivot(ArticulationBody joint) => joint.transform.TransformPoint(joint.anchorPosition);

    static float ClampToLimits(ArticulationBody joint, float angle)
    {
        var drive = joint.xDrive;
        return joint.twistLock == ArticulationDofLock.LimitedMotion
            ? Mathf.Clamp(angle, drive.lowerLimit, drive.upperLimit) : angle;
    }

    static float KeyAxis(Func<KeyCode, bool> held, KeyCode positive, KeyCode negative)
        => (held(positive) ? 1f : 0f) - (held(negative) ? 1f : 0f);

    static ControlInput ReadInput(Func<KeyCode, bool> held, bool orientationMode)
    {
        var input = new ControlInput
        {
            translation = Vector3.up * KeyAxis(held, KeyCode.W, KeyCode.S),
            baseTurn = KeyAxis(held, KeyCode.A, KeyCode.D),
            gripper = KeyAxis(held, KeyCode.C, KeyCode.Z)
        };
        if (orientationMode)
        {
            // Imported tool +Y points along the fingers. Roll is about that axis;
            // pitch is about tool +X, yaw about tool -Z (a forward/up tool frame).
            input.toolRotation = new Vector3(
                KeyAxis(held, KeyCode.P, KeyCode.H),
                KeyAxis(held, KeyCode.R, KeyCode.G),
                -KeyAxis(held, KeyCode.Y, KeyCode.L));
        }
        else
        {
            input.translation += new Vector3(KeyAxis(held, KeyCode.RightArrow, KeyCode.LeftArrow),
                0f, KeyAxis(held, KeyCode.UpArrow, KeyCode.DownArrow));
        }
        input.translation = Vector3.ClampMagnitude(input.translation, 1f);
        input.toolRotation = Vector3.ClampMagnitude(input.toolRotation, 1f);
        return input;
    }

    void Update()
    {
        if (!m_Ready || !m_ManualModeEnabled)
            return;

        if (Input.GetKeyDown(KeyCode.T))
        {
            m_OrientationMode = !m_OrientationMode;
            // Discard any tracking lag so the newly selected mode holds the current tip.
            m_DesiredPosition = m_EndEffector.position;
            m_DesiredRotation = m_EndEffector.rotation;
        }
        if (Input.GetKeyDown(KeyCode.Tab))
            m_DragPlane = (DragPlane)(((int)m_DragPlane + 1) % 3);
        if (Input.GetKeyDown(KeyCode.PageUp))
            m_SpeedScale = Mathf.Min(m_SpeedScale * 1.25f, 20f);
        if (Input.GetKeyDown(KeyCode.PageDown))
            m_SpeedScale = Mathf.Max(m_SpeedScale / 1.25f, 0.2f);

        for (var i = 0; i < k_NumRobotJoints; i++)
            if (Input.GetKeyDown(KeyCode.Alpha1 + i) || Input.GetKeyDown(KeyCode.Keypad1 + i))
                m_JointLocked[i] = !m_JointLocked[i];
        if (Input.GetKeyDown(KeyCode.Alpha0) || Input.GetKeyDown(KeyCode.Keypad0))
            Array.Clear(m_JointLocked, 0, k_NumRobotJoints);

        m_Input = ReadInput(Input.GetKey, m_OrientationMode);
        m_PendingScroll += Input.GetAxis("Mouse ScrollWheel");
        m_PendingMouseDelta += ReadMouseDrag();
    }

    // Fixed world axes (not camera-relative). Accumulated here and applied in FixedUpdate.
    Vector3 ReadMouseDrag()
    {
        var delta = Vector3.zero;
        var scale = m_MouseSensitivity * m_SpeedScale;
        if (Input.GetMouseButton(0))
        {
            var x = Input.GetAxis("Mouse X") * scale;
            var y = Input.GetAxis("Mouse Y") * scale;
            switch (m_DragPlane)
            {
                case DragPlane.XZ: delta += new Vector3(x, 0f, y); break;
                case DragPlane.XY: delta += new Vector3(x, y, 0f); break;
                case DragPlane.YZ: delta += new Vector3(0f, y, x); break;
            }
        }
        if (Input.GetMouseButton(1))
            delta += Vector3.up * (Input.GetAxis("Mouse Y") * scale);
        return delta;
    }

    void OnApplicationFocus(bool focused)
    {
        if (!focused)
        {
            m_Input = default;
            m_PendingScroll = 0f;
            m_PendingMouseDelta = Vector3.zero;
        }
    }

    void FixedUpdate()
    {
        if (!m_Ready || !m_ManualModeEnabled)
            return;

        var dt = Time.fixedDeltaTime;
        var baseJogging = Mathf.Abs(m_Input.baseTurn) > 0f && !m_JointLocked[0];
        if (baseJogging)
            JogBase(m_Input.baseTurn, dt);

        var delta = m_Input.translation * (m_MoveSpeed * m_SpeedScale * dt) + m_PendingMouseDelta;
        m_PendingMouseDelta = Vector3.zero;
        if (delta.sqrMagnitude > 0f)
        {
            var previousGoal = m_DesiredPosition;
            // Bound only the requested axes, preserving height during horizontal motion.
            for (var axis = 0; axis < 3; axis++)
                if (Mathf.Abs(delta[axis]) > 0f)
                    m_DesiredPosition[axis] = Mathf.Clamp(m_DesiredPosition[axis] + delta[axis],
                        m_EndEffector.position[axis] - k_MaxGoalLeadMeters,
                        m_EndEffector.position[axis] + k_MaxGoalLeadMeters);
            var radius = m_MaxReachMeters * m_ReachSafetyMargin;
            if (Vector3.Distance(m_DesiredPosition, m_ArmBasePosition) > radius)
                m_DesiredPosition = previousGoal; // Reject an out-of-workspace step; do not tilt the plane.
        }
        if (m_Input.toolRotation.sqrMagnitude > 0f)
        {
            var localRotation = Quaternion.AngleAxis(m_WristJogSpeed * dt * m_Input.toolRotation.magnitude,
                m_Input.toolRotation.normalized);
            m_DesiredRotation = m_DesiredRotation * localRotation;
            m_DesiredRotation = Quaternion.RotateTowards(m_EndEffector.rotation, m_DesiredRotation, k_MaxGoalLeadDegrees);
        }

        SolveAndApplyIkStep(baseJogging, dt);
        UpdateUnreachableFlag(Vector3.Distance(m_DesiredPosition, m_EndEffector.position));

        var gripperDelta = m_Input.gripper * m_GripperSpeed * dt + m_PendingScroll * m_ScrollGripperSensitivity;
        UpdateGraspAssist(gripperDelta < -0.001f);
        m_PendingScroll = 0f;
        if (Mathf.Abs(gripperDelta) > 0f)
        {
            ApplyGripperDelta(m_RightGripper, gripperDelta);
            ApplyGripperDelta(m_LeftGripper, gripperDelta);
        }
    }

    void UpdateGraspAssist(bool opening)
    {
        if (!m_GraspAssist)
        {
            m_GraspStatus = "off";
            return;
        }

        if (m_HeldBody != null && m_GraspJoint == null)
        {
            m_HeldBody = null; // the joint broke under load
            m_NoRegrabUntil = Time.time + 0.5f;
            m_GraspStatus = "joint broke";
        }

        if (m_HeldBody != null)
        {
            if (opening)
            {
                Destroy(m_GraspJoint);
                m_GraspJoint = null;
                m_HeldBody = null;
                m_BothTouchSince = -1f;
                m_NoRegrabUntil = Time.time + 0.5f;
                m_GraspStatus = "released";
            }
            else
            {
                m_GraspStatus = $"HOLDING {m_HeldBody.name}";
            }
            return;
        }

        var shared = FindSharedContact();
        if (shared == null || opening || Time.time < m_NoRegrabUntil)
        {
            m_BothTouchSince = -1f;
            if (m_GraspStatus.StartsWith("HOLDING"))
                m_GraspStatus = "armed";
            return;
        }

        if (m_BothTouchSince < 0f)
            m_BothTouchSince = Time.time;
        if (Time.time - m_BothTouchSince < m_GraspHoldDelay)
        {
            m_GraspStatus = $"gripping {shared.name}...";
            return;
        }

        m_GraspJoint = shared.gameObject.AddComponent<FixedJoint>();
        m_GraspJoint.connectedArticulationBody = m_ArmJoints[k_NumRobotJoints - 1];
        m_GraspJoint.breakForce = m_GraspBreakForce;
        m_GraspJoint.breakTorque = m_GraspBreakTorque;
        m_HeldBody = shared;
        m_GraspStatus = $"HOLDING {shared.name}";
    }

    // A rigidbody that both fingers are currently touching, or null.
    Rigidbody FindSharedContact()
    {
        foreach (var a in m_Probes)
        {
            if (a.Finger != m_RightGripper || a.OtherBody == null || Time.time - a.LastContactTime > 0.1f)
                continue;
            foreach (var b in m_Probes)
                if (b.Finger == m_LeftGripper && b.OtherBody == a.OtherBody && Time.time - b.LastContactTime <= 0.1f)
                    return a.OtherBody;
        }
        return null;
    }

    void JogBase(float input, float dt)
    {
        var joint = m_ArmJoints[0];
        var axis = JointAxis(joint);
        // Negative world-Y rotation is counterclockwise when viewed from above.
        var direction = -Mathf.Sign(Vector3.Dot(axis, Vector3.up));
        var oldTarget = joint.xDrive.target;
        SetJointTarget(joint, oldTarget + input * direction * m_BaseJogSpeed * dt);
        var rotation = Quaternion.AngleAxis(joint.xDrive.target - oldTarget, axis);
        var pivot = JointPivot(joint);
        m_DesiredPosition = pivot + rotation * (m_DesiredPosition - pivot);
        m_DesiredRotation = rotation * m_DesiredRotation;
    }

    void SetJointTarget(ArticulationBody joint, float target)
    {
        var actual = ActualAngle(joint);
        var drive = joint.xDrive;
        target = Mathf.Clamp(target, actual - m_MaxTargetLeadDegrees, actual + m_MaxTargetLeadDegrees);
        drive.target = ClampToLimits(joint, target);
        joint.xDrive = drive;
    }

    static Vector3 RotationError(Quaternion desired, Quaternion actual)
    {
        var error = desired * Quaternion.Inverse(actual);
        if (error.w < 0f)
            error = new Quaternion(-error.x, -error.y, -error.z, -error.w);
        error.ToAngleAxis(out var degrees, out var axis);
        return degrees < 0.001f ? Vector3.zero : axis * (degrees * Mathf.Deg2Rad);
    }

    void SolveAndApplyIkStep(bool baseJogging, float dt)
    {
        var positionError = m_DesiredPosition - m_EndEffector.position;
        var angularError = RotationError(m_DesiredRotation, m_EndEffector.rotation);
        var linearStep = Vector3.ClampMagnitude(positionError, m_MaxStepMeters * m_SpeedScale);
        var angularStep = Vector3.ClampMagnitude(angularError, 5f * Mathf.Deg2Rad) * k_OrientationWeight;
        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            var locked = m_JointLocked[i] || (i == 0 && baseJogging);
            var axis = JointAxis(m_ArmJoints[i]);
            m_LinearColumns[i] = locked ? Vector3.zero : Vector3.Cross(axis, m_EndEffector.position - JointPivot(m_ArmJoints[i]));
            m_AngularColumns[i] = locked ? Vector3.zero : axis * k_OrientationWeight;
        }

        // Damped least squares for the full pose: (J^T J + lambda^2 I) dq = J^T e.
        var damping = Mathf.Max(m_DampingLambda, 0.001f);
        for (var row = 0; row < k_NumRobotJoints; row++)
        {
            for (var col = 0; col < k_NumRobotJoints; col++)
                m_NormalSystem[row, col] = Vector3.Dot(m_LinearColumns[row], m_LinearColumns[col])
                    + Vector3.Dot(m_AngularColumns[row], m_AngularColumns[col])
                    + (row == col ? damping * damping : 0f);
            m_NormalSystem[row, k_NumRobotJoints] = Vector3.Dot(m_LinearColumns[row], linearStep)
                + Vector3.Dot(m_AngularColumns[row], angularStep);
        }
        if (!SolveLinearSystem(m_NormalSystem))
        {
            m_DebugStatus = "Pose solver could not find a stable step";
            return;
        }

        var largestStep = 0f;
        for (var i = 0; i < k_NumRobotJoints; i++)
            largestStep = Mathf.Max(largestStep, Mathf.Abs((float)m_NormalSystem[i, k_NumRobotJoints]) * Mathf.Rad2Deg);
        var scale = largestStep > k_MaxJointSpeed * dt ? k_MaxJointSpeed * dt / largestStep : 1f;
        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            if (m_JointLocked[i] || (i == 0 && baseJogging))
                continue;
            var delta = (float)m_NormalSystem[i, k_NumRobotJoints] * Mathf.Rad2Deg * scale;
            SetJointTarget(m_ArmJoints[i], ActualAngle(m_ArmJoints[i]) + delta);
        }
        m_DebugStatus = $"Tip error: {positionError.magnitude * 1000f:F1} mm | Angle error: {angularError.magnitude * Mathf.Rad2Deg:F1} deg";
    }

    static bool SolveLinearSystem(double[,] a)
    {
        for (var col = 0; col < k_NumRobotJoints; col++)
        {
            var pivot = col;
            for (var row = col + 1; row < k_NumRobotJoints; row++)
                if (Math.Abs(a[row, col]) > Math.Abs(a[pivot, col]))
                    pivot = row;
            if (Math.Abs(a[pivot, col]) < 1e-12)
                return false;
            for (var j = col; j <= k_NumRobotJoints; j++)
            {
                var temp = a[col, j];
                a[col, j] = a[pivot, j];
                a[pivot, j] = temp;
            }
            var divisor = a[col, col];
            for (var j = col; j <= k_NumRobotJoints; j++)
                a[col, j] /= divisor;
            for (var row = 0; row < k_NumRobotJoints; row++)
            {
                if (row == col)
                    continue;
                var factor = a[row, col];
                for (var j = col; j <= k_NumRobotJoints; j++)
                    a[row, j] -= factor * a[col, j];
            }
        }
        return true;
    }

    void UpdateUnreachableFlag(float errorMag)
    {
        if (errorMag < m_UnreachableErrorThreshold)
        {
            m_WindowStartTime = -1f;
            m_TargetUnreachable = false;
            return;
        }
        if (m_WindowStartTime < 0f)
        {
            m_WindowStartTime = Time.unscaledTime;
            m_WindowStartErrorMag = errorMag;
        }
        if (Time.unscaledTime - m_WindowStartTime < m_UnreachableWindowSeconds)
            return;
        m_TargetUnreachable = (m_WindowStartErrorMag - errorMag) / m_WindowStartErrorMag < m_UnreachableMinImprovementFrac;
        m_WindowStartTime = Time.unscaledTime;
        m_WindowStartErrorMag = errorMag;
    }

    void ApplyGripperDelta(ArticulationBody gripper, float delta)
    {
        var drive = gripper.xDrive;
        var actual = ActualAngle(gripper);
        var target = Mathf.Clamp(drive.target + delta, actual - m_GripperMaxTargetLeadDegrees, actual + m_GripperMaxTargetLeadDegrees);
        drive.target = ClampToLimits(gripper, target);
        gripper.xDrive = drive;
    }

    void OnGUI()
    {
        if (!m_ManualModeEnabled)
            return;
        GUILayout.BeginArea(new Rect(10, 60, 560, 470), GUI.skin.box);
        GUILayout.Label("Jaco gripper control");
        GUILayout.Label(m_OrientationMode ? "ORIENTATION MODE - arrows off (T to switch)" : "PLANE MODE - arrow keys on (T to switch)");
        GUILayout.Label("Arrows: horizontal XZ plane | W: up | S: down");
        GUILayout.Label("A: base anticlockwise | D: clockwise (viewed from above)");
        GUILayout.Label("Orientation mode: R/G roll +/- | P/H pitch +/- | Y/L yaw +/-");
        GUILayout.Label("Rotations follow the gripper; roll turns around the finger direction.");
        GUILayout.Label($"Mouse: left-drag moves {m_DragPlane} plane (Tab to cycle) | right-drag: world Y");
        GUILayout.Label("Z: open | C: close gripper | scroll: gripper");
        GUILayout.Label("1-6: joint locks | 0: unlock all | PageUp/Down: move speed");
        GUILayout.Label($"Speed x{m_SpeedScale:F2} | {m_DebugStatus}");
        var locks = "";
        for (var i = 0; i < k_NumRobotJoints; i++)
            if (m_JointLocked[i]) locks += $" J{i + 1}";
        if (locks.Length > 0) GUILayout.Label("Locked:" + locks);
        if (m_TargetUnreachable) GUILayout.Label("Target unreachable with the current pose / joint locks");
        GUILayout.Label($"Fingers R: {ActualAngle(m_RightGripper):F1} -> {m_RightGripper.xDrive.target:F1} deg | L: {ActualAngle(m_LeftGripper):F1} -> {m_LeftGripper.xDrive.target:F1} deg");
        var touching = 0;
        foreach (var probe in m_Probes)
        {
            if (Time.time - probe.LastContactTime > 0.25f)
                continue;
            touching++;
            GUILayout.Label($"  touch {probe.Label} <-> {probe.OtherName} ({probe.ContactPoints} pts, impulse {probe.Impulse:F3})");
        }
        if (touching == 0) GUILayout.Label("  fingers touching nothing");
        GUILayout.Label($"Grasp assist: {m_GraspStatus}");
        GUILayout.EndArea();
    }
}

/// <summary>Records what a finger collider is touching so the on-screen panel can show it.</summary>
public class FingerContactProbe : MonoBehaviour
{
    public string Label;
    public ArticulationBody Finger;
    public Rigidbody OtherBody;
    public string OtherName = "";
    public int ContactPoints;
    public float Impulse;
    public float LastContactTime = -10f;

    void OnCollisionEnter(Collision collision) => Record(collision);

    void OnCollisionStay(Collision collision) => Record(collision);

    void Record(Collision collision)
    {
        var body = collision.collider.attachedRigidbody;
        OtherBody = body;
        OtherName = body != null ? body.name : collision.collider.name;
        ContactPoints = collision.contactCount;
        Impulse = collision.impulse.magnitude;
        LastContactTime = Time.time;
    }
}
