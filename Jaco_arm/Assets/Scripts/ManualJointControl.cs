using System;
using System.Collections;
using UnityEngine;

/// <summary>
/// Cartesian control of the Jaco gripper tip. Arrows translate in world XZ, W/S
/// change height, and A/D rotate the base counterclockwise/clockwise from above.
/// T exchanges arrow control for tool-local roll (R/G), pitch (P/H), yaw (Y/L).
/// The pose IK holds orientation during translation and position during rotation.
/// </summary>
public class ManualJointControl : MonoBehaviour
{
    const int k_NumRobotJoints = 6;
    // This j2n6s200 pose puts the tip at (0, 0.4, -0.4), with the elbow raised
    // and the URDF tool approach axis (+Y after import) pointing down.
    static readonly float[] k_StartAngles = { -7.66934f, 195.41690f, 99.62644f, -31.45060f, 101.46753f, -129.11994f };

    [SerializeField] GameObject m_j2n6s200;
    public GameObject j2n6s200 { get => m_j2n6s200; set => m_j2n6s200 = value; }
    [SerializeField] bool m_ManualModeEnabled = true;
    public bool ManualModeEnabled { get => m_ManualModeEnabled; set => m_ManualModeEnabled = value; }

    [SerializeField] float m_MoveSpeed = 0.3f;
    [SerializeField] float m_SpeedScale = 3f;
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
    Vector3 m_DesiredPosition, m_ArmBasePosition;
    Quaternion m_DesiredRotation;
    float m_MaxReachMeters, m_PendingScroll;
    bool m_Ready, m_OrientationMode;
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
        ConfigureDrive(m_RightGripper, m_GripperDriveStiffness, m_GripperDriveDamping, ActualAngle(m_RightGripper));
        ConfigureDrive(m_LeftGripper, m_GripperDriveStiffness, m_GripperDriveDamping, ActualAngle(m_LeftGripper));

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

    static void ConfigureDrive(ArticulationBody joint, float stiffness, float damping, float angle)
    {
        var drive = joint.xDrive;
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
    }

    void OnApplicationFocus(bool focused)
    {
        if (!focused)
        {
            m_Input = default;
            m_PendingScroll = 0f;
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

        if (m_Input.translation.sqrMagnitude > 0f)
        {
            var previousGoal = m_DesiredPosition;
            var delta = m_Input.translation * (m_MoveSpeed * m_SpeedScale * dt);
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
        m_PendingScroll = 0f;
        if (Mathf.Abs(gripperDelta) > 0f)
        {
            ApplyGripperDelta(m_RightGripper, gripperDelta);
            ApplyGripperDelta(m_LeftGripper, gripperDelta);
        }
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

    static void ApplyGripperDelta(ArticulationBody gripper, float delta)
    {
        var drive = gripper.xDrive;
        drive.target = ClampToLimits(gripper, drive.target + delta);
        gripper.xDrive = drive;
    }

    void OnGUI()
    {
        if (!m_ManualModeEnabled)
            return;
        GUILayout.BeginArea(new Rect(10, 60, 440, 310), GUI.skin.box);
        GUILayout.Label("Jaco gripper control");
        GUILayout.Label(m_OrientationMode ? "ORIENTATION MODE - arrows off (T to switch)" : "PLANE MODE - arrow keys on (T to switch)");
        GUILayout.Label("Arrows: horizontal XZ plane | W: up | S: down");
        GUILayout.Label("A: base anticlockwise | D: clockwise (viewed from above)");
        GUILayout.Label("Orientation mode: R/G roll +/- | P/H pitch +/- | Y/L yaw +/-");
        GUILayout.Label("Rotations follow the gripper; roll turns around the finger direction.");
        GUILayout.Label("Z: open | C: close gripper | scroll: gripper");
        GUILayout.Label("1-6: joint locks | 0: unlock all | PageUp/Down: move speed");
        GUILayout.Label($"Speed x{m_SpeedScale:F2} | {m_DebugStatus}");
        var locks = "";
        for (var i = 0; i < k_NumRobotJoints; i++)
            if (m_JointLocked[i]) locks += $" J{i + 1}";
        if (locks.Length > 0) GUILayout.Label("Locked:" + locks);
        if (m_TargetUnreachable) GUILayout.Label("Target unreachable with the current pose / joint locks");
        GUILayout.EndArea();
    }
}
