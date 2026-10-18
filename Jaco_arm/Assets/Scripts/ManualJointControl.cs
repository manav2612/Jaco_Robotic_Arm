using UnityEngine;

/// <summary>
///     Manual Cartesian jog control for the j2n6s200 arm's end effector, for testing
///     without ROS running. Dragging the mouse moves the end effector and a closed-loop
///     damped least squares (DLS) / singularity-robust inverse solve on the geometric
///     Jacobian updates all 6 arm joints together in sync to follow it. A diagonal drag
///     combines both mouse components into a single 2D plane move (not clamped to one
///     axis): double-click toggles between the horizontal ground plane (mouse X -> world
///     right, mouse Y -> world forward/back, an XZ-cross move) and the vertical plane
///     (mouse X -> camera-right, mouse Y -> world up, an XY-cross move). Left/Right
///     arrows move the target along camera-forward (depth); Up/Down arrows open/close
///     the gripper.
/// </summary>
public class ManualJointControl : MonoBehaviour
{
    const int k_NumRobotJoints = 6;

    [SerializeField]
    GameObject m_j2n6s200;
    public GameObject j2n6s200 { get => m_j2n6s200; set => m_j2n6s200 = value; }

    [SerializeField]
    bool m_ManualModeEnabled = true;
    public bool ManualModeEnabled { get => m_ManualModeEnabled; set => m_ManualModeEnabled = value; }

    [SerializeField]
    float m_Sensitivity = 0.01f; // meters of end-effector motion per unit of mouse delta

    [SerializeField]
    float m_DepthSpeed = 0.3f; // meters/sec while holding Left/Right arrow

    [SerializeField]
    float m_GripperSpeed = 60f; // degrees/sec while holding Up/Down arrow

    [SerializeField]
    float m_DoubleClickWindow = 0.3f; // seconds between clicks to count as a double-click

    [SerializeField]
    float m_DampingLambda = 0.03f; // base DLS damping; larger = more stable near singularities, slower

    [SerializeField]
    float m_MaxStepMeters = 0.015f; // clamp on the position error fed to the solver each frame, prevents overshoot/jitter

    [SerializeField]
    float m_ManipulabilityThreshold = 0.0025f; // below this, damping ramps up (singularity-robust inverse)

    ArticulationBody[] m_ArmJoints;
    ArticulationBody m_RightGripper;
    ArticulationBody m_LeftGripper;
    Transform m_EndEffector;
    Camera m_Camera;

    bool m_VerticalDragMode;
    float m_LastClickTime = -1f;
    Vector3 m_DesiredPosition;

    [SerializeField]
    float m_TrajectoryDuration = 0.12f; // seconds for the minimum-jerk re-plan to catch up to a new goal

    Vector3 m_TrajStartPos;
    float m_TrajStartTime;
    Vector3 m_TrajEndPos;

    [SerializeField]
    float m_UnreachableErrorThreshold = 0.05f; // meters of sustained error before flagging "unreachable"

    [SerializeField]
    float m_UnreachableWindowSeconds = 1.5f; // how long error must fail to meaningfully shrink before flagging

    [SerializeField]
    float m_UnreachableMinImprovementFrac = 0.15f; // must shrink by at least this fraction over the window, or it's "stuck"

    float m_WindowStartTime = -1f;
    float m_WindowStartErrorMag;
    bool m_TargetUnreachable;

    float m_MaxReachMeters;
    Vector3 m_ArmBasePosition;

    void Start()
    {
        m_ArmJoints = new ArticulationBody[k_NumRobotJoints];

        var linkName = string.Empty;
        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            linkName += SourceDestinationPublisher.LinkNames[i];
            m_ArmJoints[i] = m_j2n6s200.transform.Find(linkName).GetComponent<ArticulationBody>();
        }

        m_EndEffector = m_ArmJoints[k_NumRobotJoints - 1].transform;

        var rightGripper = linkName + "/j2n6s200_link_finger_1";
        var leftGripper = linkName + "/j2n6s200_link_finger_2";
        m_RightGripper = m_j2n6s200.transform.Find(rightGripper).GetComponent<ArticulationBody>();
        m_LeftGripper = m_j2n6s200.transform.Find(leftGripper).GetComponent<ArticulationBody>();

        m_Camera = Camera.main;
        m_DesiredPosition = m_EndEffector.position;
        m_TrajStartPos = m_EndEffector.position;
        m_TrajEndPos = m_EndEffector.position;
        m_TrajStartTime = Time.time;

        // Reach limit = sum of consecutive joint-anchor distances from base to end effector —
        // the standard max-workspace-radius bound for a serial manipulator. Any desired
        // position further than this from the base is provably unreachable regardless of
        // joint limits, so it's a useful sanity bound alongside the live solver.
        m_ArmBasePosition = m_ArmJoints[0].transform.TransformPoint(m_ArmJoints[0].anchorPosition);
        var prevPos = m_ArmBasePosition;
        m_MaxReachMeters = 0f;
        for (var i = 1; i < k_NumRobotJoints; i++)
        {
            var jointPos = m_ArmJoints[i].transform.TransformPoint(m_ArmJoints[i].anchorPosition);
            m_MaxReachMeters += Vector3.Distance(prevPos, jointPos);
            prevPos = jointPos;
        }
        m_MaxReachMeters += Vector3.Distance(prevPos, m_EndEffector.position);
    }

    // Debug diagnostics shown in OnGUI to help track down why dragging might not move the arm.
    string m_DebugStatus = "no drag yet";

    void Update()
    {
        if (!m_ManualModeEnabled)
        {
            return;
        }

        if (Input.GetMouseButtonDown(0))
        {
            if (m_LastClickTime >= 0f && Time.unscaledTime - m_LastClickTime <= m_DoubleClickWindow)
            {
                m_VerticalDragMode = !m_VerticalDragMode;
                m_LastClickTime = -1f; // consume the double-click so a third click doesn't toggle again
            }
            else
            {
                m_LastClickTime = Time.unscaledTime;
            }
        }

        var worldDelta = Vector3.zero;

        if (Input.GetMouseButton(0) && m_Camera != null)
        {
            var mouseX = Input.GetAxis("Mouse X");
            var mouseY = Input.GetAxis("Mouse Y");

            if (m_VerticalDragMode)
            {
                // Vertical plane: combines camera-right (X) and world-up (Y) so a
                // diagonal drag moves both axes at once (XY-cross) instead of only
                // ever reading mouse Y.
                worldDelta += m_Camera.transform.right * (mouseX * m_Sensitivity);
                worldDelta += Vector3.up * (mouseY * m_Sensitivity);
            }
            else
            {
                // Horizontal (ground) plane: project the camera's right and forward
                // onto world XZ so a diagonal drag combines both world X and world Z
                // components (XZ-cross) instead of only ever reading mouse X.
                var groundRight = Vector3.ProjectOnPlane(m_Camera.transform.right, Vector3.up).normalized;
                var groundForward = Vector3.ProjectOnPlane(m_Camera.transform.forward, Vector3.up).normalized;
                worldDelta += groundRight * (mouseX * m_Sensitivity);
                worldDelta += groundForward * (mouseY * m_Sensitivity);
            }
        }

        if (m_Camera != null)
        {
            if (Input.GetKey(KeyCode.RightArrow))
            {
                worldDelta += m_Camera.transform.forward * (m_DepthSpeed * Time.deltaTime);
            }
            else if (Input.GetKey(KeyCode.LeftArrow))
            {
                worldDelta -= m_Camera.transform.forward * (m_DepthSpeed * Time.deltaTime);
            }
        }

        if (worldDelta.sqrMagnitude > 0f)
        {
            m_DesiredPosition += worldDelta;

            // Re-plan the minimum-jerk trajectory: start from wherever the smoothed
            // point currently is (for continuity, no pop) and ease toward the new goal
            // over m_TrajectoryDuration. Re-planning every time the goal moves keeps
            // this responsive to the live mouse drag while still removing the jerk of
            // feeding the raw goal straight into the IK solver.
            m_TrajStartPos = EvaluateMinimumJerkTrajectory(Time.time);
            m_TrajStartTime = Time.time;
            m_TrajEndPos = m_DesiredPosition;
        }
        else if (Input.GetMouseButton(0))
        {
            m_DebugStatus = m_Camera == null ? "camera is null" : "mouse held but delta is zero";
        }

        // Closed-loop: every frame, drive the actual end effector toward the smoothed
        // (minimum-jerk) point on the current trajectory, not the raw mouse-driven goal.
        // This self-corrects any linearization error from the previous step instead of
        // blindly trusting an open-loop delta application, while the trajectory layer
        // keeps the motion profile smooth (zero velocity/acceleration at each re-plan).
        var smoothedTarget = EvaluateMinimumJerkTrajectory(Time.time);
        var error = smoothedTarget - m_EndEffector.position;
        SolveAndApplyIkStep(error);
        UpdateUnreachableFlag(Vector3.Distance(m_DesiredPosition, m_EndEffector.position));

        if (Input.GetKey(KeyCode.UpArrow))
        {
            JogGripper(m_GripperSpeed * Time.deltaTime);
        }
        else if (Input.GetKey(KeyCode.DownArrow))
        {
            JogGripper(-m_GripperSpeed * Time.deltaTime);
        }
    }

    /// <summary>
    ///     Minimum-jerk trajectory: the quintic polynomial s(t) = 10t^3 - 15t^4 + 6t^5
    ///     (Flash &amp; Hogan's minimum-jerk model of smooth point-to-point motion — the
    ///     unique trajectory shape that minimizes the integral of squared jerk subject to
    ///     zero velocity and acceleration at both endpoints). Re-planned from the current
    ///     smoothed point every time the mouse goal moves, so the end effector always
    ///     eases in and out instead of snapping to follow the cursor 1:1.
    /// </summary>
    Vector3 EvaluateMinimumJerkTrajectory(float time)
    {
        if (m_TrajectoryDuration <= 0f)
        {
            return m_TrajEndPos;
        }

        var t = Mathf.Clamp01((time - m_TrajStartTime) / m_TrajectoryDuration);
        var s = 10f * t * t * t - 15f * t * t * t * t + 6f * t * t * t * t * t;
        return Vector3.LerpUnclamped(m_TrajStartPos, m_TrajEndPos, s);
    }

    /// <summary>
    ///     Closed-loop damped least squares (DLS) / singularity-robust inverse IK step.
    ///     Builds the 3x6 positional geometric Jacobian for the arm chain directly from
    ///     live transforms (column i = axis_i x (p_endEffector - p_i) for a revolute
    ///     joint), clamps the position error to a max step, ramps up damping when the
    ///     arm is near a singularity (low manipulability), and solves
    ///     dq = J^T (J J^T + lambda^2 I)^-1 * error
    ///     applying it to all 6 joints at once so they move together in sync.
    /// </summary>
    void SolveAndApplyIkStep(Vector3 positionError)
    {
        if (positionError.magnitude < 1e-5f)
        {
            m_DebugStatus = "converged (error < 0.01mm)";
            return;
        }

        var clampedError = Vector3.ClampMagnitude(positionError, m_MaxStepMeters);
        var endEffectorPos = m_EndEffector.position;

        var jacobianColumns = new Vector3[k_NumRobotJoints];
        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            var joint = m_ArmJoints[i];
            var axisWorld = joint.transform.TransformDirection(joint.anchorRotation * Vector3.right).normalized;
            var jointPos = joint.transform.TransformPoint(joint.anchorPosition);
            jacobianColumns[i] = Vector3.Cross(axisWorld, endEffectorPos - jointPos);
        }

        // J * J^T (3x3), built from the 3-vectors above (row i of J^T == column i of J == jacobianColumns[i]).
        float m00 = 0f, m01 = 0f, m02 = 0f, m11 = 0f, m12 = 0f, m22 = 0f;
        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            var c = jacobianColumns[i];
            m00 += c.x * c.x;
            m01 += c.x * c.y;
            m02 += c.x * c.z;
            m11 += c.y * c.y;
            m12 += c.y * c.z;
            m22 += c.z * c.z;
        }

        // Singularity-robust inverse: manipulability ~ sqrt(det(J J^T)) measures how far
        // the arm is from a singular pose. Ramp damping up sharply as it drops toward 0
        // instead of using a fixed lambda, so motion stays smooth near full
        // extension/retraction instead of spiking.
        var rawDet = m00 * (m11 * m22 - m12 * m12) - m01 * (m01 * m22 - m12 * m02) + m02 * (m01 * m12 - m11 * m02);
        var manipulability = Mathf.Sqrt(Mathf.Max(rawDet, 0f));
        var singularityFactor = manipulability < m_ManipulabilityThreshold
            ? 1f + Mathf.Pow(1f - manipulability / m_ManipulabilityThreshold, 2f) * 4f
            : 1f;
        var effectiveLambda = m_DampingLambda * singularityFactor;
        var lambdaSq = effectiveLambda * effectiveLambda;

        m00 += lambdaSq;
        m11 += lambdaSq;
        m22 += lambdaSq;

        if (!Invert3x3Symmetric(m00, m01, m02, m11, m12, m22, out var inv))
        {
            m_DebugStatus = "singular Jacobian (det~0), skipped";
            return; // singular, skip this frame
        }

        // weighted = (J J^T + lambda^2 I)^-1 * clampedError
        var wx = inv.m00 * clampedError.x + inv.m01 * clampedError.y + inv.m02 * clampedError.z;
        var wy = inv.m01 * clampedError.x + inv.m11 * clampedError.y + inv.m12 * clampedError.z;
        var wz = inv.m02 * clampedError.x + inv.m12 * clampedError.y + inv.m22 * clampedError.z;
        var weighted = new Vector3(wx, wy, wz);

        var totalDqDeg = 0f;
        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            var dq = Vector3.Dot(jacobianColumns[i], weighted); // radians
            var joint = m_ArmJoints[i];
            var drive = joint.xDrive;
            var deltaDeg = dq * Mathf.Rad2Deg;
            var newTarget = drive.target + deltaDeg;

            if (drive.lowerLimit != 0f || drive.upperLimit != 0f)
            {
                newTarget = Mathf.Clamp(newTarget, drive.lowerLimit, drive.upperLimit);
            }

            drive.target = newTarget;
            joint.xDrive = drive;
            totalDqDeg += Mathf.Abs(deltaDeg);
        }

        m_DebugStatus = $"err={positionError.magnitude:F4}m manip={manipulability:F5} lambda={effectiveLambda:F3} totalDq={totalDqDeg:F2}deg";
    }

    struct Sym3x3
    {
        public float m00, m01, m02, m11, m12, m22;
    }

    static bool Invert3x3Symmetric(float m00, float m01, float m02, float m11, float m12, float m22, out Sym3x3 inv)
    {
        // Full symmetric matrix:
        // | m00 m01 m02 |
        // | m01 m11 m12 |
        // | m02 m12 m22 |
        var c00 = m11 * m22 - m12 * m12;
        var c01 = -(m01 * m22 - m12 * m02);
        var c02 = m01 * m12 - m11 * m02;
        var det = m00 * c00 + m01 * c01 + m02 * c02;

        if (Mathf.Abs(det) < 1e-8f)
        {
            inv = default;
            return false;
        }

        var invDet = 1f / det;
        var c11 = m00 * m22 - m02 * m02;
        var c12 = -(m00 * m12 - m01 * m02);
        var c22 = m00 * m11 - m01 * m01;

        inv = new Sym3x3
        {
            m00 = c00 * invDet,
            m01 = c01 * invDet,
            m02 = c02 * invDet,
            m11 = c11 * invDet,
            m12 = c12 * invDet,
            m22 = c22 * invDet
        };
        return true;
    }

    /// <summary>
    ///     Flags the target as unreachable (shown in red) only when the tracking error
    ///     fails to meaningfully shrink over a multi-second window — this happens when
    ///     the desired position is outside the arm's workspace or past a joint limit, so
    ///     the solver keeps stepping at max speed but never catches up. A short window
    ///     would false-trigger on ordinary PD-drive lag (the physical joints take real
    ///     time to catch up to a new xDrive target even for perfectly reachable points),
    ///     so this compares against the error recorded at the start of a longer window
    ///     rather than frame-to-frame.
    /// </summary>
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
            return;
        }

        if (Time.unscaledTime - m_WindowStartTime < m_UnreachableWindowSeconds)
        {
            return; // still within the grace window, let the PD drive catch up
        }

        var improvement = (m_WindowStartErrorMag - errorMag) / m_WindowStartErrorMag;
        m_TargetUnreachable = improvement < m_UnreachableMinImprovementFrac;

        // start a fresh window either way, so we keep re-checking rather than latching forever
        m_WindowStartTime = Time.unscaledTime;
        m_WindowStartErrorMag = errorMag;
    }

    void JogGripper(float degreesDelta)
    {
        ApplyGripperDelta(m_RightGripper, degreesDelta);
        ApplyGripperDelta(m_LeftGripper, degreesDelta);
    }

    static void ApplyGripperDelta(ArticulationBody gripper, float degreesDelta)
    {
        if (gripper == null)
        {
            return;
        }

        var drive = gripper.xDrive;
        var newTarget = drive.target + degreesDelta;

        if (drive.lowerLimit != 0f || drive.upperLimit != 0f)
        {
            newTarget = Mathf.Clamp(newTarget, drive.lowerLimit, drive.upperLimit);
        }

        drive.target = newTarget;
        gripper.xDrive = drive;
    }

    void OnGUI()
    {
        if (!m_ManualModeEnabled)
        {
            return;
        }

        GUILayout.BeginArea(new Rect(10, 60, 360, 260), GUI.skin.box);
        GUILayout.Label("Manual IK Control (all joints move in sync)");
        GUILayout.Label($"Max reach from base: {m_MaxReachMeters:F3} m (base: {m_ArmBasePosition:F3})");
        GUILayout.Label($"Debug: {m_DebugStatus}");
        GUILayout.Label($"Drag mode: {(m_VerticalDragMode ? "Vertical plane (X=camera-right, Y=world-up)" : "Horizontal plane (X=ground-right, Y=ground-forward)")} — double-click to switch, diagonal drag combines both axes");
        GUILayout.Label("Left/Right arrow: move depth (camera forward/back)");
        GUILayout.Label("Up/Down arrow: open/close gripper");
        if (m_EndEffector != null)
        {
            var desiredColor = m_TargetUnreachable ? Color.red : GUI.skin.label.normal.textColor;
            var prevColor = GUI.contentColor;
            GUI.contentColor = desiredColor;
            GUILayout.Label($"Desired pos: {m_DesiredPosition:F3}" + (m_TargetUnreachable ? "  (UNREACHABLE)" : ""));
            GUI.contentColor = prevColor;

            GUILayout.Label($"Trajectory pt: {EvaluateMinimumJerkTrajectory(Time.time):F3} (minimum-jerk smoothed goal)");
            GUILayout.Label($"Actual pos:  {m_EndEffector.position:F3}");
            GUILayout.Label($"Tracking error: {Vector3.Distance(m_DesiredPosition, m_EndEffector.position):F4} m");
        }
        GUILayout.EndArea();
    }
}
