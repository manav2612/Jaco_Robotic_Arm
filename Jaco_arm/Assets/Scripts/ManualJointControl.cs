using UnityEngine;

/// <summary>
///     Manual mouse- and keyboard-driven jog control for every DOF of the j2n6s200 arm
///     (6 arm joints + 2 gripper fingers), for testing without ROS running.
///     Hold left mouse button and drag horizontally, or hold Left/Right arrow keys,
///     to jog the selected joint. Number keys 1-8 select the active joint
///     (7/8 = right/left gripper finger).
/// </summary>
public class ManualJointControl : MonoBehaviour
{
    const int k_NumRobotJoints = 6;
    const int k_NumControllable = k_NumRobotJoints + 2; // + 2 gripper fingers

    [SerializeField]
    GameObject m_j2n6s200;
    public GameObject j2n6s200 { get => m_j2n6s200; set => m_j2n6s200 = value; }

    [SerializeField]
    bool m_ManualModeEnabled = true;
    public bool ManualModeEnabled { get => m_ManualModeEnabled; set => m_ManualModeEnabled = value; }

    [SerializeField]
    float m_Sensitivity = 2f;

    [SerializeField]
    float m_KeySpeed = 30f; // degrees per second while holding an arrow key

    ArticulationBody[] m_Joints;
    string[] m_JointLabels;
    int m_SelectedJoint;

    void Start()
    {
        m_Joints = new ArticulationBody[k_NumControllable];
        m_JointLabels = new string[k_NumControllable];

        var linkName = string.Empty;
        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            linkName += SourceDestinationPublisher.LinkNames[i];
            m_Joints[i] = m_j2n6s200.transform.Find(linkName).GetComponent<ArticulationBody>();
            m_JointLabels[i] = $"Joint {i + 1}";
        }

        var rightGripper = linkName + "/j2n6s200_link_finger_1";
        var leftGripper = linkName + "/j2n6s200_link_finger_2";

        m_Joints[6] = m_j2n6s200.transform.Find(rightGripper).GetComponent<ArticulationBody>();
        m_JointLabels[6] = "Right Gripper";
        m_Joints[7] = m_j2n6s200.transform.Find(leftGripper).GetComponent<ArticulationBody>();
        m_JointLabels[7] = "Left Gripper";
    }

    void Update()
    {
        if (!m_ManualModeEnabled)
        {
            return;
        }

        for (var key = 0; key < k_NumControllable; key++)
        {
            if (Input.GetKeyDown((KeyCode)((int)KeyCode.Alpha1 + key)))
            {
                m_SelectedJoint = key;
            }
        }

        if (Input.GetMouseButton(0))
        {
            var deltaX = Input.GetAxis("Mouse X");
            JogSelectedJoint(deltaX * m_Sensitivity);
        }

        if (Input.GetKey(KeyCode.RightArrow))
        {
            JogSelectedJoint(m_KeySpeed * Time.deltaTime);
        }
        else if (Input.GetKey(KeyCode.LeftArrow))
        {
            JogSelectedJoint(-m_KeySpeed * Time.deltaTime);
        }
    }

    void JogSelectedJoint(float degreesDelta)
    {
        var joint = m_Joints[m_SelectedJoint];
        if (joint == null)
        {
            return;
        }

        var drive = joint.xDrive;
        var newTarget = drive.target + degreesDelta;

        if (drive.lowerLimit != 0f || drive.upperLimit != 0f)
        {
            newTarget = Mathf.Clamp(newTarget, drive.lowerLimit, drive.upperLimit);
        }

        drive.target = newTarget;
        joint.xDrive = drive;
    }

    void OnGUI()
    {
        if (!m_ManualModeEnabled)
        {
            return;
        }

        GUILayout.BeginArea(new Rect(10, 60, 280, 220), GUI.skin.box);
        GUILayout.Label("Manual Joint Control");
        GUILayout.Label("Keys 1-8 select joint, hold Left/Right arrow or drag mouse to jog");

        for (var i = 0; i < k_NumControllable; i++)
        {
            var joint = m_Joints[i];
            var angle = joint != null ? joint.xDrive.target : 0f;
            var marker = i == m_SelectedJoint ? ">" : " ";
            GUILayout.Label($"{marker} {m_JointLabels[i]}: {angle:F1} deg");
        }

        GUILayout.EndArea();
    }
}
