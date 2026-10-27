using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Reflection;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

// Runs against the real imported robot, without ROS or keyboard automation.
public static class ManualControlValidation
{
    const BindingFlags Flags = BindingFlags.NonPublic | BindingFlags.Instance | BindingFlags.Static;
    static readonly Type ControllerType = typeof(ManualJointControl);
    static readonly List<string> Results = new List<string>();
    static ManualJointControl control;
    static Transform tip;

    static object Field(string name) => ControllerType.GetField(name, Flags).GetValue(control);
    static void Set(string name, object value) => ControllerType.GetField(name, Flags).SetValue(control, value);
    static object Call(string name, params object[] args) => ControllerType.GetMethod(name, Flags).Invoke(control, args);
    static void Check(bool condition, string description)
    {
        if (!condition) throw new Exception(description);
        Results.Add("PASS " + description);
        Debug.Log("[control-validation] PASS " + description);
    }

    static object Input(bool rotation, params KeyCode[] keys)
    {
        var pressed = new HashSet<KeyCode>(keys);
        return Call("ReadInput", new Func<KeyCode, bool>(pressed.Contains), rotation);
    }

    static void Step(int frames, bool rotation = false, params KeyCode[] keys)
    {
        Set("m_Input", Input(rotation, keys));
        for (var i = 0; i < frames; i++)
        {
            Call("FixedUpdate");
            Physics.Simulate(Time.fixedDeltaTime);
        }
    }

    static void Reset()
    {
        EditorSceneManager.OpenScene("Assets/Scenes/ManualControlTestScene.unity");
        control = UnityEngine.Object.FindFirstObjectByType<ManualJointControl>();
        if (control == null) throw new Exception("ManualJointControl missing from test scene");
        var start = (IEnumerator)Call("Start");
        start.MoveNext();
        Physics.Simulate(Time.fixedDeltaTime);
        start.MoveNext();
        tip = (Transform)Field("m_EndEffector");
        Step(100);
    }

    [MenuItem("Jaco/Validate Manual Controls")]
    public static void Run()
    {
        if (EditorApplication.isPlayingOrWillChangePlaymode)
        {
            Debug.LogWarning("Stop Play mode before validating manual controls.");
            return;
        }
        var output = Path.GetFullPath("control-validation.txt");
        var previousSimulation = Physics.simulationMode;
        var previousFixedDelta = Time.fixedDeltaTime;
        var previousScene = EditorSceneManager.GetActiveScene().path;
        if (!Application.isBatchMode && !EditorSceneManager.SaveCurrentModifiedScenesIfUserWantsTo())
            return;
        Results.Clear();
        try
        {
            Physics.simulationMode = SimulationMode.Script;
            Time.fixedDeltaTime = 0.02f;
            Reset();
            Results.Add($"START position={tip.position:F5} approach={tip.up:F5}");
            Check(Vector3.Distance(tip.position, new Vector3(0f, 0.4f, -0.4f)) < 0.01f, "Arched start places tip over work area");
            Check(Vector3.Dot(tip.up, Vector3.down) > 0.999f, "Start tool points down");
            var joints = (ArticulationBody[])Field("m_ArmJoints");
            Check(joints[2].transform.position.y > tip.position.y + 0.15f, "Elbow starts above gripper");
            var startRotation = tip.rotation;
            var startPosition = tip.position;

            foreach (var key in new[] { KeyCode.UpArrow, KeyCode.DownArrow, KeyCode.LeftArrow, KeyCode.RightArrow })
            {
                Reset();
                var before = tip.position;
                var orientation = tip.rotation;
                Step(8, false, key);
                Step(160);
                var expected = key == KeyCode.UpArrow ? Vector3.forward : key == KeyCode.DownArrow ? Vector3.back : key == KeyCode.RightArrow ? Vector3.right : Vector3.left;
                var movement = tip.position - before;
                Results.Add($"{key} delta={movement:F5} angle={Quaternion.Angle(orientation, tip.rotation):F4}");
                Check(Vector3.Dot(movement, expected) > 0.025f, key + " moves in requested direction");
                Check(Mathf.Abs(movement.y) < 0.005f, key + " holds horizontal plane");
                Check(Quaternion.Angle(orientation, tip.rotation) < 1f, key + " holds tool orientation");
            }

            foreach (var key in new[] { KeyCode.W, KeyCode.S })
            {
                Reset();
                var before = tip.position;
                Step(6, false, key);
                Step(160);
                var delta = tip.position - before;
                Results.Add($"{key} delta={delta:F5}");
                Check((key == KeyCode.W ? delta.y : -delta.y) > 0.025f, key + " changes height in correct direction");
                Check(new Vector2(delta.x, delta.z).magnitude < 0.005f, key + " holds horizontal position");
            }

            foreach (var key in new[] { KeyCode.A, KeyCode.D })
            {
                Reset();
                var before = tip.position;
                var baseJoint = ((ArticulationBody[])Field("m_ArmJoints"))[0];
                var beforeAngle = baseJoint.jointPosition[0];
                Step(20, false, key);
                Step(160);
                var turn = Vector3.SignedAngle(new Vector3(before.x, 0, before.z), new Vector3(tip.position.x, 0, tip.position.z), Vector3.up);
                Results.Add($"{key} baseDelta={(baseJoint.jointPosition[0]-beforeAngle)*Mathf.Rad2Deg:F4} worldTurn={turn:F4}");
                Check(key == KeyCode.A ? turn < -3f : turn > 3f, key + " rotates base correctly from above");
                Check(Mathf.Abs(tip.position.y - before.y) < 0.005f, key + " preserves height");
            }

            var positive = new[] { KeyCode.R, KeyCode.P, KeyCode.Y };
            var negative = new[] { KeyCode.G, KeyCode.H, KeyCode.L };
            var axes = new[] { Vector3.up, Vector3.right, Vector3.back };
            for (var axis = 0; axis < 3; axis++)
            {
                foreach (var sign in new[] { 1, -1 })
                {
                    Reset();
                    var key = sign > 0 ? positive[axis] : negative[axis];
                    var before = tip.position;
                    var orientation = tip.rotation;
                    Step(20, true, key);
                    Step(200);
                    var localDelta = Quaternion.Inverse(orientation) * tip.rotation;
                    var rotation = (Vector3)Call("RotationError", localDelta, Quaternion.identity);
                    Results.Add($"{key} localRotDegrees={rotation * Mathf.Rad2Deg:F4} drift={Vector3.Distance(tip.position,before):F5}");
                    Check(Vector3.Dot(rotation, axes[axis]) * sign > 5f * Mathf.Deg2Rad, key + " rotates about correct tool-local axis and sign");
                    Check(Vector3.Cross(rotation, axes[axis]).magnitude < 1.5f * Mathf.Deg2Rad, key + " isolates selected rotation axis");
                    Check(Vector3.Distance(tip.position, before) < 0.005f, key + " holds gripper tip position");
                }
            }

            Reset();
            startPosition = tip.position;
            startRotation = tip.rotation;
            Step(30, true, KeyCode.UpArrow, KeyCode.LeftArrow);
            Step(50);
            Check(Vector3.Distance(tip.position, startPosition) < 0.001f, "Orientation mode ignores arrow keys");
            Step(30, false, KeyCode.R, KeyCode.P, KeyCode.Y);
            Step(50);
            Check(Quaternion.Angle(tip.rotation, startRotation) < 0.1f, "Plane mode ignores rotation keys");
            Step(30, true, KeyCode.R, KeyCode.G, KeyCode.P, KeyCode.H, KeyCode.Y, KeyCode.L);
            Check(Quaternion.Angle(tip.rotation, startRotation) < 0.1f, "Opposite rotation keys cancel");

            // Direction must remain local after the tool has already rotated.
            Step(30, true, KeyCode.R);
            Step(180);
            startPosition = tip.position;
            startRotation = tip.rotation;
            Step(20, true, KeyCode.P);
            Step(180);
            var composed = (Vector3)Call("RotationError", Quaternion.Inverse(startRotation) * tip.rotation, Quaternion.identity);
            Check(composed.x > 5f * Mathf.Deg2Rad && new Vector2(composed.y, composed.z).magnitude < 0.04f, "Pitch follows rotated tool frame");
            Check(Vector3.Distance(tip.position, startPosition) < 0.005f, "Sequential rotations preserve tip position");
            Results.Add("ALL CHECKS PASSED");
            File.WriteAllLines(output, Results);
            if (Application.isBatchMode) EditorApplication.Exit(0);
        }
        catch (Exception exception)
        {
            Results.Add("FAIL " + exception);
            File.WriteAllLines(output, Results);
            Debug.LogException(exception);
            if (Application.isBatchMode) EditorApplication.Exit(1);
            else throw;
        }
        finally
        {
            Physics.simulationMode = previousSimulation;
            Time.fixedDeltaTime = previousFixedDelta;
            if (!Application.isBatchMode && !string.IsNullOrEmpty(previousScene))
                EditorSceneManager.OpenScene(previousScene);
        }
    }
}
