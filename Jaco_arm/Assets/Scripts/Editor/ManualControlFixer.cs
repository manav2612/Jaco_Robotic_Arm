using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using Unity.Robotics.UrdfImporter.Control;

public static class ManualControlFixer
{
    public static void CreateManualTestScene()
    {
        var sourcePath = "Assets/Scenes/GarbageSortingScene.unity";
        var newPath = "Assets/Scenes/ManualControlTestScene.unity";

        EditorSceneManager.OpenScene(sourcePath);

        // Remove anything that writes to the joints autonomously.
        var publisher = GameObject.Find("Publisher");
        if (publisher != null)
        {
            Object.DestroyImmediate(publisher);
            Debug.Log("Removed Publisher (GarbageSorting autonomous loop) from the test scene.");
        }

        var j2n6s200 = GameObject.Find("j2n6s200");
        if (j2n6s200 == null)
        {
            Debug.LogError("j2n6s200 GameObject not found.");
            return;
        }

        var controller = j2n6s200.GetComponent<Controller>();
        if (controller != null)
        {
            Object.DestroyImmediate(controller);
        }

        var manual = j2n6s200.GetComponent<ManualJointControl>();
        if (manual == null)
        {
            manual = j2n6s200.AddComponent<ManualJointControl>();
            Debug.Log("Added ManualJointControl to j2n6s200.");
        }
        manual.j2n6s200 = j2n6s200;
        manual.ManualModeEnabled = true;

        var activeScene = EditorSceneManager.GetActiveScene();
        var saved = EditorSceneManager.SaveScene(activeScene, newPath, true);
        if (!saved)
        {
            Debug.LogError("Failed to save ManualControlTestScene.");
            return;
        }

        Debug.Log("ManualControlTestScene created and saved at " + newPath);
    }


    public static void OpenSceneAndRemoveController()
    {
        var scenePath = "Assets/Scenes/GarbageSortingScene.unity";
        EditorSceneManager.OpenScene(scenePath);

        var j2n6s200 = GameObject.Find("j2n6s200");
        if (j2n6s200 == null)
        {
            Debug.LogError("j2n6s200 GameObject not found in GarbageSortingScene.");
            return;
        }

        var controller = j2n6s200.GetComponent<Controller>();
        if (controller != null)
        {
            Object.DestroyImmediate(controller);
            Debug.Log("Removed Controller component from j2n6s200.");
        }
        else
        {
            Debug.Log("No Controller component found on j2n6s200 (already removed).");
        }

        var jointControls = j2n6s200.GetComponentsInChildren<JointControl>(true);
        foreach (var jc in jointControls)
        {
            Object.DestroyImmediate(jc);
        }
        Debug.Log($"Removed {jointControls.Length} JointControl components from j2n6s200 children.");

        EditorSceneManager.MarkSceneDirty(EditorSceneManager.GetActiveScene());
        EditorSceneManager.SaveScene(EditorSceneManager.GetActiveScene());
        Debug.Log("GarbageSortingScene opened, Controller removed, and scene saved.");
    }
}
