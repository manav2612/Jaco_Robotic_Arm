using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using Unity.Robotics.UrdfImporter.Control;

public static class ManualControlFixer
{
    public static void ImportTmpEssentialsAndOpenTestScene()
    {
        var pkgPath = "Packages/com.unity.ugui/Package Resources/TMP Essential Resources.unitypackage";
        if (System.IO.File.Exists(pkgPath))
        {
            AssetDatabase.importPackageCompleted += _ =>
            {
                EditorSceneManager.OpenScene("Assets/Scenes/ManualControlTestScene.unity");
                Debug.Log("TMP essentials imported and ManualControlTestScene opened.");
            };
            AssetDatabase.ImportPackage(pkgPath, false);
        }
        else
        {
            Debug.LogError("TMP Essential Resources package not found at " + pkgPath);
            EditorSceneManager.OpenScene("Assets/Scenes/ManualControlTestScene.unity");
        }
    }

    public static void ResetManualJointControlAndOpen()
    {
        EditorSceneManager.OpenScene("Assets/Scenes/ManualControlTestScene.unity");

        var j2n6s200 = GameObject.Find("j2n6s200");
        if (j2n6s200 == null)
        {
            Debug.LogError("j2n6s200 GameObject not found.");
            return;
        }

        var existing = j2n6s200.GetComponent<ManualJointControl>();
        if (existing != null)
        {
            Object.DestroyImmediate(existing);
            Debug.Log("Removed stale ManualJointControl component.");
        }

        var fresh = j2n6s200.AddComponent<ManualJointControl>();
        fresh.j2n6s200 = j2n6s200;
        fresh.ManualModeEnabled = true;
        Debug.Log("Added fresh ManualJointControl component with clean defaults.");

        var activeScene = EditorSceneManager.GetActiveScene();
        EditorSceneManager.MarkSceneDirty(activeScene);
        EditorSceneManager.SaveScene(activeScene);
        Debug.Log("ManualControlTestScene saved with reset ManualJointControl.");
    }

    public static void OpenManualTestScene()
    {
        EditorSceneManager.OpenScene("Assets/Scenes/ManualControlTestScene.unity");
        Debug.Log("Opened ManualControlTestScene.");
    }

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
