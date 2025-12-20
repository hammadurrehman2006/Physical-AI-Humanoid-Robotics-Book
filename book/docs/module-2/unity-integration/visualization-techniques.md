---
sidebar_position: 3
---

# Visualization Techniques in Unity

This tutorial covers advanced visualization techniques in Unity for robotics applications. You'll learn how to create high-fidelity visualizations of robots and environments that accurately represent real-world physics and sensor data.

## Core Visualization Concepts

### Realistic Lighting and Shadows

Creating realistic lighting that matches the physical environment:

#### Lighting Setup for Robotics

1. **Directional Light for Sun Simulation**:
   - Use a single directional light to simulate sunlight
   - Set appropriate color temperature (typically 6500K for daylight)
   - Configure shadows for realistic robot-environment interaction

2. **Environment Lighting**:
   - Use reflection probes for accurate environment reflections
   - Configure ambient lighting to match real-world conditions
   - Implement image-based lighting (IBL) for complex lighting scenarios

```csharp
// Dynamic lighting configuration based on time of day
public class RobotEnvironmentLighting : MonoBehaviour
{
    public Light sunLight;
    public float dayLengthSeconds = 120f; // 2 minutes for full day/night cycle

    void Update()
    {
        float timeOfDay = (Time.time % dayLengthSeconds) / dayLengthSeconds;
        UpdateLighting(timeOfDay);
    }

    void UpdateLighting(float timeOfDay)
    {
        // Adjust sun position
        float sunAngle = timeOfDay * 360f - 90f; // Start at horizon
        sunLight.transform.rotation = Quaternion.Euler(sunAngle, 0, 0);

        // Adjust light color and intensity
        if (timeOfDay < 0.25f || timeOfDay > 0.75f)
        {
            // Night time - dim and blue
            sunLight.color = Color.blue * 0.2f;
            sunLight.intensity = 0.1f;
        }
        else if (timeOfDay < 0.3f || timeOfDay > 0.7f)
        {
            // Dawn/dusk - orange and moderate
            sunLight.color = new Color(1f, 0.7f, 0.4f);
            sunLight.intensity = 0.5f;
        }
        else
        {
            // Daytime - white and bright
            sunLight.color = Color.white;
            sunLight.intensity = 1f;
        }
    }
}
```

### Material Properties and Textures

Creating realistic materials for robot components:

#### Robot Material Setup

1. **Metallic Materials for Robot Parts**:
   - Use high metallic values for metal components
   - Configure appropriate smoothness for different metal types
   - Add normal maps for surface details

2. **Plastic Materials for Casings**:
   - Lower metallic values (0-0.2)
   - Appropriate smoothness for different plastic types
   - Color variations to match real robot appearance

```csharp
// Dynamic material switching based on robot state
public class RobotMaterialController : MonoBehaviour
{
    public Material normalMaterial;
    public Material warningMaterial;
    public Material errorMaterial;
    public Renderer[] robotRenderers;

    public void SetRobotState(RobotState state)
    {
        Material targetMaterial = normalMaterial;

        switch (state)
        {
            case RobotState.Warning:
                targetMaterial = warningMaterial;
                break;
            case RobotState.Error:
                targetMaterial = errorMaterial;
                break;
        }

        foreach (Renderer renderer in robotRenderers)
        {
            renderer.material = targetMaterial;
        }
    }
}

public enum RobotState { Normal, Warning, Error }
```

## Animation and Kinematic Visualization

### Joint Visualization

Visualizing robot joint movements accurately:

```csharp
// Joint angle visualization
public class JointVisualizer : MonoBehaviour
{
    public Transform jointTransform;
    public TextMeshProUGUI angleDisplay; // Requires TextMeshPro package
    public float minAngle = -90f;
    public float maxAngle = 90f;

    [Range(0, 1)] public float jointValue = 0f; // 0 to 1 normalized value

    void Update()
    {
        // Convert normalized value to actual joint angle
        float actualAngle = Mathf.Lerp(minAngle, maxAngle, jointValue);

        // Apply rotation to joint
        jointTransform.localRotation = Quaternion.Euler(0, actualAngle, 0);

        // Display angle
        if (angleDisplay != null)
        {
            angleDisplay.text = $"Angle: {actualAngle:F1}°";
        }
    }

    // Method to set joint from external data (e.g., ROS joint state)
    public void SetJointAngle(float angle)
    {
        jointValue = Mathf.InverseLerp(minAngle, maxAngle, angle);
    }
}
```

### Path and Trajectory Visualization

Visualizing robot movement paths and planned trajectories:

```csharp
// Trajectory visualization
public class TrajectoryVisualizer : MonoBehaviour
{
    public LineRenderer lineRenderer;
    public int maxPoints = 100;
    private List<Vector3> trajectoryPoints = new List<Vector3>();

    void Start()
    {
        if (lineRenderer == null)
        {
            lineRenderer = gameObject.AddComponent<LineRenderer>();
            lineRenderer.material = new Material(Shader.Find("Sprites/Default"));
            lineRenderer.color = Color.blue;
            lineRenderer.startWidth = 0.1f;
            lineRenderer.endWidth = 0.1f;
        }
    }

    public void AddTrajectoryPoint(Vector3 point)
    {
        trajectoryPoints.Add(point);

        if (trajectoryPoints.Count > maxPoints)
        {
            trajectoryPoints.RemoveAt(0);
        }

        UpdateTrajectoryLine();
    }

    void UpdateTrajectoryLine()
    {
        lineRenderer.positionCount = trajectoryPoints.Count;
        lineRenderer.SetPositions(trajectoryPoints.ToArray());
    }

    public void ClearTrajectory()
    {
        trajectoryPoints.Clear();
        lineRenderer.positionCount = 0;
    }
}
```

## Sensor Visualization

### LiDAR Data Visualization

Visualizing LiDAR scan data in Unity:

```csharp
// LiDAR scan visualization
public class LidarVisualizer : MonoBehaviour
{
    public GameObject scanPointPrefab;
    public Transform robotTransform;
    private List<GameObject> scanPoints = new List<GameObject>();
    private int maxScanPoints = 1000;

    public void UpdateLidarScan(float[] ranges, float angleMin, float angleIncrement)
    {
        // Clear previous scan points
        foreach (GameObject point in scanPoints)
        {
            DestroyImmediate(point);
        }
        scanPoints.Clear();

        // Create new scan points
        for (int i = 0; i < ranges.Length; i++)
        {
            float distance = ranges[i];

            if (distance > 0.1f && distance < 10.0f) // Valid range
            {
                float angle = angleMin + i * angleIncrement;

                Vector3 pointPos = new Vector3(
                    distance * Mathf.Cos(angle),
                    0.1f, // Height above ground
                    distance * Mathf.Sin(angle)
                );

                // Transform to world space relative to robot
                Vector3 worldPos = robotTransform.TransformPoint(pointPos);

                GameObject pointObj = Instantiate(scanPointPrefab, worldPos, Quaternion.identity);
                scanPoints.Add(pointObj);

                if (scanPoints.Count >= maxScanPoints) break;
            }
        }
    }
}
```

### Camera Feed Integration

Integrating camera feeds for visualization:

```csharp
// Camera feed visualization
public class CameraFeedVisualizer : MonoBehaviour
{
    public Renderer cameraDisplayRenderer;
    private Texture2D cameraTexture;
    private bool textureInitialized = false;

    // Method to update camera feed from ROS image message
    public void UpdateCameraFeed(byte[] imageData, int width, int height)
    {
        if (!textureInitialized)
        {
            cameraTexture = new Texture2D(width, height, TextureFormat.RGB24, false);
            cameraDisplayRenderer.material.mainTexture = cameraTexture;
            textureInitialized = true;
        }

        if (imageData.Length == width * height * 3) // RGB format
        {
            // Convert byte array to Color array
            Color32[] colors = new Color32[width * height];
            for (int i = 0; i < imageData.Length; i += 3)
            {
                colors[i / 3] = new Color32(
                    imageData[i],     // R
                    imageData[i + 1], // G
                    imageData[i + 2], // B
                    255               // A
                );
            }

            cameraTexture.SetPixels32(colors);
            cameraTexture.Apply();
        }
    }
}
```

## Performance Optimization

### Level of Detail (LOD) for Robotics

Implementing LOD for complex robot models:

```csharp
// LOD system for robot models
public class RobotLODController : MonoBehaviour
{
    public Transform[] lodLevels; // Assign LOD0, LOD1, LOD2, etc.
    public float[] lodDistances; // Distance thresholds for each LOD
    private Camera mainCamera;

    void Start()
    {
        mainCamera = Camera.main;
    }

    void Update()
    {
        if (mainCamera != null)
        {
            float distance = Vector3.Distance(transform.position, mainCamera.transform.position);
            ActivateLOD(distance);
        }
    }

    void ActivateLOD(float distance)
    {
        for (int i = 0; i < lodLevels.Length; i++)
        {
            if (distance <= lodDistances[i])
            {
                lodLevels[i].gameObject.SetActive(true);
            }
            else
            {
                lodLevels[i].gameObject.SetActive(false);
            }
        }
    }
}
```

### Occlusion Culling for Environments

Using Unity's occlusion culling for large environments:

1. **Mark Static Objects**: Mark environment objects as "Static" in Unity
2. **Bake Occlusion Culling**: Window → Rendering → Lighting → Occlusion Culling → Bake
3. **Optimize Portals**: Create occlusion portals for buildings and rooms

## Advanced Visualization Features

### Point Cloud Visualization

For 3D sensor data like depth cameras or 3D LiDAR:

```csharp
// Point cloud visualization
public class PointCloudVisualizer : MonoBehaviour
{
    public GameObject pointPrefab;
    private List<GameObject> pointObjects = new List<GameObject>();

    public void UpdatePointCloud(Vector3[] points, Color[] colors = null)
    {
        // Clear previous points
        foreach (GameObject point in pointObjects)
        {
            DestroyImmediate(point);
        }
        pointObjects.Clear();

        for (int i = 0; i < points.Length; i++)
        {
            GameObject pointObj = Instantiate(pointPrefab, points[i], Quaternion.identity);
            pointObjects.Add(pointObj);

            // Apply color if provided
            if (colors != null && i < colors.Length)
            {
                Renderer pointRenderer = pointObj.GetComponent<Renderer>();
                if (pointRenderer != null)
                {
                    pointRenderer.material.color = colors[i];
                }
            }

            if (pointObjects.Count > 10000) // Limit for performance
            {
                Debug.LogWarning("Point cloud limited to 10,000 points for performance");
                break;
            }
        }
    }
}
```

### Sensor Fusion Visualization

Combining multiple sensor visualizations:

```csharp
// Sensor fusion visualization controller
public class SensorFusionVisualizer : MonoBehaviour
{
    public GameObject lidarVisualizer;
    public GameObject cameraVisualizer;
    public GameObject imuVisualizer;
    public GameObject trajectoryVisualizer;

    public void ToggleSensorVisualization(string sensorType, bool visible)
    {
        GameObject sensorGO = null;

        switch (sensorType.ToLower())
        {
            case "lidar":
                sensorGO = lidarVisualizer;
                break;
            case "camera":
                sensorGO = cameraVisualizer;
                break;
            case "imu":
                sensorGO = imuVisualizer;
                break;
            case "trajectory":
                sensorGO = trajectoryVisualizer;
                break;
        }

        if (sensorGO != null)
        {
            sensorGO.SetActive(visible);
        }
    }
}
```

## Best Practices

### Visualization Guidelines

1. **Color Coding**: Use consistent colors for different robot states
2. **Scale Accuracy**: Maintain real-world scale for accurate perception
3. **Frame Consistency**: Ensure coordinate frame consistency with ROS
4. **Performance Monitoring**: Monitor frame rates and optimize accordingly

### Accessibility Considerations

1. **Color Blindness**: Use patterns and textures in addition to colors
2. **Text Size**: Ensure text is readable at various distances
3. **Contrast**: Maintain high contrast for visibility
4. **Alternative Views**: Provide multiple visualization modes

## Testing Visualization Quality

### Quality Assurance Checklist

- [ ] Robot models maintain correct scale relative to environment
- [ ] Joint movements are accurately represented
- [ ] Sensor data visualization is clear and informative
- [ ] Lighting and shadows appear realistic
- [ ] Frame rate remains stable during visualization
- [ ] Coordinate systems align with ROS conventions
- [ ] Materials and textures are appropriate for robotics

## Next Steps

After implementing these visualization techniques:

1. Review [Unity Troubleshooting](./unity-troubleshooting.md) for common issues
2. Test your complete Unity integration with real robot data
3. Optimize visualization performance for your target hardware
4. Consider advanced topics like AR/VR integration for immersive robotics visualization