---
sidebar_position: 3
---

# Unity میں وژولائزیشن تکنیک

یہ ٹیوٹوریل روبوٹکس ایپلی کیشنز کے لیے Unity میں جدید وژولائزیشن تکنیکوں کا احاطہ کرتا ہے۔ آپ سیکھیں گے کہ روبوٹس اور ماحول کی ہائی فیڈیلیٹی وژولائزیشن کیسے بنائی جائے جو حقیقی دنیا کی طبیعیات اور سینسر ڈیٹا کی درست نمائندگی کرے۔

## بنیادی وژولائزیشن تصورات

### حقیقت پسندانہ لائٹنگ اور سائے (Shadows)

ایسی لائٹنگ بنانا جو طبعی ماحول سے مطابقت رکھتی ہو:

#### روبوٹکس کے لیے لائٹنگ سیٹ اپ

1. **سورج کی نقل کے لیے ڈائریکشنل لائٹ**:
   - سورج کی روشنی کی نقل کرنے کے لیے ایک ہی ڈائریکشنل لائٹ استعمال کریں
   - مناسب رنگ کا درجہ حرارت سیٹ کریں (عام طور پر دن کی روشنی کے لیے 6500K)
   - حقیقت پسندانہ روبوٹ-ماحول تعامل کے لیے سائے کنفیگر کریں

2. **ماحولیاتی لائٹنگ**:
   - درست ماحول کے عکس (reflections) کے لیے ریفلیکشن پروبز استعمال کریں
   - حقیقی دنیا کے حالات سے مطابقت رکھنے کے لیے محیطی روشنی (ambient lighting) کنفیگر کریں
   - پیچیدہ لائٹنگ منظرناموں کے لیے امیج بیسڈ لائٹنگ (IBL) نافذ کریں

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

### میٹریل خصوصیات اور ٹیکسچرز

روبوٹ کے اجزاء کے لیے حقیقت پسندانہ میٹریل بنانا:

#### روبوٹ میٹریل سیٹ اپ

1. **روبوٹ کے حصوں کے لیے دھاتی میٹریل**:
   - دھاتی اجزاء کے لیے اعلی دھاتی اقدار (high metallic values) استعمال کریں
   - دھات کی مختلف اقسام کے لیے مناسب ہمواری (smoothness) کنفیگر کریں
   - سطح کی تفصیلات کے لیے نارمل میپس شامل کریں

2. **کیسنگز کے لیے پلاسٹک میٹریل**:
   - کم دھاتی اقدار (0-0.2)
   - پلاسٹک کی مختلف اقسام کے لیے مناسب ہمواری
   - حقیقی روبوٹ کی ظاہری شکل سے مطابقت رکھنے کے لیے رنگوں کے تغیرات

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

## اینیمیشن اور کائینیٹک وژولائزیشن

### جوائنٹ وژولائزیشن

روبوٹ جوائنٹ کی نقل و حرکت کو درست طریقے سے وژولائز کرنا:

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

### پاتھ اور ٹریجیکٹری وژولائزیشن

روبوٹ کی نقل و حرکت کے راستوں اور منصوبہ بند ٹریجیکٹریز کو وژولائز کرنا:

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

## سینسر وژولائزیشن

### LiDAR ڈیٹا وژولائزیشن

Unity میں LiDAR اسکین ڈیٹا کو وژولائز کرنا:

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

### کیمرہ فیڈ انضمام

وژولائزیشن کے لیے کیمرہ فیڈز کو ضم کرنا:

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

## کارکردگی کی اصلاح (Performance Optimization)

### روبوٹکس کے لیے لیول آف ڈیٹیل (LOD)

پیچیدہ روبوٹ ماڈلز کے لیے LOD نافذ کرنا:

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

### ماحول کے لیے Occlusion Culling

بڑے ماحول کے لیے Unity کی occlusion culling کا استعمال:

1. **جامد اشیاء کو نشان زد کریں**: Unity میں ماحول کی اشیاء کو "Static" کے طور پر نشان زد کریں
2. **Occlusion Culling بیک کریں**: Window → Rendering → Lighting → Occlusion Culling → Bake
3. **پورٹلز کو بہتر بنائیں**: عمارتوں اور کمروں کے لیے occlusion portals بنائیں

## جدید وژولائزیشن خصوصیات

### پوائنٹ کلاؤڈ وژولائزیشن

3D سینسر ڈیٹا جیسے ڈیپتھ کیمروں یا 3D LiDAR کے لیے:

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

### سینسر فیوژن وژولائزیشن

متعدد سینسر وژولائزیشنز کو اکٹھا کرنا:

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

## بہترین طریقے

### وژولائزیشن کے رہنما خطوط

1. **رنگ کوڈنگ**: روبوٹ کی مختلف حالتوں کے لیے مستقل رنگ استعمال کریں
2. **اسکیل کی درستگی**: درست ادراک کے لیے حقیقی دنیا کے اسکیل کو برقرار رکھیں
3. **فریم کی مستقل مزاجی**: ROS کے ساتھ کوآرڈینیٹ فریم کی مستقل مزاجی کو یقینی بنائیں
4. **کارکردگی کی نگرانی**: فریم ریٹس کی نگرانی کریں اور اس کے مطابق اصلاح کریں

### رسائی کے تحفظات (Accessibility Considerations)

1. **کلر بلائنڈنس**: رنگوں کے علاوہ پیٹرنز اور ٹیکسچرز کا استعمال کریں
2. **ٹیکسٹ سائز**: یقینی بنائیں کہ ٹیکسٹ مختلف فاصلوں پر پڑھنے کے قابل ہے
3. **تضاد (Contrast)**: مرئیت کے لیے ہائی کنٹراسٹ برقرار رکھیں
4. **متبادل نظارے**: متعدد وژولائزیشن موڈز فراہم کریں

## وژولائزیشن کوالٹی کی جانچ

### کوالٹی اشورنس چیک لسٹ

- [ ] روبوٹ ماڈلز ماحول کے مقابلے میں درست اسکیل برقرار رکھتے ہیں
- [ ] جوائنٹ کی نقل و حرکت کی درست نمائندگی کی گئی ہے
- [ ] سینسر ڈیٹا کی وژولائزیشن واضح اور معلوماتی ہے
- [ ] لائٹنگ اور سائے حقیقت پسندانہ دکھائی دیتے ہیں
- [ ] وژولائزیشن کے دوران فریم ریٹ مستحکم رہتا ہے
- [ ] کوآرڈینیٹ سسٹمز ROS کنونشنز کے ساتھ ہم آہنگ ہیں
- [ ] میٹریلز اور ٹیکسچرز روبوٹکس کے لیے موزوں ہیں

## اگلے اقدامات

ان وژولائزیشن تکنیکوں کو نافذ کرنے کے بعد:

1. عام مسائل کے لیے [Unity ٹربل شوٹنگ](./unity-troubleshooting.md) کا جائزہ لیں
2. حقیقی روبوٹ ڈیٹا کے ساتھ اپنے مکمل Unity انضمام کو ٹیسٹ کریں
3. اپنے ٹارگٹ ہارڈویئر کے لیے وژولائزیشن کی کارکردگی کو بہتر بنائیں
4. عمیق روبوٹکس وژولائزیشن کے لیے AR/VR انضمام جیسے جدید موضوعات پر غور کریں
