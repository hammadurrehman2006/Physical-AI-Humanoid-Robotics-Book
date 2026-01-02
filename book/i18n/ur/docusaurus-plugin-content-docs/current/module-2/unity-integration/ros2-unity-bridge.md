---
sidebar_position: 2
---

# ROS 2 سے Unity برج

یہ ٹیوٹوریل ROS 2 اور Unity کے درمیان حقیقی وقت کے ڈیٹا کی مطابقت پذیری کے لیے برج کو نافذ کرنے کا احاطہ کرتا ہے۔ آپ اپنے ROS 2 سسٹم کو Unity وژولائزیشن سے جوڑنا سیکھیں گے۔

## برج آرکیٹیکچر

ROS 2 سے Unity برج کے اجزاء کو سمجھنا:

### مواصلاتی تہیں (Communication Layers)

برج عام طور پر مندرجہ ذیل پر مشتمل ہوتا ہے:
- **نیٹ ورک لیئر**: ROS 2 اور Unity کے درمیان مواصلات کو سنبھالتی ہے
- **پیغام کا ترجمہ**: ROS 2 پیغامات کو Unity ڈیٹا اسٹرکچرز میں تبدیل کرتا ہے
- **مطابقت پذیری (Synchronization)**: یقینی بناتی ہے کہ Unity وژولائزیشن ROS 2 اسٹیٹ سے میل کھاتی ہے
- **کارکردگی کا انتظام**: ڈیٹا کی منتقلی کی شرح کو بہتر بناتا ہے

### عام برج کے حل

1. **Unity ROS TCP Connector**: ایک مقبول اوپن سورس حل
2. **ROS#**: Unity کے لیے ROS کلائنٹ کا C# نفاذ
3. **کسٹم WebSocket برجز**: ویب پر مبنی تعیناتیوں کے لیے
4. **DDS پر مبنی براہ راست انضمام**: زیادہ جدید لیکن موثر

## نفاذ کے اقدامات

### مرحلہ 1: مواصلاتی تہہ کو ترتیب دیں

Unity ROS TCP کنیکٹر اپروچ کے لیے:

1. **Unity ROS TCP کنیکٹر پیکیج انسٹال کریں**:
   - Unity Asset Store یا GitHub سے ڈاؤن لوڈ کریں
   - اپنے Unity پروجیکٹ میں درآمد کریں
   - نیٹ ورک کی ترتیبات کو کنفیگر کریں

2. **ROS 2 سائیڈ کو کنفیگر کریں**:
   ```bash
   # Install required ROS 2 packages
   sudo apt install ros-humble-rosbridge-suite
   ```

3. **نیٹ ورک کنفیگریشن**:
   ```bash
   # Launch the ROS bridge server
   ros2 launch rosbridge_server rosbridge_websocket_launch.xml
   ```

### مرحلہ 2: پیغام کی اقسام کی وضاحت کریں

روبوٹکس میں استعمال ہونے والی عام ROS 2 پیغام کی اقسام:

```csharp
// Example of handling geometry_msgs/Twist in Unity
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector;

public class RobotController : MonoBehaviour
{
    private ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<TwistMsg>("cmd_vel");
    }

    void Update()
    {
        // Send velocity commands to ROS
        if (Input.GetKey(KeyCode.W))
        {
            var twist = new TwistMsg();
            twist.linear = new Vector3Msg(0.5f, 0, 0); // Move forward
            ros.Publish("cmd_vel", twist);
        }
    }
}
```

### مرحلہ 3: ڈیٹا پبلشرز/سبسکرائبرز کا نفاذ

```csharp
// Subscriber example for sensor data
using RosMessageTypes.Sensor;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

public class SensorVisualizer : MonoBehaviour
{
    void Start()
    {
        ROSConnection.GetOrCreateInstance()
            .Subscribe<LaserScanMsg>("scan", OnLaserScanReceived);
    }

    void OnLaserScanReceived(LaserScanMsg scan)
    {
        // Process laser scan data for visualization
        Debug.Log($"Received scan with {scan.ranges.Length} points");

        // Update visualization based on scan data
        UpdateLaserVisualization(scan);
    }

    void UpdateLaserVisualization(LaserScanMsg scan)
    {
        // Implement visualization logic here
    }
}
```

### مرحلہ 4: کنکشن کی جانچ

```csharp
// Connection testing script
public class BridgeTester : MonoBehaviour
{
    private ROSConnection ros;
    private float lastMessageTime = 0;
    private bool isConnected = false;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterTopicListUpdatedCallback(OnTopicsUpdated);
    }

    void OnTopicsUpdated(string[] topics)
    {
        isConnected = topics.Length > 0;
        Debug.Log($"Connected to ROS. Available topics: {topics.Length}");
    }

    void Update()
    {
        if (Time.time - lastMessageTime > 5.0f) // Test every 5 seconds
        {
            TestConnection();
            lastMessageTime = Time.time;
        }
    }

    void TestConnection()
    {
        // Send a test message
        var testMsg = new RosMessageTypes.Std.StringMsg("Unity Test Message");
        ros.Publish("unity_test", testMsg);
    }
}
```

## کارکردگی کی اصلاح

### ڈیٹا ریٹ کا انتظام

1. **اعلی تعدد والے پیغامات کو تھروٹل کریں**:
   - LiDAR اسکینز: 10-30 Hz عام طور پر کافی ہے
   - کیمرہ امیجز: ایپلی کیشن کے لحاظ سے 5-15 Hz
   - جوائنٹ اسٹیٹس: درست کنٹرول کے لیے 50-100 Hz

2. **ڈیٹا کمپریشن نافذ کریں**:
   - پوائنٹ کلاؤڈز جیسے بڑے ڈیٹا کو کمپریس کریں
   - مناسب ڈیٹا ٹائپس استعمال کریں (غیر ضروری درستگی سے گریز کریں)
   - وژولائزیشن کے لیے سب سیمپلنگ پر غور کریں

### نیٹ ورک کی اصلاح

```csharp
// Example of rate-limited publisher
public class RateLimitedPublisher : MonoBehaviour
{
    private ROSConnection ros;
    private float publishInterval = 0.1f; // 10 Hz
    private float lastPublishTime = 0f;

    void Update()
    {
        if (Time.time - lastPublishTime >= publishInterval)
        {
            PublishData();
            lastPublishTime = Time.time;
        }
    }

    void PublishData()
    {
        // Publish your data here
    }
}
```

## حفاظتی تحفظات

### نیٹ ورک سیکیورٹی

1. **محفوظ کنکشن استعمال کریں**: پروڈکشن تعیناتیوں کے لیے TLS/SSL پر غور کریں
2. **نیٹ ورک سیگمنٹیشن**: جب ممکن ہو روبوٹکس نیٹ ورکس کو الگ کریں
3. **تصدیق (Authentication)**: مناسب تصدیقی میکانزم نافذ کریں
4. **فائر وال رولز**: ROS 2 پورٹس کے لیے مناسب فائر وال رولز کنفیگر کریں

## عام مسائل کا حل

### کنکشن کے مسائل

**مسئلہ**: Unity ROS 2 سے منسلک نہیں ہو سکتا
**حل**:
- IP ایڈریسز اور پورٹ نمبرز کی تصدیق کریں
- فائر وال کی ترتیبات چیک کریں
- یقینی بنائیں کہ ROS برج سرور چل رہا ہے
- تصدیق کریں کہ ROS_DOMAIN_ID دونوں طرف مماثل ہے

**مسئلہ**: مواصلات میں زیادہ تاخیر (Latency)
**حل**:
- نیٹ ورک بینڈوتھ چیک کریں
- پیغام کے سائز کو بہتر بنائیں
- جب ممکن ہو لوکل نیٹ ورک استعمال کریں
- پیغام کی تعدد کو کم کریں

### ڈیٹا کی مطابقت پذیری

**مسئلہ**: Unity وژولائزیشن ROS 2 سیمولیشن سے پیچھے رہ جاتی ہے
**حل**:
- مناسب ٹائم سنکرونائزیشن نافذ کریں
- ہموار وژولائزیشن کے لیے انٹرپولیشن کا استعمال کریں
- تیزی سے چلنے والے روبوٹس کے لیے پیشین گوئی الگورتھم پر غور کریں

## انضمام کی مثالیں

### بنیادی روبوٹ اسٹیٹ وژولائزیشن

```csharp
// Synchronize robot joint states
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;

public class JointStateVisualizer : MonoBehaviour
{
    public Transform[] jointTransforms; // Assign in inspector
    private Dictionary<string, int> jointMap = new Dictionary<string, int>();

    void Start()
    {
        ROSConnection.GetOrCreateInstance()
            .Subscribe<JointStateMsg>("joint_states", OnJointStatesReceived);

        // Map joint names to transforms
        for (int i = 0; i < jointTransforms.Length; i++)
        {
            jointMap[jointTransforms[i].name] = i;
        }
    }

    void OnJointStatesReceived(JointStateMsg jointState)
    {
        for (int i = 0; i < jointState.name.Count; i++)
        {
            string jointName = jointState.name[i];
            if (jointMap.ContainsKey(jointName))
            {
                int jointIndex = jointMap[jointName];
                float jointAngle = (float)jointState.position[i];

                // Update the joint transform
                jointTransforms[jointIndex].localRotation =
                    Quaternion.Euler(0, jointAngle * Mathf.Rad2Deg, 0);
            }
        }
    }
}
```

## اگلے اقدامات

ROS 2 سے Unity برج کو نافذ کرنے کے بعد:

1. جدید رینڈرنگ کے لیے [وژولائزیشن تکنیک](./visualization-techniques.md) جاری رکھیں
2. عام برج کے مسائل کے لیے [Unity ٹربل شوٹنگ](./unity-troubleshooting.md) کے بارے میں جانیں
3. اپنے مکمل Unity-ROS 2 انضمام کی جانچ کریں
