---
sidebar_position: 4
---

# ROS 2 برج کے مسائل کے لیے Unity ٹربل شوٹنگ

یہ ٹیوٹوریل Unity کو ROS 2 سسٹمز سے جوڑتے وقت پیش آنے والے عام مسائل کا احاطہ کرتا ہے اور ان مسائل کے حل اور ٹربل شوٹنگ فراہم کرتا ہے

## عام Unity-ROS 2 برج کے مسائل

### 1. کنکشن کے مسائل

#### مسئلہ: Unity اور ROS 2 بات چیت نہیں کر سکتے
**علامات**: Unity اور ROS 2 نوڈز کے درمیان کوئی ڈیٹا نہیں جا رہا
**حل**:
1. **نیٹ ورک کنفیگریشن چیک کریں**:
   ```bash
   # Verify ROS 2 environment
   echo $ROS_DOMAIN_ID
   echo $ROS_LOCALHOST_ONLY

   # For Unity to connect to ROS 2, they usually need to be on the same network
   # If running locally, set:
   export ROS_LOCALHOST_ONLY=0
   ```

2. **IP ایڈریسز اور پورٹس کی تصدیق کریں**:
   ```bash
   # Check if both systems can see each other
   ping <unity_machine_ip>
   ping <ros_machine_ip>

   # Check if ROS 2 ports are accessible
   nmap -p 11811 <target_ip>  # Default ROS 2 port
   ```

3. **فائر وال سیٹنگز چیک کریں**:
   ```bash
   # Ubuntu firewall
   sudo ufw status
   sudo ufw allow from <unity_ip> to any port <ros_port>
   ```

#### مسئلہ: DDS مواصلات کی ناکامیاں
**علامات**: DDS شرکاء کے بارے میں ایرر پیغامات، غیر مماثل QoS پروفائلز
**حل**:
1. **ROS 2 ڈسٹری بیوشن کی مطابقت کی تصدیق کریں**:
   ```bash
   # Ensure both Unity bridge and ROS 2 nodes use the same distribution
   ros2 --version
   # Unity bridge should be built for the same ROS 2 version
   ```

2. **QoS پروفائل سیٹنگز چیک کریں**:
   ```csharp
   // In Unity C# scripts, ensure QoS profiles match ROS 2 defaults
   var qos = QoSProfile.Default;
   // Or explicitly set matching profiles
   var qos = QoSProfile.SensorData;
   ```

### 2. کارکردگی کے مسائل

#### مسئلہ: Unity اور ROS 2 کے درمیان زیادہ تاخیر (Latency)
**علامات**: تاخیر سے ردعمل، فریم ڈراپس، جھٹکے دار حرکتیں
**حل**:
1. **نیٹ ورک بینڈوڈتھ کو بہتر بنائیں**:
   - نیٹ ورک اسٹریمنگ کے لیے Unity رینڈرنگ کوالٹی کم کریں
   - ریئل ٹائم ایپلیکیشنز کے لیے کم ریزولیوشن والے ٹیکسچرز استعمال کریں
   - لیول آف ڈیٹیل (LOD) سسٹمز نافذ کریں

2. **اپ ڈیٹ ریٹس کو ایڈجسٹ کریں**:
   ```csharp
   // In Unity, don't update too frequently
   private float updateInterval = 0.033f; // ~30 Hz
   private float lastUpdateTime = 0f;

   void Update() {
       if (Time.time - lastUpdateTime > updateInterval) {
           UpdateROSData();
           lastUpdateTime = Time.time;
       }
   }
   ```

3. **پیغام کے سائز کو بہتر بنائیں**:
   - بڑے ڈیٹا کو کمپریس کریں (تصاویر، پوائنٹ کلاؤڈز)
   - مناسب ڈیٹا کی اقسام استعمال کریں (غیر ضروری درستگی سے گریز کریں)
   - ہائی فریکوئنسی پیغامات کو محدود کریں

#### مسئلہ: Unity فریم ریٹ ڈراپس
**علامات**: رکاوٹ، کم FPS، خراب وژولائزیشن کوالٹی
**حل**:
1. **Unity سین کو بہتر بنائیں**:
   - 3D ماڈلز میں پولیگون کی تعداد کم کریں
   - occlusion culling کا استعمال کریں
   - متحرک اشیاء کے لیے آبجیکٹ پولنگ نافذ کریں

2. **رینڈرنگ سیٹنگز کو ایڈجسٹ کریں**:
   ```csharp
   // In Unity C# scripts
   QualitySettings.SetQualityLevel(2); // Lower quality for real-time
   Application.targetFrameRate = 30;   // Cap frame rate for stability
   ```

### 3. ڈیٹا سنکرونائزیشن کے مسائل

#### مسئلہ: ٹائم سنکرونائزیشن کے مسائل
**علامات**: Unity میں روبوٹ کی حرکتیں ROS 2 سمولیشن ٹائم سے میل نہیں کھاتیں
**حل**:
1. **مناسب ٹائم ہینڈلنگ نافذ کریں**:
   ```csharp
   using RosMessageTypes.BuiltinInterfaces;
   using Unity.Robotics.ROSTCPConnector;

   // Use ROS time instead of Unity time
   builtin_interfaces.msg.Time rosTime = new builtin_interfaces.msg.Time();
   rosTime.sec = (int)System.DateTime.UtcNow.Subtract(
       new System.DateTime(1970, 1, 1)).TotalSeconds;
   rosTime.nanosec = (uint)(System.DateTime.UtcNow.Millisecond * 1000000);
   ```

2. **سمولیشن ٹائم کو سنکرونائز کریں**:
   ```csharp
   // If using Gazebo + Unity, sync the simulation time
   // Send simulation time from Gazebo to Unity
   // Use the same time base for both systems
   ```

#### مسئلہ: کوآرڈینیٹ سسٹم کی عدم مطابقت
**علامات**: روبوٹ Unity میں غلط پوزیشن/واقفیت میں ظاہر ہوتا ہے
**حل**:
1. **کوآرڈینیٹ سسٹم کے فرق کو سمجھیں**:
   - ROS: X آگے، Y بائیں، Z اوپر
   - Unity: X دائیں، Y اوپر، Z آگے
   - مناسب ٹرانسفارمیشنز کا اطلاق کریں

2. **کوآرڈینیٹ ٹرانسفارمیشن نافذ کریں**:
   ```csharp
   // Convert ROS coordinates to Unity coordinates
   public Vector3 RosToUnityPosition(Vector3 rosPos) {
       return new Vector3(rosPos.y, rosPos.z, rosPos.x);
   }

   public Quaternion RosToUnityRotation(Quaternion rosRot) {
       // Convert quaternion from ROS to Unity coordinate system
       return new Quaternion(rosRot.y, rosRot.z, rosRot.x, -rosRot.w);
   }
   ```

## Unity مخصوص ٹربل شوٹنگ

### 1. Unity ایڈیٹر بمقابلہ بلڈ کے مسائل

#### مسئلہ: برج Unity ایڈیٹر میں کام کرتا ہے لیکن بلڈ میں نہیں
**علامات**: ایڈیٹر میں کنکشن کامیاب، اسٹینڈ لون بلڈ میں ناکام
**حل**:
1. **بلڈ سیٹنگز چیک کریں**:
   - یقینی بنائیں کہ تمام ضروری DLLs بلڈ میں شامل ہیں
   - پلیٹ فارم کے لیے مخصوص سیٹنگز کی تصدیق کریں
   - ٹارگٹ تعیناتی والے پلیٹ فارم پر ٹیسٹ کریں

2. **بلڈز میں ایڈریس ریزولیوشن**:
   ```csharp
   // Use configurable IP addresses
   [SerializeField] private string rosIpAddress = "127.0.0.1";
   [SerializeField] private int rosPort = 8888;
   ```

### 2. اثاثہ (Asset) اور سین مینجمنٹ

#### مسئلہ: بڑے Unity سینز کارکردگی کے مسائل پیدا کر رہے ہیں
**علامات**: طویل لوڈنگ ٹائم، میموری کے مسائل، کریشز
**حل**:
1. **بڑے اثاثوں کے لیے AssetBundles کا استعمال کریں**:
   ```csharp
   // Load assets on demand
   using (var request = UnityWebRequestAssetBundle.GetAssetBundle(url)) {
       yield return request.SendWebRequest();
       var bundle = DownloadHandlerAssetBundle.GetContent(request);
       var prefab = bundle.LoadAsset<GameObject>("RobotPrefab");
   }
   ```

2. **سین اسٹریمنگ نافذ کریں**:
   - بڑے سینز کو چھوٹے حصوں میں توڑیں
   - ڈائنامک لوڈنگ کے لیے Addressables سسٹم استعمال کریں
   - دور کی اشیاء کے لیے LOD نافذ کریں

## ROS 2 برج ٹربل شوٹنگ

### 1. میسج ٹائپ کے مسائل

#### مسئلہ: میسج سیریلائزیشن/ڈی سیریلائزیشن کے مسائل
**علامات**: پیغامات موصول نہیں ہوئے، ڈیٹا کرپشن، ٹائپ کی عدم مطابقت
**حل**:
1. **میسج کی تعریفوں کی تصدیق کریں**:
   ```bash
   # Check if message types are properly installed
   ros2 interface show std_msgs/msg/String
   ros2 interface show geometry_msgs/msg/Twist
   ```

2. **میسج کی مطابقت چیک کریں**:
   ```csharp
   // In Unity C# scripts, ensure message types match
   var publisher = rosConnection.Publish<geometry_msgs.msg.Twist>("cmd_vel");
   // Must match ROS 2 node subscriber type
   ```

### 2. نیٹ ورک کنفیگریشن کے مسائل

#### مسئلہ: متعدد ROS 2 ڈومینز کا مداخلت کرنا
**علامات**: غلط روبوٹس سے پیغامات، ڈومین تنازعات
**حل**:
1. **مخصوص ڈومین آئی ڈیز استعمال کریں**:
   ```bash
   # Set ROS domain for specific robot/area
   export ROS_DOMAIN_ID=10  # Use different domains for different robots
   ```

2. **نیٹ ورک کی تنہائی (Isolation)**:
   ```bash
   # Use different networks for different robot systems
   # Or use ROS 2 namespaces
   ros2 run your_package your_node --ros-args --remap __ns:=/robot1
   ```

## ڈیبگنگ ٹولز اور تکنیکیں

### 1. Unity ڈیبگنگ

#### Unity Profiler کا استعمال
```csharp
// Profile network operations
using UnityEngine.Profiling;

void SendToROS() {
    Profiler.BeginSample("SendToROS");
    // Your network code here
    Profiler.EndSample();
}
```

#### حسب ضرورت ڈیبگنگ ٹولز
```csharp
// Create a debug panel in Unity
public class ROSDebugPanel : MonoBehaviour {
    private string debugText = "";

    void OnGUI() {
        debugText = $"Connected: {rosConnection.IsConnected}\n" +
                   $"Messages sent: {messagesSent}\n" +
                   $"Messages received: {messagesReceived}";
        GUI.Label(new Rect(10, 10, 300, 100), debugText);
    }
}
```

### 2. ROS 2 ڈیبگنگ

#### ROS 2 ٹولز کا استعمال
```bash
# Monitor topics
ros2 topic list
ros2 topic echo /your_topic

# Check service availability
ros2 service list
ros2 service call /your_service std_srvs/srv/Trigger

# Monitor nodes
ros2 node list
ros2 run demo_nodes_cpp talker
```

### 3. نیٹ ورک ڈیبگنگ

#### نیٹ ورک مانیٹرنگ ٹولز کا استعمال
```bash
# Monitor network traffic
sudo tcpdump -i any port 8888  # Default Unity ROS TCP port

# Check network connections
netstat -tuln | grep 8888

# Monitor bandwidth usage
iftop -i <interface>
```

## عام خرابی کے پیغامات اور حل

### ایرر: "Failed to connect to ROS"
**وجہ**: نیٹ ورک کنیکٹیویٹی کے مسائل
**حل**:
1. IP ایڈریسز اور پورٹس کی تصدیق کریں
2. فائر وال سیٹنگز چیک کریں
3. یقینی بنائیں کہ ROS 2 ڈیمن چل رہا ہے: `ros2 daemon start`

### ایرر: "Message type not found"
**وجہ**: میسج کی تعریفیں غائب ہیں
**حل**:
1. ROS 2 سیٹ اپ سورس کریں: `source /opt/ros/humble/setup.bash`
2. یقینی بنائیں کہ ضروری پیکیجز انسٹال ہیں
3. میسج ٹائپ کی ہجے اور کیس کی تصدیق کریں

### ایرر: "DDS participant creation failed"
**وجہ**: DDS کنفیگریشن کے مسائل
**حل**:
1. ROS_DOMAIN_ID ماحولیاتی متغیر چیک کریں
2. FastDDS/RMW کنفیگریشن کی تصدیق کریں
3. ROS 2 ڈیمن کو دوبارہ شروع کریں

## ٹیسٹنگ اور توثیق

### 1. کنکشن ٹیسٹنگ

Unity-ROS 2 کنکشن کی تصدیق کے لیے ایک سادہ ٹیسٹ بنائیں:

```csharp
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std_msgs;
using UnityEngine;

public class ConnectionTester : MonoBehaviour {
    private ROSConnection ros;
    private float testInterval = 5f;
    private float lastTestTime = 0f;

    void Start() {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisteredTopicListUpdated.AddListener(OnTopicsUpdated);
    }

    void Update() {
        if (Time.time - lastTestTime > testInterval) {
            TestConnection();
            lastTestTime = Time.time;
        }
    }

    void TestConnection() {
        if (ros.IsConnected) {
            // Send a simple message to test
            var msg = new StringMsg("Connection test at: " + Time.time);
            ros.Publish("unity_test", msg);
            Debug.Log("Connection test: OK");
        } else {
            Debug.LogError("Connection test: FAILED - Not connected");
        }
    }

    void OnTopicsUpdated(string[] topics) {
        Debug.Log("Available topics: " + string.Join(", ", topics));
    }
}
```

### 2. ڈیٹا کی توثیق

```csharp
// Validate received data
void OnMessageReceived(MessageType msg) {
    // Validate message contents
    if (msg != null) {
        // Check for reasonable values
        if (IsMessageValid(msg)) {
            ProcessMessage(msg);
        } else {
            Debug.LogWarning("Received invalid message data");
        }
    }
}

bool IsMessageValid(MessageType msg) {
    // Implement validation logic
    // Check for NaN, infinity, reasonable ranges, etc.
    return true; // or false based on validation
}
```

## استحکام کے لیے بہترین طریقے

### 1. کنکشن کا انتظام
- دوبارہ کنکشن (reconnection) کی منطق نافذ کریں
- کنکشن اسٹیٹس انڈیکیٹرز استعمال کریں
- منقطع ہونے کو احسن طریقے سے ہینڈل کریں

### 2. ایرر ہینڈلنگ
- نیٹ ورک آپریشنز کے لیے try-catch بلاکس نافذ کریں
- فال بیک (fallback) رویے فراہم کریں
- ڈیبگنگ کے لیے غلطیوں کو لاگ کریں

### 3. وسائل کا انتظام
- نیٹ ورک کنکشنز کو مناسب طریقے سے نمٹائیں
- میموری کے استعمال کی نگرانی کریں
- پیغامات کے لیے کوڑا کرکٹ جمع کرنے (garbage collection) کو نافذ کریں

## اگلے اقدامات

ایک بار جب آپ Unity-ROS 2 برج کے مسائل حل کر لیتے ہیں:

1. اپنے علم کا اطلاق کرنے کے لیے [اسیسمنٹ پروجیکٹ](../assessment-project/project-overview.md) پر جاری رکھیں
2. جامع سمجھ کو یقینی بنانے کے لیے ماڈیول 2 کے تمام مواد کا جائزہ لیں
3. Gazebo سے Unity تک اپنی مکمل سمولیشن پائپ لائن کو ٹیسٹ کریں
