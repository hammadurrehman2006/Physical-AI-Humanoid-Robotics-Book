---
sidebar_position: 4
---

# Unity Troubleshooting for ROS 2 Bridge Issues

This tutorial covers common issues encountered when connecting Unity to ROS 2 systems and provides solutions for troubleshooting and resolving these problems.

## Common Unity-ROS 2 Bridge Issues

### 1. Connection Problems

#### Issue: Unity and ROS 2 cannot communicate
**Symptoms**: No data flowing between Unity and ROS 2 nodes
**Solutions**:
1. **Check network configuration**:
   ```bash
   # Verify ROS 2 environment
   echo $ROS_DOMAIN_ID
   echo $ROS_LOCALHOST_ONLY

   # For Unity to connect to ROS 2, they usually need to be on the same network
   # If running locally, set:
   export ROS_LOCALHOST_ONLY=0
   ```

2. **Verify IP addresses and ports**:
   ```bash
   # Check if both systems can see each other
   ping <unity_machine_ip>
   ping <ros_machine_ip>

   # Check if ROS 2 ports are accessible
   nmap -p 11811 <target_ip>  # Default ROS 2 port
   ```

3. **Check firewall settings**:
   ```bash
   # Ubuntu firewall
   sudo ufw status
   sudo ufw allow from <unity_ip> to any port <ros_port>
   ```

#### Issue: DDS communication failures
**Symptoms**: Error messages about DDS participants, mismatched QoS profiles
**Solutions**:
1. **Verify ROS 2 distribution compatibility**:
   ```bash
   # Ensure both Unity bridge and ROS 2 nodes use the same distribution
   ros2 --version
   # Unity bridge should be built for the same ROS 2 version
   ```

2. **Check QoS profile settings**:
   ```csharp
   // In Unity C# scripts, ensure QoS profiles match ROS 2 defaults
   var qos = QoSProfile.Default;
   // Or explicitly set matching profiles
   var qos = QoSProfile.SensorData;
   ```

### 2. Performance Issues

#### Issue: High latency between Unity and ROS 2
**Symptoms**: Delayed response, frame drops, jerky movements
**Solutions**:
1. **Optimize network bandwidth**:
   - Reduce Unity rendering quality for network streaming
   - Use lower resolution textures for real-time applications
   - Implement Level of Detail (LOD) systems

2. **Adjust update rates**:
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

3. **Optimize message sizes**:
   - Compress large data (images, point clouds)
   - Use appropriate data types (avoid unnecessary precision)
   - Throttle high-frequency messages

#### Issue: Unity frame rate drops
**Symptoms**: Stuttering, low FPS, poor visualization quality
**Solutions**:
1. **Optimize Unity scene**:
   - Reduce polygon count in 3D models
   - Use occlusion culling
   - Implement object pooling for dynamic objects

2. **Adjust rendering settings**:
   ```csharp
   // In Unity C# scripts
   QualitySettings.SetQualityLevel(2); // Lower quality for real-time
   Application.targetFrameRate = 30;   // Cap frame rate for stability
   ```

### 3. Data Synchronization Problems

#### Issue: Time synchronization issues
**Symptoms**: Robot movements in Unity don't match ROS 2 simulation time
**Solutions**:
1. **Implement proper time handling**:
   ```csharp
   using RosMessageTypes.BuiltinInterfaces;
   using Unity.Robotics.ROSTCPConnector;

   // Use ROS time instead of Unity time
   builtin_interfaces.msg.Time rosTime = new builtin_interfaces.msg.Time();
   rosTime.sec = (int)System.DateTime.UtcNow.Subtract(
       new System.DateTime(1970, 1, 1)).TotalSeconds;
   rosTime.nanosec = (uint)(System.DateTime.UtcNow.Millisecond * 1000000);
   ```

2. **Synchronize simulation time**:
   ```csharp
   // If using Gazebo + Unity, sync the simulation time
   // Send simulation time from Gazebo to Unity
   // Use the same time base for both systems
   ```

#### Issue: Coordinate system mismatches
**Symptoms**: Robot appears in wrong position/orientation in Unity
**Solutions**:
1. **Understand coordinate system differences**:
   - ROS: X forward, Y left, Z up
   - Unity: X right, Y up, Z forward
   - Apply proper transformations

2. **Implement coordinate transformation**:
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

## Unity-Specific Troubleshooting

### 1. Unity Editor vs Build Issues

#### Issue: Bridge works in Unity Editor but not in build
**Symptoms**: Connection successful in editor, fails in standalone build
**Solutions**:
1. **Check build settings**:
   - Ensure all required DLLs are included in build
   - Verify platform-specific settings
   - Test on same platform as target deployment

2. **Address resolution in builds**:
   ```csharp
   // Use configurable IP addresses
   [SerializeField] private string rosIpAddress = "127.0.0.1";
   [SerializeField] private int rosPort = 8888;
   ```

### 2. Asset and Scene Management

#### Issue: Large Unity scenes causing performance problems
**Symptoms**: Long loading times, memory issues, crashes
**Solutions**:
1. **Use AssetBundles for large assets**:
   ```csharp
   // Load assets on demand
   using (var request = UnityWebRequestAssetBundle.GetAssetBundle(url)) {
       yield return request.SendWebRequest();
       var bundle = DownloadHandlerAssetBundle.GetContent(request);
       var prefab = bundle.LoadAsset<GameObject>("RobotPrefab");
   }
   ```

2. **Implement scene streaming**:
   - Break large scenes into smaller chunks
   - Use Addressables system for dynamic loading
   - Implement LOD for distant objects

## ROS 2 Bridge Troubleshooting

### 1. Message Type Issues

#### Issue: Message serialization/deserialization problems
**Symptoms**: Messages not received, data corruption, type mismatches
**Solutions**:
1. **Verify message definitions**:
   ```bash
   # Check if message types are properly installed
   ros2 interface show std_msgs/msg/String
   ros2 interface show geometry_msgs/msg/Twist
   ```

2. **Check message compatibility**:
   ```csharp
   // In Unity C# scripts, ensure message types match
   var publisher = rosConnection.Publish<geometry_msgs.msg.Twist>("cmd_vel");
   // Must match ROS 2 node subscriber type
   ```

### 2. Network Configuration Issues

#### Issue: Multiple ROS 2 domains interfering
**Symptoms**: Messages from wrong robots, domain conflicts
**Solutions**:
1. **Use specific domain IDs**:
   ```bash
   # Set ROS domain for specific robot/area
   export ROS_DOMAIN_ID=10  # Use different domains for different robots
   ```

2. **Network isolation**:
   ```bash
   # Use different networks for different robot systems
   # Or use ROS 2 namespaces
   ros2 run your_package your_node --ros-args --remap __ns:=/robot1
   ```

## Debugging Tools and Techniques

### 1. Unity Debugging

#### Using Unity Profiler
```csharp
// Profile network operations
using UnityEngine.Profiling;

void SendToROS() {
    Profiler.BeginSample("SendToROS");
    // Your network code here
    Profiler.EndSample();
}
```

#### Custom debugging tools
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

### 2. ROS 2 Debugging

#### Using ROS 2 tools
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

### 3. Network Debugging

#### Using network monitoring tools
```bash
# Monitor network traffic
sudo tcpdump -i any port 8888  # Default Unity ROS TCP port

# Check network connections
netstat -tuln | grep 8888

# Monitor bandwidth usage
iftop -i <interface>
```

## Common Error Messages and Solutions

### Error: "Failed to connect to ROS"
**Cause**: Network connectivity issues
**Solution**:
1. Verify IP addresses and ports
2. Check firewall settings
3. Ensure ROS 2 daemon is running: `ros2 daemon start`

### Error: "Message type not found"
**Cause**: Missing message definitions
**Solution**:
1. Source ROS 2 setup: `source /opt/ros/humble/setup.bash`
2. Ensure required packages are installed
3. Verify message type spelling and case

### Error: "DDS participant creation failed"
**Cause**: DDS configuration issues
**Solution**:
1. Check ROS_DOMAIN_ID environment variable
2. Verify FastDDS/RMW configuration
3. Restart ROS 2 daemon

## Testing and Validation

### 1. Connection Testing

Create a simple test to verify Unity-ROS 2 connection:

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

### 2. Data Validation

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

## Best Practices for Stability

### 1. Connection Management
- Implement reconnection logic
- Use connection status indicators
- Gracefully handle disconnections

### 2. Error Handling
- Implement try-catch blocks for network operations
- Provide fallback behaviors
- Log errors for debugging

### 3. Resource Management
- Properly dispose of network connections
- Monitor memory usage
- Implement garbage collection for messages

## Next Steps

Once you've resolved Unity-ROS 2 bridge issues:

1. Continue to the [Assessment Project](../assessment-project/project-overview.md) to apply your knowledge
2. Review all Module 2 content to ensure comprehensive understanding
3. Test your complete simulation pipeline from Gazebo to Unity