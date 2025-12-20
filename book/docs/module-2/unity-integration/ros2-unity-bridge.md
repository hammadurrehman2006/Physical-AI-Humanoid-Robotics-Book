---
sidebar_position: 2
---

# ROS 2 to Unity Bridge

This tutorial covers implementing the bridge between ROS 2 and Unity for real-time data synchronization. You'll learn how to connect your ROS 2 system to Unity visualization.

## Bridge Architecture

Understanding the ROS 2 to Unity bridge components:

### Communication Layers

The bridge typically consists of:
- **Network Layer**: Handles communication between ROS 2 and Unity
- **Message Translation**: Converts ROS 2 messages to Unity data structures
- **Synchronization**: Ensures Unity visualization matches ROS 2 state
- **Performance Management**: Optimizes data transfer rates

### Common Bridge Solutions

1. **Unity ROS TCP Connector**: A popular open-source solution
2. **ROS#**: C# implementation of ROS client for Unity
3. **Custom WebSocket bridges**: For web-based deployments
4. **DDS-based direct integration**: More advanced but efficient

## Implementation Steps

### Step 1: Set up the Communication Layer

For the Unity ROS TCP Connector approach:

1. **Install the Unity ROS TCP Connector package**:
   - Download from the Unity Asset Store or GitHub
   - Import into your Unity project
   - Configure network settings

2. **Configure ROS 2 side**:
   ```bash
   # Install required ROS 2 packages
   sudo apt install ros-humble-rosbridge-suite
   ```

3. **Network Configuration**:
   ```bash
   # Launch the ROS bridge server
   ros2 launch rosbridge_server rosbridge_websocket_launch.xml
   ```

### Step 2: Define Message Types

Common ROS 2 message types used in robotics:

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

### Step 3: Implement Data Publishers/Subscribers

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

### Step 4: Testing the Connection

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

## Performance Optimization

### Data Rate Management

1. **Throttle high-frequency messages**:
   - LiDAR scans: 10-30 Hz is typically sufficient
   - Camera images: 5-15 Hz depending on application
   - Joint states: 50-100 Hz for precise control

2. **Implement data compression**:
   - Compress large data like point clouds
   - Use appropriate data types (avoid unnecessary precision)
   - Consider subsampling for visualization

### Network Optimization

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

## Security Considerations

### Network Security

1. **Use secure connections**: Consider TLS/SSL for production deployments
2. **Network segmentation**: Isolate robotics networks when possible
3. **Authentication**: Implement proper authentication mechanisms
4. **Firewall rules**: Configure appropriate firewall rules for ROS 2 ports

## Troubleshooting Common Issues

### Connection Problems

**Issue**: Unity cannot connect to ROS 2
**Solutions**:
- Verify IP addresses and port numbers
- Check firewall settings
- Ensure ROS bridge server is running
- Verify ROS_DOMAIN_ID matches on both sides

**Issue**: High latency in communication
**Solutions**:
- Check network bandwidth
- Optimize message sizes
- Use local network when possible
- Reduce message frequency

### Data Synchronization

**Issue**: Unity visualization lags behind ROS 2 simulation
**Solutions**:
- Implement proper time synchronization
- Use interpolation for smooth visualization
- Consider prediction algorithms for fast-moving robots

## Integration Examples

### Basic Robot State Visualization

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

## Next Steps

After implementing the ROS 2 to Unity bridge:

1. Continue to [Visualization Techniques](./visualization-techniques.md) for advanced rendering
2. Learn about [Unity Troubleshooting](./unity-troubleshooting.md) for common bridge issues
3. Test your complete Unity-ROS 2 integration