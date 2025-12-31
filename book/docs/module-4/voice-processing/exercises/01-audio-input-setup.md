# Exercise 1.1: Audio Input Setup and Configuration

## Objective
Learn to set up audio input hardware and configure audio parameters for optimal voice processing in the Vision-Language-Action system.

## Prerequisites
- Python 3.10+
- ROS 2 Humble Hawksbill installed
- Microphone hardware (real or simulated)
- Basic understanding of audio processing concepts

## Exercise Steps

### Step 1: Install Audio Processing Dependencies
First, install the required audio processing libraries:

```bash
# Create a virtual environment for this exercise
python3 -m venv voice_processing_env
source voice_processing_env/bin/activate
pip install pyaudio numpy scipy librosa pydub vosk rclpy

# On Ubuntu, install system dependencies
sudo apt-get update
sudo apt-get install portaudio19-dev python3-pyaudio
```

### Step 2: Create Audio Input Node
Create a new file `audio_input_node.py`:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import AudioData
from std_msgs.msg import Header
import pyaudio
import numpy as np
import threading
import time

class AudioInputNode(Node):
    def __init__(self):
        super().__init__('audio_input_node')

        # Audio configuration parameters
        self.declare_parameter('sample_rate', 44100)
        self.declare_parameter('channels', 1)
        self.declare_parameter('chunk_size', 1024)
        self.declare_parameter('format', 16)  # 16-bit

        self.sample_rate = self.get_parameter('sample_rate').value
        self.channels = self.get_parameter('channels').value
        self.chunk_size = self.get_parameter('chunk_size').value
        self.audio_format = pyaudio.paInt16

        # Publisher for audio data
        self.audio_publisher = self.create_publisher(AudioData, 'audio_stream', 10)

        # Initialize PyAudio
        self.audio = pyaudio.PyAudio()

        # Audio stream
        self.stream = None
        self.is_recording = False

        self.get_logger().info(f'Audio input node initialized with sample_rate: {self.sample_rate}, channels: {self.channels}')

    def start_audio_capture(self):
        """Start audio capture from microphone"""
        try:
            self.stream = self.audio.open(
                format=self.audio_format,
                channels=self.channels,
                rate=self.sample_rate,
                input=True,
                frames_per_buffer=self.chunk_size
            )
            self.is_recording = True

            # Start recording in a separate thread
            self.recording_thread = threading.Thread(target=self.record_audio)
            self.recording_thread.daemon = True
            self.recording_thread.start()

            self.get_logger().info('Audio capture started successfully')
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to start audio capture: {e}')
            return False

    def record_audio(self):
        """Record audio in a continuous loop"""
        while self.is_recording:
            try:
                # Read audio data from stream
                data = self.stream.read(self.chunk_size, exception_on_overflow=False)

                # Convert to AudioData message
                audio_msg = AudioData()
                audio_msg.data = data

                # Add timestamp
                audio_msg.header = Header()
                audio_msg.header.stamp = self.get_clock().now().to_msg()

                # Publish audio data
                self.audio_publisher.publish(audio_msg)

            except Exception as e:
                self.get_logger().error(f'Error recording audio: {e}')
                break

    def stop_audio_capture(self):
        """Stop audio capture"""
        self.is_recording = False
        if self.stream:
            self.stream.stop_stream()
            self.stream.close()
        self.audio.terminate()

def main(args=None):
    rclpy.init(args=args)

    node = AudioInputNode()

    # Start audio capture
    if node.start_audio_capture():
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            node.get_logger().info('Interrupted, shutting down...')
        finally:
            node.stop_audio_capture()
            node.destroy_node()
            rclpy.shutdown()
    else:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 3: Test Audio Input
Create a test script `test_audio_input.py` to verify your audio setup:

```python
#!/usr/bin/env python3
import pyaudio
import numpy as np
import wave
import time

def list_audio_devices():
    """List all available audio input devices"""
    audio = pyaudio.PyAudio()

    print("Available audio input devices:")
    for i in range(audio.get_device_count()):
        device_info = audio.get_device_info_by_index(i)
        if device_info['maxInputChannels'] > 0:
            print(f"Device {i}: {device_info['name']}")
            print(f"  Sample rate: {device_info['defaultSampleRate']}")
            print(f"  Max input channels: {device_info['maxInputChannels']}")
    audio.terminate()

def test_microphone_input():
    """Test microphone input and save to file"""
    # Audio configuration
    chunk = 1024
    format = pyaudio.paInt16
    channels = 1
    sample_rate = 44100
    record_seconds = 5

    audio = pyaudio.PyAudio()

    try:
        # Open stream
        stream = audio.open(
            format=format,
            channels=channels,
            rate=sample_rate,
            input=True,
            frames_per_buffer=chunk
        )

        print(f"Recording for {record_seconds} seconds...")
        frames = []

        for i in range(0, int(sample_rate / chunk * record_seconds)):
            data = stream.read(chunk)
            frames.append(data)

        print("Recording finished!")

        # Save to WAV file
        filename = f"test_audio_{int(time.time())}.wav"
        with wave.open(filename, 'wb') as wf:
            wf.setnchannels(channels)
            wf.setsampwidth(audio.get_sample_size(format))
            wf.setframerate(sample_rate)
            wf.writeframes(b''.join(frames))

        print(f"Audio saved to {filename}")

        # Close stream
        stream.stop_stream()
        stream.close()

    except Exception as e:
        print(f"Error during recording: {e}")
    finally:
        audio.terminate()

if __name__ == "__main__":
    print("Testing audio input setup...")
    list_audio_devices()
    test_microphone_input()
```

### Step 4: Create Launch File
Create a launch file `audio_input.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'sample_rate',
            default_value='44100',
            description='Audio sample rate'
        ),
        DeclareLaunchArgument(
            'channels',
            default_value='1',
            description='Number of audio channels'
        ),
        DeclareLaunchArgument(
            'chunk_size',
            default_value='1024',
            description='Audio chunk size'
        ),

        # Audio input node
        Node(
            package='your_robot_package',  # Replace with your package name
            executable='audio_input_node',
            name='audio_input_node',
            parameters=[
                {
                    'sample_rate': LaunchConfiguration('sample_rate'),
                    'channels': LaunchConfiguration('channels'),
                    'chunk_size': LaunchConfiguration('chunk_size')
                }
            ],
            output='screen'
        )
    ])
```

### Step 5: Run the Exercise
1. Make sure your virtual environment is activated:
```bash
source voice_processing_env/bin/activate
```

2. Run the audio test script:
```bash
python3 test_audio_input.py
```

3. Check that your microphone is listed and test recording works.

4. Run the ROS 2 audio input node:
```bash
python3 audio_input_node.py
```

5. In another terminal, listen to the audio stream:
```bash
# First install ros2 topic tools if needed
ros2 topic echo /audio_stream
```

## Expected Outcomes
- Successfully list available audio input devices
- Record and save a 5-second audio sample
- Run the ROS 2 audio input node
- Verify audio data is being published to the `/audio_stream` topic

## Verification Steps
1. Check that audio devices are properly detected
2. Verify the test recording file was created and contains audio
3. Confirm the ROS 2 node is publishing audio messages
4. Validate audio parameters (sample rate, channels, etc.) match configuration

## Troubleshooting
- If no audio devices are found, check microphone connections and permissions
- If recording fails, verify PyAudio is properly installed
- If ROS 2 node fails to start, ensure rclpy is installed and ROS 2 environment is sourced

## Next Exercise
Continue to Exercise 1.2: Audio Preprocessing and Noise Reduction