"""
Voice Processor ROS 2 Node

This module implements the ROS 2 node for voice processing functionality,
handling audio input, speech recognition, and voice command processing.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String, Bool
from sensor_msgs.msg import AudioData
from geometry_msgs.msg import Pose
import threading
import asyncio
import logging
from typing import Optional
import tempfile
import os

from ..services.voice_processor import VoiceProcessor
from ..services.whisper_service import WhisperService
from ..models.voice_command import VoiceCommand, VoiceCommandStatus


class VoiceProcessorNode(Node):
    """
    ROS 2 node for voice processing functionality.
    Handles audio input, speech recognition, and voice command processing.
    """

    def __init__(self):
        """Initialize the voice processor node."""
        super().__init__('voice_processor_node')

        # Setup logging
        self.logger = self.get_logger()

        # Initialize services
        self.whisper_service = WhisperService()
        self.voice_processor = VoiceProcessor(whisper_service=self.whisper_service)

        # Setup QoS profiles
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Create subscribers
        self.audio_sub = self.create_subscription(
            AudioData,
            'audio_input',
            self.audio_callback,
            qos_profile
        )

        self.command_sub = self.create_subscription(
            String,
            'voice_command',
            self.command_callback,
            qos_profile
        )

        # Create publishers
        self.transcript_pub = self.create_publisher(
            String,
            'voice_transcript',
            qos_profile
        )

        self.command_status_pub = self.create_publisher(
            String,
            'voice_command_status',
            qos_profile
        )

        self.voice_command_pub = self.create_publisher(
            String,
            'processed_voice_command',
            qos_profile
        )

        # Setup parameters
        self.declare_parameter('language', 'en')
        self.declare_parameter('model_size', 'base')
        self.declare_parameter('use_api', False)

        self.language = self.get_parameter('language').value
        self.model_size = self.get_parameter('model_size').value
        self.use_api = self.get_parameter('use_api').value

        # Setup internal state
        self.active_commands = {}
        self.command_queue = []

        # Setup processing timer
        self.processing_timer = self.create_timer(0.1, self.process_command_queue)

        self.logger.info("Voice Processor Node initialized")

    def audio_callback(self, msg: AudioData):
        """
        Callback for audio input messages.

        Args:
            msg: AudioData message containing audio data
        """
        try:
            self.logger.info("Received audio data")

            # Save audio data to temporary file
            with tempfile.NamedTemporaryFile(delete=False, suffix='.wav') as temp_file:
                temp_file.write(msg.data)
                temp_path = temp_file.name

            # Process the audio asynchronously
            threading.Thread(
                target=self._process_audio_thread,
                args=(temp_path,)
            ).start()

        except Exception as e:
            self.logger.error(f"Error in audio callback: {str(e)}")

    def command_callback(self, msg: String):
        """
        Callback for voice command messages.

        Args:
            msg: String message containing voice command
        """
        try:
            self.logger.info(f"Received voice command: {msg.data}")

            # Create a voice command from the string
            voice_cmd = VoiceCommand(
                transcript=msg.data,
                language=self.language,
                status=VoiceCommandStatus.RECEIVED
            )

            # Add to processing queue
            self.command_queue.append(voice_cmd)

        except Exception as e:
            self.logger.error(f"Error in command callback: {str(e)}")

    def _process_audio_thread(self, audio_path: str):
        """
        Process audio in a separate thread to avoid blocking ROS callbacks.

        Args:
            audio_path: Path to the audio file to process
        """
        try:
            # Process the audio using the voice processor
            voice_command = self.voice_processor.process_audio_sync(audio_path)

            # Publish the transcript
            transcript_msg = String()
            transcript_msg.data = voice_command.transcript or ""
            self.transcript_pub.publish(transcript_msg)

            # Publish command status
            status_msg = String()
            status_msg.data = f"{voice_command.id}:{voice_command.status.value}"
            self.command_status_pub.publish(status_msg)

            # Publish processed command if successful
            if voice_command.status != VoiceCommandStatus.FAILED_PROCESSING:
                command_msg = String()
                command_msg.data = f"{voice_command.transcript}:{voice_command.confidence}"
                self.voice_command_pub.publish(command_msg)

                # Store the command for potential later use
                self.active_commands[voice_command.id] = voice_command

            self.logger.info(f"Processed audio: {voice_command.transcript[:50]}...")

        except Exception as e:
            self.logger.error(f"Error processing audio {audio_path}: {str(e)}")

            # Publish error status
            status_msg = String()
            status_msg.data = f"error:{str(e)}"
            self.command_status_pub.publish(status_msg)

        finally:
            # Clean up temporary file
            try:
                os.unlink(audio_path)
            except:
                pass

    def process_command_queue(self):
        """
        Process commands in the queue periodically.
        """
        if not self.command_queue:
            return

        # Process one command at a time to avoid overwhelming the system
        voice_command = self.command_queue.pop(0)

        try:
            # If the command doesn't have audio data but has a transcript,
            # we can still process it (e.g., for simulation purposes)
            if voice_command.transcript:
                # Update status to processing
                voice_command.update_status(VoiceCommandStatus.PROCESSING)

                # Publish status update
                status_msg = String()
                status_msg.data = f"{voice_command.id}:{voice_command.status.value}"
                self.command_status_pub.publish(status_msg)

                # For text-based commands, we can skip the audio processing
                # and go directly to intent extraction (this would typically
                # be handled by the NLU service)
                voice_command.update_status(VoiceCommandStatus.PROCESSED)

                # Publish processed command
                command_msg = String()
                command_msg.data = f"{voice_command.transcript}:{voice_command.confidence}"
                self.voice_command_pub.publish(command_msg)

                # Store the processed command
                self.active_commands[voice_command.id] = voice_command

                self.logger.info(f"Processed text command: {voice_command.transcript[:50]}...")

        except Exception as e:
            self.logger.error(f"Error processing command queue: {str(e)}")

            # Update command status to failed
            voice_command.update_status(VoiceCommandStatus.FAILED_PROCESSING)

            # Publish error status
            status_msg = String()
            status_msg.data = f"{voice_command.id}:failed_processing"
            self.command_status_pub.publish(status_msg)

    def get_command_status(self, command_id: str) -> Optional[VoiceCommandStatus]:
        """
        Get the status of a specific command.

        Args:
            command_id: ID of the command to check

        Returns:
            Status of the command, or None if not found
        """
        command = self.active_commands.get(command_id)
        return command.status if command else None

    def cleanup(self):
        """
        Clean up resources before shutdown.
        """
        self.logger.info("Cleaning up voice processor node...")

        # Cancel any active commands
        for cmd_id in list(self.active_commands.keys()):
            cmd = self.active_commands[cmd_id]
            if cmd.status in [VoiceCommandStatus.RECEIVED, VoiceCommandStatus.PROCESSING]:
                cmd.update_status(VoiceCommandStatus.FAILED_PROCESSING)


def main(args=None):
    """
    Main function to run the voice processor node.
    """
    rclpy.init(args=args)

    voice_processor_node = VoiceProcessorNode()

    try:
        rclpy.spin(voice_processor_node)
    except KeyboardInterrupt:
        pass
    finally:
        voice_processor_node.cleanup()
        voice_processor_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()