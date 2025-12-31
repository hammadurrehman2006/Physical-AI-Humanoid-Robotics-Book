"""
Vision-Language-Action API

This module implements the API endpoints for the Vision-Language-Action system,
providing RESTful interfaces for voice processing, visual perception, action execution,
and multi-modal context management.
"""

from fastapi import FastAPI, HTTPException, UploadFile, File, BackgroundTasks
from fastapi.middleware.cors import CORSMiddleware
from typing import Dict, Any, List, Optional
import asyncio
import logging
import uuid
import tempfile
import os
from datetime import datetime

from ..models.voice_command import VoiceCommand, VoiceCommandStatus
from ..models.visual_perception import VisualPerception, DetectedObject
from ..models.action_plan import ActionPlan, ActionStatus
from ..models.multi_modal_context import MultiModalContext

from ..services.voice_processor import VoiceProcessor
from ..services.whisper_service import WhisperService
from ..services.nlu_service import NLUService
from ..services.command_parser import CommandParser
from ..services.action_executor import ActionExecutor
from ..services.multi_modal_fusion import MultiModalFusionService


class VisionLanguageActionAPI:
    """
    API class for the Vision-Language-Action system.
    Provides endpoints for voice processing, visual perception, action execution,
    and multi-modal context management.
    """

    def __init__(self, nlu_service: Optional[NLUService] = None):
        """
        Initialize the API with required services.

        Args:
            nlu_service: Optional NLU service. If not provided, one will be created.
        """
        self.app = FastAPI(
            title="Vision-Language-Action API",
            description="API for integrating vision, language, and action in robotics",
            version="1.0.0"
        )

        # Setup CORS
        self.app.add_middleware(
            CORSMiddleware,
            allow_origins=["*"],
            allow_credentials=True,
            allow_methods=["*"],
            allow_headers=["*"],
        )

        # Initialize services
        self.whisper_service = WhisperService()
        self.voice_processor = VoiceProcessor(whisper_service=self.whisper_service)
        self.nlu_service = nlu_service
        self.command_parser = CommandParser()
        self.action_executor = ActionExecutor()
        self.fusion_service = MultiModalFusionService()

        # If NLU service is not provided, try to create one (requires API key)
        if self.nlu_service is None:
            import os
            api_key = os.getenv("OPENAI_API_KEY")
            if api_key:
                self.nlu_service = NLUService(api_key=api_key)
            else:
                print("Warning: NLU service not initialized - OPENAI_API_KEY not set")
                self.nlu_service = None

        # In-memory storage for demonstration
        self.voice_commands: Dict[str, VoiceCommand] = {}
        self.visual_perceptions: Dict[str, VisualPerception] = {}
        self.action_plans: Dict[str, ActionPlan] = {}
        self.multi_modal_contexts: Dict[str, MultiModalContext] = {}

        # Setup logging
        self.logger = logging.getLogger(__name__)

        # Setup API routes
        self._setup_routes()

    def _setup_routes(self):
        """
        Setup all API routes.
        """
        # Voice processing endpoints
        @self.app.post("/api/vision-language-action/voice/commands")
        async def submit_voice_command(file: UploadFile = File(...)):
            return await self._handle_voice_command_upload(file)

        @self.app.post("/api/vision-language-action/voice/commands/text")
        async def submit_text_command(text: str):
            return await self._handle_text_command(text)

        @self.app.get("/api/vision-language-action/voice/commands/{command_id}")
        async def get_voice_command(command_id: str):
            return await self._get_voice_command(command_id)

        @self.app.get("/api/vision-language-action/voice/commands")
        async def list_voice_commands():
            return await self._list_voice_commands()

        # Visual perception endpoints
        @self.app.post("/api/vision-language-action/vision/snapshots")
        async def capture_visual_snapshot(file: UploadFile = File(...)):
            return await self._handle_visual_snapshot(file)

        @self.app.get("/api/vision-language-action/vision/snapshots/{snapshot_id}")
        async def get_visual_snapshot(snapshot_id: str):
            return await self._get_visual_snapshot(snapshot_id)

        @self.app.get("/api/vision-language-action/vision/objects")
        async def get_current_objects():
            return await self._get_current_objects()

        # Action execution endpoints
        @self.app.post("/api/vision-language-action/actions/plans")
        async def create_action_plan(plan_data: Dict[str, Any]):
            return await self._create_action_plan(plan_data)

        @self.app.post("/api/vision-language-action/actions/plans/{plan_id}/execute")
        async def execute_action_plan(plan_id: str):
            return await self._execute_action_plan(plan_id)

        @self.app.get("/api/vision-language-action/actions/plans/{plan_id}")
        async def get_action_plan(plan_id: str):
            return await self._get_action_plan(plan_id)

        @self.app.put("/api/vision-language-action/actions/plans/{plan_id}/cancel")
        async def cancel_action_plan(plan_id: str):
            return await self._cancel_action_plan(plan_id)

        # Multi-modal context endpoints
        @self.app.get("/api/vision-language-action/context/current")
        async def get_current_context():
            return await self._get_current_context()

        @self.app.get("/api/vision-language-action/context/history")
        async def get_context_history():
            return await self._get_context_history()

    async def _handle_voice_command_upload(self, file: UploadFile):
        """
        Handle voice command upload.

        Args:
            file: Audio file containing the voice command

        Returns:
            Processed voice command data
        """
        try:
            # Save uploaded file to temporary location
            with tempfile.NamedTemporaryFile(delete=False, suffix=os.path.splitext(file.filename)[1]) as temp_file:
                temp_file.write(await file.read())
                temp_path = temp_file.name

            try:
                # Process the audio using voice processor
                voice_command = self.voice_processor.process_audio_sync(temp_path)

                # Store the command
                self.voice_commands[voice_command.id] = voice_command

                # If NLU service is available, interpret the command
                if self.nlu_service:
                    intent, parameters = self.nlu_service.interpret_command_sync(voice_command)
                    voice_command.set_intent_and_parameters(intent, parameters)

                return {
                    "id": voice_command.id,
                    "transcript": voice_command.transcript,
                    "confidence": voice_command.confidence,
                    "status": voice_command.status.value,
                    "intent": voice_command.intent,
                    "parameters": voice_command.parameters,
                    "timestamp": voice_command.timestamp.isoformat()
                }

            finally:
                # Clean up temporary file
                os.unlink(temp_path)

        except Exception as e:
            self.logger.error(f"Error processing voice command: {str(e)}")
            raise HTTPException(status_code=500, detail=f"Error processing voice command: {str(e)}")

    async def _handle_text_command(self, text: str):
        """
        Handle text command submission.

        Args:
            text: Text of the voice command

        Returns:
            Processed voice command data
        """
        try:
            # Create a voice command from text
            voice_command = VoiceCommand(
                transcript=text,
                confidence=1.0,  # Perfect confidence for text input
                status=VoiceCommandStatus.PROCESSED
            )

            # If NLU service is available, interpret the command
            if self.nlu_service:
                intent, parameters = self.nlu_service.interpret_command_sync(voice_command)
                voice_command.set_intent_and_parameters(intent, parameters)

            # Store the command
            self.voice_commands[voice_command.id] = voice_command

            return {
                "id": voice_command.id,
                "transcript": voice_command.transcript,
                "confidence": voice_command.confidence,
                "status": voice_command.status.value,
                "intent": voice_command.intent,
                "parameters": voice_command.parameters,
                "timestamp": voice_command.timestamp.isoformat()
            }

        except Exception as e:
            self.logger.error(f"Error processing text command: {str(e)}")
            raise HTTPException(status_code=500, detail=f"Error processing text command: {str(e)}")

    async def _get_voice_command(self, command_id: str):
        """
        Get a specific voice command by ID.

        Args:
            command_id: ID of the voice command

        Returns:
            Voice command data
        """
        command = self.voice_commands.get(command_id)
        if not command:
            raise HTTPException(status_code=404, detail="Voice command not found")

        return {
            "id": command.id,
            "transcript": command.transcript,
            "confidence": command.confidence,
            "status": command.status.value,
            "intent": command.intent,
            "parameters": command.parameters,
            "timestamp": command.timestamp.isoformat()
        }

    async def _list_voice_commands(self):
        """
        List recent voice commands.

        Returns:
            List of recent voice commands
        """
        commands = list(self.voice_commands.values())
        # Sort by timestamp, most recent first
        commands.sort(key=lambda x: x.timestamp, reverse=True)
        # Return most recent 50 commands
        commands = commands[:50]

        return [
            {
                "id": cmd.id,
                "transcript": cmd.transcript,
                "confidence": cmd.confidence,
                "status": cmd.status.value,
                "intent": cmd.intent,
                "parameters": cmd.parameters,
                "timestamp": cmd.timestamp.isoformat()
            }
            for cmd in commands
        ]

    async def _handle_visual_snapshot(self, file: UploadFile):
        """
        Handle visual snapshot upload.

        Args:
            file: Image file containing the visual snapshot

        Returns:
            Processed visual perception data
        """
        try:
            # Save uploaded file to temporary location
            with tempfile.NamedTemporaryFile(delete=False, suffix=os.path.splitext(file.filename)[1]) as temp_file:
                temp_file.write(await file.read())
                temp_path = temp_file.name

            try:
                # For this implementation, we'll create a mock visual perception
                # In a real implementation, this would involve object detection, etc.
                visual_perception = VisualPerception(
                    camera_source="front_camera",
                    image_path=temp_path,
                    confidence_threshold=0.5,
                    detection_accuracy=0.85
                )

                # Add some mock detected objects
                red_ball = DetectedObject(
                    class_name="ball",
                    bounding_box=[100.0, 200.0, 50.0, 50.0],
                    confidence=0.95,
                    position_3d=[1.0, 2.0, 0.5],
                    size=[0.1, 0.1, 0.1],
                    color="red"
                )

                blue_cube = DetectedObject(
                    class_name="cube",
                    bounding_box=[300.0, 150.0, 60.0, 60.0],
                    confidence=0.88,
                    position_3d=[2.0, 1.0, 0.3],
                    size=[0.15, 0.15, 0.15],
                    color="blue"
                )

                visual_perception.add_object(red_ball)
                visual_perception.add_object(blue_cube)

                # Store the visual perception
                self.visual_perceptions[visual_perception.id] = visual_perception

                return {
                    "id": visual_perception.id,
                    "camera_source": visual_perception.camera_source,
                    "detection_accuracy": visual_perception.detection_accuracy,
                    "objects": [
                        {
                            "object_id": obj.object_id,
                            "class_name": obj.class_name,
                            "confidence": obj.confidence,
                            "position_3d": obj.position_3d,
                            "color": obj.color
                        }
                        for obj in visual_perception.objects
                    ],
                    "timestamp": visual_perception.timestamp.isoformat()
                }

            finally:
                # Clean up temporary file
                os.unlink(temp_path)

        except Exception as e:
            self.logger.error(f"Error processing visual snapshot: {str(e)}")
            raise HTTPException(status_code=500, detail=f"Error processing visual snapshot: {str(e)}")

    async def _get_visual_snapshot(self, snapshot_id: str):
        """
        Get a specific visual snapshot by ID.

        Args:
            snapshot_id: ID of the visual snapshot

        Returns:
            Visual perception data
        """
        snapshot = self.visual_perceptions.get(snapshot_id)
        if not snapshot:
            raise HTTPException(status_code=404, detail="Visual snapshot not found")

        return {
            "id": snapshot.id,
            "camera_source": snapshot.camera_source,
            "detection_accuracy": snapshot.detection_accuracy,
            "objects": [
                {
                    "object_id": obj.object_id,
                    "class_name": obj.class_name,
                    "confidence": obj.confidence,
                    "position_3d": obj.position_3d,
                    "color": obj.color
                }
                for obj in snapshot.objects
            ],
            "timestamp": snapshot.timestamp.isoformat()
        }

    async def _get_current_objects(self):
        """
        Get currently detected objects.

        Returns:
            List of currently detected objects
        """
        # In a real implementation, this would get the most recent visual perception
        # For now, we'll return objects from the most recent snapshot
        if not self.visual_perceptions:
            return []

        # Get the most recent visual perception
        latest_snapshot = max(
            self.visual_perceptions.values(),
            key=lambda x: x.timestamp
        )

        return [
            {
                "object_id": obj.object_id,
                "class_name": obj.class_name,
                "confidence": obj.confidence,
                "position_3d": obj.position_3d,
                "color": obj.color
            }
            for obj in latest_snapshot.objects
        ]

    async def _create_action_plan(self, plan_data: Dict[str, Any]):
        """
        Create a new action plan.

        Args:
            plan_data: Data for creating the action plan

        Returns:
            Created action plan data
        """
        try:
            # Create actions from the provided data
            actions = []
            for action_data in plan_data.get('actions', []):
                from ..models.action_plan import Action, ActionType
                action = Action(
                    type=ActionType[action_data.get('type', 'MANIPULATION')],
                    parameters=action_data.get('parameters', {}),
                    timeout=action_data.get('timeout', 30.0),
                    preconditions=action_data.get('preconditions', []),
                    postconditions=action_data.get('postconditions', []),
                    success_threshold=action_data.get('success_threshold', 0.9)
                )
                actions.append(action)

            # Create the action plan
            action_plan = ActionPlan(
                command_id=plan_data.get('command_id', ''),
                actions=actions,
                priority=plan_data.get('priority', 3),
                estimated_duration=plan_data.get('estimated_duration', 0.0),
                required_resources=plan_data.get('required_resources', []),
                success_criteria=plan_data.get('success_criteria', []),
                status=ActionStatus.PENDING
            )

            # Store the action plan
            self.action_plans[action_plan.id] = action_plan

            return {
                "id": action_plan.id,
                "command_id": action_plan.command_id,
                "status": action_plan.status.value,
                "priority": action_plan.priority,
                "estimated_duration": action_plan.estimated_duration,
                "required_resources": action_plan.required_resources,
                "success_criteria": action_plan.success_criteria,
                "actions": [
                    {
                        "action_id": action.action_id,
                        "type": action.type.value,
                        "parameters": action.parameters,
                        "timeout": action.timeout,
                        "success_threshold": action.success_threshold
                    }
                    for action in action_plan.actions
                ]
            }

        except Exception as e:
            self.logger.error(f"Error creating action plan: {str(e)}")
            raise HTTPException(status_code=500, detail=f"Error creating action plan: {str(e)}")

    async def _execute_action_plan(self, plan_id: str):
        """
        Execute an action plan.

        Args:
            plan_id: ID of the action plan to execute

        Returns:
            Execution result
        """
        action_plan = self.action_plans.get(plan_id)
        if not action_plan:
            raise HTTPException(status_code=404, detail="Action plan not found")

        try:
            # Execute the action plan
            executed_plan = self.action_executor.execute_action_plan(action_plan)

            # Update stored plan
            self.action_plans[executed_plan.id] = executed_plan

            return {
                "id": executed_plan.id,
                "status": executed_plan.status.value,
                "execution_log": executed_plan.execution_log,
                "completed_actions": len([log for log in executed_plan.execution_log if log.get('result') == 'success'])
            }

        except Exception as e:
            self.logger.error(f"Error executing action plan: {str(e)}")
            raise HTTPException(status_code=500, detail=f"Error executing action plan: {str(e)}")

    async def _get_action_plan(self, plan_id: str):
        """
        Get a specific action plan by ID.

        Args:
            plan_id: ID of the action plan

        Returns:
            Action plan data
        """
        action_plan = self.action_plans.get(plan_id)
        if not action_plan:
            raise HTTPException(status_code=404, detail="Action plan not found")

        return {
            "id": action_plan.id,
            "command_id": action_plan.command_id,
            "status": action_plan.status.value,
            "priority": action_plan.priority,
            "estimated_duration": action_plan.estimated_duration,
            "required_resources": action_plan.required_resources,
            "success_criteria": action_plan.success_criteria,
            "actions": [
                {
                    "action_id": action.action_id,
                    "type": action.type.value,
                    "parameters": action.parameters,
                    "timeout": action.timeout,
                    "success_threshold": action.success_threshold
                }
                for action in action_plan.actions
            ],
            "execution_log": action_plan.execution_log
        }

    async def _cancel_action_plan(self, plan_id: str):
        """
        Cancel an action plan.

        Args:
            plan_id: ID of the action plan to cancel

        Returns:
            Cancellation result
        """
        success = self.action_executor.cancel_action_plan(plan_id)
        if not success:
            raise HTTPException(status_code=404, detail="Action plan not found or could not be cancelled")

        return {
            "success": success,
            "plan_id": plan_id
        }

    async def _get_current_context(self):
        """
        Get the current multi-modal context.

        Returns:
            Current multi-modal context
        """
        # In a real implementation, this would combine the most recent voice, visual, and action data
        # For now, we'll create a mock context
        if not self.voice_commands and not self.visual_perceptions:
            return {"error": "No context available"}

        # Get the most recent voice command
        latest_voice = max(
            self.voice_commands.values(),
            key=lambda x: x.timestamp
        ) if self.voice_commands else None

        # Get the most recent visual perception
        latest_visual = max(
            self.visual_perceptions.values(),
            key=lambda x: x.timestamp
        ) if self.visual_perceptions else None

        # Create a multi-modal context
        context = MultiModalContext(
            voice_context={
                "transcript": latest_voice.transcript if latest_voice else "",
                "intent": latest_voice.intent if latest_voice else "",
                "confidence": latest_voice.confidence if latest_voice else 0.0
            } if latest_voice else {},
            visual_context={
                "detection_accuracy": latest_visual.detection_accuracy if latest_visual else 0.0,
                "objects_count": len(latest_visual.objects) if latest_visual else 0
            } if latest_visual else {},
            confidence_score=0.8  # Mock confidence
        )

        return {
            "id": context.id,
            "timestamp": context.timestamp.isoformat(),
            "voice_context": context.voice_context,
            "visual_context": context.visual_context,
            "confidence_score": context.confidence_score
        }

    async def _get_context_history(self):
        """
        Get the multi-modal context history.

        Returns:
            Context history
        """
        # In a real implementation, this would return historical context data
        # For now, we'll return a mock history
        return {
            "contexts": [
                {
                    "id": str(uuid.uuid4()),
                    "timestamp": (datetime.now().timestamp() - i*60).__str__(),  # Mock timestamps
                    "voice_transcript": f"Mock command {i}",
                    "confidence": 0.8 + (i * 0.01)  # Mock confidence values
                }
                for i in range(5)  # Last 5 contexts
            ]
        }

    def get_app(self):
        """
        Get the FastAPI application instance.

        Returns:
            FastAPI application
        """
        return self.app

    def run(self, host: str = "0.0.0.0", port: int = 8000):
        """
        Run the API server.

        Args:
            host: Host address to bind to
            port: Port to bind to
        """
        import uvicorn
        uvicorn.run(self.app, host=host, port=port)


# Example usage and testing
if __name__ == "__main__":
    import os
    import threading
    import time
    import requests

    # Create the API
    api = VisionLanguageActionAPI()

    # Run the API in a separate thread for testing
    def run_api():
        api.run(host="127.0.0.1", port=8000)

    api_thread = threading.Thread(target=run_api, daemon=True)
    api_thread.start()

    # Give the API time to start
    time.sleep(2)

    print("Vision-Language-Action API is running on http://127.0.0.1:8000")
    print("API endpoints available:")
    print("  POST /api/vision-language-action/voice/commands - Submit voice command")
    print("  POST /api/vision-language-action/voice/commands/text - Submit text command")
    print("  GET /api/vision-language-action/voice/commands/{id} - Get voice command")
    print("  POST /api/vision-language-action/vision/snapshots - Submit visual snapshot")
    print("  POST /api/vision-language-action/actions/plans - Create action plan")
    print("  POST /api/vision-language-action/actions/plans/{id}/execute - Execute plan")
    print("  GET /api/vision-language-action/context/current - Get current context")

    try:
        # Keep the main thread alive
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nShutting down API server...")