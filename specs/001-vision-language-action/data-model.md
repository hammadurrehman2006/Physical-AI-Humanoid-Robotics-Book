# Data Model: Vision-Language-Action Module

## Overview
This document defines the key data structures and entities for the Vision-Language-Action module, which integrates voice processing, language understanding, computer vision, and action execution for humanoid robotics applications.

## Core Entities

### Voice Command
Represents a spoken instruction that undergoes speech processing, language understanding, and intent extraction.

**Fields:**
- `id`: Unique identifier for the command
- `audio_data`: Raw audio input or path to audio file
- `transcript`: Text transcription of the spoken command
- `confidence`: Confidence score for speech recognition (0.0-1.0)
- `timestamp`: Time when command was received
- `language`: Detected language of the command
- `intent`: Parsed intent from natural language processing
- `parameters`: Extracted parameters from the command (objects, locations, etc.)
- `status`: Current processing status (received, processing, executed, failed)

**Validation Rules:**
- `confidence` must be between 0.0 and 1.0
- `transcript` must not be empty
- `timestamp` must be in ISO 8601 format

### Visual Perception
Represents the robot's understanding of its environment through visual processing, including object detection, recognition, and spatial relationships.

**Fields:**
- `id`: Unique identifier for the perception snapshot
- `timestamp`: Time when perception was captured
- `objects`: List of detected objects with their properties
- `environment_map`: Spatial representation of the environment
- `camera_source`: Identifier of the camera that captured the data
- `image_path`: Path to the source image (if applicable)
- `confidence_threshold`: Minimum confidence for object detection
- `detection_accuracy`: Overall accuracy score for the detection

**Object Properties:**
- `object_id`: Unique identifier for the detected object
- `class`: Object category (e.g., "ball", "cup", "table")
- `bounding_box`: Coordinates of the object's bounding box [x, y, width, height]
- `confidence`: Confidence score for object detection (0.0-1.0)
- `position_3d`: 3D coordinates in robot's coordinate system [x, y, z]
- `size`: Estimated size of the object [width, height, depth]
- `color`: Dominant color of the object

**Validation Rules:**
- `confidence_threshold` must be between 0.0 and 1.0
- `objects` list must contain valid object structures
- `position_3d` coordinates must be within robot's operational space

### Action Plan
Represents a sequence of physical movements and behaviors generated to fulfill a user command.

**Fields:**
- `id`: Unique identifier for the action plan
- `command_id`: Reference to the originating voice command
- `actions`: Ordered list of actions to execute
- `priority`: Priority level for execution (1-5, 5 being highest)
- `estimated_duration`: Estimated time to complete the plan (in seconds)
- `required_resources`: List of resources needed for execution
- `success_criteria`: Conditions that define successful completion
- `fallback_plan`: Alternative plan if primary plan fails
- `status`: Current execution status (pending, executing, completed, failed)
- `execution_log`: Log of executed actions and their outcomes

**Action Properties:**
- `action_id`: Unique identifier for the action
- `type`: Type of action (navigation, manipulation, interaction, etc.)
- `parameters`: Specific parameters for the action
- `timeout`: Maximum time allowed for this action (in seconds)
- `preconditions`: Conditions that must be met before execution
- `postconditions`: Expected state after execution
- `success_threshold`: Success threshold for the action (0.0-1.0)

**Validation Rules:**
- `priority` must be between 1 and 5
- `estimated_duration` must be positive
- `actions` list must not be empty
- `status` must be one of the defined status values

### Multi-Modal Context
Represents the combined understanding of visual, linguistic, and situational information used for decision making.

**Fields:**
- `id`: Unique identifier for the context
- `timestamp`: Time when context was created
- `voice_context`: Information from voice command processing
- `visual_context`: Information from visual perception
- `action_context`: Information from action planning
- `environment_context`: Information about the environment
- `user_context`: Information about the user (if available)
- `temporal_context`: Information about time and sequence
- `confidence_score`: Overall confidence in the context (0.0-1.0)
- `fusion_strategy`: Strategy used to combine modalities

**Validation Rules:**
- `confidence_score` must be between 0.0 and 1.0
- All context fields must be properly structured
- `timestamp` must be in ISO 8601 format

## Relationships

### Voice Command → Action Plan
- One-to-Many: A single voice command can generate multiple action plans in complex scenarios
- The `command_id` in Action Plan references the `id` in Voice Command

### Visual Perception → Action Plan
- Many-to-Many: Multiple perception snapshots may inform a single action plan
- Action plans may trigger new visual perception requests
- Connected through execution context and object references

### Action Plan → Multi-Modal Context
- Many-to-One: Multiple action plans may contribute to a single multi-modal context
- Context provides the foundation for action plan generation and execution

## State Transitions

### Voice Command States
```
received → processing → processed → action_planned → executed → completed
                                    ↓
                                failed_processing
```

### Action Plan States
```
pending → validating → executing → completed
                             ↓
                         partially_failed → retrying → completed
                             ↓              ↓
                         failed    → fallback_executing → fallback_completed
```

## API Endpoints

### Voice Processing
- `POST /api/vision-language-action/voice/commands` - Submit a new voice command
- `GET /api/vision-language-action/voice/commands/{id}` - Get command status
- `GET /api/vision-language-action/voice/commands` - List recent commands

### Visual Perception
- `POST /api/vision-language-action/vision/snapshots` - Capture new perception data
- `GET /api/vision-language-action/vision/snapshots/{id}` - Get specific snapshot
- `GET /api/vision-language-action/vision/objects` - Get current object detections

### Action Execution
- `POST /api/vision-language-action/actions/plans` - Create new action plan
- `POST /api/vision-language-action/actions/plans/{id}/execute` - Execute plan
- `GET /api/vision-language-action/actions/plans/{id}` - Get plan status
- `PUT /api/vision-language-action/actions/plans/{id}/cancel` - Cancel execution

### Multi-Modal Context
- `GET /api/vision-language-action/context/current` - Get current context
- `GET /api/vision-language-action/context/history` - Get context history