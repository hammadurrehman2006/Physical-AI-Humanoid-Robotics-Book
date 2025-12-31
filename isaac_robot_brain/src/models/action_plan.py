"""
Action Plan Data Model

This module defines the ActionPlan data model representing a sequence of
physical movements and behaviors generated to fulfill a user command.
"""
from dataclasses import dataclass, field
from datetime import datetime
from typing import Dict, Any, List, Optional
from enum import Enum
import uuid


class ActionStatus(Enum):
    """Enumeration of possible action statuses"""
    PENDING = "pending"
    VALIDATING = "validating"
    EXECUTING = "executing"
    COMPLETED = "completed"
    PARTIALLY_FAILED = "partially_failed"
    FAILED = "failed"
    RETRYING = "retrying"
    FALLBACK_EXECUTING = "fallback_executing"
    FALLBACK_COMPLETED = "fallback_completed"


class ActionType(Enum):
    """Enumeration of possible action types"""
    NAVIGATION = "navigation"
    MANIPULATION = "manipulation"
    INTERACTION = "interaction"
    PERCEPTION = "perception"
    COMMUNICATION = "communication"


@dataclass
class Action:
    """
    Represents a single action within an action plan.

    Attributes:
        action_id: Unique identifier for the action
        type: Type of action (navigation, manipulation, interaction, etc.)
        parameters: Specific parameters for the action
        timeout: Maximum time allowed for this action (in seconds)
        preconditions: Conditions that must be met before execution
        postconditions: Expected state after execution
        success_threshold: Success threshold for the action (0.0-1.0)
    """

    action_id: str = field(default_factory=lambda: str(uuid.uuid4()))
    type: ActionType = ActionType.MANIPULATION
    parameters: Dict[str, Any] = field(default_factory=dict)
    timeout: float = 10.0  # seconds
    preconditions: Dict[str, Any] = field(default_factory=dict)
    postconditions: Dict[str, Any] = field(default_factory=dict)
    success_threshold: float = 0.8

    def __post_init__(self):
        """Validate the action after initialization"""
        self.validate()

    def validate(self) -> bool:
        """
        Validate the action according to the specified validation rules.

        Returns:
            bool: True if validation passes, False otherwise

        Raises:
            ValueError: If validation fails
        """
        if not 0.0 <= self.success_threshold <= 1.0:
            raise ValueError(f"Success threshold must be between 0.0 and 1.0, got {self.success_threshold}")

        if self.timeout <= 0:
            raise ValueError(f"Timeout must be positive, got {self.timeout}")

        if self.type not in ActionType:
            raise ValueError(f"Invalid action type: {self.type}")

        return True


@dataclass
class ActionPlan:
    """
    Represents a sequence of physical movements and behaviors generated to fulfill a user command.

    Attributes:
        id: Unique identifier for the action plan
        command_id: Reference to the originating voice command
        actions: Ordered list of actions to execute
        priority: Priority level for execution (1-5, 5 being highest)
        estimated_duration: Estimated time to complete the plan (in seconds)
        required_resources: List of resources needed for execution
        success_criteria: Conditions that define successful completion
        fallback_plan: Alternative plan if primary plan fails
        status: Current execution status
        execution_log: Log of executed actions and their outcomes
    """

    id: str = field(default_factory=lambda: str(uuid.uuid4()))
    command_id: str = ""
    actions: List[Action] = field(default_factory=list)
    priority: int = 3  # 1-5, 5 being highest
    estimated_duration: float = 0.0  # seconds
    required_resources: List[str] = field(default_factory=list)
    success_criteria: Dict[str, Any] = field(default_factory=dict)
    fallback_plan: Optional['ActionPlan'] = None
    status: ActionStatus = ActionStatus.PENDING
    execution_log: List[Dict[str, Any]] = field(default_factory=list)

    def __post_init__(self):
        """Validate the action plan after initialization"""
        self.validate()

    def validate(self) -> bool:
        """
        Validate the action plan according to the specified validation rules.

        Returns:
            bool: True if validation passes, False otherwise

        Raises:
            ValueError: If validation fails
        """
        if not 1 <= self.priority <= 5:
            raise ValueError(f"Priority must be between 1 and 5, got {self.priority}")

        if self.estimated_duration < 0:
            raise ValueError(f"Estimated duration must be non-negative, got {self.estimated_duration}")

        if not self.actions:
            raise ValueError("Actions list must not be empty")

        if self.status not in ActionStatus:
            raise ValueError(f"Invalid status: {self.status}")

        # Validate each action
        for action in self.actions:
            if not isinstance(action, Action):
                raise ValueError(f"All actions must be of type Action, got {type(action)}")
            action.validate()

        # Validate fallback plan if exists
        if self.fallback_plan is not None:
            if not isinstance(self.fallback_plan, ActionPlan):
                raise ValueError(f"Fallback plan must be of type ActionPlan, got {type(self.fallback_plan)}")
            self.fallback_plan.validate()

        return True

    def add_action(self, action: Action):
        """Add an action to the plan"""
        action.validate()
        self.actions.append(action)
        return self

    def update_status(self, new_status: ActionStatus):
        """Update the action plan status"""
        self.status = new_status
        return self

    def log_execution(self, action_id: str, result: str, details: Dict[str, Any] = None):
        """Log the execution of an action"""
        log_entry = {
            "action_id": action_id,
            "timestamp": datetime.now().isoformat(),
            "result": result,
            "details": details or {}
        }
        self.execution_log.append(log_entry)
        return self

    def get_action_by_id(self, action_id: str) -> Optional[Action]:
        """Get an action by its ID"""
        for action in self.actions:
            if action.action_id == action_id:
                return action
        return None

    def to_dict(self) -> Dict[str, Any]:
        """Convert the action plan to a dictionary representation"""
        return {
            "id": self.id,
            "command_id": self.command_id,
            "actions": [action.to_dict() for action in self.actions],
            "priority": self.priority,
            "estimated_duration": self.estimated_duration,
            "required_resources": self.required_resources,
            "success_criteria": self.success_criteria,
            "fallback_plan": self.fallback_plan.to_dict() if self.fallback_plan else None,
            "status": self.status.value,
            "execution_log": self.execution_log
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'ActionPlan':
        """Create an ActionPlan instance from a dictionary"""
        actions = []
        for action_data in data.get("actions", []):
            action = Action(
                action_id=action_data.get("action_id", str(uuid.uuid4())),
                type=ActionType(action_data.get("type", "manipulation")),
                parameters=action_data.get("parameters", {}),
                timeout=action_data.get("timeout", 10.0),
                preconditions=action_data.get("preconditions", {}),
                postconditions=action_data.get("postconditions", {}),
                success_threshold=action_data.get("success_threshold", 0.8)
            )
            actions.append(action)

        fallback_plan = None
        if data.get("fallback_plan"):
            fallback_plan = cls.from_dict(data["fallback_plan"])

        return cls(
            id=data.get("id", str(uuid.uuid4())),
            command_id=data.get("command_id", ""),
            actions=actions,
            priority=data.get("priority", 3),
            estimated_duration=data.get("estimated_duration", 0.0),
            required_resources=data.get("required_resources", []),
            success_criteria=data.get("success_criteria", {}),
            fallback_plan=fallback_plan,
            status=ActionStatus(data.get("status", "pending")),
            execution_log=data.get("execution_log", [])
        )


# Add the to_dict method to Action after the class is fully defined
def action_to_dict(self) -> Dict[str, Any]:
    """Convert the action to a dictionary representation"""
    return {
        "action_id": self.action_id,
        "type": self.type.value,
        "parameters": self.parameters,
        "timeout": self.timeout,
        "preconditions": self.preconditions,
        "postconditions": self.postconditions,
        "success_threshold": self.success_threshold
    }


def action_from_dict(data: Dict[str, Any]) -> Action:
    """Create an Action instance from a dictionary"""
    return Action(
        action_id=data.get("action_id", str(uuid.uuid4())),
        type=ActionType(data.get("type", "manipulation")),
        parameters=data.get("parameters", {}),
        timeout=data.get("timeout", 10.0),
        preconditions=data.get("preconditions", {}),
        postconditions=data.get("postconditions", {}),
        success_threshold=data.get("success_threshold", 0.8)
    )


# Attach the methods to the Action class
Action.to_dict = action_to_dict
Action.from_dict = action_from_dict