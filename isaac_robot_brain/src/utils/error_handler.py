"""
Error Handler Utility

This module provides comprehensive error handling and exception management
for the Vision-Language-Action system.
"""

import logging
from typing import Dict, Any, Optional, Callable, Union
from enum import Enum
import traceback
import functools
from datetime import datetime


class ErrorCategory(Enum):
    """Enumeration of error categories"""
    VOICE_PROCESSING_ERROR = "voice_processing_error"
    LANGUAGE_UNDERSTANDING_ERROR = "language_understanding_error"
    ACTION_EXECUTION_ERROR = "action_execution_error"
    VISUAL_PERCEPTION_ERROR = "visual_perception_error"
    FUSION_ERROR = "fusion_error"
    API_ERROR = "api_error"
    SYSTEM_ERROR = "system_error"
    VALIDATION_ERROR = "validation_error"


class VLAError(Exception):
    """Base exception for Vision-Language-Action system"""

    def __init__(self, message: str, category: ErrorCategory, details: Optional[Dict[str, Any]] = None):
        super().__init__(message)
        self.message = message
        self.category = category
        self.details = details or {}
        self.timestamp = datetime.now()

    def to_dict(self) -> Dict[str, Any]:
        """Convert error to dictionary representation"""
        return {
            "message": self.message,
            "category": self.category.value,
            "details": self.details,
            "timestamp": self.timestamp.isoformat()
        }


class VoiceProcessingError(VLAError):
    """Exception for voice processing errors"""
    def __init__(self, message: str, details: Optional[Dict[str, Any]] = None):
        super().__init__(message, ErrorCategory.VOICE_PROCESSING_ERROR, details)


class LanguageUnderstandingError(VLAError):
    """Exception for language understanding errors"""
    def __init__(self, message: str, details: Optional[Dict[str, Any]] = None):
        super().__init__(message, ErrorCategory.LANGUAGE_UNDERSTANDING_ERROR, details)


class ActionExecutionError(VLAError):
    """Exception for action execution errors"""
    def __init__(self, message: str, details: Optional[Dict[str, Any]] = None):
        super().__init__(message, ErrorCategory.ACTION_EXECUTION_ERROR, details)


class VisualPerceptionError(VLAError):
    """Exception for visual perception errors"""
    def __init__(self, message: str, details: Optional[Dict[str, Any]] = None):
        super().__init__(message, ErrorCategory.VISUAL_PERCEPTION_ERROR, details)


class FusionError(VLAError):
    """Exception for fusion errors"""
    def __init__(self, message: str, details: Optional[Dict[str, Any]] = None):
        super().__init__(message, ErrorCategory.FUSION_ERROR, details)


class APIError(VLAError):
    """Exception for API errors"""
    def __init__(self, message: str, details: Optional[Dict[str, Any]] = None):
        super().__init__(message, ErrorCategory.API_ERROR, details)


class ValidationError(VLAError):
    """Exception for validation errors"""
    def __init__(self, message: str, details: Optional[Dict[str, Any]] = None):
        super().__init__(message, ErrorCategory.VALIDATION_ERROR, details)


class ErrorHandler:
    """Centralized error handling for the Vision-Language-Action system"""

    def __init__(self):
        self.logger = logging.getLogger(__name__)
        self.error_history = []

    def handle_error(
        self,
        error: Exception,
        context: Optional[Dict[str, Any]] = None,
        reraise: bool = True
    ) -> Optional[VLAError]:
        """
        Handle an error with proper logging and context.

        Args:
            error: The error to handle
            context: Additional context information
            reraise: Whether to reraise the error after handling

        Returns:
            VLAError if converted, None if not reraised
        """
        # Create a VLAError from the original error if needed
        if not isinstance(error, VLAError):
            vla_error = self._convert_to_vla_error(error, context)
        else:
            vla_error = error

        # Add context to error details
        if context:
            vla_error.details.update(context)

        # Log the error
        self._log_error(vla_error)

        # Store in history
        self.error_history.append(vla_error)

        # Clean up old errors (keep last 1000)
        if len(self.error_history) > 1000:
            self.error_history = self.error_history[-1000:]

        if reraise:
            raise vla_error
        else:
            return vla_error

    def _convert_to_vla_error(self, error: Exception, context: Optional[Dict[str, Any]] = None) -> VLAError:
        """
        Convert a standard exception to a VLAError.

        Args:
            error: The original error
            context: Context information

        Returns:
            Converted VLAError
        """
        error_type = type(error).__name__
        error_msg = str(error)

        # Determine category based on error type
        if "voice" in error_type.lower() or "audio" in error_type.lower():
            category = ErrorCategory.VOICE_PROCESSING_ERROR
        elif "nlu" in error_type.lower() or "language" in error_type.lower() or "gpt" in error_type.lower():
            category = ErrorCategory.LANGUAGE_UNDERSTANDING_ERROR
        elif "action" in error_type.lower() or "execution" in error_type.lower():
            category = ErrorCategory.ACTION_EXECUTION_ERROR
        elif "vision" in error_type.lower() or "visual" in error_type.lower() or "detection" in error_type.lower():
            category = ErrorCategory.VISUAL_PERCEPTION_ERROR
        elif "fusion" in error_type.lower():
            category = ErrorCategory.FUSION_ERROR
        elif "api" in error_type.lower() or "http" in error_type.lower():
            category = ErrorCategory.API_ERROR
        else:
            category = ErrorCategory.SYSTEM_ERROR

        details = {
            "original_error_type": error_type,
            "traceback": traceback.format_exc(),
            "context": context or {}
        }

        return VLAError(error_msg, category, details)

    def _log_error(self, vla_error: VLAError):
        """Log the error with appropriate level based on category."""
        log_msg = f"[{vla_error.category.value}] {vla_error.message}"

        if vla_error.category in [
            ErrorCategory.SYSTEM_ERROR,
            ErrorCategory.API_ERROR
        ]:
            self.logger.error(log_msg, extra=vla_error.to_dict())
        else:
            self.logger.warning(log_msg, extra=vla_error.to_dict())

    def get_error_summary(self) -> Dict[str, Any]:
        """Get a summary of recent errors."""
        if not self.error_history:
            return {"total_errors": 0, "categories": {}}

        categories = {}
        for error in self.error_history:
            cat = error.category.value
            categories[cat] = categories.get(cat, 0) + 1

        return {
            "total_errors": len(self.error_history),
            "categories": categories,
            "recent_errors": [
                {
                    "category": error.category.value,
                    "message": error.message,
                    "timestamp": error.timestamp.isoformat()
                }
                for error in self.error_history[-10:]  # Last 10 errors
            ]
        }

    def clear_error_history(self):
        """Clear the error history."""
        self.error_history.clear()

    def retry_on_error(
        self,
        func: Callable,
        max_retries: int = 3,
        delay: float = 1.0,
        backoff: float = 2.0,
        exceptions: tuple = (Exception,),
        context: Optional[Dict[str, Any]] = None
    ) -> Any:
        """
        Execute a function with retry logic on specific exceptions.

        Args:
            func: Function to execute
            max_retries: Maximum number of retries
            delay: Initial delay between retries
            backoff: Multiplier for delay after each retry
            exceptions: Tuple of exceptions to catch
            context: Context information for error handling

        Returns:
            Function result
        """
        import time

        last_exception = None

        for attempt in range(max_retries + 1):
            try:
                return func()
            except exceptions as e:
                last_exception = e

                if attempt == max_retries:
                    # Final attempt, handle the error
                    break

                # Wait before retry
                time.sleep(delay * (backoff ** attempt))

        # If we get here, all retries failed
        self.handle_error(last_exception, context, reraise=True)

    def safe_execute(
        self,
        func: Callable,
        default_return: Any = None,
        handle_exception: bool = True,
        context: Optional[Dict[str, Any]] = None
    ) -> Any:
        """
        Safely execute a function, catching exceptions and returning a default value.

        Args:
            func: Function to execute
            default_return: Default value to return if function fails
            handle_exception: Whether to handle the exception with error handler
            context: Context information for error handling

        Returns:
            Function result or default value
        """
        try:
            return func()
        except Exception as e:
            if handle_exception:
                self.handle_error(e, context, reraise=False)
            else:
                # Just log the error without reraising
                error_msg = f"Error in safe execution: {str(e)}"
                self.logger.warning(error_msg, extra={"context": context})
            return default_return


def error_handler_decorator(
    error_handler: ErrorHandler,
    default_return: Any = None,
    handle_exception: bool = True,
    context: Optional[Dict[str, Any]] = None
):
    """
    Decorator for adding error handling to functions.

    Args:
        error_handler: ErrorHandler instance
        default_return: Default return value on error
        handle_exception: Whether to handle exceptions
        context: Context information
    """
    def decorator(func: Callable) -> Callable:
        @functools.wraps(func)
        def wrapper(*args, **kwargs):
            return error_handler.safe_execute(
                lambda: func(*args, **kwargs),
                default_return=default_return,
                handle_exception=handle_exception,
                context=context
            )
        return wrapper
    return decorator


# Example usage and testing
if __name__ == "__main__":
    import logging

    # Setup logging
    logging.basicConfig(level=logging.INFO)

    # Create error handler
    handler = ErrorHandler()

    # Test different error types
    try:
        raise VoiceProcessingError("Audio file not found", {"file_path": "/path/to/audio.wav"})
    except VoiceProcessingError as e:
        print(f"Caught VoiceProcessingError: {e.message}")
        print(f"Category: {e.category.value}")

    try:
        raise LanguageUnderstandingError("GPT API call failed", {"api_endpoint": "https://api.openai.com"})
    except LanguageUnderstandingError as e:
        print(f"Caught LanguageUnderstandingError: {e.message}")

    # Test safe execution
    def failing_function():
        raise ValueError("This function always fails")

    result = handler.safe_execute(failing_function, default_return="fallback")
    print(f"Safe execution result: {result}")

    # Test error summary
    summary = handler.get_error_summary()
    print(f"Error summary: {summary}")