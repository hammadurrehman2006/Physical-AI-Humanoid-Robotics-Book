# Data Model: Floating Chatbot UI

## Entity: FloatingChatbot

**Description**: The main UI component that appears as a fixed-position element with the logo icon, containing the chat interface when expanded.

**Attributes**:
- `isVisible` (boolean): Whether the chatbot is currently visible on the screen
- `isOpen` (boolean): Whether the chat interface is currently expanded
- `position` (string): Current position of the chatbot (e.g., "bottom-right")
- `iconSrc` (string): Path to the logo image file
- `status` (string): Current status (e.g., "online", "offline", "away")

## Entity: ChatInterface

**Description**: The expanded view that appears when the chatbot is activated, containing message history, input area, and conversation controls.

**Attributes**:
- `messages` (array): List of messages in the conversation
- `isOpen` (boolean): Whether the chat interface is currently open
- `title` (string): Title displayed at the top of the chat window
- `isMinimized` (boolean): Whether the chat window is minimized
- `unreadCount` (number): Number of unread messages

## Entity: ChatMessage

**Description**: Individual message component within the chat interface.

**Attributes**:
- `id` (string): Unique identifier for the message
- `content` (string): The message text content
- `sender` (string): Who sent the message ("user" or "assistant")
- `timestamp` (Date): When the message was sent
- `status` (string): Delivery status ("sent", "delivered", "read")

## Entity: AccessibilityConfig

**Description**: Configuration settings that ensure the chatbot meets accessibility standards for various user needs and preferences.

**Attributes**:
- `keyboardNavigationEnabled` (boolean): Whether keyboard navigation is enabled
- `screenReaderCompatibility` (boolean): Whether screen reader support is active
- `reducedMotion` (boolean): Whether to respect user's reduced motion preferences
- `highContrastMode` (boolean): Whether to use high contrast colors
- `fontSize` (string): Font size setting for accessibility

## Relationships

- `FloatingChatbot` 1 → 1 `ChatInterface`: Each floating chatbot has one associated chat interface
- `ChatInterface` 1 → * `ChatMessage`: Each chat interface contains multiple messages
- `FloatingChatbot` 1 → 1 `AccessibilityConfig`: Each chatbot has accessibility configuration settings