# Floating Chatbot UI Interface Contracts

## Component: FloatingChatbot

### Props Interface
```typescript
interface FloatingChatbotProps {
  iconSrc?: string;           // Path to the chatbot icon, defaults to logo.png
  initialPosition?: string;   // Initial position of the chatbot (default: "bottom-right")
  accessibilityConfig?: AccessibilityConfig; // Accessibility settings
  onOpen?: () => void;       // Callback when chat interface opens
  onClose?: () => void;      // Callback when chat interface closes
  title?: string;            // Title to display in the chat interface
}
```

### State Interface
```typescript
interface FloatingChatbotState {
  isVisible: boolean;        // Whether the chatbot is visible
  isOpen: boolean;           // Whether the chat interface is open
  position: string;          // Current position (e.g., "bottom-right")
  status: 'online' | 'offline' | 'away'; // Chatbot status
}
```

## Component: ChatWindow

### Props Interface
```typescript
interface ChatWindowProps {
  isOpen: boolean;           // Whether the window is open
  title: string;             // Title of the chat window
  messages: ChatMessage[];   // Array of messages to display
  onSendMessage: (message: string) => void; // Callback for sending messages
  onClose: () => void;       // Callback for closing the window
  onMinimize?: () => void;   // Optional callback for minimizing the window
  isMinimized?: boolean;     // Whether the window is minimized
}
```

### State Interface
```typescript
interface ChatWindowState {
  messages: ChatMessage[];   // Current messages in the chat
  inputText: string;         // Current text in the input field
  isMinimized: boolean;      // Whether the window is minimized
}
```

## Component: ChatMessage

### Props Interface
```typescript
interface ChatMessageProps {
  message: {
    id: string;
    content: string;
    sender: 'user' | 'assistant';
    timestamp: Date;
    status?: 'sent' | 'delivered' | 'read';
  };
}
```

## Component: InputArea

### Props Interface
```typescript
interface InputAreaProps {
  value: string;             // Current input value
  onChange: (value: string) => void; // Callback for input changes
  onSend: (value: string) => void;   // Callback for sending the message
  placeholder?: string;      // Placeholder text for the input
  disabled?: boolean;        // Whether the input is disabled
}
```

### State Interface
```typescript
interface InputAreaState {
  inputValue: string;        // Current input value
  isFocused: boolean;        // Whether the input is focused
}
```

## Accessibility Contract

### Keyboard Navigation
- `Tab`: Navigate between focusable elements in the chat interface
- `Enter`: Send message when input area is focused
- `Escape`: Close the chat interface
- `Alt+M`: Minimize/maximize the chat window

### Screen Reader Support
- Proper ARIA labels for all interactive elements
- Live regions for new messages
- Semantic HTML structure
- Focus management when opening/closing

### Reduced Motion Support
- Animation disabling when user prefers reduced motion
- CSS media query: `@media (prefers-reduced-motion: reduce)`

## Responsive Design Contract

### Breakpoints
- Mobile: 320px - 768px
- Tablet: 769px - 1024px
- Desktop: 1025px - 1440px
- Large Desktop: 1441px+

### Behavior at Different Sizes
- Mobile: Chat window takes full width with bottom positioning
- Tablet: Chat window with max-width of 400px
- Desktop: Chat window with fixed width of 380px