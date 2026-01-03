// Type definitions for the floating chatbot components

export interface Message {
  id: string;
  content: string;
  sender: 'user' | 'bot';
  timestamp: Date;
}

export interface ChatWindowProps {
  onClose: () => void;
}

export interface ChatMessageProps {
  message: Message;
}

export interface InputAreaProps {
  value: string;
  onChange: (value: string) => void;
  onSend: (message: string) => void;
}

export interface FloatingChatbotState {
  isOpen: boolean;
  isVisible: boolean;
}