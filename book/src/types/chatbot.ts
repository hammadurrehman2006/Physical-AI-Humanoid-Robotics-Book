// Type definitions for the chatbot component

export interface Message {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  timestamp: Date;
  sources?: Source[];
}

export interface Source {
  title: string;
  chapter: string;
  lesson: string;
  url: string;
  relevance_score: number;
}

export interface Conversation {
  id: string;
  title: string;
  createdAt: Date;
  updatedAt: Date;
  messages: Message[];
}

export interface ChatRequest {
  query: string;
  session_id: string;
  conversation_id?: string;
  include_sources?: boolean;
}

export interface SelectedTextChatRequest extends ChatRequest {
  selected_text: string;
}

export interface ChatResponse {
  response: string;
  sources: Source[];
  conversation_id: string;
  message_id: string;
}

export interface ApiResponse<T> {
  data?: T;
  error?: string;
  message?: string;
}