// API service for chatbot interactions
import { ChatRequest, SelectedTextChatRequest, ChatResponse, Message, Conversation } from '../types/chatbot';

const API_BASE_URL = process.env.REACT_APP_API_BASE_URL || 'http://localhost:8000/api/v1';

class ApiService {
  private async request<T>(endpoint: string, options: RequestInit = {}): Promise<T> {
    const response = await fetch(`${API_BASE_URL}${endpoint}`, {
      headers: {
        'Content-Type': 'application/json',
        ...options.headers,
      },
      ...options,
    });

    if (!response.ok) {
      throw new Error(`API request failed: ${response.status} ${response.statusText}`);
    }

    return response.json();
  }

  async healthCheck(): Promise<{ status: string; version: string }> {
    return this.request('/health');
  }

  async chat(request: ChatRequest): Promise<ChatResponse> {
    return this.request('/chat', {
      method: 'POST',
      body: JSON.stringify(request),
    });
  }

  async chatSelectedText(request: SelectedTextChatRequest): Promise<ChatResponse> {
    return this.request('/chat/selected-text', {
      method: 'POST',
      body: JSON.stringify(request),
    });
  }

  async getConversation(conversationId: string): Promise<{ conversation: Conversation }> {
    return this.request(`/conversations/${conversationId}`);
  }

  async getConversations(userId: string): Promise<{ conversations: Conversation[]; total_count: number }> {
    return this.request(`/conversations?user_id=${userId}`);
  }
}

export default new ApiService();