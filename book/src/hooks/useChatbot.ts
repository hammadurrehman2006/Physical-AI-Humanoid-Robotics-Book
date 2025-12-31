// Custom hook for chatbot functionality
import { useState, useCallback } from 'react';
import { Message } from '../types/chatbot';
import chatbotService from '../services/chatbot';

const useChatbot = () => {
  const [messages, setMessages] = useState<Message[]>([]);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const sendMessage = useCallback(async (query: string, selectedText?: string) => {
    if (!query.trim()) return;

    // Add user message to UI immediately
    const userMessage: Message = {
      id: `user_${Date.now()}`,
      role: 'user',
      content: query,
      timestamp: new Date(),
    };

    setMessages(prev => [...prev, userMessage]);
    setIsLoading(true);
    setError(null);

    try {
      const response = await chatbotService.sendMessage(query, selectedText);
      setMessages(prev => [...prev, response]);
    } catch (err) {
      setError(err instanceof Error ? err.message : 'An error occurred while sending the message');
      console.error('Chat error:', err);
    } finally {
      setIsLoading(false);
    }
  }, []);

  const clearConversation = useCallback(() => {
    setMessages([]);
    chatbotService.resetConversation();
    setError(null);
  }, []);

  return {
    messages,
    isLoading,
    error,
    sendMessage,
    clearConversation,
  };
};

export default useChatbot;