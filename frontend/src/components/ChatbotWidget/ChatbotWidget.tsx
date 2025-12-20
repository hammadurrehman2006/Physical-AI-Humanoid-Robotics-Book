import React, { useState, useEffect, useRef } from 'react';
import { ChatWindow } from './ChatWindow';
import { InputArea } from './InputArea';
import { Message } from './Message';
import { apiClient } from '../../services/apiClient';
import './chatbot.css';

interface ChatMessage {
  id: string;
  content: string;
  sender: 'user' | 'assistant';
  timestamp: Date;
  sources?: Array<{
    source_file: string;
    section_title: string;
    chapter: string;
    lesson: string;
    relevance_score: number;
  }>;
}

interface ChatbotWidgetProps {
  initialQuery?: string;
  position?: 'bottom-right' | 'bottom-left' | 'embedded';
  size?: 'small' | 'medium' | 'large';
}

export const ChatbotWidget: React.FC<ChatbotWidgetProps> = ({
  initialQuery = '',
  position = 'bottom-right',
  size = 'medium'
}) => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<ChatMessage[]>([]);
  const [isLoading, setIsLoading] = useState(false);
  const [selectedText, setSelectedText] = useState<string | null>(null);
  const [queryType, setQueryType] = useState<'FULL_BOOK' | 'SELECTED_TEXT'>('FULL_BOOK');
  const messagesEndRef = useRef<HTMLDivElement>(null);

  // Handle text selection
  useEffect(() => {
    const handleSelection = () => {
      const selectedText = window.getSelection()?.toString().trim();
      if (selectedText && selectedText.length > 10) { // Only consider meaningful selections
        setSelectedText(selectedText);
        setQueryType('SELECTED_TEXT');
      } else {
        setSelectedText(null);
        setQueryType('FULL_BOOK');
      }
    };

    document.addEventListener('mouseup', handleSelection);
    return () => {
      document.removeEventListener('mouseup', handleSelection);
    };
  }, []);

  // Scroll to bottom when messages change
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  // Add initial message if provided
  useEffect(() => {
    if (initialQuery) {
      const initialMsg: ChatMessage = {
        id: 'initial',
        content: initialQuery,
        sender: 'user',
        timestamp: new Date(),
      };
      setMessages([initialMsg]);
      sendMessage(initialQuery);
    }
  }, [initialQuery]);

  const sendMessage = async (query: string) => {
    if (!query.trim() || isLoading) return;

    setIsLoading(true);

    // Add user message
    const userMessage: ChatMessage = {
      id: Date.now().toString(),
      content: query,
      sender: 'user',
      timestamp: new Date(),
    };

    setMessages(prev => [...prev, userMessage]);

    try {
      // Prepare the request
      const requestBody = {
        query: query,
        query_type: selectedText ? 'SELECTED_TEXT' : 'FULL_BOOK',
        selected_text: selectedText || undefined,
      };

      // Call the backend API
      const response = await apiClient.chat(requestBody);

      // Add assistant message
      const assistantMessage: ChatMessage = {
        id: Date.now().toString() + '-assistant',
        content: response.response,
        sender: 'assistant',
        timestamp: new Date(),
        sources: response.sources || [],
      };

      setMessages(prev => [...prev, assistantMessage]);
    } catch (error) {
      console.error('Error sending message:', error);

      // Add error message
      const errorMessage: ChatMessage = {
        id: Date.now().toString() + '-error',
        content: 'Sorry, I encountered an error processing your request. Please try again.',
        sender: 'assistant',
        timestamp: new Date(),
      };

      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  const clearChat = () => {
    setMessages([]);
  };

  return (
    <div className={`chatbot-container chatbot-${position} chatbot-${size}`}>
      {isOpen ? (
        <div className="chatbot-window">
          <div className="chatbot-header">
            <h3>Physical AI & Robotics Assistant</h3>
            <div className="chatbot-controls">
              <button onClick={clearChat} className="chatbot-clear-btn" title="Clear chat">
                ✕
              </button>
              <button onClick={toggleChat} className="chatbot-close-btn" title="Close chat">
                −
              </button>
            </div>
          </div>

          <ChatWindow messages={messages} isLoading={isLoading} />

          <InputArea
            onSend={sendMessage}
            isLoading={isLoading}
            queryType={queryType}
            selectedText={selectedText}
          />

          <div ref={messagesEndRef} />
        </div>
      ) : (
        <button className="chatbot-launcher" onClick={toggleChat}>
          <span>🤖 Ask AI</span>
        </button>
      )}
    </div>
  );
};