import React, { useState, useRef, useEffect } from 'react';
import { Message, ChatWindowProps } from './types';
import ChatMessage from './ChatMessage';
import InputArea from './InputArea';
import styles from './styles.module.css';
import { useAuth } from '../../auth/hooks/useAuth';

const CHAT_MESSAGES_STORAGE_KEY = 'docusaurus.chat.messages';

const ChatWindow: React.FC<ChatWindowProps> = ({ onClose }) => {
  const { isAuthenticated, isLoading } = useAuth();
  const [messages, setMessages] = useState<Message[]>(() => {
    // Initialize with welcome message or load from storage
    if (typeof window !== 'undefined') {
      const storedMessages = localStorage.getItem(CHAT_MESSAGES_STORAGE_KEY);
      if (storedMessages) {
        try {
          const parsed = JSON.parse(storedMessages);
          // Convert timestamp strings back to Date objects
          return parsed.map((msg: any) => ({
            ...msg,
            timestamp: new Date(msg.timestamp)
          }));
        } catch (e) {
          console.warn('Failed to parse stored messages, using default');
        }
      }
    }

    // Default welcome message if no stored messages
    const welcomeMessages: Message[] = [
      {
        id: '1',
        content: 'Hello! How can I help you with the Physical AI & Humanoid Robotics content today?',
        sender: 'bot',
        timestamp: new Date(),
      }
    ];

    // Add authentication reminder if user is not authenticated
    if (!isAuthenticated && !isLoading) {
      welcomeMessages.push({
        id: '2',
        content: 'Please sign in to access the full chatbot functionality. This feature is available to registered users only.',
        sender: 'bot',
        timestamp: new Date(),
      });
    }

    return welcomeMessages;
  });

  const [inputValue, setInputValue] = useState('');
  const messagesEndRef = useRef<HTMLDivElement>(null);

  // Save messages to localStorage whenever they change
  useEffect(() => {
    if (typeof window !== 'undefined') {
      try {
        localStorage.setItem(
          CHAT_MESSAGES_STORAGE_KEY,
          JSON.stringify(messages.map(msg => ({
            ...msg,
            timestamp: msg.timestamp.toISOString()
          })))
        );
      } catch (e) {
        console.warn('Failed to save messages to localStorage:', e);
      }
    }
  }, [messages]);

  const handleSend = async (message: string) => {
    if (message.trim() === '') return;

    // Check if user is authenticated
    if (!isAuthenticated && !isLoading) {
      // Add a message prompting the user to sign in
      const authPromptMessage: Message = {
        id: Date.now().toString(),
        content: 'Please sign in to continue using the chatbot. This feature is available to registered users only.',
        sender: 'bot',
        timestamp: new Date(),
      };
      setMessages(prev => [...prev, authPromptMessage]);
      return;
    }

    // Add user message
    const userMessage: Message = {
      id: Date.now().toString(),
      content: message,
      sender: 'user',
      timestamp: new Date(),
    };

    setMessages(prev => [...prev, userMessage]);
    setInputValue('');

    try {
      // Call the backend API
      // Using localhost for development/testing as per CLI environment
      const response = await fetch('http://localhost:8000/api/v1/chat/', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        credentials: 'include', // Send cookies for authentication
        body: JSON.stringify({ message }),
      });

      if (!response.ok) {
        if (response.status === 401) {
           throw new Error("Unauthorized. Please sign in again.");
        }
        throw new Error(`Server error: ${response.status}`);
      }

      const data = await response.json();

      const botMessage: Message = {
        id: (Date.now() + 1).toString(),
        content: data.response,
        sender: 'bot',
        timestamp: new Date(),
      };
      setMessages(prev => [...prev, botMessage]);

    } catch (error) {
      console.error('Chat error:', error);
      const errorMessage: Message = {
        id: (Date.now() + 1).toString(),
        content: error instanceof Error ? `Error: ${error.message}` : "Sorry, I'm having trouble connecting to the server.",
        sender: 'bot',
        timestamp: new Date(),
      };
      setMessages(prev => [...prev, errorMessage]);
    }
  };

  // Scroll to bottom when messages change
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  return (
    <div className={styles.chatWindow}>
      <div className={styles.chatHeader}>
        <h3>AI Assistant</h3>
        <button
          className={styles.closeButton}
          onClick={onClose}
          aria-label="Close chat"
        >
          ×
        </button>
      </div>
      <div className={styles.messagesContainer}>
        {messages.map((message) => (
          <ChatMessage key={message.id} message={message} />
        ))}
        <div ref={messagesEndRef} />
      </div>
      <InputArea
        value={inputValue}
        onChange={setInputValue}
        onSend={handleSend}
        isAuthenticated={isAuthenticated}
      />
    </div>
  );
};

export default ChatWindow;