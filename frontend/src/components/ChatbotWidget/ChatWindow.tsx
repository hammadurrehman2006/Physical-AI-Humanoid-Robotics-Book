import React from 'react';
import { Message } from './Message';
import { ChatMessage } from './ChatbotWidget';

interface ChatWindowProps {
  messages: ChatMessage[];
  isLoading: boolean;
}

export const ChatWindow: React.FC<ChatWindowProps> = ({ messages, isLoading }) => {
  return (
    <div className="chat-window">
      {messages.length === 0 ? (
        <div className="chat-welcome">
          <h4>Hello! I'm your Physical AI & Humanoid Robotics Assistant.</h4>
          <p>Ask me anything about the book content, or select text on the page to get clarifications.</p>
          <div className="welcome-tips">
            <div className="tip">
              <strong>💡 Tip:</strong> Select text on the page and ask questions about it
            </div>
            <div className="tip">
              <strong>📚 Tip:</strong> Ask about specific chapters, lessons, or concepts
            </div>
          </div>
        </div>
      ) : (
        <div className="messages-container">
          {messages.map((message) => (
            <Message key={message.id} message={message} />
          ))}
          {isLoading && (
            <div className="message assistant-message">
              <div className="message-content">
                <div className="typing-indicator">
                  <span></span>
                  <span></span>
                  <span></span>
                </div>
              </div>
            </div>
          )}
        </div>
      )}
    </div>
  );
};