import React from 'react';
import { ChatMessage } from './ChatbotWidget';

interface MessageProps {
  message: ChatMessage;
}

export const Message: React.FC<MessageProps> = ({ message }) => {
  const isUser = message.sender === 'user';

  return (
    <div className={`message ${isUser ? 'user-message' : 'assistant-message'}`}>
      <div className="message-content">
        <div className="message-text">
          {message.content.split('\n').map((line, i) => (
            <React.Fragment key={i}>
              {line}
              {i < message.content.split('\n').length - 1 && <br />}
            </React.Fragment>
          ))}
        </div>

        {message.sources && message.sources.length > 0 && !isUser && (
          <div className="message-sources">
            <h5>Sources:</h5>
            <ul>
              {message.sources.slice(0, 3).map((source, index) => (
                <li key={index}>
                  <strong>{source.chapter} - {source.lesson}</strong>: {source.section_title}
                </li>
              ))}
              {message.sources.length > 3 && (
                <li>... and {message.sources.length - 3} more sources</li>
              )}
            </ul>
          </div>
        )}
      </div>
      <div className="message-timestamp">
        {message.timestamp.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
      </div>
    </div>
  );
};