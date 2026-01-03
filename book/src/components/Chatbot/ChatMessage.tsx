import React from 'react';
import { Message } from './types';
import styles from './styles.module.css';

interface ChatMessageProps {
  message: Message;
}

const ChatMessage: React.FC<ChatMessageProps> = ({ message }) => {
  const isUser = message.sender === 'user';

  return (
    <div className={`${styles.message} ${isUser ? styles.userMessage : styles.botMessage}`}>
      <div className={styles.messageContent}>
        {message.content}
      </div>
      <div className={styles.messageTimestamp}>
        {message.timestamp.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
      </div>
    </div>
  );
};

export default ChatMessage;