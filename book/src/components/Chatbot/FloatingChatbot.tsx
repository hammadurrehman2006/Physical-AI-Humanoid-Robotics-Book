import React, { useState, useEffect } from 'react';
import ChatWindow from './ChatWindow';
import styles from './styles.module.css';

const FloatingChatbot = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [isVisible, setIsVisible] = useState(true);

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  const closeChat = () => {
    setIsOpen(false);
  };

  // Check if logo.png exists in the static folder
  const logoPath = '/img/logo.png';

  return (
    <div className={styles.floatingChatbotContainer}>
      {isOpen ? (
        <ChatWindow onClose={closeChat} />
      ) : (
        <button
          className={`${styles.floatingButton} ${styles.animateFloat}`}
          onClick={toggleChat}
          aria-label="Open chatbot"
          aria-expanded={isOpen}
        >
          <img
            src={logoPath}
            alt="Chatbot"
            className={styles.chatIcon}
            onError={(e) => {
              // Fallback to a default icon if logo.png doesn't exist
              e.currentTarget.src = 'data:image/svg+xml;utf8,<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24" fill="currentColor"><path d="M20 2H4c-1.1 0-1.99.9-1.99 2L2 22l4-4h14c1.1 0 2-.9 2-2V4c0-1.1-.9-2-2-2zm-2 12H6v-2h12v2zm0-3H6V9h12v2zm0-3H6V6h12v2z"/></svg>';
            }}
          />
        </button>
      )}
    </div>
  );
};

export default FloatingChatbot;