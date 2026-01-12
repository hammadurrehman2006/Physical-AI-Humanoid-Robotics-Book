import React, { useState, useRef } from 'react';
import { InputAreaProps } from './types';
import styles from './styles.module.css';

interface EnhancedInputAreaProps extends InputAreaProps {
  isAuthenticated?: boolean;
}

const InputArea: React.FC<EnhancedInputAreaProps> = ({ value, onChange, onSend, isAuthenticated = true }) => {
  const [inputValue, setInputValue] = useState(value);
  const textareaRef = useRef<HTMLTextAreaElement>(null);

  const handleInputChange = (e: React.ChangeEvent<HTMLTextAreaElement>) => {
    if (!isAuthenticated) return; // Don't allow input if not authenticated
    const newValue = e.target.value;
    setInputValue(newValue);
    onChange(newValue);
  };

  const handleKeyDown = (e: React.KeyboardEvent<HTMLTextAreaElement>) => {
    if (!isAuthenticated) return; // Don't allow input if not authenticated
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSend();
    }
  };

  const handleSend = () => {
    if (!isAuthenticated) return; // Don't allow sending if not authenticated
    if (inputValue.trim()) {
      onSend(inputValue);
      setInputValue('');
      onChange('');
    }
  };

  // Auto-resize textarea based on content
  const handleInput = (e: React.FormEvent<HTMLTextAreaElement>) => {
    if (!isAuthenticated) return; // Don't allow input if not authenticated
    const target = e.target as HTMLTextAreaElement;
    target.style.height = 'auto';
    target.style.height = `${Math.min(target.scrollHeight, 100)}px`;
  };

  return (
    <div className={styles.inputArea}>
      <textarea
        ref={textareaRef}
        className={`${styles.textInput} ${!isAuthenticated ? styles.disabledInput : ''}`}
        value={inputValue}
        onChange={handleInputChange}
        onKeyDown={handleKeyDown}
        onInput={handleInput}
        placeholder={isAuthenticated ? "Type your message here..." : "Sign in to use the chatbot"}
        rows={1}
        aria-label="Chat message input"
        disabled={!isAuthenticated}
      />
      <button
        className={`${styles.sendButton} ${!isAuthenticated ? styles.disabledButton : ''}`}
        onClick={handleSend}
        disabled={!isAuthenticated || !inputValue.trim()}
        aria-label="Send message"
      >
        {isAuthenticated ? "Send" : "Sign In"}
      </button>
    </div>
  );
};

export default InputArea;