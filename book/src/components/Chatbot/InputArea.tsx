import React, { useState, useRef } from 'react';
import { InputAreaProps } from './types';
import styles from './styles.module.css';

const InputArea: React.FC<InputAreaProps> = ({ value, onChange, onSend }) => {
  const [inputValue, setInputValue] = useState(value);
  const textareaRef = useRef<HTMLTextAreaElement>(null);

  const handleInputChange = (e: React.ChangeEvent<HTMLTextAreaElement>) => {
    const newValue = e.target.value;
    setInputValue(newValue);
    onChange(newValue);
  };

  const handleKeyDown = (e: React.KeyboardEvent<HTMLTextAreaElement>) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSend();
    }
  };

  const handleSend = () => {
    if (inputValue.trim()) {
      onSend(inputValue);
      setInputValue('');
      onChange('');
    }
  };

  // Auto-resize textarea based on content
  const handleInput = (e: React.FormEvent<HTMLTextAreaElement>) => {
    const target = e.target as HTMLTextAreaElement;
    target.style.height = 'auto';
    target.style.height = `${Math.min(target.scrollHeight, 100)}px`;
  };

  return (
    <div className={styles.inputArea}>
      <textarea
        ref={textareaRef}
        className={styles.textInput}
        value={inputValue}
        onChange={handleInputChange}
        onKeyDown={handleKeyDown}
        onInput={handleInput}
        placeholder="Type your message here..."
        rows={1}
        aria-label="Chat message input"
      />
      <button
        className={styles.sendButton}
        onClick={handleSend}
        disabled={!inputValue.trim()}
        aria-label="Send message"
      >
        Send
      </button>
    </div>
  );
};

export default InputArea;