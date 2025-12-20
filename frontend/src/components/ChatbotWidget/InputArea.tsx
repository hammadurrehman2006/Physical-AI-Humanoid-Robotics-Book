import React, { useState, KeyboardEvent } from 'react';

interface InputAreaProps {
  onSend: (message: string) => void;
  isLoading: boolean;
  queryType: 'FULL_BOOK' | 'SELECTED_TEXT';
  selectedText: string | null;
}

export const InputArea: React.FC<InputAreaProps> = ({
  onSend,
  isLoading,
  queryType,
  selectedText
}) => {
  const [inputValue, setInputValue] = useState('');

  const handleSubmit = () => {
    if (inputValue.trim() && !isLoading) {
      onSend(inputValue.trim());
      setInputValue('');
    }
  };

  const handleKeyDown = (e: KeyboardEvent<HTMLTextAreaElement>) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSubmit();
    }
  };

  return (
    <div className="input-area">
      {queryType === 'SELECTED_TEXT' && selectedText && (
        <div className="selected-text-preview">
          <small>Selected: "{selectedText.substring(0, 60)}{selectedText.length > 60 ? '...' : ''}"</small>
        </div>
      )}

      <div className="input-container">
        <textarea
          value={inputValue}
          onChange={(e) => setInputValue(e.target.value)}
          onKeyDown={handleKeyDown}
          placeholder={
            queryType === 'SELECTED_TEXT'
              ? "Ask about the selected text..."
              : "Ask about the book content..."
          }
          disabled={isLoading}
          rows={2}
        />
        <button
          onClick={handleSubmit}
          disabled={isLoading || !inputValue.trim()}
          className="send-button"
        >
          {isLoading ? 'Sending...' : 'Send'}
        </button>
      </div>

      <div className="input-hints">
        <small>Press Enter to send, Shift+Enter for new line</small>
      </div>
    </div>
  );
};