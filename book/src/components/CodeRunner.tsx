import React from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';

// A simple code runner component for displaying and potentially executing code
const CodeRunner = ({ code, language = 'python', title = 'Code Example', description = '' }) => {
  return (
    <div className="code-runner">
      <div className="code-header">
        <span className="code-title">{title}</span>
        <span className="code-language">{language}</span>
      </div>
      {description && <div className="code-description">{description}</div>}
      <div className="code-content">
        <pre>
          <code className={`language-${language}`}>
            {code}
          </code>
        </pre>
      </div>
      <div className="code-actions">
        <button className="btn-copy" onClick={() => navigator.clipboard.writeText(code)}>
          Copy
        </button>
      </div>
    </div>
  );
};

export default CodeRunner;