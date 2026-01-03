# Quickstart: Floating Chatbot UI Implementation

## Prerequisites

- Node.js 18+ installed
- Docusaurus 3.x project set up
- Basic knowledge of React and TypeScript
- Access to the logo.png file in the img directory

## Setup Steps

### 1. Create the Component Directory

```bash
mkdir -p book/src/components/Chatbot
```

### 2. Create the Main Component Files

Create the following files in the `book/src/components/Chatbot/` directory:

- `FloatingChatbot.tsx` - Main floating chatbot component
- `ChatWindow.tsx` - Expanded chat interface component
- `ChatMessage.tsx` - Individual message component
- `InputArea.tsx` - Input area with send button
- `styles.module.css` - Component-specific styles

### 3. Create the Component Export File

Create `book/src/components/index.tsx` to export the components:

```typescript
export { default as FloatingChatbot } from './Chatbot/FloatingChatbot';
export { default as ChatWindow } from './Chatbot/ChatWindow';
export { default as ChatMessage } from './Chatbot/ChatMessage';
export { default as InputArea } from './Chatbot/InputArea';
```

### 4. Add the Chatbot to Layout

Modify `book/src/theme/Layout/index.js` to include the floating chatbot:

```javascript
import React from 'react';
import Layout from '@theme-original/Layout';
import { FloatingChatbot } from '@site/src/components';

export default function LayoutWrapper(props) {
  return (
    <>
      <Layout {...props}>
        {props.children}
        <FloatingChatbot />
      </Layout>
    </>
  );
}
```

### 5. Add Global Styles

Add the following to `book/src/css/custom.css` to ensure proper positioning:

```css
/* Floating chatbot positioning */
.floating-chatbot-container {
  position: fixed;
  bottom: 20px;
  right: 20px;
  z-index: 1000;
  transition: all 0.3s ease;
}

@media (max-width: 768px) {
  .floating-chatbot-container {
    bottom: 15px;
    right: 15px;
  }
}
```

### 6. Verify Asset Location

Ensure the `logo.png` file exists in the `book/static/img/` directory for use as the chatbot icon.

## Running and Testing

### Development

```bash
cd book
npm start
```

The floating chatbot should appear in the bottom-right corner of all pages.

### Testing

1. Verify the chatbot appears on all pages
2. Test opening and closing the chat interface
3. Verify responsive behavior on different screen sizes
4. Test keyboard navigation accessibility
5. Check theme consistency across light/dark modes