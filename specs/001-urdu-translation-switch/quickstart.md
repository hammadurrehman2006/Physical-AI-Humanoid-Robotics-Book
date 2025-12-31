# Quickstart: Urdu Translation Switch

## Overview
This guide provides a quick overview of implementing the Urdu translation switch feature in the Physical AI & Humanoid Robotics book. The feature allows readers to toggle between English and Urdu translations on any page with proper Google font rendering for Urdu text.

## Prerequisites
- Node.js 18+ and npm/yarn
- Docusaurus 3.x project setup
- Google Fonts access for Urdu fonts (Jameel Noori Nastaleeq Kasheeda, Gulzar, or Noto Nastaliq Urdu)

## Implementation Steps

### 1. Setup Internationalization Structure
Create the i18n directory structure in your Docusaurus project:
```bash
mkdir -p i18n/en
mkdir -p i18n/ur
```

### 2. Create Language Context
Create a React context for managing language state:
```javascript
// src/context/LanguageContext.js
import React, { createContext, useContext, useState } from 'react';

const LanguageContext = createContext();

export const useLanguage = () => {
  const context = useContext(LanguageContext);
  if (!context) {
    throw new Error('useLanguage must be used within a LanguageProvider');
  }
  return context;
};

export const LanguageProvider = ({ children }) => {
  const [language, setLanguage] = useState('en');

  const switchLanguage = (newLanguage) => {
    setLanguage(newLanguage);
    localStorage.setItem('preferred-language', newLanguage);
  };

  return (
    <LanguageContext.Provider value={{ language, switchLanguage }}>
      {children}
    </LanguageContext.Provider>
  );
};
```

### 3. Create Language Switcher Component
Create a React component for the language switcher UI:
```javascript
// src/components/LanguageSwitcher/index.js
import React from 'react';
import { useLanguage } from '../../context/LanguageContext';

const LanguageSwitcher = () => {
  const { language, switchLanguage } = useLanguage();

  const handleLanguageChange = (newLanguage) => {
    switchLanguage(newLanguage);
  };

  return (
    <div className="language-switcher">
      <button
        className={language === 'en' ? 'active' : ''}
        onClick={() => handleLanguageChange('en')}
      >
        English
      </button>
      <button
        className={language === 'ur' ? 'active' : ''}
        onClick={() => handleLanguageChange('ur')}
      >
        اردو
      </button>
    </div>
  );
};

export default LanguageSwitcher;
```

### 4. Integrate with Layout
Wrap your main layout with the language provider:
```javascript
// src/theme/Layout/index.js
import React from 'react';
import OriginalLayout from '@theme-original/Layout';
import { LanguageProvider } from '../../context/LanguageContext';
import LanguageSwitcher from '../../components/LanguageSwitcher';

export default function Layout(props) {
  return (
    <LanguageProvider>
      <OriginalLayout {...props}>
        <LanguageSwitcher />
        {props.children}
      </OriginalLayout>
    </LanguageProvider>
  );
}
```

### 5. Configure Google Fonts
Add Google Fonts for Urdu text rendering in your CSS:
```css
/* src/css/urdu-styles.css */
@import url('https://fonts.googleapis.com/css2?family=Jameel+Noori+Nastaleeq+Kasheeda:wght@400&family=Gulzar&family=Noto+Nastaliq+Urdu&display=swap');

.urdu-text {
  font-family: 'Jameel Noori Nastaleeq Kasheeda', 'Gulzar', 'Noto Nastaliq Urdu', 'Noto Naskh Arabic', serif;
  direction: rtl;
  text-align: right;
}
```

### 6. Update Docusaurus Configuration
Add i18n configuration to your docusaurus.config.js:
```javascript
module.exports = {
  i18n: {
    defaultLocale: 'en',
    locales: ['en', 'ur'],
    localeConfigs: {
      en: {
        label: 'English',
      },
      ur: {
        label: 'اردو',
      },
    },
  },
  // ... other config
};
```

## Running the Implementation
1. Add translated content to the i18n directories
2. Start your Docusaurus development server:
```bash
npm run start
```
3. The language switcher will appear on all pages, allowing users to toggle between English and Urdu

## Testing
- Verify the language switcher appears on all pages
- Test switching between languages
- Ensure Urdu text renders properly with Google Fonts
- Confirm language preference persists across sessions
- Verify that all content is properly translated

## Next Steps
1. Add translated content to all book pages
2. Implement RTL layout support for Urdu content
3. Add font loading optimization
4. Create translation management tools