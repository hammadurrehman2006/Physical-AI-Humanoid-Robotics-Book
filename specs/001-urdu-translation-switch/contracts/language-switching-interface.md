# API Contract: Language Switching Interface

## Overview
This document defines the interface contracts for the language switching functionality in the Physical AI & Humanoid Robotics book. Since this is a client-side implementation using Docusaurus, the "API" refers to the component interfaces and data contracts.

## Language Switcher Component Interface

### Interface: ILanguageSwitcher
```typescript
interface ILanguageSwitcher {
  // Current selected language ('en' or 'ur')
  currentLanguage: string;

  // Function to switch language
  switchLanguage: (language: 'en' | 'ur') => Promise<void>;

  // Function to check if translation exists for current page
  hasTranslation: (pageId: string, language: 'en' | 'ur') => boolean;

  // Function to get available languages for current page
  getAvailableLanguages: (pageId: string) => string[];
}
```

### Events
- `onLanguageChanged`: Emitted when language is successfully switched
- `onTranslationLoading`: Emitted when translation content is being loaded
- `onTranslationError`: Emitted when translation fails to load

## Translation Content Interface

### Interface: ITranslationContent
```typescript
interface ITranslationContent {
  // Unique identifier for the content
  id: string;

  // Page identifier
  pageId: string;

  // Language code
  language: 'en' | 'ur';

  // Translated title
  title: string;

  // Translated body content
  body: string;

  // Creation timestamp
  createdAt: Date;

  // Last update timestamp
  updatedAt: Date;

  // Version of the translation
  version: string;
}
```

## Language Context Interface

### Interface: ILanguageContext
```typescript
interface ILanguageContext {
  // Current language setting
  language: 'en' | 'ur';

  // Available languages
  availableLanguages: Array<'en' | 'ur'>;

  // Function to change language
  setLanguage: (lang: 'en' | 'ur') => void;

  // Function to get translation for current page
  getTranslation: (pageId: string, targetLanguage: 'en' | 'ur') => ITranslationContent | null;

  // Function to check if translation exists
  hasTranslation: (pageId: string, targetLanguage: 'en' | 'ur') => boolean;

  // Function to get current page content in selected language
  getCurrentContent: (pageId: string) => ITranslationContent | null;
}
```

## Font Configuration Interface

### Interface: IFontConfiguration
```typescript
interface IFontConfiguration {
  // Font family for Urdu text (e.g., 'Jameel Noori Nastaleeq Kasheeda', 'Gulzar', 'Noto Nastaliq Urdu')
  fontFamily: string;

  // URL to Google Font resource
  fontUrl: string;

  // Fallback font families
  fallbackFonts: string[];

  // CSS properties for proper rendering
  cssProperties: Record<string, any>;

  // Associated language code
  languageCode: 'ur';
}
```

## Data Flow Contracts

### Language Switching Flow
1. User triggers language switch via UI component
2. System checks if translation exists for current page in target language
3. If translation exists:
   - Load required fonts (if switching to Urdu)
   - Update content with translated version
   - Update UI to reflect new language
4. If translation doesn't exist:
   - Show error message or fallback to English
   - Maintain current language setting

### Content Loading Contract
- The system must provide translated content within 2 seconds of language switch
- If content is not available in target language, the system must maintain the current language
- All content must be validated before rendering to prevent XSS vulnerabilities

### Font Loading Contract
- Urdu fonts must be loaded asynchronously without blocking page rendering
- If font fails to load, the system must use appropriate fallback fonts
- Font loading must follow web performance best practices (font-display: swap)

## Error Handling Contracts

### Translation Not Found
- **Condition**: Requested translation does not exist for a page
- **Response**: Return to current language with notification to user
- **Fallback**: Maintain existing content in original language

### Font Loading Error
- **Condition**: Google Font fails to load
- **Response**: Use system fallback fonts
- **Fallback**: Ensure text remains readable with default fonts

### Invalid Language Code
- **Condition**: Attempt to switch to unsupported language
- **Response**: Ignore request and maintain current language
- **Fallback**: Log error and continue with current language