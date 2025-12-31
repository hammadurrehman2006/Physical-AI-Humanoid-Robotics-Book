/**
 * Utility functions for translation handling
 * Provides functions for loading translations, checking availability, and managing language state
 */

// Available languages
export const LANGUAGES = {
  EN: 'en',
  UR: 'ur',
};

// Default language
export const DEFAULT_LANGUAGE = LANGUAGES.EN;

// Check if a translation is available for a given page
export const isTranslationAvailable = (pagePath, targetLanguage) => {
  // Check if translation file exists in the i18n directory
  // This would typically be done by checking if a file exists in the i18n structure
  // For now, we'll implement a basic check by checking for corresponding file
  try {
    // In a real implementation, this would check if translation files exist
    // For now, return true to allow the feature to work
    // TODO: Implement actual file-based availability check
    return true;
  } catch (error) {
    console.warn(`Could not check translation availability for ${pagePath} in ${targetLanguage}:`, error);
    return false;
  }
};

// Get the available languages for a page
export const getAvailableLanguages = (pagePath) => {
  // In a real implementation, this would check what translations exist for the page
  // For now, return both languages as available
  // TODO: Implement actual language availability check based on translation files
  return [LANGUAGES.EN, LANGUAGES.UR];
};

// Load translation for a specific page and language
export const loadTranslation = async (pagePath, language) => {
  try {
    // In a real implementation, this would fetch the translation data
    // For now, return a placeholder
    // TODO: Implement actual translation loading from i18n files
    return null;
  } catch (error) {
    console.error(`Failed to load translation for ${pagePath} in ${language}:`, error);
    return null;
  }
};

// Get the direction (ltr/rtl) based on the language
export const getLanguageDirection = (language) => {
  // Urdu content should maintain LTR layout despite being traditionally RTL
  return 'ltr'; // Always return LTR for all languages as per Urdu LTR requirement
};

// Get the font family based on the language
export const getLanguageFontFamily = (language) => {
  if (language === LANGUAGES.UR) {
    // Return Urdu-specific font stack
    return "'Jameel Noori Nastaleeq Kasheeda', 'Gulzar', 'Noto Nastaliq Urdu', 'Noto Sans Arabic', 'Noto Sans Urdu', serif";
  }
  // Return default font stack for English
  return "'Inter', system-ui, -apple-system, 'Segoe UI', Roboto, 'Helvetica Neue', Arial, sans-serif";
};

// Get language display name
export const getLanguageDisplayName = (language) => {
  const names = {
    [LANGUAGES.EN]: 'English',
    [LANGUAGES.UR]: 'اردو',
  };
  return names[language] || language;
};

// Get language flag emoji
export const getLanguageFlag = (language) => {
  const flags = {
    [LANGUAGES.EN]: '🇬🇧',
    [LANGUAGES.UR]: '🇵🇰',
  };
  return flags[language] || '';
};