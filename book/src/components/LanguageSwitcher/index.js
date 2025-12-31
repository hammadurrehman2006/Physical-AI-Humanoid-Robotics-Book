import React from 'react';
import { useLanguage } from '../context/LanguageContext';
import { LANGUAGES, getLanguageDisplayName, getLanguageFlag } from '../utils/translation-utils';
import '../css/urdu-styles.css';

/**
 * LanguageSwitcher Component
 * Provides UI for switching between available languages with enhanced accessibility
 */
const LanguageSwitcher = () => {
  const { currentLanguage, changeLanguage } = useLanguage();

  const handleLanguageChange = (language) => {
    // For Docusaurus i18n, we need to redirect to the appropriate locale path
    if (language !== currentLanguage) {
      let newUrl;
      const currentPath = window.location.pathname;

      if (language === 'ur') {
        // If we're on the default locale (en), we need to add /ur/ to the path
        if (currentPath.startsWith('/ur/')) {
          // Already on Urdu locale, no change needed
          return;
        } else {
          // Add ur prefix to current path
          newUrl = `/ur${currentPath}`;
        }
      } else {
        // Switching to English (default), remove /ur/ prefix if present
        if (currentPath.startsWith('/ur/')) {
          newUrl = currentPath.replace(/^\/ur\//, '/');
          if (newUrl === '/') {
            // If it's the root, just redirect to /
            newUrl = '/';
          }
        } else {
          // Already on English locale, no change needed
          return;
        }
      }

      // Redirect to new locale
      window.location.href = newUrl;
    }
  };

  return (
    <div className="language-switcher" role="region" aria-label="Language selection">
      <button
        className={`language-switcher-button ${currentLanguage === LANGUAGES.EN ? 'active' : ''}`}
        onClick={() => handleLanguageChange(LANGUAGES.EN)}
        aria-label={`Switch to ${getLanguageDisplayName(LANGUAGES.EN)}`}
        title={getLanguageDisplayName(LANGUAGES.EN)}
        aria-pressed={currentLanguage === LANGUAGES.EN}
        aria-current={currentLanguage === LANGUAGES.EN ? 'true' : 'false'}
      >
        <span>{getLanguageDisplayName(LANGUAGES.EN)}</span>
      </button>
      <button
        className={`language-switcher-button ${currentLanguage === LANGUAGES.UR ? 'active' : ''}`}
        onClick={() => handleLanguageChange(LANGUAGES.UR)}
        aria-label={`Switch to ${getLanguageDisplayName(LANGUAGES.UR)}`}
        title={getLanguageDisplayName(LANGUAGES.UR)}
        aria-pressed={currentLanguage === LANGUAGES.UR}
        aria-current={currentLanguage === LANGUAGES.UR ? 'true' : 'false'}
      >
        <span>{getLanguageDisplayName(LANGUAGES.UR)}</span>
      </button>
    </div>
  );
};

export default LanguageSwitcher;