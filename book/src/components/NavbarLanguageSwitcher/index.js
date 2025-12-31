import React from 'react';
import { useLanguage } from '../../context/LanguageContext';
import '../../css/urdu-styles.css';

const NavbarLanguageSwitcher = () => {
  // Check if we're in the browser environment
  const isBrowser = typeof window !== 'undefined';

  // Use language context if available, fallback to browser detection
  let currentLanguage = 'en';
  let changeLanguage = null;

  try {
    if (isBrowser) {
      const languageContext = useLanguage();
      if (languageContext) {
        currentLanguage = languageContext.currentLanguage;
        changeLanguage = languageContext.changeLanguage;
      }
    }
  } catch (error) {
    // Fallback to URL-based detection if context is not available during SSR
    if (isBrowser) {
      currentLanguage = window.location.pathname.startsWith('/ur/') ? 'ur' : 'en';
    }
  }

  const handleLanguageChange = (language) => {
    // For Docusaurus i18n, we need to redirect to the appropriate locale path
    if (isBrowser && language !== currentLanguage) {
      let newUrl;
      const currentPath = window.location.pathname;

      if (language === 'ur') {
        // If we're on the default locale (en), we need to add /ur/ to the path
        if (currentPath.startsWith('/ur/')) {
          // Already on Urdu locale, no change needed
          return;
        } else {
          // Add ur prefix to current path, handling special cases
          if (currentPath === '/') {
            newUrl = '/ur/';
          } else {
            newUrl = `/ur${currentPath}`;
          }
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
    <div className="navbar__item language-switcher-navbar">
      <div className="language-switcher" role="region" aria-label="Language selection">
        <button
          className={`language-switcher-button ${currentLanguage === 'en' ? 'active' : ''}`}
          onClick={() => handleLanguageChange('en')}
          aria-label="Switch to English"
          title="English"
          aria-pressed={currentLanguage === 'en'}
          aria-current={currentLanguage === 'en' ? 'true' : 'false'}
        >
          <span aria-hidden="true">🇬🇧</span>
          <span className="sr-only">English</span>
        </button>
        <button
          className={`language-switcher-button ${currentLanguage === 'ur' ? 'active' : ''}`}
          onClick={() => handleLanguageChange('ur')}
          aria-label="Switch to Urdu"
          title="اردو"
          aria-pressed={currentLanguage === 'ur'}
          aria-current={currentLanguage === 'ur' ? 'true' : 'false'}
        >
          <span aria-hidden="true">🇵🇰</span>
          <span className="sr-only">Urdu</span>
        </button>
      </div>
    </div>
  );
};

export default React.memo(NavbarLanguageSwitcher);