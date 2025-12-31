import React, { createContext, useContext, useState, useEffect } from 'react';
import { LANGUAGES, DEFAULT_LANGUAGE } from '../utils/translation-utils';

// Create the context
const LanguageContext = createContext();

// Custom hook to use the language context
export const useLanguage = () => {
  const context = useContext(LanguageContext);
  if (!context) {
    throw new Error('useLanguage must be used within a LanguageProvider');
  }
  return context;
};

// LanguageProvider component
export const LanguageProvider = ({ children }) => {
  // Get the initial language from URL or default to English
  const getInitialLanguage = () => {
    if (typeof window !== 'undefined') {
      // Check URL path for locale
      const path = window.location.pathname;
      if (path.startsWith('/ur/')) {
        return LANGUAGES.UR;
      }
      return DEFAULT_LANGUAGE;
    }
    return DEFAULT_LANGUAGE;
  };

  const [currentLanguage, setCurrentLanguage] = useState(getInitialLanguage);

  // Update language when URL changes
  useEffect(() => {
    const handleUrlChange = () => {
      const path = window.location.pathname;
      if (path.startsWith('/ur/')) {
        setCurrentLanguage(LANGUAGES.UR);
      } else {
        setCurrentLanguage(DEFAULT_LANGUAGE);
      }
    };

    // Listen for URL changes (pushState, replaceState, and popstate events)
    const originalPushState = window.history.pushState;
    const originalReplaceState = window.history.replaceState;

    window.history.pushState = function (...args) {
      const result = originalPushState.apply(this, args);
      handleUrlChange();
      return result;
    };

    window.history.replaceState = function (...args) {
      const result = originalReplaceState.apply(this, args);
      handleUrlChange();
      return result;
    };

    window.addEventListener('popstate', handleUrlChange);

    // Initial check
    handleUrlChange();

    // Cleanup
    return () => {
      window.history.pushState = originalPushState;
      window.history.replaceState = originalReplaceState;
      window.removeEventListener('popstate', handleUrlChange);
    };
  }, []);

  const changeLanguage = (language) => {
    if (language !== currentLanguage) {
      let newUrl;
      const currentPath = window.location.pathname;

      if (language === LANGUAGES.UR) {
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

  // Get the language direction (LTR for all as per Urdu LTR requirement)
  const getDirection = () => {
    return 'ltr';
  };

  return (
    <LanguageContext.Provider
      value={{
        currentLanguage,
        changeLanguage,
        getDirection,
      }}
    >
      {children}
    </LanguageContext.Provider>
  );
};