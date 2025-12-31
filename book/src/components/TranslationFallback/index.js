import React from 'react';
import { useLanguage } from '../../context/LanguageContext';
import { LANGUAGES, getLanguageDisplayName } from '../../utils/translation-utils';
import { isTranslationAvailableForPage, getAvailableLanguagesForPage } from '../../utils/translation-availability';

/**
 * TranslationFallback Component
 * Handles cases when translation is not available for a page
 */
const TranslationFallback = ({ pagePath, children, fallbackMessage }) => {
  const { currentLanguage } = useLanguage();
  const isTranslationAvailable = isTranslationAvailableForPage(pagePath, currentLanguage);
  const availableLanguages = getAvailableLanguagesForPage(pagePath);

  // If translation is available in current language, show the content
  if (isTranslationAvailable) {
    return <>{children}</>;
  }

  // If translation is not available, show fallback content
  const fallbackText = fallbackMessage || (
    <div className="translation-fallback">
      <p>
        The content for this page is not yet available in {getLanguageDisplayName(currentLanguage)}.
        Please switch to a supported language: {availableLanguages.map(lang => getLanguageDisplayName(lang)).join(', ')}.
      </p>
      <p className="fallback-note">
        <small>
          {currentLanguage === LANGUAGES.UR
            ? 'یہ مواد ابھی تک اس زبان میں دستیاب نہیں ہے۔ براہ کرم دیگر زبان میں دیکھیں۔'
            : 'This content is not yet translated. Please check back later.'}
        </small>
      </p>
    </div>
  );

  return (
    <div className="translation-fallback-container">
      {fallbackText}
    </div>
  );
};

export default TranslationFallback;