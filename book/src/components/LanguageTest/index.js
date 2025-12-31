import React from 'react';
import { useLanguage } from '../../context/LanguageContext';
import { LANGUAGES, getLanguageDisplayName, getLanguageFlag } from '../../utils/translation-utils';

/**
 * Test component to verify language switching functionality
 */
const LanguageTest = () => {
  const { currentLanguage, changeLanguage } = useLanguage();

  return (
    <div className="language-test-component">
      <h3>Language Switching Test</h3>
      <p>Current language: {getLanguageDisplayName(currentLanguage)} {getLanguageFlag(currentLanguage)}</p>
      <p id="test-text">
        {currentLanguage === LANGUAGES.EN
          ? 'This is a test to verify language switching functionality.'
          : 'یہ زبان تبدیل کرنے کی صلاحیت کو چیک کرنے کے لیے ایک ٹیسٹ ہے۔'}
      </p>
      <div className="language-buttons">
        <button
          onClick={() => changeLanguage(LANGUAGES.EN)}
          className={currentLanguage === LANGUAGES.EN ? 'active' : ''}
        >
          English
        </button>
        <button
          onClick={() => changeLanguage(LANGUAGES.UR)}
          className={currentLanguage === LANGUAGES.UR ? 'active' : ''}
        >
          اردو
        </button>
      </div>
    </div>
  );
};

export default LanguageTest;