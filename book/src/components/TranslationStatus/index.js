import React from 'react';
import { useLanguage } from '../../context/LanguageContext';
import { LANGUAGES, getLanguageDisplayName } from '../../utils/translation-utils';
import { getTranslationStatus } from '../../utils/translation-availability';

/**
 * TranslationStatus Component
 * Shows translation availability status for the current page
 */
const TranslationStatus = ({ pagePath }) => {
  const { currentLanguage } = useLanguage();
  const status = getTranslationStatus(pagePath);

  // Determine the status indicator based on translation availability
  const getStatusIndicator = () => {
    if (status.hasUrdu && status.hasEnglish) {
      return {
        status: 'complete',
        text: 'Both languages available',
        color: 'green',
        icon: '✅',
      };
    } else if (status.hasUrdu || status.hasEnglish) {
      return {
        status: 'partial',
        text: 'One language available',
        color: 'orange',
        icon: '⚠️',
      };
    } else {
      return {
        status: 'missing',
        text: 'No translations available',
        color: 'red',
        icon: '❌',
      };
    }
  };

  const indicator = getStatusIndicator();

  return (
    <div className={`translation-status translation-status-${indicator.status}`}>
      <div className="status-header">
        <span className="status-icon" aria-label={indicator.text}>
          {indicator.icon}
        </span>
        <span className="status-text">{indicator.text}</span>
      </div>
      <div className="status-details">
        <div className={`lang-status ${status.hasEnglish ? 'available' : 'missing'}`}>
          <span className={currentLanguage === LANGUAGES.EN ? 'current' : ''}>
            {getLanguageDisplayName(LANGUAGES.EN)} {status.hasEnglish ? '✓' : '✗'}
          </span>
        </div>
        <div className={`lang-status ${status.hasUrdu ? 'available' : 'missing'}`}>
          <span className={currentLanguage === LANGUAGES.UR ? 'current' : ''}>
            {getLanguageDisplayName(LANGUAGES.UR)} {status.hasUrdu ? '✓' : '✗'}
          </span>
        </div>
      </div>
    </div>
  );
};

export default TranslationStatus;