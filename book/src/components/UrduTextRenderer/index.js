import React from 'react';
import { useLanguage } from '../../context/LanguageContext';
import { LANGUAGES, getLanguageFontFamily } from '../../utils/translation-utils';
import '../../css/urdu-styles.css';

/**
 * UrduTextRenderer Component
 * Renders text content with proper Urdu font and styling with enhanced accessibility
 */
const UrduTextRenderer = ({ children, className = '', style = {}, as: Component = 'span', ...props }) => {
  const { currentLanguage } = useLanguage();

  const isUrdu = currentLanguage === LANGUAGES.UR;
  const baseClassName = `urdu-text-renderer ${isUrdu ? 'urdu-text' : ''} ${className}`.trim();

  const baseStyle = {
    fontFamily: getLanguageFontFamily(currentLanguage),
    ...style
  };

  // Add appropriate ARIA attributes for accessibility
  const accessibilityProps = {
    dir: 'ltr', // Always use LTR layout for Urdu content as per specification
    lang: currentLanguage,
    'aria-label': isUrdu ? `Urdu text: ${typeof children === 'string' ? children : ''}` : undefined,
    ...props
  };

  return (
    <Component
      className={baseClassName}
      style={baseStyle}
      {...accessibilityProps}
    >
      {children}
    </Component>
  );
};

export default UrduTextRenderer;