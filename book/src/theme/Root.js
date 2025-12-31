import React from 'react';
import { LanguageProvider } from '../context/LanguageContext';

// Root component that wraps the entire app
export default function Root({ children }) {
  return (
    <LanguageProvider>
      {children}
    </LanguageProvider>
  );
}