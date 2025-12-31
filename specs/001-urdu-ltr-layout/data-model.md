# Data Model: Urdu Language Support with LTR Layout

## Overview
This document defines the data structures and entities required for implementing Urdu language support with Left-to-Right layout in the Docusaurus documentation site.

## Key Entities

### Urdu Locale Configuration
- **Name**: UrduLocaleConfig
- **Fields**:
  - locale: string (value: "ur")
  - label: string (value: "اردو")
  - direction: string (value: "ltr")
  - htmlLang: string (value: "ur-PK")
  - path: string (value: "ur")
  - calendar: string (optional, default: "gregory")
  - translate: boolean (optional, default: true)
- **Relationships**: Extends base LocaleConfig interface from Docusaurus
- **Validation**: direction must be "ltr" to override default RTL behavior

### Urdu Content File Structure
- **Name**: UrduContentFiles
- **Fields**:
  - docsPath: string (i18n/ur/docusaurus-plugin-content-docs/current/)
  - blogPath: string (i18n/ur/docusaurus-plugin-content-blog/)
  - themePath: string (i18n/ur/docusaurus-theme-classic/)
  - codeJsonPath: string (i18n/ur/code.json)
- **Relationships**: Maps to Docusaurus i18n directory structure
- **Validation**: All paths must exist and be properly structured

### Language Switcher State
- **Name**: LanguageSwitcherState
- **Fields**:
  - currentLanguage: string (enum: "en", "ur")
  - availableLanguages: array of strings
  - languageDirection: string (enum: "ltr", "rtl")
- **Relationships**: Connected to Docusaurus i18n context
- **State transitions**: Language change triggers URL redirect to locale path

### Urdu Font Configuration
- **Name**: UrduFontConfig
- **Fields**:
  - fontFamily: string (primary Urdu font stack)
  - fallbackFonts: array of strings (fallback font options)
  - fontDisplay: string (CSS font-display property)
  - preconnectUrls: array of strings (font loading optimization)
- **Relationships**: Used by CSS styling system
- **Validation**: Font stack must support Urdu characters

## API Contracts

### Locale Configuration API
- **Endpoint**: docusaurus.config.ts i18n configuration
- **Method**: Configuration object (not HTTP)
- **Input**: LocaleConfig object with Urdu-specific settings
- **Output**: Docusaurus i18n system with Urdu support
- **Error handling**: Invalid configuration causes build failure

### Content Translation API
- **Endpoint**: i18n/ur/ directory structure
- **Method**: File-based content delivery
- **Input**: Markdown files with Urdu content
- **Output**: Rendered Urdu documentation pages
- **Error handling**: Missing translations fall back to English

### Language Switching API
- **Endpoint**: /ur/* routes
- **Method**: GET
- **Input**: Locale path parameter
- **Output**: Urdu content with LTR layout
- **Error handling**: Invalid locale paths redirect to default

## Validation Rules

### From Requirements
- FR-010: Urdu locale direction must be 'ltr' to override default RTL behavior
- FR-011: htmlLang must be 'ur-PK' for proper language tagging
- FR-015-FR-019: Urdu content files must be properly organized in i18n/ur/ directory
- FR-020-FR-024: Proper metadata must be included for SEO
- FR-030-FR-034: CSS direction must remain LTR for Urdu content

### State Validation
- Language switcher must maintain consistent LTR layout when Urdu is selected
- Navigation structure must remain identical between English and Urdu versions
- Font loading must not block page rendering
- SEO metadata must accurately reflect language and direction

## State Transitions

### Language Switching Flow
1. User selects Urdu from language switcher
2. System detects language change request
3. System redirects to /ur/ prefixed path for current page
4. Docusaurus serves Urdu content with LTR layout
5. Language switcher updates to reflect current language
6. CSS applies LTR overrides for layout consistency

### Content Loading Flow
1. Page request made to /ur/ path
2. Docusaurus loads Urdu content from i18n/ur/ directory
3. CSS applies LTR layout overrides
4. Urdu fonts are loaded and applied
5. Content renders with proper typography and layout