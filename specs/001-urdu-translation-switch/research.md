# Research: Urdu Translation Switch Implementation

## Decision: Docusaurus Internationalization Approach
**Rationale**: Using Docusaurus' built-in i18n capabilities combined with custom React components for more flexible language switching. This approach maintains compatibility with existing Docusaurus features while providing the specific functionality needed for Urdu text rendering.

**Alternatives considered**:
1. Pure React Context API without Docusaurus i18n - Would require custom routing and content management
2. Third-party i18n libraries (i18next) - Would add complexity and potential conflicts with Docusaurus
3. Server-side language detection - Would require backend changes and not meet the requirement for user-controlled switching

## Decision: Google Font Integration Method
**Rationale**: Using Google Fonts CSS API with asynchronous loading to prevent blocking page rendering. This ensures Urdu text renders properly with Jameel Noori Nastaleeq Kasheeda, Gulzar or Noto Nastaliq Urdu fonts while maintaining performance.

**Alternatives considered**:
1. Preloading all fonts at site level - Would increase initial load time unnecessarily
2. Dynamic font loading only when Urdu is selected - May cause flickering when switching
3. Self-hosted font files - Would increase bundle size and maintenance overhead

## Decision: Bidirectional Text Handling
**Rationale**: Implementing proper RTL (right-to-left) support using CSS `direction: rtl` property and Docusaurus' RTL capabilities. This ensures proper rendering of Urdu content while maintaining layout integrity.

**Alternatives considered**:
1. Custom RTL implementation - Would be error-prone and non-standard
2. Ignoring RTL requirements - Would result in unreadable Urdu text
3. Using third-party RTL libraries - Not necessary as CSS provides sufficient capabilities

## Decision: Translation Content Storage
**Rationale**: Using Docusaurus' i18n directory structure with separate locale directories for English and Urdu content. This follows Docusaurus conventions and makes it easy to manage translations.

**Alternatives considered**:
1. JSON-based translation files - Would require custom content processing
2. Database storage - Not appropriate for static site generator
3. Single file with both languages - Would complicate content management

## Decision: Language Preference Persistence
**Rationale**: Using localStorage to save user's language preference with fallback to browser language detection. This ensures the preference persists across sessions while providing a sensible default.

**Alternatives considered**:
1. URL parameters - Would not persist across visits
2. Cookies - More complex than necessary for this use case
3. Session storage - Would reset on each browser session

## Technical Unknowns Resolved

### 1. Google Fonts for Urdu
- **Research**: Jameel Noori Nastaleeq Kasheeda, Gulzar, and Noto Nastaliq Urdu fonts are available via Google Fonts
- **Implementation**: Use CSS import or JavaScript API for asynchronous loading
- **Fallback**: Noto Naskh Arabic or system fonts for compatibility

### 2. Docusaurus i18n Integration
- **Research**: Docusaurus has built-in i18n support with locale directories
- **Implementation**: Create i18n/en/ and i18n/ur/ directories with translated content
- **Routing**: Docusaurus will handle locale-specific URLs automatically

### 3. React Component Structure
- **Research**: Need custom components for language switching and text rendering
- **Implementation**: Create LanguageSwitcher component and context for state management
- **Integration**: Components will work with Docusaurus' layout system

### 4. Performance Considerations
- **Research**: Font loading can impact performance if not handled properly
- **Implementation**: Use font-display: swap and preconnect for Google Fonts
- **Optimization**: Lazy-load Urdu fonts only when needed (optional enhancement)

## Architecture Patterns

### Frontend Architecture
- **Component-based**: React components for language switching and text rendering
- **Context-based**: React Context API for language state management
- **CSS-based**: Proper styling for RTL and font handling

### Content Architecture
- **File-based**: Separate locale directories following Docusaurus i18n patterns
- **Version-controlled**: All translations in Git alongside code
- **Structured**: Consistent format for both English and Urdu content

### Performance Architecture
- **Asynchronous loading**: Fonts and language content loaded without blocking
- **Caching**: Browser caching for translated content and fonts
- **Optimization**: Minimal impact on initial page load times