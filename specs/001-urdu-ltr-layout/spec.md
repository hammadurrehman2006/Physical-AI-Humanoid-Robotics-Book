# Feature Specification: Urdu Language Support with Left-to-Right Layout

**Feature Branch**: `001-urdu-ltr-layout`
**Created**: 2025-12-30
**Status**: Draft
**Input**: User description: "Act as the Content Architect and use Context7 to analyze our current Docusaurus directory structure and docusaurus.config.js. I need you to specify a technical requirement document for adding Urdu (ur) as a secondary language that strictly uses a Left-to-Right (LTR) layout. Clearly define the i18n configuration requirements, the specific file paths for localized docs in i18n/ur/, and the metadata rules that ensure search engines recognize the Urdu content without triggering automatic RTL browser behavior. Identify which UI strings in the navbar and footer must be localized and document the exact CSS selectors that will need overrides to lock the layout in LTR."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Urdu Content Access (Priority: P1)

Users who speak Urdu as their primary language should be able to access the Physical AI & Humanoid Robotics book content in Urdu while maintaining a left-to-right reading layout. The content should be properly displayed with appropriate Urdu fonts and styling without triggering automatic RTL browser behavior.

**Why this priority**: This is the core functionality that enables Urdu-speaking users to access the educational content in their preferred language while maintaining the layout consistency required for technical documentation.

**Independent Test**: Can be fully tested by accessing the Urdu version of the site and verifying that content displays correctly with LTR layout, appropriate fonts, and no RTL behavior.

**Acceptance Scenarios**:

1. **Given** a user visits the site, **When** they select Urdu language from the language switcher, **Then** they see all content in Urdu with left-to-right layout
2. **Given** a user is on an Urdu content page, **When** they navigate through different sections, **Then** the layout remains consistently left-to-right with proper Urdu typography

---

### User Story 2 - Language Switching (Priority: P1)

Users should be able to seamlessly switch between English and Urdu languages while maintaining their current location in the documentation, with proper LTR layout preserved when viewing Urdu content.

**Why this priority**: Essential for users who may need to switch between languages based on their comprehension level or preference for specific topics.

**Independent Test**: Can be tested by switching between English and Urdu versions and verifying that navigation and layout remain consistent.

**Acceptance Scenarios**:

1. **Given** a user is viewing English content, **When** they switch to Urdu, **Then** they see the same content in Urdu with LTR layout
2. **Given** a user is viewing Urdu content, **When** they switch back to English, **Then** they return to English content with normal layout

---

### User Story 3 - SEO and Search Engine Recognition (Priority: P2)

Search engines should properly recognize and index Urdu content with appropriate language metadata, while the LTR layout ensures proper rendering without triggering RTL behavior that might confuse the content interpretation.

**Why this priority**: Important for discoverability of the Urdu content by users searching in Urdu or looking for Urdu educational resources.

**Independent Test**: Can be tested by examining page metadata and using SEO tools to verify proper language tagging.

**Acceptance Scenarios**:

1. **Given** search engine crawlers access Urdu content, **When** they analyze the page metadata, **Then** they correctly identify the language as Urdu while recognizing the LTR layout

---

### Edge Cases

- What happens when users access the site with Urdu as their browser's default language?
- How does the system handle users with screen readers when using Urdu content with LTR layout?
- What occurs when users share links to Urdu content - is the language context preserved?

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST configure Urdu (ur) as a supported locale in Docusaurus i18n configuration with LTR direction override
- **FR-002**: System MUST ensure Urdu content displays with left-to-right layout despite Urdu being traditionally an RTL language
- **FR-003**: System MUST load appropriate Urdu fonts (e.g., Jameel Noori Nastaleeq Kasheeda) for proper text rendering
- **FR-004**: System MUST maintain proper HTML lang attributes for Urdu content (lang="ur")
- **FR-005**: System MUST override any automatic RTL behavior that might be triggered by Urdu content
- **FR-006**: System MUST ensure all UI elements maintain LTR layout when viewing Urdu content
- **FR-007**: System MUST provide proper metadata for search engines to recognize Urdu content
- **FR-008**: System MUST maintain all existing English content functionality unchanged
- **FR-009**: System MUST ensure navigation and layout consistency across all Urdu content pages

### i18n Configuration Requirements

- **FR-010**: System MUST set Urdu locale direction to 'ltr' in localeConfigs to override default RTL behavior
- **FR-011**: System MUST specify htmlLang as 'ur-PK' for Urdu locale to indicate proper language tag
- **FR-012**: System MUST ensure Urdu locale path is set to 'ur' in the configuration
- **FR-013**: System MUST provide proper label 'اردو' for the Urdu locale in the language switcher
- **FR-014**: System MUST ensure Urdu locale is included in the locales array alongside English

### File Path Requirements

- **FR-015**: System MUST store Urdu documentation files in `i18n/ur/docusaurus-plugin-content-docs/current/` directory
- **FR-016**: System MUST store Urdu blog content in `i18n/ur/docusaurus-plugin-content-blog/` directory
- **FR-017**: System MUST store Urdu theme content in `i18n/ur/docusaurus-theme-classic/` directory
- **FR-018**: System MUST store Urdu UI translations in `i18n/ur/code.json` file
- **FR-019**: System MUST ensure all existing Urdu content files are properly organized in the correct subdirectories

### Metadata and SEO Requirements

- **FR-020**: System MUST include proper `<link rel="alternate" hreflang="ur-PK" href="...">` tags for SEO
- **FR-021**: System MUST ensure HTML `<html lang="ur-PK">` attribute is set correctly for Urdu pages
- **FR-022**: System MUST include proper meta description in Urdu for search engine optimization
- **FR-023**: System MUST ensure no conflicting RTL direction attributes are present in page metadata
- **FR-024**: System MUST maintain proper canonical URLs for Urdu content versions

### UI Localization Requirements

- **FR-025**: System MUST translate navbar items (Start Reading, Introduction, Modules) to Urdu equivalents
- **FR-026**: System MUST translate footer section titles (Book Content, Resources, More) to Urdu
- **FR-027**: System MUST translate footer links to appropriate Urdu equivalents where applicable
- **FR-028**: System MUST maintain proper navigation structure with translated labels in Urdu
- **FR-029**: System MUST ensure language switcher displays both English and Urdu options with proper labels

### CSS Override Requirements

- **FR-030**: System MUST override any CSS direction: rtl rules that might be automatically applied to Urdu content
- **FR-031**: System MUST ensure body and container elements maintain direction: ltr for Urdu pages
- **FR-032**: System MUST override sidebar and navigation direction properties to maintain LTR layout
- **FR-033**: System MUST ensure text alignment remains left-aligned for Urdu content despite language
- **FR-034**: System MUST maintain consistent grid and flexbox layouts in LTR mode for Urdu pages

### Key Entities *(include if feature involves data)*

- **Urdu Locale Configuration**: Settings that define the Urdu language locale with LTR override in Docusaurus
- **Urdu Content Files**: Localized documentation files in the i18n/ur/ directory structure
- **Language Switcher**: Component that allows users to toggle between English and Urdu content

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Users can access Urdu content with consistent LTR layout across all pages without RTL behavior being triggered
- **SC-002**: Search engines correctly identify page language as Urdu while recognizing the LTR layout through proper metadata
- **SC-003**: 95% of Urdu content pages render properly with appropriate Urdu fonts and LTR layout
- **SC-004**: Language switching between English and Urdu works seamlessly with preserved navigation context
