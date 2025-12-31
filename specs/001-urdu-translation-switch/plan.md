# Implementation Plan: Urdu Translation Switch

**Branch**: `001-urdu-translation-switch` | **Date**: 2025-12-22 | **Spec**: [specs/001-urdu-translation-switch/spec.md](../specs/001-urdu-translation-switch/spec.md)
**Input**: Feature specification from `/specs/001-urdu-translation-switch/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Implementation of a language switching feature for the Physical AI & Humanoid Robotics book that allows readers to toggle between English and Urdu translations on any page. The solution will include a UI component for language switching, Google font integration for proper Urdu rendering (Jameel Noori Nastaleeq Kasheeda, Gulzar, or Noto Nastaliq Urdu), and a mechanism to store and retrieve translated content for each page. The implementation will follow Docusaurus best practices for internationalization while maintaining the existing content structure and user experience.

## Technical Context

**Language/Version**: JavaScript/TypeScript (Node.js 18+), React for Docusaurus framework, CSS for styling
**Primary Dependencies**: Docusaurus 3.x, React, Google Fonts API, React Context API for state management
**Storage**: Markdown files for content storage, localStorage for user preference persistence, Google Fonts CDN for font loading
**Testing**: Jest for unit testing, Playwright for end-to-end testing
**Target Platform**: Web-based documentation site (Docusaurus), compatible with modern browsers (Chrome, Firefox, Safari, Edge)
**Project Type**: Web application (static site generation with Docusaurus)
**Performance Goals**: Language switching within 2 seconds, page load times under 5 seconds with font resources, 100% of pages support language toggle
**Constraints**: Must maintain existing English content, support bidirectional text rendering, ensure accessibility, follow Docusaurus conventions
**Scale/Scope**: Multi-language support for entire book content, user preference persistence across sessions

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

Based on the constitution, the following gates apply:
1. **Learning Philosophy**: The translation feature supports hands-on mastery by making content accessible to Urdu-speaking learners
2. **Content Quality Standards**: The implementation must ensure technical accuracy and accessibility of translated content
3. **Technical Constraints**: Solution must work on standard consumer hardware and maintain Docusaurus performance
4. **Content Constraints**: Implementation must not interfere with the 4-6 hour chapter completion time

All gates pass as the feature enhances accessibility without compromising the core learning experience.

## Project Structure

### Documentation (this feature)

```text
specs/001-urdu-translation-switch/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
book/
├── src/
│   ├── components/
│   │   ├── LanguageSwitcher/        # React component for language toggle
│   │   ├── UrduTextRenderer/        # Component for proper Urdu text rendering
│   │   └── LayoutWrapper/           # Layout wrapper with language context
│   ├── context/
│   │   └── LanguageContext.js       # React context for language state management
│   ├── hooks/
│   │   └── useLanguage.js           # Custom hook for language switching
│   ├── css/
│   │   └── urdu-styles.css          # Styles for Urdu text rendering
│   └── utils/
│       └── translation-utils.js     # Utility functions for translation handling
├── static/
│   └── fonts/                       # Font loading utilities
├── i18n/
│   ├── en/                          # English translations
│   └── ur/                          # Urdu translations
├── docusaurus.config.js             # Updated Docusaurus config with i18n
└── sidebars.js                      # Updated sidebar with language support
```

**Structure Decision**: Web application structure selected since this is a Docusaurus-based documentation site that requires frontend components for language switching and text rendering. The implementation will follow Docusaurus internationalization patterns with React components for the language switcher and context management.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [N/A] | [N/A] | [N/A] |
