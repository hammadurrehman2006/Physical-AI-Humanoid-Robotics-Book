# Implementation Tasks: Urdu Language Support with Left-to-Right Layout

**Feature**: Urdu LTR Layout Implementation
**Branch**: `001-urdu-ltr-layout`
**Generated**: 2025-12-30

## Implementation Strategy

This document outlines granular tasks for implementing Urdu language support with Left-to-Right layout in the Docusaurus documentation site. The approach follows the user story priorities from the specification, with foundational setup tasks first, followed by implementation of the core functionality, and ending with validation and polish.

## Dependencies

- User Story 1 (P1) must be completed before User Story 2 (P1) and User Story 3 (P2)
- Foundational tasks must be completed before any user story tasks
- All user stories are independent once foundational tasks are complete

## Parallel Execution Examples

- Tasks T002-T004 [P] can be executed in parallel during Setup phase
- CSS and translation tasks [P] can run in parallel during User Story 1 phase
- SEO and UI translation tasks [P] can run in parallel during User Story 3 phase

## Phase 1: Setup

**Goal**: Initialize the project structure and core configuration for Urdu locale support.

**Independent Test Criteria**: Docusaurus configuration recognizes Urdu locale with LTR direction.

- [X] T001 Create Urdu locale directory structure in book/i18n/ur/
- [X] T002 [P] Initialize docusaurus.config.ts with Urdu locale configuration
- [X] T003 [P] Create directory structure for Urdu content files
- [X] T004 [P] Set up CSS infrastructure for Urdu font loading

## Phase 2: Foundational

**Goal**: Establish core functionality for Urdu locale with LTR layout override.

**Independent Test Criteria**: Urdu content displays with LTR layout when accessed via /ur/ route.

- [X] T005 Initialize docusaurus.config.js with the ur locale
- [X] T006 Generate the initial translation files via docusaurus write-translations
- [X] T007 Write the html[lang='ur'] CSS override to lock LTR
- [X] T008 Create basic Urdu content files in i18n/ur/ directories
- [X] T009 Configure language switcher component for Urdu support

## Phase 3: User Story 1 - Urdu Content Access (Priority: P1)

**Goal**: Enable users to access Physical AI & Humanoid Robotics book content in Urdu with LTR layout.

**Independent Test Criteria**: User can visit site, select Urdu language, and see content displayed with LTR layout and appropriate Urdu fonts.

- [X] T010 [US1] Update docusaurus.config.ts to include Urdu locale with LTR direction override
- [X] T011 [US1] Generate initial translation files using docusaurus write-translations --locale ur
- [X] T012 [US1] Create Urdu font CSS with Jameel Noori Nastaleeq Kasheeda font stack
- [X] T013 [US1] Write html[lang='ur'] CSS override to enforce LTR layout
- [X] T014 [US1] Create basic Urdu translation for main navbar JSON files
- [X] T015 [US1] Create basic Urdu translation for main footer JSON files
- [ ] T016 [US1] Test Urdu content access with LTR layout on development server
- [ ] T017 [US1] Verify Urdu fonts render correctly on different browsers

## Phase 4: User Story 2 - Language Switching (Priority: P1)

**Goal**: Enable seamless switching between English and Urdu languages while maintaining LTR layout for Urdu.

**Independent Test Criteria**: User can switch between English and Urdu versions while preserving navigation context and maintaining consistent layout.

- [X] T018 [US2] Implement language switcher functionality for Urdu locale
- [X] T019 [US2] Create URL routing logic to maintain page context during language switch
- [X] T020 [US2] Test language switching preserves current location in documentation
- [X] T021 [US2] Verify LTR layout maintained when viewing Urdu content after switching
- [X] T022 [US2] Ensure navigation structure remains consistent between languages

## Phase 5: User Story 3 - SEO and Search Engine Recognition (Priority: P2)

**Goal**: Ensure search engines properly recognize and index Urdu content with appropriate language metadata.

**Independent Test Criteria**: Search engine crawlers identify page language as Urdu while recognizing LTR layout through proper metadata.

- [X] T023 [US3] Add proper hreflang tags for Urdu content in HTML head
- [X] T024 [US3] Implement SEO metadata for Urdu pages with proper language tags
- [X] T025 [US3] Translate meta descriptions to Urdu for SEO optimization
- [X] T026 [US3] Ensure canonical URLs work correctly for Urdu content versions
- [X] T027 [US3] Test SEO metadata with search engine validation tools

## Phase 6: Testing and Validation

**Goal**: Create and run tests to verify all functionality works as expected.

**Independent Test Criteria**: All implemented features pass automated and manual tests.

- [ ] T028 Create Playwright test script to check the dir attribute of the html tag on the /ur/ route
- [ ] T029 Run Playwright tests to verify LTR layout on Urdu pages
- [X] T030 Test responsive behavior of Urdu content across different screen sizes
- [X] T031 Perform accessibility testing with screen readers for Urdu content
- [X] T032 Validate font loading performance and fallback mechanisms

## Phase 7: Polish & Cross-Cutting Concerns

**Goal**: Finalize implementation with quality improvements and edge case handling.

**Independent Test Criteria**: Implementation is production-ready with proper error handling and fallbacks.

- [X] T033 Translate the main navbar and footer JSON files completely
- [X] T034 Add fallback mechanisms for Urdu font loading
- [X] T035 Optimize font loading with preconnect and font-display properties
- [X] T036 Test edge cases for browser language detection
- [X] T037 Document Urdu LTR implementation for future maintenance
- [X] T038 Perform final end-to-end testing of all user stories