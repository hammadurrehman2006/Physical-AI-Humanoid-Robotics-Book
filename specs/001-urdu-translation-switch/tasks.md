---
description: "Task list for Urdu translation switch feature implementation"
---

# Tasks: Urdu Translation Switch

**Input**: Design documents from `/specs/001-urdu-translation-switch/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: The examples below include test tasks. Tests are OPTIONAL - only include them if explicitly requested in the feature specification.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Docusaurus project**: `book/src/`, `book/static/`, `book/i18n/`
- Paths shown below assume Docusaurus project structure - adjust based on plan.md structure

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [x] T001 Create i18n directory structure in book/ for English and Urdu translations
- [x] T002 [P] Create source directory structure per implementation plan in book/src/
- [x] T003 [P] Update package.json with any required dependencies for i18n

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [x] T004 Setup Docusaurus i18n configuration in docusaurus.config.js for English and Urdu
- [x] T005 Create LanguageContext for managing language state in book/src/context/LanguageContext.js
- [x] T006 Create custom hook useLanguage in book/src/hooks/useLanguage.js
- [x] T007 Create utility functions for translation handling in book/src/utils/translation-utils.js
- [x] T008 Configure Google Fonts CSS import for Urdu fonts in book/src/css/urdu-styles.css

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Language Switching (Priority: P1) 🎯 MVP

**Goal**: Enable users to switch between English and Urdu translations on any page with proper font rendering

**Independent Test**: Verify that a language switcher appears on each page and successfully toggles content between English and Urdu with proper font rendering

### Implementation for User Story 1

- [x] T009 Create LanguageSwitcher React component in book/src/components/LanguageSwitcher/index.js
- [x] T010 Integrate LanguageProvider with main layout in book/src/theme/Layout/index.js
- [x] T011 Add language switcher UI styling in book/src/css/urdu-styles.css
- [x] T012 Implement language preference persistence using localStorage
- [x] T013 Add visual indicators for current language selection
- [x] T014 Test language switching functionality on sample pages

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Font Rendering for Urdu Content (Priority: P1)

**Goal**: Display Urdu translation in appropriate Google fonts like Jameel Noori Nastaleeq Kasheeda, Gulzar, or Noto Nastaliq Urdu

**Independent Test**: Verify that Urdu text appears in the specified Google fonts with proper character rendering and styling

### Implementation for User Story 2

- [x] T015 Implement RTL (right-to-left) text direction for Urdu content
- [x] T016 Configure font loading strategy for Urdu fonts with async loading
- [x] T017 Create UrduTextRenderer component for proper text rendering in book/src/components/UrduTextRenderer/index.js
- [x] T018 Add bidirectional text handling for Urdu content
- [x] T019 Test font rendering and character display for Urdu text
- [x] T020 Validate font fallback mechanism works properly

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - Translation Availability (Priority: P2)

**Goal**: Ensure translations exist clearly for each page so users can switch languages without encountering missing or incomplete translations

**Independent Test**: Verify that each page has complete translations available in both languages without content gaps

### Implementation for User Story 3

- [x] T021 Create mechanism to check translation availability for each page
- [x] T022 Implement fallback handling when translation is not available
- [x] T023 Add translated content for sample pages in book/i18n/ur/
- [x] T024 Create translation status indicators in the UI
- [x] T025 Test navigation between pages with different translation availability
- [x] T026 Validate that code examples and technical content remain readable in both languages

**Checkpoint**: All user stories should now be independently functional

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [x] T027 [P] Update sidebars.js to support language-specific navigation
- [x] T028 [P] Add documentation updates in specs/001-urdu-translation-switch/
- [x] T029 Code cleanup and refactoring
- [x] T030 Performance optimization for font loading and language switching
- [x] T031 Accessibility improvements for translated content
- [x] T032 Run quickstart.md validation to ensure all steps work as expected
- [x] T033 Update docusaurus.config.js with final i18n settings
- [x] T034 Test complete user flow across all pages

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3+)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3)
- **Polish (Final Phase)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P1)**: Can start after Foundational (Phase 2) - May integrate with US1 but should be independently testable
- **User Story 3 (P2)**: Can start after Foundational (Phase 2) - May integrate with US1/US2 but should be independently testable

### Within Each User Story

- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, all user stories can start in parallel (if team capacity allows)
- Different user stories can be worked on in parallel by different team members

---

## Parallel Example: User Story 1

```bash
# Launch all components for User Story 1 together:
Task: "Create LanguageSwitcher React component in book/src/components/LanguageSwitcher/index.js"
Task: "Integrate LanguageProvider with main layout in book/src/theme/Layout/index.js"
Task: "Add language switcher UI styling in book/src/css/urdu-styles.css"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1
4. **STOP and VALIDATE**: Test User Story 1 independently
5. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 → Test independently → Deploy/Demo
4. Add User Story 3 → Test independently → Deploy/Demo
5. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1
   - Developer B: User Story 2
   - Developer C: User Story 3
3. Stories complete and integrate independently

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Verify tests fail before implementing
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence