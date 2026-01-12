# Tasks: User Authentication and Profile Management

**Input**: Design documents from `/specs/001-auth-user-management/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: Tests are NOT explicitly requested in the specification, so test tasks are NOT included. Focus is on implementation only.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Web app**: `book/src/`, `book/plugins/`, `book/tests/`
- Paths follow Docusaurus project structure with authentication module in `book/src/auth/`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [ ] T001 Create NeonDB project at https://console.neon.tech/ and obtain connection string
- [ ] T002 Create environment file book/.env.local with DATABASE_URL, BETTER_AUTH_SECRET, BETTER_AUTH_URL
- [ ] T003 [P] Install auth dependencies: npm install better-auth @neondatabase/serverless drizzle-orm ws zod
- [ ] T004 [P] Install dev dependencies: npm install -D drizzle-kit @types/ws
- [ ] T005 [P] Create directory structure: book/src/auth/, book/src/auth/db/, book/src/auth/validators/, book/src/auth/utils/, book/src/auth/hooks/
- [ ] T006 [P] Create directory structure: book/src/components/auth/, book/src/pages/api/auth/, book/plugins/auth-plugin/

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [ ] T007 Create Drizzle configuration file book/drizzle.config.ts with schema and migration paths
- [ ] T008 Create database schema with Drizzle in book/src/auth/db/schema.ts (users table with id, email, password_hash, software_background enum, hardware_background enum, created_at, last_login_at)
- [ ] T009 Create database schema in book/src/auth/db/schema.ts (sessions table with id, user_id FK, token_hash, expires_at, created_at, ip_address, user_agent)
- [ ] T010 Generate initial database migration: npx drizzle-kit generate
- [ ] T011 Push migration to NeonDB: npx drizzle-kit push
- [ ] T012 Create NeonDB connection with Pool in book/src/auth/db/client.ts (import Pool from @neondatabase/serverless, configure WebSocket, create drizzle instance)
- [ ] T013 Create better-auth server instance in book/src/auth/auth.ts (configure baseURL, secret, drizzle adapter, emailAndPassword with minPasswordLength 8, custom user fields software_background and hardware_background, session expiresIn 7 days, rateLimit 5 attempts per 15 min)
- [ ] T014 [P] Create Zod validation schemas in book/src/auth/validators/schemas.ts (emailSchema, passwordSchema with uppercase/lowercase/number regex, softwareBackgroundSchema enum, hardwareBackgroundSchema enum, signupSchema, signinSchema)
- [ ] T015 [P] Create authentication error types in book/src/auth/utils/errors.ts (AuthError interface with code/message/field, AUTH_ERRORS object with INVALID_CREDENTIALS, DUPLICATE_EMAIL, WEAK_PASSWORD, SESSION_EXPIRED, RATE_LIMIT_EXCEEDED)
- [ ] T016 Create Docusaurus auth plugin in book/plugins/auth-plugin/index.js (register /api/auth/* routes, mount better-auth handlers)
- [ ] T017 Register auth plugin in book/docusaurus.config.js plugins array
- [ ] T018 Create API handler in book/src/pages/api/auth/[[...all]].tsx (import auth from src/auth/auth.ts, export auth.handler function)
- [ ] T019 Create better-auth React client in book/src/auth/client.ts (createAuthClient with baseURL, inferAdditionalFields plugin for software_background and hardware_background, export signIn/signUp/signOut/useSession)
- [ ] T020 Create AuthProvider context in book/src/auth/AuthProvider.tsx (wrap useSession hook, provide isAuthenticated boolean, provide error state)
- [ ] T021 Create useAuth hook in book/src/auth/hooks/useAuth.ts (useContext wrapper with error if used outside AuthProvider)
- [ ] T022 Swizzle Docusaurus Root component: npm run swizzle @docusaurus/theme-classic Root -- --wrap
- [ ] T023 Wrap Root component with AuthProvider in book/src/theme/Root.tsx (import AuthProvider, wrap children)

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - New User Registration (Priority: P1) 🎯 MVP

**Goal**: Enable new visitors to create accounts with email, password, and background information, automatically logging them in upon successful registration

**Independent Test**: Navigate to /signup page, complete form with valid email (test@example.com), password (SecurePass123), select background options, submit, verify account created in database, user automatically logged in, session cookie set

### Implementation for User Story 1

- [ ] T024 [P] [US1] Create SignUpForm component in book/src/components/auth/SignUpForm.tsx (form with email input, password input, confirm password input, software_background select dropdown with Beginner/Intermediate/Advanced/Expert options, hardware_background select dropdown with No Experience/Basic Knowledge/Hands-on Experience/Professional Experience options)
- [ ] T025 [P] [US1] Add form validation to SignUpForm (validate email format, password min 8 chars with uppercase/lowercase/number, confirm password matches, display error messages with role="alert" aria-live="polite")
- [ ] T026 [P] [US1] Add loading state to SignUpForm (disable form during submission, show "Creating account..." button text, use aria-busy attribute)
- [ ] T027 [P] [US1] Add password strength indicator to SignUpForm (visual feedback showing weak/medium/strong based on length and complexity)
- [ ] T028 [P] [US1] Style SignUpForm component (match Docusaurus theme variables, mobile-responsive 320px+, focus indicators, proper spacing)
- [ ] T029 [US1] Integrate SignUpForm with better-auth client (call signUp from useAuth hook with email, password, software_background, hardware_background, handle success with redirect, handle errors with display)
- [ ] T030 [US1] Create signup page in book/src/pages/signup.tsx (import SignUpForm, add page title "Create Account", add "Already have an account? Sign in" link to /signin)
- [ ] T031 [US1] Add duplicate email error handling in SignUpForm (catch AUTH002 error code, display "An account with this email already exists", show link to signin page)
- [ ] T032 [US1] Add ARIA labels to all SignUpForm inputs (aria-label for email "Email address", password "Password", confirm password "Confirm password", software_background "Software experience level", hardware_background "Hardware experience level")
- [ ] T033 [US1] Add keyboard navigation support to SignUpForm (Tab through all fields, Enter to submit, Escape to clear errors)
- [ ] T034 [US1] Test SignUpForm validation (test empty email, invalid email format, password too short, password missing complexity, mismatched confirm password, verify error messages display correctly)
- [ ] T035 [US1] Test SignUpForm successful submission (create test account, verify redirect to homepage, verify session cookie set, verify authenticated UI state)
- [ ] T036 [US1] Test SignUpForm duplicate email scenario (attempt to register with existing email, verify error message displays, verify form not submitted)

**Checkpoint**: At this point, User Story 1 should be fully functional - users can register and are automatically authenticated

---

## Phase 4: User Story 2 - Existing User Sign In (Priority: P1)

**Goal**: Enable returning users to authenticate with email and password, maintaining session across browser restarts for 7 days

**Independent Test**: Create test user account first (via signup), navigate to /signin page, enter correct email and password, submit, verify user authenticated, verify session persists after browser restart

### Implementation for User Story 2

- [ ] T037 [P] [US2] Create SignInForm component in book/src/components/auth/SignInForm.tsx (form with email input type="email" autoComplete="email", password input type="password" autoComplete="current-password", submit button)
- [ ] T038 [P] [US2] Add form validation to SignInForm (validate email format, password not empty, display error messages with role="alert")
- [ ] T039 [P] [US2] Add loading state to SignInForm (disable form during submission, show "Signing in..." button text, use aria-busy attribute)
- [ ] T040 [P] [US2] Style SignInForm component (match Docusaurus theme variables, mobile-responsive, focus indicators)
- [ ] T041 [US2] Integrate SignInForm with better-auth client (call signIn from useAuth hook with email and password, handle success with redirect to returnTo URL or homepage, handle errors with generic message "Invalid email or password")
- [ ] T042 [US2] Create signin page in book/src/pages/signin.tsx (import SignInForm, add page title "Sign In", add "Don't have an account? Sign up" link to /signup, handle returnTo query parameter for redirect after login)
- [ ] T043 [US2] Add generic error handling to SignInForm (for both wrong password and non-existent email, display same message "Invalid email or password" to prevent account enumeration)
- [ ] T044 [US2] Add ARIA labels to SignInForm inputs (aria-label for email "Email address", password "Password")
- [ ] T045 [US2] Add keyboard navigation support to SignInForm (Tab through fields, Enter to submit)
- [ ] T046 [US2] Test SignInForm with correct credentials (signin with valid account, verify authentication success, verify redirect to homepage or returnTo URL)
- [ ] T047 [US2] Test SignInForm with incorrect password (signin with wrong password, verify generic error message "Invalid email or password", verify no authentication)
- [ ] T048 [US2] Test SignInForm with non-existent email (signin with email not in database, verify same generic error message, verify no authentication)
- [ ] T049 [US2] Test session persistence (signin, close browser, reopen within 7 days, verify still authenticated without re-signin)
- [ ] T050 [US2] Test rate limiting (attempt 6 signin attempts in 15 minutes, verify 6th attempt blocked with RATE_LIMIT_EXCEEDED error, verify error message "Too many attempts. Please try again in 15 minutes.")

**Checkpoint**: At this point, User Stories 1 AND 2 should both work - users can register or signin

---

## Phase 5: User Story 3 - Access Protected Features (Priority: P1)

**Goal**: Restrict Urdu translation feature to authenticated users only, showing disabled state for unauthenticated users with tooltip

**Independent Test**: Test as authenticated user (signin first), navigate to any docs page, click Urdu toggle, verify content switches to Urdu. Test as unauthenticated user, verify Urdu toggle disabled with tooltip "Sign in to access Urdu translation"

### Implementation for User Story 3

- [ ] T051 [P] [US3] Create ProtectedRoute component in book/src/components/auth/ProtectedRoute.tsx (check isAuthenticated from useAuth, if false redirect to /signin with returnTo param, if loading show spinner, if authenticated render children)
- [ ] T052 [P] [US3] Enhance UrduTranslationToggle component in book/src/components/UrduTranslationToggle.tsx (check isAuthenticated from useAuth hook, if false show disabled button with Tooltip content="Sign in to access Urdu translation", if true show functional toggle with GlobeIcon and اردو/English text)
- [ ] T053 [P] [US3] Add locale persistence to UrduTranslationToggle (use useDocusaurusContext to get currentLocale and setLocale, persist preference in localStorage, restore on page load)
- [ ] T054 [P] [US3] Style UrduTranslationToggle (use .urdu-toggle and .urdu-toggle-disabled classes, ensure proper RTL layout for Urdu text, use Jameel Noori Nastaleeq font from Google Fonts)
- [ ] T055 [US3] Add Urdu translation toggle to Docusaurus navbar (swizzle Navbar component, integrate UrduTranslationToggle component, position near language selector or search bar)
- [ ] T056 [US3] Add session expiration handling (when session expires while viewing Urdu content, redirect to /signin with message "Your session has expired. Please sign in to continue.", preserve returnTo URL with current page)
- [ ] T057 [US3] Test protected route access as authenticated user (signin, navigate to docs page, click Urdu toggle, verify content switches to Urdu, verify font renders correctly)
- [ ] T058 [US3] Test protected route access as unauthenticated user (without signin, navigate to docs page, verify Urdu toggle disabled, verify tooltip displays "Sign in to access Urdu translation")
- [ ] T059 [US3] Test locale persistence (signin, enable Urdu, navigate between pages, verify Urdu persists, close browser, reopen, verify Urdu still enabled)
- [ ] T060 [US3] Test session expiration redirect (signin, enable Urdu, manually expire session in database, interact with page, verify redirect to /signin with expiration message and returnTo URL)

**Checkpoint**: At this point, all P1 user stories are complete - MVP is fully functional with signup, signin, and protected Urdu translation

---

## Phase 6: User Story 4 - Session Management and Sign Out (Priority: P2)

**Goal**: Allow authenticated users to securely terminate their session, clearing cookies and invalidating session in database

**Independent Test**: Signin as user, verify authenticated state in navbar, click "Sign Out" button in user menu, verify redirect to homepage, verify unauthenticated state, verify session cookie cleared, verify cannot access Urdu toggle

### Implementation for User Story 4

- [ ] T061 [P] [US4] Create UserMenu component in book/src/components/auth/UserMenu.tsx (dropdown trigger button with user email, isOpen state, dropdown menu with "Profile" link and "Sign Out" button)
- [ ] T062 [P] [US4] Add dropdown interaction to UserMenu (toggle isOpen on button click, close on outside click with useEffect and document.addEventListener, close on Escape key, add aria-expanded and aria-haspopup attributes)
- [ ] T063 [P] [US4] Add Sign Out functionality to UserMenu (onClick call signOut from useAuth hook, clear session cookie, redirect to homepage, update UI to unauthenticated state)
- [ ] T064 [P] [US4] Style UserMenu component (position dropdown below trigger, z-index to appear above content, box-shadow for elevation, match Docusaurus theme)
- [ ] T065 [US4] Create AuthNavbarItems component in book/src/components/auth/AuthNavbarItems.tsx (if isLoading show skeleton, if isAuthenticated show UserMenu with user data, if not authenticated show "Sign In" and "Sign Up" buttons)
- [ ] T066 [US4] Integrate AuthNavbarItems into Docusaurus navbar (swizzle Navbar/Content component in book/src/theme/Navbar/Content/index.tsx, import AuthNavbarItems, render in appropriate position)
- [ ] T067 [US4] Add keyboard navigation to UserMenu (Tab to focus trigger, Enter/Space to toggle, Tab through menu items, Enter to activate, Escape to close)
- [ ] T068 [US4] Add ARIA attributes to UserMenu (aria-label="User menu" on trigger, role="menu" on dropdown, role="menuitem" on links/buttons)
- [ ] T069 [US4] Test sign out functionality (signin, verify authenticated navbar, click Sign Out, verify redirect to homepage, verify unauthenticated navbar with Sign In/Sign Up buttons)
- [ ] T070 [US4] Test sign out session invalidation (signin, sign out, verify session cookie cleared, verify cannot access Urdu toggle, verify protected routes redirect to signin)
- [ ] T071 [US4] Test sign out multi-tab behavior (signin, open multiple tabs, sign out in one tab, interact with other tabs, verify they reflect signed-out state)

**Checkpoint**: Session management complete - users can sign in and sign out securely

---

## Phase 7: User Story 5 - Profile Information Display (Priority: P3)

**Goal**: Display user profile information including email and background details on a dedicated profile page

**Independent Test**: Signin as user, navigate to /profile page, verify email address displayed, verify software_background displayed (or "Not specified" if null), verify hardware_background displayed (or "Not specified" if null)

### Implementation for User Story 5

- [ ] T072 [P] [US5] Create profile page in book/src/pages/profile.tsx (ProtectedRoute wrapper, display user data from useAuth hook, sections for "Account Information" with email/created_at/last_login_at and "Background" with software_background/hardware_background)
- [ ] T073 [P] [US5] Add profile link to UserMenu (add "Profile" link to /profile in dropdown menu before Sign Out button)
- [ ] T074 [P] [US5] Style profile page (card layout with sections, definition lists <dl> for key-value pairs, responsive layout, match Docusaurus theme)
- [ ] T075 [P] [US5] Add null handling for background fields (if software_background is null display "Not specified", same for hardware_background)
- [ ] T076 [P] [US5] Add date formatting for profile timestamps (format created_at and last_login_at with toLocaleDateString(), display "N/A" if last_login_at is null)
- [ ] T077 [US5] Add profile page title and breadcrumb (h1 "Profile", add to Docusaurus navigation if needed)
- [ ] T078 [US5] Test profile page with complete user data (signin with user who has both background fields, navigate to /profile, verify all fields display correctly)
- [ ] T079 [US5] Test profile page with null background fields (create user without background info, signin, navigate to /profile, verify "Not specified" displays for null fields)
- [ ] T080 [US5] Test profile page access control (attempt to access /profile without authentication, verify redirect to /signin with returnTo=/profile)

**Checkpoint**: All user stories complete - full feature set implemented

---

## Phase 8: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [ ] T081 [P] Add mobile responsive styles to all auth components (test on 320px, 768px, 1024px viewports, ensure forms usable on mobile, ensure navbar auth UI works on mobile)
- [ ] T082 [P] Add loading skeleton to AuthNavbarItems (show placeholder while session loading, prevent layout shift)
- [ ] T083 [P] Add error boundary for auth components (wrap AuthProvider with error boundary, display user-friendly error message if auth system fails, provide retry button)
- [ ] T084 [P] Add accessibility improvements (run axe-core audit, fix any WCAG 2.1 AA violations, test with keyboard only, test with screen reader)
- [ ] T085 [P] Add focus indicators to all interactive auth elements (visible focus ring on all buttons, inputs, links, 3px solid with primary color)
- [ ] T086 [P] Optimize bundle size for auth module (code-split auth routes, lazy load SignUpForm and SignInForm, ensure auth module <50KB)
- [ ] T087 [P] Add security headers to production config (Content-Security-Policy, X-Frame-Options, X-Content-Type-Options in docusaurus.config.js or server config)
- [ ] T088 [P] Add input sanitization for user data display (HTML entity encoding for email and background values when displaying in profile page)
- [ ] T089 [P] Create database cleanup job for expired sessions (script to delete sessions where expires_at < NOW(), schedule to run daily)
- [ ] T090 [P] Verify HTTPS configuration for production (ensure BETTER_AUTH_URL uses https://, ensure cookies have Secure flag in production)
- [ ] T091 Verify quickstart.md instructions (follow quickstart.md step-by-step in fresh environment, verify all steps work, update any incorrect paths or commands)
- [ ] T092 Update docum entation (update README with auth feature description, document environment variables needed, add screenshots of signup/signin/profile pages)
- [ ] T093 Add performance monitoring (log session validation latency, track authentication errors, monitor rate limiting triggers)

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-7)**: All depend on Foundational phase completion
  - User Story 1 (P1): Can start after Foundational - No dependencies on other stories
  - User Story 2 (P1): Can start after Foundational - Independent from US1 (but integrates in navbar)
  - User Story 3 (P1): Depends on US1 and US2 (needs authentication to work) - Can start after US1+US2
  - User Story 4 (P2): Can start after US2 (needs signin to test signout) - Independent from US3 and US5
  - User Story 5 (P3): Can start after US2 (needs authentication) - Independent from US3 and US4
- **Polish (Phase 8)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P1)**: Can start after Foundational (Phase 2) - Independent from US1
- **User Story 3 (P1)**: Depends on US1 and US2 (needs working authentication)
- **User Story 4 (P2)**: Depends on US2 (needs signin to test signout)
- **User Story 5 (P3)**: Depends on US2 (needs authentication)

### Within Each User Story

- Component creation before integration
- Form validation before submission logic
- Styling before testing
- Basic functionality before accessibility enhancements
- Story complete before moving to next priority

### Parallel Opportunities

- **Phase 1 (Setup)**: T003, T004, T005, T006 can all run in parallel
- **Phase 2 (Foundational)**: T014, T015 can run in parallel with T007-T013
- **User Story 1**: T024, T025, T026, T027, T028 can all run in parallel (different aspects of SignUpForm)
- **User Story 2**: T037, T038, T039, T040 can all run in parallel (different aspects of SignInForm)
- **User Story 3**: T051, T052, T053, T054 can all run in parallel (different components)
- **User Story 4**: T061, T062, T063, T064 can all run in parallel (different aspects of UserMenu)
- **User Story 5**: T072, T073, T074, T075, T076 can all run in parallel (different aspects of profile page)
- **Phase 8 (Polish)**: T081-T090 can all run in parallel (different cross-cutting concerns)
- **Once Foundational completes**: User Stories 1 and 2 can be worked on in parallel by different team members

---

## Parallel Example: User Story 1 (New User Registration)

```bash
# Launch all parallel tasks for SignUpForm component:
Task T024: "Create SignUpForm component in book/src/components/auth/SignUpForm.tsx"
Task T025: "Add form validation to SignUpForm"
Task T026: "Add loading state to SignUpForm"
Task T027: "Add password strength indicator to SignUpForm"
Task T028: "Style SignUpForm component"

# Then complete sequential tasks:
Task T029: "Integrate SignUpForm with better-auth client" (needs T024-T028)
Task T030: "Create signup page" (needs T029)
Task T031-T036: Testing and refinement (needs T030)
```

---

## Parallel Example: Foundational Phase

```bash
# Can work on these in parallel:
Task T014: "Create Zod validation schemas"
Task T015: "Create authentication error types"

# While these run sequentially:
Task T007: "Create Drizzle configuration"
Task T008: "Create users table schema"
Task T009: "Create sessions table schema"
Task T010: "Generate migration"
Task T011: "Push migration"
Task T012: "Create NeonDB connection"
Task T013: "Create better-auth instance"
```

---

## Implementation Strategy

### MVP First (User Stories 1, 2, 3 - All P1)

1. Complete Phase 1: Setup (T001-T006)
2. Complete Phase 2: Foundational (T007-T023) - CRITICAL
3. Complete Phase 3: User Story 1 (T024-T036)
4. **VALIDATE**: Test signup independently
5. Complete Phase 4: User Story 2 (T037-T050)
6. **VALIDATE**: Test signin independently
7. Complete Phase 5: User Story 3 (T051-T060)
8. **VALIDATE**: Test protected Urdu toggle
9. **STOP and DEMO**: MVP complete with signup, signin, protected translation

### Incremental Delivery (Add P2 and P3)

10. Add Phase 6: User Story 4 (T061-T071) - Sign out
11. **VALIDATE**: Test session management
12. Add Phase 7: User Story 5 (T072-T080) - Profile page
13. **VALIDATE**: Test profile display
14. Complete Phase 8: Polish (T081-T093)
15. **FINAL VALIDATION**: Run full E2E test suite

### Parallel Team Strategy

With 3 developers:

1. **Together**: Complete Setup (Phase 1) + Foundational (Phase 2)
2. **Once Foundational done**:
   - **Developer A**: User Story 1 (T024-T036) - Signup
   - **Developer B**: User Story 2 (T037-T050) - Signin
   - **Developer C**: Start planning User Story 3 (blocked until US1+US2)
3. **After US1+US2**:
   - **Developer A**: User Story 4 (T061-T071) - Sign out
   - **Developer B**: User Story 5 (T072-T080) - Profile
   - **Developer C**: User Story 3 (T051-T060) - Protected features
4. **Together**: Polish phase (T081-T093)

---

## Checkpoint Summary

| Phase | Checkpoint | What Works |
|-------|-----------|------------|
| Phase 2 | Foundation Ready | Database, auth instance, schemas configured |
| Phase 3 | US1 Complete | Users can register and are auto-authenticated |
| Phase 4 | US2 Complete | Users can signin, sessions persist 7 days |
| Phase 5 | US3 Complete | Urdu toggle protected, works for authenticated users |
| Phase 6 | US4 Complete | Users can sign out securely |
| Phase 7 | US5 Complete | Profile page displays user information |
| Phase 8 | All Complete | Production-ready with polish and security |

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Tests are NOT included as they were not explicitly requested in spec
- Focus on implementation quality and security best practices
- Verify all FR-001 through FR-031 requirements from spec are covered

---

## Task Coverage Summary

- **Total Tasks**: 93 tasks
- **Setup Phase**: 6 tasks (T001-T006)
- **Foundational Phase**: 17 tasks (T007-T023) - BLOCKS all stories
- **User Story 1 (P1)**: 13 tasks (T024-T036) - Signup
- **User Story 2 (P1)**: 14 tasks (T037-T050) - Signin
- **User Story 3 (P1)**: 10 tasks (T051-T060) - Protected features
- **User Story 4 (P2)**: 11 tasks (T061-T071) - Sign out
- **User Story 5 (P3)**: 9 tasks (T072-T080) - Profile page
- **Polish Phase**: 13 tasks (T081-T093) - Cross-cutting concerns

**Parallel Opportunities**: 32 tasks marked [P] can run in parallel within their phase
**MVP Scope**: Phases 1-5 (T001-T060) = 60 tasks for complete MVP with all P1 user stories
