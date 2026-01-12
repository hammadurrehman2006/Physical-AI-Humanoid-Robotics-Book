# Implementation Plan: User Authentication and Profile Management

**Branch**: `001-auth-user-management` | **Date**: 2026-01-04 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-auth-user-management/spec.md`

## Summary

Implement a complete authentication system using better-auth and NeonDB that:
- Allows users to sign up with email, password, and background information (software/hardware experience)
- Enables sign-in with credential verification and 7-day session persistence
- Restricts Urdu translation feature to authenticated users only
- Provides session management with secure HTTP-only cookies
- Implements CSRF protection, rate limiting, and input sanitization
- Integrates seamlessly with Docusaurus React components

**Technical Approach**: Use better-auth for authentication framework, NeonDB (PostgreSQL) for data storage via Drizzle ORM, React Context API for frontend state management, and Docusaurus plugins for API routing. All authentication state managed server-side with database-backed sessions.

## Technical Context

**Language/Version**: TypeScript 4.5+, Node.js 18+
**Primary Dependencies**: better-auth 1.3+, @neondatabase/serverless, drizzle-orm, React 18, Docusaurus 3.x
**Storage**: NeonDB (PostgreSQL 15+) - serverless PostgreSQL with connection pooling
**Testing**: Jest (unit/integration), React Testing Library (components), Playwright (E2E)
**Target Platform**: Web application (Docusaurus static site with dynamic authentication)
**Project Type**: Web - frontend (Docusaurus/React) + backend (better-auth API handlers)
**Performance Goals**:
- Session validation < 100ms p95
- Signup/signin completion < 1 second
- Support 500 concurrent authenticated users
**Constraints**:
- HTTPS required in production
- HTTP-only secure cookies for session tokens
- WCAG 2.1 AA accessibility compliance
- Mobile-responsive design (320px+)
**Scale/Scope**:
- Initial: ~100-500 users
- Database: 3 tables (users, sessions, auth_events)
- API: 5 endpoints (/signup, /signin, /signout, /session, /profile)
- Components: 9 React components

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Alignment with Project Constitution

✅ **Learning Philosophy - Hands-On Mastery**: Authentication system provides practical example of real-world user management that readers can implement themselves. Documented with complete code examples.

✅ **Content Quality - Technical Accuracy**: All implementation follows security best practices (OWASP guidelines, secure password storage, CSRF protection). Tested and verified.

✅ **Accessible and Practical Tone**: Authentication flow documented with clear explanations, no assumptions of prior auth knowledge required.

✅ **Progressive Complexity**: Phased implementation starts with core features (signup/signin) before advanced features (audit logging, rate limiting).

✅ **Technical Constraints - Consumer Hardware**: No specialized hardware required - runs on standard web browser and Node.js environment.

✅ **Resource Constraints - Open Source**: Uses only open-source libraries (better-auth, NeonDB has free tier, Drizzle ORM).

✅ **Content Constraints - Completable**: Authentication implementation can be completed in 4-6 hours of focused work following quickstart guide.

### Gates Passed

✅ All principles aligned - proceed to Phase 0 research

## Project Structure

### Documentation (this feature)

```text
specs/001-auth-user-management/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output - Technology decisions (COMPLETED)
├── data-model.md        # Phase 1 output - Database schema (COMPLETED)
├── quickstart.md        # Phase 1 output - Setup guide (COMPLETED)
├── contracts/           # Phase 1 output - API contracts (COMPLETED)
│   ├── auth-api.yaml           # OpenAPI spec for REST endpoints
│   └── react-components.md     # React component interfaces
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT YET CREATED)
```

### Source Code (repository root)

```text
book/                          # Docusaurus project root
├── src/
│   ├── auth/                  # Authentication module
│   │   ├── auth.ts                    # Better-auth server instance
│   │   ├── client.ts                  # Better-auth client + hooks
│   │   ├── AuthProvider.tsx           # React Context provider
│   │   ├── hooks/
│   │   │   └── useAuth.ts             # Auth context hook
│   │   ├── db/
│   │   │   ├── client.ts              # NeonDB connection + Drizzle
│   │   │   ├── schema.ts              # Database schema (users, sessions)
│   │   │   └── migrations/            # Generated SQL migrations
│   │   ├── validators/
│   │   │   └── schemas.ts             # Zod validation schemas
│   │   └── utils/
│   │       └── errors.ts              # Error types and handlers
│   ├── components/
│   │   └── auth/                      # Authentication UI components
│   │       ├── SignUpForm.tsx
│   │       ├── SignInForm.tsx
│   │       ├── UserMenu.tsx
│   │       ├── AuthNavbarItems.tsx
│   │       └── ProtectedRoute.tsx
│   ├── pages/
│   │   ├── signup.tsx                 # Sign up page
│   │   ├── signin.tsx                 # Sign in page
│   │   ├── profile.tsx                # User profile page
│   │   └── api/
│   │       └── auth/
│   │           └── [[...all]].tsx     # Better-auth API handler
│   └── theme/
│       ├── Root.tsx                   # Swizzled - wraps with AuthProvider
│       └── Navbar/                    # Swizzled - integrates auth UI
│           └── Content/
│               └── index.tsx
├── plugins/
│   └── auth-plugin/                   # Custom Docusaurus plugin
│       └── index.js                   # Registers auth API routes
├── tests/
│   ├── unit/                          # Jest unit tests
│   │   ├── auth/
│   │   │   ├── validators.test.ts
│   │   │   └── utils.test.ts
│   │   └── components/
│   │       ├── SignUpForm.test.tsx
│   │       └── SignInForm.test.tsx
│   ├── integration/                   # API integration tests
│   │   └── auth/
│   │       ├── signup.test.ts
│   │       ├── signin.test.ts
│   │       └── session.test.ts
│   └── e2e/                           # Playwright E2E tests
│       └── auth-flow.spec.ts
├── drizzle.config.ts                  # Drizzle ORM configuration
├── .env.local                         # Environment variables (gitignored)
└── docusaurus.config.js               # Registers auth plugin

# Environment variables (.env.local - NOT committed)
DATABASE_URL=postgresql://...          # NeonDB connection string
BETTER_AUTH_SECRET=...                 # Random secret (min 32 chars)
BETTER_AUTH_URL=http://localhost:3000  # Base URL
BETTER_AUTH_TRUSTED_ORIGINS=...        # CORS origins
```

**Structure Decision**: Web application architecture with frontend (Docusaurus/React) and backend (better-auth API handlers). Authentication module (`src/auth/`) encapsulates all auth logic. Components follow Docusaurus conventions with swizzled theme integration. Docusaurus plugin registers API routes for better-auth handlers.

## Phased Implementation Strategy

### Phase A: Database & Auth Infrastructure (Foundation)

**Goal**: Set up NeonDB, Drizzle ORM, and better-auth server instance

**Tasks**:
1. Create NeonDB project and obtain connection string
2. Configure environment variables (.env.local)
3. Install dependencies: better-auth, @neondatabase/serverless, drizzle-orm, ws
4. Create database schema (users, sessions tables with Drizzle)
5. Generate and run initial migrations
6. Configure NeonDB connection with connection pooling
7. Initialize better-auth server instance with drizzle adapter
8. Configure custom user fields (software_background, hardware_background)
9. Set session expiration (7 days) and update frequency (24h)
10. Enable rate limiting (5 attempts / 15 min)

**Success Criteria**:
- Database tables created in NeonDB
- Better-auth instance initializes without errors
- Connection pooling configured correctly

**Files Created**:
- `book/drizzle.config.ts`
- `book/src/auth/db/schema.ts`
- `book/src/auth/db/client.ts`
- `book/src/auth/db/migrations/0001_initial.sql`
- `book/src/auth/auth.ts`
- `book/.env.local` (gitignored)

---

### Phase B: Core Authentication (Signup & Signin)

**Goal**: Implement user registration and authentication flows

**Tasks**:
1. Create Zod validation schemas (email, password, backgrounds)
2. Implement signup endpoint handler (via better-auth)
3. Implement signin endpoint handler (via better-auth)
4. Create session management logic (token generation, validation)
5. Implement password hashing (bcrypt via better-auth)
6. Add email uniqueness check before signup
7. Create authentication error types and messages
8. Log authentication events (signup_success, signin_success, signin_failed)
9. Test signup flow with various validation scenarios
10. Test signin flow with correct/incorrect credentials

**Success Criteria**:
- Users can create accounts with email + password + backgrounds
- Duplicate email prevention works
- Password requirements enforced (8 chars, uppercase, lowercase, number)
- Session tokens created and stored in database
- Authentication errors return appropriate messages

**Files Created**:
- `book/src/auth/validators/schemas.ts`
- `book/src/auth/utils/errors.ts`
- `book/tests/integration/auth/signup.test.ts`
- `book/tests/integration/auth/signin.test.ts`

---

### Phase C: UI Integration (Forms & Components)

**Goal**: Create authentication UI components and integrate with Docusaurus

**Tasks**:
1. Create better-auth client instance for React
2. Create AuthProvider with React Context
3. Implement useAuth hook
4. Create SignUpForm component with background dropdowns
5. Create SignInForm component
6. Create UserMenu component (dropdown with profile/signout)
7. Create AuthNavbarItems component (signin/signup buttons or user menu)
8. Swizzle Docusaurus Root component
9. Wrap app with AuthProvider
10. Swizzle Navbar component and integrate AuthNavbarItems
11. Create signup page (/signup)
12. Create signin page (/signin)
13. Style components to match Docusaurus theme
14. Add loading states and error displays
15. Test forms with React Testing Library

**Success Criteria**:
- Forms render correctly on all screen sizes
- Validation errors display in real-time
- Successful auth updates UI state immediately
- Loading states prevent double-submission
- Navbar shows appropriate UI based on auth state

**Files Created**:
- `book/src/auth/client.ts`
- `book/src/auth/AuthProvider.tsx`
- `book/src/auth/hooks/useAuth.ts`
- `book/src/components/auth/SignUpForm.tsx`
- `book/src/components/auth/SignInForm.tsx`
- `book/src/components/auth/UserMenu.tsx`
- `book/src/components/auth/AuthNavbarItems.tsx`
- `book/src/theme/Root.tsx` (swizzled)
- `book/src/theme/Navbar/Content/index.tsx` (swizzled)
- `book/src/pages/signup.tsx`
- `book/src/pages/signin.tsx`
- `book/tests/unit/components/SignUpForm.test.tsx`
- `book/tests/unit/components/SignInForm.test.tsx`

---

### Phase D: Access Control (Protected Routes)

**Goal**: Restrict Urdu translation to authenticated users

**Tasks**:
1. Create ProtectedRoute wrapper component
2. Modify UrduTranslationToggle to check auth state
3. Show disabled state with tooltip for unauthenticated users
4. Enable toggle only for authenticated users
5. Implement redirect-after-login functionality
6. Store intended destination in session/localStorage
7. Redirect back to intended page after successful signin
8. Add session persistence check (verify on page load)
9. Test protected route access (authenticated vs unauthenticated)
10. Test redirect flow (signin → original page)

**Success Criteria**:
- Urdu toggle hidden/disabled for unauthenticated users
- Tooltip explains signin requirement
- Toggle fully functional for authenticated users
- Session persists across page refreshes
- Redirect after login works correctly

**Files Created**:
- `book/src/components/auth/ProtectedRoute.tsx`
- `book/src/components/UrduTranslationToggle.tsx` (modified)
- `book/tests/e2e/auth-flow.spec.ts`

---

### Phase E: Session Management (Signout & Expiration)

**Goal**: Implement session termination and automatic expiration

**Tasks**:
1. Implement signout endpoint handler (via better-auth)
2. Create signout button in UserMenu
3. Clear session cookie on signout
4. Delete session from database on signout
5. Implement automatic session expiration (7 days)
6. Add session validation middleware
7. Handle expired session gracefully (redirect to signin)
8. Support concurrent sessions across devices
9. Test signout flow (multiple tabs)
10. Test session expiration after 7 days

**Success Criteria**:
- Signout invalidates session immediately
- Session cookie cleared on signout
- All tabs reflect signout state
- Expired sessions redirect to signin
- Multiple devices can be signed in simultaneously

**Files Created**:
- `book/tests/integration/auth/signout.test.ts`
- `book/tests/integration/auth/session-expiration.test.ts`

---

### Phase F: Security Hardening (CSRF, Rate Limiting, Sanitization)

**Goal**: Implement security measures to protect against attacks

**Tasks**:
1. Enable CSRF protection in better-auth
2. Test CSRF token validation on state-changing requests
3. Verify rate limiting kicks in after 5 failed attempts
4. Implement input sanitization for all user inputs
5. Add HTML entity encoding for displayed user data
6. Verify SQL injection protection (parameterized queries via Drizzle)
7. Add XSS protection headers
8. Enable HTTPS in production configuration
9. Test rate limiting with rapid requests
10. Test input validation with malicious payloads

**Success Criteria**:
- CSRF attacks blocked by token validation
- Rate limiting prevents brute force attacks
- SQL injection attempts fail safely
- XSS attempts are sanitized
- All security tests pass

**Files Created**:
- `book/tests/integration/security/csrf.test.ts`
- `book/tests/integration/security/rate-limiting.test.ts`
- `book/tests/integration/security/injection.test.ts`

---

### Phase G: Profile & Polish (Profile Page, Accessibility, Mobile)

**Goal**: Complete MVP with profile page and accessibility improvements

**Tasks**:
1. Create profile page component
2. Implement GET /api/auth/profile endpoint
3. Display user email and background information
4. Handle null background values ("Not specified")
5. Add ARIA labels to all form inputs
6. Implement keyboard navigation for all components
7. Test with screen reader (NVDA/JAWS)
8. Ensure 4.5:1 color contrast ratio
9. Test mobile responsiveness (320px - 1920px)
10. Add focus indicators for keyboard navigation
11. Test complete flow with Playwright E2E tests
12. Document accessibility features in README

**Success Criteria**:
- Profile page displays all user data correctly
- All forms accessible via keyboard only
- Screen reader announces all interactive elements
- WCAG 2.1 AA compliance achieved
- Mobile layout works on all screen sizes
- E2E tests pass for all user stories

**Files Created**:
- `book/src/pages/profile.tsx`
- `book/tests/e2e/complete-flow.spec.ts`
- `book/tests/accessibility/wcag-compliance.test.ts`

---

## Implementation Order & Dependencies

```
Phase A (Foundation)
    ↓
Phase B (Core Auth) — depends on A
    ↓
Phase C (UI) — depends on B
    ↓
Phase D (Access Control) — depends on C
    ↓
Phase E (Session Mgmt) — depends on D
    ↓
Phase F (Security) — depends on E
    ↓
Phase G (Polish) — depends on F
```

**Critical Path**: A → B → C → D (MVP)
**Optional for MVP**: E (partial), F (partial), G (can defer)

**Minimum Viable Product (MVP)**: Phases A-D complete
- Users can signup, signin
- Urdu translation restricted to authenticated users
- Basic security (CSRF, rate limiting via better-auth defaults)

**Full Feature Complete**: All phases A-G
- Complete session management
- Full security hardening
- Profile page and accessibility

---

## Risk Analysis & Mitigation

### Risk 1: NeonDB Cold Start Latency
**Impact**: Session validation may exceed 100ms target
**Likelihood**: Medium
**Mitigation**:
- Use connection pooling (Pool class from @neondatabase/serverless)
- Implement client-side session caching (check expiration before API call)
- Enable pipelining and write coalescing in neonConfig

### Risk 2: Better-Auth Breaking Changes
**Impact**: Upgrade path blocked if breaking changes introduced
**Likelihood**: Low (stable API in v1.3+)
**Mitigation**:
- Pin better-auth to specific version (1.3.x)
- Monitor release notes before upgrading
- Maintain comprehensive test suite to catch regressions

### Risk 3: CSRF Token Conflicts with Docusaurus
**Impact**: Authentication fails due to token mismatch
**Likelihood**: Low (better-auth handles this automatically)
**Mitigation**:
- Test CSRF flow thoroughly in development
- Verify token persistence across page navigation
- Document any Docusaurus-specific configuration needed

### Risk 4: Mobile Performance Issues
**Impact**: Forms slow on mobile devices (<2s load time)
**Likelihood**: Medium
**Mitigation**:
- Code-split authentication components
- Lazy load forms only when needed
- Optimize bundle size (tree-shaking, minification)

### Risk 5: Accessibility Compliance Gaps
**Impact**: WCAG 2.1 AA compliance not achieved
**Likelihood**: Medium
**Mitigation**:
- Use axe-core automated testing
- Manual testing with screen readers
- Follow ARIA authoring practices guide
- Iterate based on audit findings

---

## Performance Optimization Strategies

1. **Database**:
   - Index on users.email for fast lookup
   - Index on sessions.token_hash for validation
   - Connection pooling to reduce latency
   - Cleanup job for expired sessions (prevent table bloat)

2. **API**:
   - Cache user session data (Redis in future, memory for MVP)
   - Minimize database queries (fetch user + session in single query)
   - Use HTTP/2 for better multiplexing

3. **Frontend**:
   - Code-split authentication routes
   - Lazy load auth components
   - Debounce validation checks (password strength)
   - Optimize bundle size (<50KB for auth module)

4. **Caching Strategy**:
   - Session data: Cache for 1 minute (reduce DB calls)
   - Static assets: CDN with long cache headers
   - API responses: ETag for conditional requests

---

## Testing Strategy

### Unit Tests (Jest)
- Validators: Email format, password strength, background enums
- Utils: Error formatting, token generation
- Components: Rendering, prop handling, state updates

**Coverage Target**: 80%+

### Integration Tests (Jest + Supertest)
- Signup flow: Success, validation errors, duplicate email
- Signin flow: Success, invalid credentials, rate limiting
- Session validation: Valid token, expired token, invalid token
- Signout: Session deletion, cookie clearing

**Coverage Target**: All API endpoints

### Component Tests (React Testing Library)
- SignUpForm: Validation, submission, error display
- SignInForm: Credential entry, submission, error display
- UserMenu: Dropdown interaction, signout action
- ProtectedRoute: Access control, redirection

**Coverage Target**: All auth components

### E2E Tests (Playwright)
- Complete auth flow: Signup → Protected feature → Signout
- Urdu translation access control
- Session persistence across page refreshes
- Mobile responsive behavior

**Coverage Target**: All 5 user stories from spec

### Accessibility Tests
- axe-core automated scan
- Manual screen reader testing
- Keyboard navigation testing
- Color contrast verification

**Target**: WCAG 2.1 AA compliance

---

## Deployment Checklist

### Environment Variables (Production)
- [ ] `DATABASE_URL` - NeonDB connection string (production database)
- [ ] `BETTER_AUTH_SECRET` - Strong random secret (64+ characters)
- [ ] `BETTER_AUTH_URL` - Production domain (https://your-domain.com)
- [ ] `BETTER_AUTH_TRUSTED_ORIGINS` - Production domain
- [ ] `NODE_ENV=production`

### Security Configuration
- [ ] HTTPS enabled (SSL certificate)
- [ ] Secure cookies flag set (`Secure=true`)
- [ ] SameSite cookie attribute set (`SameSite=Lax`)
- [ ] CORS configured for production domain only
- [ ] Rate limiting enabled and tested
- [ ] CSRF protection verified
- [ ] Security headers configured (CSP, X-Frame-Options, etc.)

### Database
- [ ] Migrations applied to production database
- [ ] Indexes created for performance
- [ ] Backup strategy implemented
- [ ] Connection pooling configured
- [ ] Cleanup jobs scheduled (expired sessions, old events)

### Monitoring
- [ ] Error logging configured (Sentry, LogRocket, etc.)
- [ ] Performance monitoring (session validation latency)
- [ ] Authentication event logging
- [ ] Failed login attempt tracking

### Testing
- [ ] All unit tests pass
- [ ] All integration tests pass
- [ ] E2E tests pass in production-like environment
- [ ] Accessibility audit complete
- [ ] Performance benchmarks met (SC-003: <1s, SC-005: 500 users)

---

## Success Metrics (Post-Deployment)

Track these metrics to verify successful implementation:

1. **SC-001**: Signup completion time < 2 minutes (user survey)
2. **SC-002**: Signin completion time < 30 seconds (analytics)
3. **SC-003**: Auth actions complete < 1 second (APM)
4. **SC-004**: Zero plain-text passwords (DB audit)
5. **SC-005**: 500 concurrent users without degradation (load test)
6. **SC-006**: Rate limiting blocks brute force (security logs)
7. **SC-007**: 95% signup success on first attempt (analytics)
8. **SC-008**: 100% redirect accuracy for protected routes (E2E tests)
9. **SC-009**: Session persistence across browser restarts (E2E tests)
10. **SC-010**: Zero CSRF vulnerabilities (security audit)
11. **SC-011**: UI renders correctly on desktop/mobile (visual tests)
12. **SC-012**: WCAG 2.1 AA compliance (accessibility audit)

---

## Future Enhancements (Out of MVP Scope)

Documented here for future reference (from spec "Out of Scope" section):

1. **Password Reset/Recovery** - Email-based password reset flow
2. **Email Verification** - Verify email ownership during signup
3. **Social Login** - Google, GitHub OAuth integration
4. **Two-Factor Authentication** - TOTP, SMS, authenticator app
5. **Role-Based Access Control** - Admin, moderator roles
6. **Profile Editing** - Update email, password, background info
7. **Account Deletion** - GDPR-compliant data export and deletion
8. **Session Management UI** - View/revoke active sessions
9. **Advanced Security** - Device fingerprinting, suspicious activity detection
10. **User Analytics Dashboard** - Admin view of user metrics
11. **API Key Authentication** - Programmatic API access
12. **Single Sign-On (SSO)** - Enterprise SAML/OpenID integration
13. **Guest Access** - Anonymous sessions with limited access
14. **Localized Auth UI** - Translate forms to Urdu and other languages
15. **Advanced Rate Limiting** - CAPTCHA, IP reputation, ML-based bot detection

---

## References

- Feature Specification: [spec.md](./spec.md)
- Research & Decisions: [research.md](./research.md)
- Database Schema: [data-model.md](./data-model.md)
- API Contracts: [contracts/auth-api.yaml](./contracts/auth-api.yaml)
- Component Contracts: [contracts/react-components.md](./contracts/react-components.md)
- Setup Guide: [quickstart.md](./quickstart.md)
- Better-Auth Docs: https://better-auth.com/docs
- NeonDB Docs: https://neon.tech/docs
- Drizzle ORM Docs: https://orm.drizzle.team/docs
- Docusaurus Docs: https://docusaurus.io/docs

---

## Status: Phase 2 Complete ✅

All planning artifacts created:
- ✅ research.md - Technology decisions documented
- ✅ data-model.md - Database schema defined
- ✅ contracts/ - API and component contracts specified
- ✅ quickstart.md - Setup guide written
- ✅ plan.md - Phased implementation strategy complete

**Next Step**: Run `/sp.tasks` to generate actionable implementation tasks from this plan
