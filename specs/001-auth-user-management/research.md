# Research & Technology Decisions: Authentication System

**Feature**: 001-auth-user-management
**Date**: 2026-01-04
**Status**: Completed

## Overview

This document captures all research findings and architectural decisions made during the planning phase for the authentication system. All "NEEDS CLARIFICATION" items from the Technical Context have been resolved through research and documented here.

## Phase 0: Research Findings

### 1. Better-Auth Library Integration

**Decision**: Use better-auth v1.3+ as the primary authentication framework

**Rationale**:
- **Framework-agnostic**: Works seamlessly with Docusaurus/React without vendor lock-in
- **Built-in security**: Provides bcrypt/argon2 password hashing, CSRF protection, secure session management
- **Custom fields support**: Supports `additionalFields` configuration for software_background and hardware_background
- **PostgreSQL native**: First-class support for PostgreSQL via drizzle adapter
- **Type-safe**: Full TypeScript support with type inference for session and user objects
- **Extensible**: Plugin architecture for future features (2FA, OAuth, etc.)

**Alternatives Considered**:
- **NextAuth.js**: Rejected - tightly coupled to Next.js, not suitable for Docusaurus
- **Passport.js**: Rejected - requires more boilerplate, no built-in session management
- **Auth0/Clerk**: Rejected - third-party services with monthly costs, network dependency

**Implementation Details**:
```typescript
// Server configuration structure
betterAuth({
  baseURL: process.env.BETTER_AUTH_URL,
  secret: process.env.BETTER_AUTH_SECRET,
  database: drizzleAdapter(db, { provider: "pg" }),
  emailAndPassword: {
    enabled: true,
    minPasswordLength: 8,
    requireEmailVerification: false, // MVP requirement
  },
  user: {
    additionalFields: {
      software_background: { type: "string", required: false },
      hardware_background: { type: "string", required: false },
    },
  },
  session: {
    expiresIn: 60 * 60 * 24 * 7, // 7 days as per spec
    updateAge: 60 * 60 * 24, // Refresh token daily
  },
})
```

**References**:
- better-auth documentation: https://github.com/better-auth/better-auth
- Custom fields: https://context7.com/better-auth/better-auth

---

### 2. NeonDB PostgreSQL Configuration

**Decision**: Use @neondatabase/serverless with Pool-based connections

**Rationale**:
- **Serverless-optimized**: Built for edge/serverless environments with WebSocket support
- **Connection pooling**: Efficient connection reuse with `Pool` class
- **Low latency**: Sub-10ms cold starts with HTTP fetch fallback
- **Cost-effective**: Pay-per-query pricing model, auto-scaling compute
- **Drizzle compatible**: Works seamlessly with better-auth's drizzle adapter

**Alternatives Considered**:
- **Supabase**: Rejected - introduces unnecessary complexity with built-in auth (conflicts with better-auth)
- **PlanetScale**: Rejected - MySQL-based, better-auth optimized for PostgreSQL
- **Traditional PostgreSQL (RDS)**: Rejected - higher operational overhead, not serverless

**Connection Strategy**:
```typescript
// Use Pool for efficient connection management
import { Pool } from '@neondatabase/serverless';
import { drizzle } from 'drizzle-orm/neon-serverless';

const pool = new Pool({ connectionString: process.env.DATABASE_URL });
export const db = drizzle(pool);

// Pool configuration
neonConfig.webSocketConstructor = ws; // For Node.js v21 and earlier
neonConfig.pipelineConnect = 'password'; // Optimize startup
neonConfig.coalesceWrites = true; // Batch writes
```

**References**:
- Neon serverless driver: https://context7.com/neondatabase/serverless
- Drizzle adapter: https://context7.com/drizzle-team/drizzle-orm

---

### 3. Database Schema Design

**Decision**: Use Drizzle ORM for schema definition and migrations

**Rationale**:
- **Type-safe**: Auto-generated TypeScript types from schema
- **Better-auth integration**: Official adapter available
- **Migration system**: Built-in migration generation and execution
- **Lightweight**: Minimal runtime overhead compared to Prisma/TypeORM
- **PostgreSQL features**: Full support for enums, JSON columns, constraints

**Schema Structure**:
- **users table**: Core user data with email, hashed password, background info
- **sessions table**: Session tokens with expiration, device tracking
- **auth_events table**: Audit log for authentication events (optional in MVP)

**Migration Strategy**:
- Use drizzle-kit for generating migrations from schema changes
- Version-controlled migrations in `auth/migrations/` directory
- Automated migration execution on deployment

**References**:
- Drizzle ORM: https://context7.com/drizzle-team/drizzle-orm
- Better-auth drizzle adapter: https://github.com/better-auth/better-auth/adapters

---

### 4. Docusaurus Integration Architecture

**Decision**: Use React Context API with custom AuthProvider wrapper

**Rationale**:
- **Native to Docusaurus**: No additional state management library needed
- **SSR compatible**: Works with Docusaurus static site generation
- **Theme integration**: Can wrap Root component via swizzling
- **Hooks support**: useSession() hook for components
- **Lightweight**: Minimal bundle size impact

**Integration Points**:
1. **Root wrapper**: Swizzle Root component to add AuthProvider
2. **Navbar**: Swizzle Navbar to add signin/signup buttons and user menu
3. **Protected routes**: Client-side check before rendering Urdu translation toggle
4. **API routes**: Use Docusaurus plugin for /api/auth/* endpoints

**Implementation Pattern**:
```typescript
// book/src/theme/Root.tsx (swizzled)
import { AuthProvider } from '@/auth/AuthProvider';

export default function Root({ children }) {
  return <AuthProvider>{children}</AuthProvider>;
}

// book/src/components/UrduToggle.tsx
import { useSession } from '@/auth/hooks';

export function UrduToggle() {
  const { data: session, isLoading } = useSession();

  if (!session) return <SignInPrompt />;
  return <LanguageToggle />;
}
```

**References**:
- Docusaurus swizzling: https://docusaurus.io/docs/swizzling
- React Context API: https://react.dev/reference/react/useContext

---

### 5. API Endpoint Architecture

**Decision**: Use Docusaurus plugin with Express-style handlers for auth endpoints

**Rationale**:
- **Integrated**: Runs within Docusaurus build/serve process
- **Better-auth handlers**: Expose better-auth's built-in endpoints
- **Custom endpoints**: Add profile endpoint for user data
- **Middleware support**: CSRF, rate limiting, session validation

**Endpoint Structure**:
- `POST /api/auth/signup` - Create user account (handled by better-auth)
- `POST /api/auth/signin` - Authenticate user (handled by better-auth)
- `POST /api/auth/signout` - Invalidate session (handled by better-auth)
- `GET /api/auth/session` - Get current session (handled by better-auth)
- `GET /api/auth/profile` - Get user profile (custom endpoint)

**Plugin Configuration**:
```typescript
// docusaurus.config.js
plugins: [
  [
    './plugins/auth-plugin',
    {
      authInstance: auth, // better-auth instance
      routes: {
        prefix: '/api/auth',
        enableRateLimit: true,
        maxAttempts: 5,
        windowMs: 15 * 60 * 1000, // 15 minutes
      },
    },
  ],
],
```

**References**:
- Docusaurus plugins: https://docusaurus.io/docs/advanced/plugins
- Better-auth handlers: https://context7.com/better-auth/better-auth

---

### 6. Session Management Strategy

**Decision**: Database-backed sessions with HTTP-only cookies

**Rationale**:
- **Security**: HTTP-only cookies prevent XSS attacks on tokens
- **Persistence**: Database storage enables cross-device sessions
- **Invalidation**: Can revoke sessions server-side (signout, security events)
- **Scalability**: Better-auth handles session lifecycle automatically
- **7-day expiration**: Balances security and UX as per spec

**Session Flow**:
1. User signs in → better-auth creates session in database
2. Session token stored in HTTP-only cookie (SameSite=Lax, Secure=true)
3. Each request → middleware validates token against database
4. Session updates → better-auth refreshes token every 24h (updateAge)
5. Expiration → session deleted from database, cookie cleared

**Alternatives Considered**:
- **JWT tokens**: Rejected - cannot be revoked server-side without additional infrastructure
- **Redis sessions**: Rejected - adds dependency, database sufficient for MVP scale
- **LocalStorage**: Rejected - vulnerable to XSS attacks

**References**:
- Better-auth session config: https://context7.com/better-auth/better-auth
- Cookie security: MDN Web Docs on Cookies

---

### 7. CSRF Protection Implementation

**Decision**: Use better-auth's built-in CSRF protection

**Rationale**:
- **Automatic**: Enabled by default in better-auth
- **Token-based**: Uses double-submit cookie pattern
- **Framework integration**: Works seamlessly with React forms
- **No additional config**: Handles token generation/validation automatically

**How It Works**:
1. better-auth generates CSRF token on session creation
2. Token stored in cookie and returned in response body
3. Client includes token in POST request headers
4. Server validates token matches cookie value
5. Invalid/missing token → 403 Forbidden

**References**:
- Better-auth CSRF: https://github.com/better-auth/better-auth/security
- OWASP CSRF Guide: https://owasp.org/www-community/attacks/csrf

---

### 8. Rate Limiting Strategy

**Decision**: Use better-auth's built-in rate limiting with configurable thresholds

**Rationale**:
- **Built-in**: No additional middleware needed
- **Configurable**: Set per-endpoint limits (5 attempts / 15 min as per spec)
- **IP-based**: Tracks attempts by IP address
- **Sliding window**: More accurate than fixed window approach

**Configuration**:
```typescript
betterAuth({
  rateLimit: {
    enabled: true,
    window: 60 * 15, // 15 minutes in seconds
    max: 5, // 5 attempts per window
  },
})
```

**Alternatives Considered**:
- **express-rate-limit**: Rejected - better-auth provides this functionality
- **Redis-backed limiter**: Rejected - overkill for MVP, adds dependency
- **CAPTCHA**: Out of scope for MVP (per spec), consider for future

**References**:
- Better-auth rate limiting: https://context7.com/better-auth/better-auth
- Rate limiting patterns: https://cloud.google.com/architecture/rate-limiting-strategies

---

### 9. Input Validation & Sanitization

**Decision**: Multi-layer validation with Zod schemas and better-auth validators

**Rationale**:
- **Type-safe**: Zod provides TypeScript type inference
- **Reusable**: Define schemas once, use on client and server
- **Better-auth integration**: Supports custom validators for additional fields
- **XSS prevention**: HTML entity encoding for user-provided data
- **SQL injection prevention**: Parameterized queries via Drizzle ORM

**Validation Layers**:
1. **Frontend**: Zod schema validation in React forms (instant feedback)
2. **API**: Zod schema validation in endpoint handlers (security)
3. **Database**: PostgreSQL constraints (data integrity)

**Example Schema**:
```typescript
import { z } from 'zod';

export const signupSchema = z.object({
  email: z.string().email().max(255),
  password: z
    .string()
    .min(8, "Password must be at least 8 characters")
    .regex(/[A-Z]/, "Must contain uppercase")
    .regex(/[a-z]/, "Must contain lowercase")
    .regex(/[0-9]/, "Must contain number"),
  software_background: z.enum([
    "Beginner",
    "Intermediate",
    "Advanced",
    "Expert",
  ]).optional(),
  hardware_background: z.enum([
    "No Experience",
    "Basic Knowledge",
    "Hands-on Experience",
    "Professional Experience",
  ]).optional(),
});
```

**References**:
- Zod documentation: https://zod.dev
- OWASP Input Validation: https://cheatsheetseries.owasp.org/cheatsheets/Input_Validation_Cheat_Sheet.html

---

### 10. Authentication Event Logging

**Decision**: Create audit_events table with structured logging

**Rationale**:
- **Security**: Track suspicious activity (failed logins, unusual patterns)
- **Compliance**: Audit trail for security reviews
- **Debugging**: Troubleshoot authentication issues
- **Analytics**: Understand user authentication patterns (optional)

**Event Types**:
- `signup_success` - New user registration
- `signup_failed` - Registration attempt failed (duplicate email, validation)
- `signin_success` - Successful authentication
- `signin_failed` - Failed login attempt
- `signout` - User-initiated signout
- `session_expired` - Automatic session expiration

**Logged Data**:
- Timestamp (UTC)
- Event type
- User ID (if applicable)
- IP address
- User agent string
- Event metadata (error messages, etc.)

**References**:
- Security logging best practices: OWASP Logging Cheat Sheet
- GDPR considerations: Keep IP addresses pseudonymized, set retention period

---

### 11. Frontend State Management

**Decision**: React Context API with useSession() custom hook

**Rationale**:
- **Minimal complexity**: No Redux/Zustand needed for auth state only
- **Better-auth integration**: Works with better-auth/react client
- **SSR friendly**: Compatible with Docusaurus build process
- **Performance**: Context updates don't trigger unnecessary re-renders

**State Structure**:
```typescript
interface AuthState {
  session: Session | null;
  isLoading: boolean;
  isAuthenticated: boolean;
  user: User | null;
  signIn: (email: string, password: string) => Promise<void>;
  signUp: (data: SignUpData) => Promise<void>;
  signOut: () => Promise<void>;
}
```

**References**:
- React Context API: https://react.dev/reference/react/useContext
- Better-auth React client: https://context7.com/better-auth/better-auth

---

### 12. Error Handling Strategy

**Decision**: Typed error responses with user-friendly messages

**Rationale**:
- **Security**: Generic messages prevent information leakage (no "user not found")
- **UX**: Clear, actionable error messages for users
- **Debugging**: Detailed server logs separate from user-facing messages
- **Type-safe**: TypeScript error types for consistent handling

**Error Response Format**:
```typescript
interface AuthError {
  code: string;
  message: string;
  field?: string; // For field-specific validation errors
}

// Example errors
const AUTH_ERRORS = {
  INVALID_CREDENTIALS: {
    code: 'AUTH001',
    message: 'Invalid email or password',
  },
  DUPLICATE_EMAIL: {
    code: 'AUTH002',
    message: 'An account with this email already exists',
  },
  WEAK_PASSWORD: {
    code: 'AUTH003',
    message: 'Password does not meet requirements',
  },
  SESSION_EXPIRED: {
    code: 'AUTH004',
    message: 'Your session has expired. Please sign in again.',
  },
  RATE_LIMIT_EXCEEDED: {
    code: 'AUTH005',
    message: 'Too many attempts. Please try again in 15 minutes.',
  },
};
```

**References**:
- Error handling patterns: https://kentcdodds.com/blog/get-a-catch-block-error-message-with-typescript
- Security messaging: OWASP Authentication Cheat Sheet

---

### 13. Testing Strategy

**Decision**: Multi-layer testing with Jest, React Testing Library, and Playwright

**Rationale**:
- **Unit tests**: Test individual functions (validation, helpers)
- **Integration tests**: Test API endpoints with test database
- **Component tests**: Test React components in isolation
- **E2E tests**: Test complete authentication flows

**Test Coverage Targets**:
- Unit tests: 80%+ coverage for utils, validators, helpers
- Integration tests: All API endpoints with success/error scenarios
- Component tests: All auth UI components
- E2E tests: 5 user stories from spec (signup, signin, protected access, signout, profile)

**Testing Tools**:
```typescript
// Unit & Integration: Jest + Supertest
import { describe, it, expect } from '@jest/globals';
import request from 'supertest';

// Component: React Testing Library
import { render, screen, userEvent } from '@testing-library/react';

// E2E: Playwright
import { test, expect } from '@playwright/test';
```

**References**:
- Jest: https://jestjs.io/docs/getting-started
- React Testing Library: https://testing-library.com/docs/react-testing-library/intro
- Playwright: https://playwright.dev

---

## Summary of Key Decisions

| Area | Decision | Rationale |
|------|----------|-----------|
| **Auth Framework** | better-auth v1.3+ | Type-safe, PostgreSQL native, custom fields support |
| **Database** | NeonDB with @neondatabase/serverless | Serverless-optimized, connection pooling, low latency |
| **ORM** | Drizzle ORM | Lightweight, type-safe, better-auth integration |
| **Frontend State** | React Context API | Minimal complexity, SSR friendly |
| **API Architecture** | Docusaurus plugin with Express handlers | Integrated with build process, middleware support |
| **Session Storage** | Database with HTTP-only cookies | Secure, revocable, persistent |
| **CSRF Protection** | better-auth built-in | Automatic double-submit cookie pattern |
| **Rate Limiting** | better-auth built-in | IP-based, sliding window, configurable |
| **Validation** | Zod schemas | Type-safe, reusable, multi-layer |
| **Logging** | Structured audit_events table | Security, compliance, debugging |
| **Testing** | Jest + RTL + Playwright | Unit, integration, component, E2E coverage |

---

## Open Questions (For Future Iterations)

1. **Email Verification**: How should we implement this post-MVP?
   - Options: better-auth email plugin, third-party service (SendGrid, Postmark)

2. **Password Reset**: What's the user flow for forgotten passwords?
   - Options: better-auth reset password flow, magic links

3. **OAuth Providers**: Which social login providers should we support?
   - Options: Google, GitHub, Apple (better-auth has plugins for all)

4. **Advanced Rate Limiting**: Should we add CAPTCHA for persistent attackers?
   - Options: hCaptcha, reCAPTCHA, Cloudflare Turnstile

5. **Session Management UI**: Should users be able to view/revoke active sessions?
   - Implementation: Dashboard page with sessions table query

---

## Next Steps

With all research complete, proceed to:
1. ✅ Phase 1: Design database schema (data-model.md)
2. ✅ Phase 1: Define API contracts (contracts/)
3. ✅ Phase 1: Write quickstart guide (quickstart.md)
4. ✅ Phase 2: Create implementation plan (plan.md)
5. Phase 3: Generate tasks (tasks.md via /sp.tasks)
