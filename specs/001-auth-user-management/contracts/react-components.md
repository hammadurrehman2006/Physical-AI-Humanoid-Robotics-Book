# React Component Contracts

**Feature**: 001-auth-user-management
**Date**: 2026-01-04
**Framework**: React 18+ (Docusaurus)

## Overview

This document defines the interfaces and contracts for all React components in the authentication system. All components are designed to integrate seamlessly with Docusaurus theme and follow React best practices.

---

## Core Components

### 1. AuthProvider (Context Provider)

**Purpose**: Provides authentication state to all components via React Context.

**Location**: `book/src/auth/AuthProvider.tsx`

**Props**: None (wraps children)

**Context Value**:
```typescript
interface AuthContextValue {
  // Session state
  session: SessionWithUser | null;
  isLoading: boolean;
  isAuthenticated: boolean;

  // Authentication actions
  signUp: (data: SignUpData) => Promise<void>;
  signIn: (email: string, password: string) => Promise<void>;
  signOut: () => Promise<void>;

  // Error state
  error: AuthError | null;
  clearError: () => void;
}

interface SessionWithUser {
  session: {
    id: string;
    expires_at: Date;
    created_at: Date;
  };
  user: PublicUser;
}

interface PublicUser {
  id: string;
  email: string;
  software_background: string | null;
  hardware_background: string | null;
  created_at: Date;
  last_login_at: Date | null;
}

interface SignUpData {
  email: string;
  password: string;
  software_background?: string;
  hardware_background?: string;
}

interface AuthError {
  code: string;
  message: string;
  field?: string;
}
```

**Usage**:
```typescript
import { AuthProvider } from '@/auth/AuthProvider';

export default function Root({ children }) {
  return <AuthProvider>{children}</AuthProvider>;
}
```

**Behavior**:
- Fetches initial session on mount via GET /api/auth/session
- Provides auth methods to all child components
- Handles loading and error states
- Automatically updates on auth state changes

---

### 2. useAuth Hook

**Purpose**: Hook for accessing authentication state and methods in components.

**Location**: `book/src/auth/hooks/useAuth.ts`

**Return Type**: `AuthContextValue` (see above)

**Usage**:
```typescript
import { useAuth } from '@/auth/hooks/useAuth';

function MyComponent() {
  const { isAuthenticated, user, signOut } = useAuth();

  if (!isAuthenticated) {
    return <div>Please sign in</div>;
  }

  return (
    <div>
      <p>Welcome, {user?.email}</p>
      <button onClick={signOut}>Sign Out</button>
    </div>
  );
}
```

**Error Handling**:
```typescript
// Throws error if used outside AuthProvider
if (!context) {
  throw new Error('useAuth must be used within AuthProvider');
}
```

---

### 3. SignUpForm Component

**Purpose**: User registration form with email, password, and background questions.

**Location**: `book/src/components/auth/SignUpForm.tsx`

**Props**:
```typescript
interface SignUpFormProps {
  onSuccess?: (user: PublicUser) => void;
  onError?: (error: AuthError) => void;
  redirectTo?: string; // URL to redirect after signup
}
```

**State**:
```typescript
interface SignUpFormState {
  email: string;
  password: string;
  confirmPassword: string;
  software_background: string | null;
  hardware_background: string | null;
  isSubmitting: boolean;
  errors: Partial<Record<keyof SignUpFormState, string>>;
}
```

**UI Structure**:
```tsx
<form onSubmit={handleSubmit}>
  {/* Email field */}
  <input
    type="email"
    name="email"
    required
    aria-label="Email address"
    aria-describedby="email-error"
  />

  {/* Password field */}
  <input
    type="password"
    name="password"
    required
    minLength={8}
    aria-label="Password"
    aria-describedby="password-requirements password-error"
  />
  <div id="password-requirements" className="password-hint">
    Must contain: 8+ characters, uppercase, lowercase, number
  </div>

  {/* Confirm password field */}
  <input
    type="password"
    name="confirmPassword"
    required
    aria-label="Confirm password"
    aria-describedby="confirm-password-error"
  />

  {/* Software background dropdown */}
  <select
    name="software_background"
    aria-label="Software experience level"
  >
    <option value="">Select your software experience</option>
    <option value="Beginner">Beginner</option>
    <option value="Intermediate">Intermediate</option>
    <option value="Advanced">Advanced</option>
    <option value="Expert">Expert</option>
  </select>

  {/* Hardware background dropdown */}
  <select
    name="hardware_background"
    aria-label="Hardware experience level"
  >
    <option value="">Select your hardware experience</option>
    <option value="No Experience">No Experience</option>
    <option value="Basic Knowledge">Basic Knowledge</option>
    <option value="Hands-on Experience">Hands-on Experience</option>
    <option value="Professional Experience">Professional Experience</option>
  </select>

  {/* Submit button */}
  <button
    type="submit"
    disabled={isSubmitting}
    aria-busy={isSubmitting}
  >
    {isSubmitting ? 'Creating account...' : 'Sign Up'}
  </button>

  {/* Error display */}
  {error && (
    <div role="alert" aria-live="polite">
      {error.message}
    </div>
  )}
</form>
```

**Validation**:
- Client-side validation using Zod schema
- Real-time password strength indicator
- Confirm password must match
- Background fields are optional

**Behavior**:
- On submit → call signUp() from useAuth hook
- On success → redirect to redirectTo or homepage
- On error → display error message
- Loading state → disable form and show spinner

---

### 4. SignInForm Component

**Purpose**: User authentication form with email and password.

**Location**: `book/src/components/auth/SignInForm.tsx`

**Props**:
```typescript
interface SignInFormProps {
  onSuccess?: (user: PublicUser) => void;
  onError?: (error: AuthError) => void;
  redirectTo?: string; // URL to redirect after signin
}
```

**State**:
```typescript
interface SignInFormState {
  email: string;
  password: string;
  isSubmitting: boolean;
  error: AuthError | null;
}
```

**UI Structure**:
```tsx
<form onSubmit={handleSubmit}>
  {/* Email field */}
  <input
    type="email"
    name="email"
    required
    autoComplete="email"
    aria-label="Email address"
  />

  {/* Password field */}
  <input
    type="password"
    name="password"
    required
    autoComplete="current-password"
    aria-label="Password"
  />

  {/* Submit button */}
  <button type="submit" disabled={isSubmitting}>
    {isSubmitting ? 'Signing in...' : 'Sign In'}
  </button>

  {/* Error display */}
  {error && (
    <div role="alert" aria-live="polite">
      {error.message}
    </div>
  )}

  {/* Sign up link */}
  <p>
    Don't have an account? <Link to="/signup">Sign up</Link>
  </p>
</form>
```

**Behavior**:
- On submit → call signIn() from useAuth hook
- On success → redirect to redirectTo or previous page
- On error → display generic error (no email enumeration)
- Loading state → disable form and show spinner

---

### 5. UserMenu Component

**Purpose**: Dropdown menu showing user info and sign-out button.

**Location**: `book/src/components/auth/UserMenu.tsx`

**Props**:
```typescript
interface UserMenuProps {
  user: PublicUser;
  onSignOut: () => void;
}
```

**State**:
```typescript
interface UserMenuState {
  isOpen: boolean;
}
```

**UI Structure**:
```tsx
<div className="user-menu">
  {/* Trigger button */}
  <button
    onClick={() => setIsOpen(!isOpen)}
    aria-expanded={isOpen}
    aria-haspopup="menu"
    aria-label="User menu"
  >
    <UserIcon />
    <span>{user.email}</span>
  </button>

  {/* Dropdown menu */}
  {isOpen && (
    <div role="menu" className="user-menu-dropdown">
      <Link to="/profile" role="menuitem">
        Profile
      </Link>
      <button onClick={onSignOut} role="menuitem">
        Sign Out
      </button>
    </div>
  )}
</div>
```

**Behavior**:
- Click outside → close menu
- Escape key → close menu
- Sign out → call onSignOut callback
- Keyboard navigation support

---

### 6. AuthNavbarItems Component

**Purpose**: Navbar integration showing sign-in/sign-up or user menu based on auth state.

**Location**: `book/src/components/auth/AuthNavbarItems.tsx`

**Props**: None (uses useAuth hook)

**UI Structure**:
```tsx
function AuthNavbarItems() {
  const { isAuthenticated, isLoading, user, signOut } = useAuth();

  if (isLoading) {
    return <div className="navbar-auth-skeleton">Loading...</div>;
  }

  if (isAuthenticated && user) {
    return <UserMenu user={user} onSignOut={signOut} />;
  }

  return (
    <div className="navbar-auth-buttons">
      <Link to="/signin" className="button button--secondary">
        Sign In
      </Link>
      <Link to="/signup" className="button button--primary">
        Sign Up
      </Link>
    </div>
  );
}
```

**Integration**: Swizzled into Docusaurus Navbar component.

---

### 7. ProtectedRoute Component

**Purpose**: Wrapper component that restricts access to authenticated users only.

**Location**: `book/src/components/auth/ProtectedRoute.tsx`

**Props**:
```typescript
interface ProtectedRouteProps {
  children: React.ReactNode;
  redirectTo?: string; // Default: '/signin'
  loadingComponent?: React.ReactNode;
}
```

**Behavior**:
```tsx
function ProtectedRoute({ children, redirectTo = '/signin', loadingComponent }) {
  const { isAuthenticated, isLoading } = useAuth();
  const location = useLocation();

  if (isLoading) {
    return loadingComponent || <LoadingSpinner />;
  }

  if (!isAuthenticated) {
    // Redirect to sign-in with return URL
    return <Navigate to={`${redirectTo}?returnTo=${location.pathname}`} />;
  }

  return <>{children}</>;
}
```

**Usage**:
```tsx
// Wrap protected content
<ProtectedRoute>
  <UrduTranslationToggle />
</ProtectedRoute>
```

---

### 8. UrduTranslationToggle Component (Enhanced)

**Purpose**: Language toggle that only shows for authenticated users.

**Location**: `book/src/components/UrduTranslationToggle.tsx`

**Props**: None (uses useAuth hook and i18n context)

**UI Structure**:
```tsx
function UrduTranslationToggle() {
  const { isAuthenticated } = useAuth();
  const { currentLocale, setLocale } = useDocusaurusContext();

  if (!isAuthenticated) {
    return (
      <Tooltip content="Sign in to access Urdu translation">
        <button className="urdu-toggle-disabled" disabled>
          <GlobeIcon /> اردو
        </button>
      </Tooltip>
    );
  }

  return (
    <button
      onClick={() => setLocale(currentLocale === 'en' ? 'ur' : 'en')}
      className="urdu-toggle"
      aria-label={`Switch to ${currentLocale === 'en' ? 'Urdu' : 'English'}`}
    >
      <GlobeIcon />
      {currentLocale === 'en' ? 'اردو' : 'English'}
    </button>
  );
}
```

**Behavior**:
- Show disabled state for unauthenticated users
- Toggle between 'en' and 'ur' locales for authenticated users
- Persist locale preference in localStorage
- Tooltip explains why disabled when not authenticated

---

### 9. ProfilePage Component

**Purpose**: Display user profile information.

**Location**: `book/src/pages/profile.tsx`

**Props**: None (uses useAuth hook)

**UI Structure**:
```tsx
function ProfilePage() {
  const { user, isLoading } = useAuth();

  if (isLoading) return <LoadingSpinner />;
  if (!user) return <Navigate to="/signin" />;

  return (
    <div className="profile-page">
      <h1>Profile</h1>

      <section className="profile-section">
        <h2>Account Information</h2>
        <dl>
          <dt>Email</dt>
          <dd>{user.email}</dd>

          <dt>Member since</dt>
          <dd>{new Date(user.created_at).toLocaleDateString()}</dd>

          <dt>Last login</dt>
          <dd>
            {user.last_login_at
              ? new Date(user.last_login_at).toLocaleDateString()
              : 'N/A'}
          </dd>
        </dl>
      </section>

      <section className="profile-section">
        <h2>Background</h2>
        <dl>
          <dt>Software Experience</dt>
          <dd>{user.software_background || 'Not specified'}</dd>

          <dt>Hardware Experience</dt>
          <dd>{user.hardware_background || 'Not specified'}</dd>
        </dl>
      </section>
    </div>
  );
}
```

---

## Component Testing Contracts

### Testing Requirements

All components must have:
1. **Unit tests**: Render without errors, handle props correctly
2. **Integration tests**: Interact with authentication context
3. **Accessibility tests**: WCAG 2.1 AA compliance
4. **Visual regression tests**: Maintain consistent styling

### Test Examples

```typescript
// SignUpForm.test.tsx
describe('SignUpForm', () => {
  it('renders all form fields', () => {
    render(<SignUpForm />);
    expect(screen.getByLabelText(/email/i)).toBeInTheDocument();
    expect(screen.getByLabelText(/^password$/i)).toBeInTheDocument();
    expect(screen.getByLabelText(/software experience/i)).toBeInTheDocument();
  });

  it('validates email format', async () => {
    const user = userEvent.setup();
    render(<SignUpForm />);

    await user.type(screen.getByLabelText(/email/i), 'invalid-email');
    await user.click(screen.getByRole('button', { name: /sign up/i }));

    expect(await screen.findByText(/invalid email/i)).toBeInTheDocument();
  });

  it('calls onSuccess when signup succeeds', async () => {
    const onSuccess = jest.fn();
    const user = userEvent.setup();

    render(<SignUpForm onSuccess={onSuccess} />);

    await user.type(screen.getByLabelText(/email/i), 'test@example.com');
    await user.type(screen.getByLabelText(/^password$/i), 'SecurePass123');
    await user.type(screen.getByLabelText(/confirm/i), 'SecurePass123');
    await user.click(screen.getByRole('button', { name: /sign up/i }));

    await waitFor(() => expect(onSuccess).toHaveBeenCalled());
  });
});

// AuthProvider.test.tsx
describe('AuthProvider', () => {
  it('provides authentication state to children', () => {
    render(
      <AuthProvider>
        <TestComponent />
      </AuthProvider>
    );

    expect(screen.getByText(/authenticated: false/i)).toBeInTheDocument();
  });

  it('fetches session on mount', async () => {
    const mockSession = { user: { email: 'test@example.com' } };
    mockFetch.mockResolvedValueOnce({ json: async () => mockSession });

    render(<AuthProvider><TestComponent /></AuthProvider>);

    await waitFor(() => {
      expect(screen.getByText(/authenticated: true/i)).toBeInTheDocument();
    });
  });
});
```

---

## Styling Guidelines

### CSS Classes

All components follow BEM naming convention:
- Block: `.auth-form`
- Element: `.auth-form__input`
- Modifier: `.auth-form__button--loading`

### Theme Integration

Components use Docusaurus CSS variables:
```css
.auth-form {
  background-color: var(--ifm-background-color);
  border: 1px solid var(--ifm-color-emphasis-300);
  border-radius: var(--ifm-global-radius);
  padding: var(--ifm-spacing-vertical) var(--ifm-spacing-horizontal);
}

.auth-form__button {
  background-color: var(--ifm-color-primary);
  color: var(--ifm-color-white);
  font-weight: var(--ifm-font-weight-semibold);
}
```

### Responsive Design

Mobile-first approach with breakpoints:
```css
/* Mobile (default) */
.auth-form {
  width: 100%;
  padding: 1rem;
}

/* Tablet */
@media (min-width: 768px) {
  .auth-form {
    max-width: 500px;
    margin: 0 auto;
    padding: 2rem;
  }
}

/* Desktop */
@media (min-width: 1024px) {
  .auth-form {
    max-width: 600px;
  }
}
```

---

## Accessibility Requirements

All components must:
1. Support keyboard navigation (Tab, Enter, Escape)
2. Include proper ARIA labels and roles
3. Provide focus indicators
4. Announce errors to screen readers (aria-live)
5. Have sufficient color contrast (WCAG AA: 4.5:1)
6. Support screen reader navigation

### Example Accessibility Features

```tsx
// Form with full accessibility
<form onSubmit={handleSubmit} aria-labelledby="form-title">
  <h2 id="form-title">Sign Up</h2>

  <label htmlFor="email">Email address</label>
  <input
    id="email"
    type="email"
    aria-required="true"
    aria-invalid={errors.email ? 'true' : 'false'}
    aria-describedby={errors.email ? 'email-error' : undefined}
  />
  {errors.email && (
    <p id="email-error" role="alert" aria-live="polite">
      {errors.email}
    </p>
  )}

  <button
    type="submit"
    disabled={isSubmitting}
    aria-busy={isSubmitting}
    aria-label="Submit sign up form"
  >
    {isSubmitting ? 'Creating account...' : 'Sign Up'}
  </button>
</form>
```

---

## Summary

All React components follow these principles:
- ✅ Type-safe with TypeScript
- ✅ Accessible (WCAG 2.1 AA)
- ✅ Responsive (mobile-first)
- ✅ Testable (unit + integration)
- ✅ Documented interfaces
- ✅ Consistent styling (Docusaurus theme)
- ✅ Error handling with user-friendly messages
- ✅ Loading states for async operations
