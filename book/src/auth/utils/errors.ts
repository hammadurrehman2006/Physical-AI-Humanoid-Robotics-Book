// Authentication error interface
export interface AuthError {
  code: string;
  message: string;
  field?: string;
}

// Authentication error constants
export const AUTH_ERRORS = {
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
    message: 'Password does not meet complexity requirements',
  },
  SESSION_EXPIRED: {
    code: 'AUTH004',
    message: 'Your session has expired, please sign in again',
  },
  RATE_LIMIT_EXCEEDED: {
    code: 'AUTH005',
    message: 'Too many attempts, please try again later',
  },
  UNAUTHORIZED: {
    code: 'AUTH006',
    message: 'You must be signed in to access this feature',
  },
} as const;

// Error factory function
export function createAuthError(code: string, message: string, field?: string): AuthError {
  return { code, message, field };
}

// Error checking function
export function isAuthError(error: any): error is AuthError {
  return error && typeof error.code === 'string' && typeof error.message === 'string';
}