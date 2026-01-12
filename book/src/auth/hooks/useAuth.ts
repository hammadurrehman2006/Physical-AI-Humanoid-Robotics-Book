// book/src/auth/hooks/useAuth.ts
import { useAuth as useAuthProvider } from '../AuthProvider';

/**
 * Custom hook that wraps the AuthProvider context
 * Provides authentication state and user information
 *
 * @returns {Object} Authentication context with user, isAuthenticated, isLoading, and error
 * @throws {Error} If used outside of AuthProvider
 */
export const useAuth = () => {
  const context = useAuthProvider();

  if (!context) {
    throw new Error('useAuth must be used within an AuthProvider');
  }

  return context;
};