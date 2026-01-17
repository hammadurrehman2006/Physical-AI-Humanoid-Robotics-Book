// book/src/auth/AuthProvider.tsx
import React, { createContext, useContext, ReactNode, useEffect, useState, useCallback } from 'react';
import { signIn as apiSignIn, signUp as apiSignUp, signOut as apiSignOut, getUser, User } from './client';

interface AuthContextType {
  user: User | null;
  isAuthenticated: boolean;
  isLoading: boolean;
  error: string | null;
  signIn: (data: any) => Promise<void>;
  signUp: (data: any) => Promise<void>;
  signOut: () => Promise<void>;
}

const AuthContext = createContext<AuthContextType | undefined>(undefined);

interface AuthProviderProps {
  children: ReactNode;
}

export const AuthProvider: React.FC<AuthProviderProps> = ({ children }) => {
  const [user, setUser] = useState<User | null>(null);
  const [isLoading, setIsLoading] = useState<boolean>(true);
  const [error, setError] = useState<string | null>(null);

  const fetchUser = useCallback(async () => {
    try {
      setIsLoading(true);
      const userData = await getUser();
      setUser(userData);
    } catch (err) {
      setUser(null);
    } finally {
      setIsLoading(false);
    }
  }, []);

  useEffect(() => {
    fetchUser();
  }, [fetchUser]);

  const signIn = async (data: any) => {
    setIsLoading(true);
    setError(null);
    try {
      await apiSignIn(data);
      // After login, fetch the user details
      await fetchUser();
    } catch (err: any) {
      setError(err.message || 'Failed to sign in');
      setIsLoading(false);
      throw err;
    }
  };

  const signUp = async (data: any) => {
    setIsLoading(true);
    setError(null);
    try {
      await apiSignUp(data);
      // Depending on config, user might be auto-logged in or need to verify email
      // For now, let's try to login automatically or just redirect to login
      // If the backend auto-logs in (sets cookie), fetching user should work.
      // fastapi-users /register usually doesn't auto-login unless configured.
      // So the user usually needs to sign in after registering.
      // But we can leave the state as is and let the component handle redirection.
      setIsLoading(false); 
    } catch (err: any) {
      setError(err.message || 'Failed to sign up');
      setIsLoading(false);
      throw err;
    }
  };

  const signOut = async () => {
    setIsLoading(true);
    try {
      await apiSignOut();
      setUser(null);
    } catch (err: any) {
      setError(err.message || 'Failed to sign out');
    } finally {
      setIsLoading(false);
    }
  };

  const value: AuthContextType = {
    user,
    isAuthenticated: !!user,
    isLoading,
    error,
    signIn,
    signUp,
    signOut,
  };

  return <AuthContext.Provider value={value}>{children}</AuthContext.Provider>;
};

const useAuth = () => {
  const context = useContext(AuthContext);
  if (context === undefined) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
};

export { useAuth };