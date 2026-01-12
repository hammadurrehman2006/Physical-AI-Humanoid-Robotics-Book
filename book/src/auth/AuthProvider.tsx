// book/src/auth/AuthProvider.tsx
import React, { createContext, useContext, ReactNode } from 'react';
import { useSession, signIn, signUp, signOut } from './client';

interface AuthContextType {
  user: any;
  isAuthenticated: boolean;
  isLoading: boolean;
  error: string | null;
  signIn: typeof signIn;
  signUp: typeof signUp;
  signOut: typeof signOut;
}

const AuthContext = createContext<AuthContextType | undefined>(undefined);

interface AuthProviderProps {
  children: ReactNode;
}

export const AuthProvider: React.FC<AuthProviderProps> = ({ children }) => {
  const { data: session, isLoading, error } = useSession();

  const value: AuthContextType = {
    user: session?.user || null,
    isAuthenticated: !!session?.user,
    isLoading,
    error: error ? (error as any).message || 'An error occurred' : null,
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