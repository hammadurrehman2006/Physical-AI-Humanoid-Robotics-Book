// book/src/theme/UrduContentWrapper.tsx
import React from 'react';
import { useAuth } from '../src/auth/hooks/useAuth';
import Layout from '@theme/Layout';

interface UrduContentWrapperProps {
  children: React.ReactNode;
}

const UrduContentWrapper: React.FC<UrduContentWrapperProps> = ({ children }) => {
  const { isAuthenticated, isLoading } = useAuth();

  if (isLoading) {
    // Show loading state while checking authentication
    return (
      <Layout title="Loading..." description="Checking authentication status">
        <div className="container mx-auto px-4 py-8">
          <div className="max-w-2xl mx-auto text-center">
            <div className="bg-white p-8 rounded-lg shadow-md">
              <h1 className="text-2xl font-bold text-gray-900 mb-4">Loading...</h1>
              <p>Please wait while we verify your access.</p>
            </div>
          </div>
        </div>
      </Layout>
    );
  }

  if (!isAuthenticated) {
    // Show access denied message with sign in option
    return (
      <Layout title="Access Denied" description="Sign in to access Urdu content">
        <div className="container mx-auto px-4 py-8">
          <div className="max-w-2xl mx-auto">
            <div className="bg-white p-8 rounded-lg shadow-md text-center">
              <h1 className="text-2xl font-bold text-gray-900 mb-4">Urdu Content Access</h1>
              <p className="mb-6">
                The Urdu translation feature is available to registered users only. Please sign in to access Urdu content.
              </p>
              <div className="flex flex-col sm:flex-row justify-center gap-4">
                <a
                  href="/signin"
                  className="button button--primary px-6 py-3"
                >
                  Sign In
                </a>
                <a
                  href="/signup"
                  className="button button--secondary px-6 py-3"
                >
                  Sign Up
                </a>
              </div>
            </div>
          </div>
        </div>
      </Layout>
    );
  }

  // If authenticated, render the original content
  return <>{children}</>;
};

export default UrduContentWrapper;