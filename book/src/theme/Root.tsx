// book/src/theme/Root.tsx
import React from 'react';
import { AuthProvider, useAuth } from '../auth/AuthProvider';
import { useLocation } from '@docusaurus/router';

function UrduAuthWrapper({ children }) {
  const { pathname } = useLocation();
  const { isAuthenticated, isLoading } = useAuth();

  // Check if this is a Urdu locale route (starts with /ur/)
  const isUrduRoute = pathname.startsWith('/ur');

  // If it's a Urdu route and user is not authenticated, show access denied
  if (isUrduRoute && !isAuthenticated && !isLoading) {
    return (
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
    );
  }

  // If loading, show a loading state
  if (isUrduRoute && isLoading) {
    return (
      <div className="container mx-auto px-4 py-8">
        <div className="max-w-2xl mx-auto text-center">
          <div className="bg-white p-8 rounded-lg shadow-md">
            <h1 className="text-2xl font-bold text-gray-900 mb-4">Loading...</h1>
            <p>Please wait while we verify your access.</p>
          </div>
        </div>
      </div>
    );
  }

  // If it's not a Urdu route or user is authenticated, render normally
  return <>{children}</>;
}

export default function Root({ children }) {
  return (
    <AuthProvider>
      <UrduAuthWrapper>
        {children}
      </UrduAuthWrapper>
    </AuthProvider>
  );
}