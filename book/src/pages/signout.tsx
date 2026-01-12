// book/src/pages/signout.tsx
import React, { useEffect } from 'react';
import { signOut } from '../auth/client';
import Layout from '@theme/Layout';

function SignoutPage() {
  useEffect(() => {
    const handleSignOut = async () => {
      try {
        await signOut();
        // Redirect to homepage after signout
        window.location.href = '/';
      } catch (error) {
        console.error('Error signing out:', error);
        // Even if there's an error, redirect to homepage
        window.location.href = '/';
      }
    };

    handleSignOut();
  }, []);

  return (
    <Layout title="Signing Out" description="You are being signed out">
      <div className="container mx-auto px-4 py-8">
        <div className="max-w-md mx-auto">
          <div className="bg-white p-8 rounded-lg shadow-md text-center">
            <h1 className="text-2xl font-bold text-gray-900 mb-4">Signing Out</h1>
            <p>Processing your sign out request...</p>
          </div>
        </div>
      </div>
    </Layout>
  );
}

export default SignoutPage;