// book/src/pages/profile.tsx
import React from 'react';
import Layout from '@theme/Layout';
import { useAuth } from '../auth/hooks/useAuth';

function ProfilePage() {
  const { user, isAuthenticated, isLoading } = useAuth();

  if (isLoading) {
    return (
      <Layout title="Profile" description="Loading your profile information">
        <div className="container mx-auto px-4 py-8">
          <div className="max-w-2xl mx-auto">
            <div className="bg-white p-8 rounded-lg shadow-md">
              <h1 className="text-2xl font-bold text-gray-900 mb-6">Profile</h1>
              <p>Loading...</p>
            </div>
          </div>
        </div>
      </Layout>
    );
  }

  if (!isAuthenticated || !user) {
    return (
      <Layout title="Profile" description="Please sign in to view your profile">
        <div className="container mx-auto px-4 py-8">
          <div className="max-w-2xl mx-auto">
            <div className="bg-white p-8 rounded-lg shadow-md">
              <h1 className="text-2xl font-bold text-gray-900 mb-6">Profile Access</h1>
              <p className="mb-4">Please sign in to view your profile information.</p>
              <a href="/signin" className="button button--primary">Sign In</a>
            </div>
          </div>
        </div>
      </Layout>
    );
  }

  return (
    <Layout title="Profile" description="Your profile information">
      <div className="container mx-auto px-4 py-8">
        <div className="max-w-2xl mx-auto">
          <div className="bg-white p-8 rounded-lg shadow-md">
            <h1 className="text-2xl font-bold text-gray-900 mb-6">Your Profile</h1>

            <div className="space-y-4">
              <div>
                <label className="block text-sm font-medium text-gray-700 mb-1">Email</label>
                <p className="text-gray-900">{user.email}</p>
              </div>

              {user.software_background && (
                <div>
                  <label className="block text-sm font-medium text-gray-700 mb-1">Software Background</label>
                  <p className="text-gray-900">{user.software_background}</p>
                </div>
              )}

              {user.hardware_background && (
                <div>
                  <label className="block text-sm font-medium text-gray-700 mb-1">Hardware Background</label>
                  <p className="text-gray-900">{user.hardware_background}</p>
                </div>
              )}

              <div>
                <label className="block text-sm font-medium text-gray-700 mb-1">Member Since</label>
                <p className="text-gray-900">{user.created_at ? new Date(user.created_at).toLocaleDateString() : 'Unknown'}</p>
              </div>
            </div>
          </div>
        </div>
      </div>
    </Layout>
  );
}

export default ProfilePage;