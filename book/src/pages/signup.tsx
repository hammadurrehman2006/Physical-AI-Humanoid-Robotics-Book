// book/src/pages/signup.tsx
import React from 'react';
import Layout from '@theme/Layout';
import SignUpForm from '../components/auth/SignUpForm';

function SignupPage() {
  return (
    <Layout title="Create Account" description="Sign up for an account">
      <div className="container mx-auto px-4 py-8 max-w-md">
        <div className="bg-white p-8 rounded-lg shadow-md">
          <h1 className="text-2xl font-bold text-gray-900 mb-6">Create Account</h1>
          <SignUpForm
            onSuccess={() => {
              // Redirect to homepage or show success message
              window.location.href = '/';
            }}
          />
          <div className="mt-4 text-center">
            <p className="text-sm text-gray-600">
              Already have an account?{' '}
              <a href="/signin" className="font-medium text-blue-600 hover:text-blue-500">
                Sign in
              </a>
            </p>
          </div>
        </div>
      </div>
    </Layout>
  );
}

export default SignupPage;