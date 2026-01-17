// book/src/pages/signin.tsx
import React from 'react';
import Layout from '@theme/Layout';
import SignInForm from '../components/auth/SignInForm';

function SigninPage() {
  return (
    <Layout title="Sign In" description="Sign in to your account">
      <div className="container mx-auto px-4 min-h-screen flex flex-col items-center">
        <div className="w-full max-w-md my-auto bg-white dark:bg-gray-800 p-8 rounded-lg shadow-md">
          <h1 className="text-2xl font-bold text-gray-900 dark:text-white mb-6">Sign In</h1>
          <SignInForm
            onSuccess={() => {
              // Redirect to homepage or show success message
              window.location.href = '/';
            }}
          />
          <div className="mt-4 text-center">
            <p className="text-sm text-gray-600 dark:text-gray-400">
              Don't have an account?{' '}
              <a href="/signup" className="font-medium text-blue-600 hover:text-blue-500 dark:text-blue-400 dark:hover:text-blue-300">
                Sign up
              </a>
            </p>
          </div>
        </div>
      </div>
    </Layout>
  );
}

export default SigninPage;