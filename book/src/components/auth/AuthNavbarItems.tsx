// book/src/components/auth/AuthNavbarItems.tsx
import React from 'react';
import { useAuth } from '../../auth/hooks/useAuth';
import Link from '@docusaurus/Link';

const AuthNavbarItems = () => {
  const { isAuthenticated, user, isLoading, signOut } = useAuth();

  if (isLoading) {
    // Show a loading state while checking auth status
    return (
      <div className="navbar__item">
        <span>Loading...</span>
      </div>
    );
  }

  if (isAuthenticated && user) {
    // Show user menu when authenticated
    return (
      <div className="navbar__item dropdown dropdown--nocaret dropdown--right">
        <button
          className="button button--secondary navbar__link"
          aria-label="User menu"
          aria-haspopup="true"
          aria-expanded="false"
        >
          {user.email}
        </button>
        <ul className="dropdown__menu">
          <li>
            <Link className="dropdown__link" to="/profile">
              Profile
            </Link>
          </li>
          <li>
            <a
              className="dropdown__link"
              href="#"
              onClick={async (e) => {
                e.preventDefault();
                try {
                  await signOut();
                  // Refresh the page to update the UI after signout
                  window.location.reload();
                } catch (error) {
                  console.error('Error signing out:', error);
                  // Refresh the page to update the UI after signout
                  window.location.reload();
                }
              }}
            >
              Sign Out
            </a>
          </li>
        </ul>
      </div>
    );
  } else {
    // Show sign in/up buttons when not authenticated
    return (
      <div className="navbar__item">
        <div className="button-group">
          <Link className="button button--secondary" to="/signin">
            Sign In
          </Link>
          <Link className="button button--primary ml-2" to="/signup">
            Sign Up
          </Link>
        </div>
      </div>
    );
  }
};

export default AuthNavbarItems;