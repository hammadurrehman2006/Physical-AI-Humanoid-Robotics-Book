// book/src/components/auth/AuthNavbarItems.tsx
import React from 'react';
import { useAuth } from '../../auth/hooks/useAuth';
import Link from '@docusaurus/Link';
import { LogOut } from 'lucide-react';

const AuthNavbarItems = () => {
  const { isAuthenticated, user, isLoading, signOut } = useAuth();

  if (isLoading) {
    // Show a loading state while checking auth status
    return (
      <div className="navbar__item">
        <div className="w-8 h-8 rounded-full bg-slate-200 animate-pulse"></div>
      </div>
    );
  }

  if (isAuthenticated && user) {
    // Show user avatar dropdown when authenticated
    return (
      <div className="navbar__item dropdown dropdown--nocaret dropdown--right">
        <button
          className="clean-btn navbar__link p-0 flex items-center justify-center"
          aria-label="User menu"
          aria-haspopup="true"
          aria-expanded="false"
        >
          <div className="avatar">
             {user.image ? (
                <img
                  className="avatar__photo avatar__photo--sm"
                  src={user.image}
                  alt={user.name || 'User'}
                />
              ) : (
                <div className="avatar__photo avatar__photo--sm flex items-center justify-center bg-slate-200 text-slate-600 font-bold">
                  {(user.name ? user.name.charAt(0) : (user.email ? user.email.charAt(0) : 'U')).toUpperCase()}
                </div>
              )}
          </div>
        </button>
        <ul className="dropdown__menu">
          <li>
            <div className="dropdown__header text-xs text-gray-500 uppercase tracking-wider font-semibold px-4 py-2">
              {user.name || user.email}
            </div>
          </li>
          <li>
            <Link className="dropdown__link" to="/profile">
              Profile
            </Link>
          </li>
          <div className="dropdown__divider" />
          <li>
            <a
              className="dropdown__link flex items-center gap-2 text-red-600 hover:text-red-700"
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
              <LogOut size={16} />
              Logout
            </a>
          </li>
        </ul>
      </div>
    );
  } else {
    // Show sign in button when not authenticated
    return (
      <div className="navbar__item">
         <Link className="button button--primary button--sm" to="/signin">
            Sign In
          </Link>
      </div>
    );
  }
};

export default AuthNavbarItems;