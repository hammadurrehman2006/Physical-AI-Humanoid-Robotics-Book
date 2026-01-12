import { createAuthClient } from 'better-auth/react';
import { inferAdditionalFields } from 'better-auth/client/plugins';

export const authClient = createAuthClient({
  baseURL: process.env.BETTER_AUTH_URL || 'http://localhost:3001', // Point to Auth Server
  plugins: [
    inferAdditionalFields({
      user: {
        software_background: { type: 'string', required: false },
        hardware_background: { type: 'string', required: false },
      },
    }),
  ],
});

export const { signIn, signUp, signOut, useSession } = authClient;
