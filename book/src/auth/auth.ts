import { betterAuth } from 'better-auth';
import { drizzleAdapter } from 'better-auth/adapters/drizzle';
import { db } from './db/client';
import * as schema from './db/schema';

export const auth = betterAuth({
  baseURL: process.env.BETTER_AUTH_URL || 'http://localhost:3001', // Running on port 3001
  secret: process.env.BETTER_AUTH_SECRET || 'your-super-secret-jwt-secret-change-in-production-min-32-chars',
  trustedOrigins: [
    'http://localhost:3000', // Frontend
    'http://localhost:3001'  // Self
  ],
  
  database: drizzleAdapter(db, {
    provider: 'pg',
    schema: {
        ...schema,
        user: schema.users,
        session: schema.sessions
    },
  }),

  emailAndPassword: {
    enabled: true,
    minPasswordLength: 8,
    requireEmailVerification: false,
  },

  user: {
    additionalFields: {
      software_background: { type: 'string', required: false },
      hardware_background: { type: 'string', required: false },
    },
  },

  session: {
    expiresIn: 60 * 60 * 24 * 7, // 7 days
    updateAge: 60 * 60 * 24, // Refresh daily
  },

  rateLimit: {
    enabled: true,
    window: 60 * 15, // 15 minutes
    max: 100, // Increased for dev
  },
});
