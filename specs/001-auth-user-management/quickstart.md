# Quickstart Guide: Authentication System

**Feature**: 001-auth-user-management
**Date**: 2026-01-04
**Estimated Setup Time**: 30-45 minutes

## Prerequisites

Before starting, ensure you have:
- Node.js 18+ installed
- NeonDB account (free tier sufficient)
- Existing Docusaurus project at `/book`
- Git repository initialized

## Step-by-Step Setup

### Phase 1: Database Setup (10 minutes)

#### 1.1 Create NeonDB Database

```bash
# Visit https://console.neon.tech/
# 1. Create new project: "physical-ai-book-auth"
# 2. Copy connection string from dashboard
# 3. Save to .env.local
```

#### 1.2 Configure Environment Variables

Create `book/.env.local`:
```env
# NeonDB connection
DATABASE_URL="postgresql://user:password@ep-example.us-east-2.aws.neon.tech/neondb?sslmode=require"

# Better-auth secrets
BETTER_AUTH_SECRET="your-random-secret-min-32-chars"
BETTER_AUTH_URL="http://localhost:3000"
BETTER_AUTH_TRUSTED_ORIGINS="http://localhost:3000"

# Node environment
NODE_ENV="development"
```

Generate secret:
```bash
node -e "console.log(require('crypto').randomBytes(32).toString('base64'))"
```

### Phase 2: Install Dependencies (5 minutes)

```bash
cd book

# Core dependencies
npm install better-auth @neondatabase/serverless drizzle-orm ws

# Dev dependencies
npm install -D drizzle-kit @types/ws

# Validation
npm install zod

# Testing (optional)
npm install -D @testing-library/react @testing-library/user-event @playwright/test
```

### Phase 3: Setup Database Schema (10 minutes)

#### 3.1 Create Drizzle Config

Create `book/drizzle.config.ts`:
```typescript
import { defineConfig } from 'drizzle-kit';

export default defineConfig({
  schema: './src/auth/db/schema.ts',
  out: './src/auth/db/migrations',
  dialect: 'postgresql',
  dbCredentials: {
    url: process.env.DATABASE_URL!,
  },
});
```

#### 3.2 Create Database Schema

Create `book/src/auth/db/schema.ts`:
```typescript
import { pgTable, uuid, varchar, timestamp, pgEnum } from 'drizzle-orm/pg-core';

export const softwareBackgroundEnum = pgEnum('software_background', [
  'Beginner', 'Intermediate', 'Advanced', 'Expert'
]);

export const hardwareBackgroundEnum = pgEnum('hardware_background', [
  'No Experience', 'Basic Knowledge', 'Hands-on Experience', 'Professional Experience'
]);

export const users = pgTable('users', {
  id: uuid('id').primaryKey().defaultRandom(),
  email: varchar('email', { length: 255 }).notNull().unique(),
  password_hash: varchar('password_hash', { length: 255 }).notNull(),
  software_background: softwareBackgroundEnum('software_background'),
  hardware_background: hardwareBackgroundEnum('hardware_background'),
  created_at: timestamp('created_at').notNull().defaultNow(),
  last_login_at: timestamp('last_login_at'),
});

export const sessions = pgTable('sessions', {
  id: uuid('id').primaryKey().defaultRandom(),
  user_id: uuid('user_id').notNull().references(() => users.id, { onDelete: 'cascade' }),
  token_hash: varchar('token_hash', { length: 255 }).notNull().unique(),
  expires_at: timestamp('expires_at').notNull(),
  created_at: timestamp('created_at').notNull().defaultNow(),
  ip_address: varchar('ip_address', { length: 45 }),
  user_agent: varchar('user_agent', { length: 1000 }),
});
```

#### 3.3 Generate and Run Migrations

```bash
# Generate migration
npx drizzle-kit generate

# Push to database
npx drizzle-kit push
```

### Phase 4: Configure Better-Auth (10 minutes)

#### 4.1 Create Database Connection

Create `book/src/auth/db/client.ts`:
```typescript
import { Pool, neonConfig } from '@neondatabase/serverless';
import { drizzle } from 'drizzle-orm/neon-serverless';
import ws from 'ws';
import * as schema from './schema';

// Configure WebSocket (Node.js < v22)
neonConfig.webSocketConstructor = ws;

const pool = new Pool({ connectionString: process.env.DATABASE_URL });
export const db = drizzle(pool, { schema });
```

#### 4.2 Create Better-Auth Instance

Create `book/src/auth/auth.ts`:
```typescript
import { betterAuth } from 'better-auth';
import { drizzleAdapter } from 'better-auth/adapters/drizzle';
import { db } from './db/client';
import * as schema from './db/schema';

export const auth = betterAuth({
  baseURL: process.env.BETTER_AUTH_URL || 'http://localhost:3000',
  secret: process.env.BETTER_AUTH_SECRET!,
  trustedOrigins: [process.env.BETTER_AUTH_TRUSTED_ORIGINS || 'http://localhost:3000'],

  database: drizzleAdapter(db, {
    provider: 'pg',
    schema: schema,
  }),

  emailAndPassword: {
    enabled: true,
    minPasswordLength: 8,
    requireEmailVerification: false, // MVP
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
    max: 5,
  },
});

export type Session = typeof auth.$Infer.Session;
```

### Phase 5: Setup API Routes (5 minutes)

#### 5.1 Create Auth Plugin

Create `book/plugins/auth-plugin/index.js`:
```javascript
const { auth } = require('../../src/auth/auth.ts');

module.exports = function authPlugin(context, options) {
  return {
    name: 'auth-plugin',
    async contentLoaded({ actions }) {
      const { addRoute } = actions;

      // Mount better-auth handlers
      addRoute({
        path: '/api/auth/*',
        component: '@site/src/pages/api/auth/[[...all]].tsx',
      });
    },

    configureWebpack(config) {
      return {
        plugins: [
          // Add auth API middleware
          {
            apply: (compiler) => {
              compiler.hooks.beforeRun.tap('AuthPlugin', () => {
                console.log('✓ Authentication API enabled at /api/auth/*');
              });
            },
          },
        ],
      };
    },
  };
};
```

#### 5.2 Create API Handler

Create `book/src/pages/api/auth/[[...all]].tsx`:
```typescript
import { auth } from '../../../auth/auth';

export default async function handler(req, res) {
  return auth.handler(req, res);
}

export const config = {
  api: {
    bodyParser: true,
  },
};
```

#### 5.3 Register Plugin

Update `book/docusaurus.config.js`:
```javascript
module.exports = {
  // ... existing config
  plugins: [
    './plugins/auth-plugin',
    // ... other plugins
  ],
};
```

### Phase 6: Setup React Context (Remaining time)

#### 6.1 Create Auth Client

Create `book/src/auth/client.ts`:
```typescript
import { createAuthClient } from 'better-auth/react';
import { inferAdditionalFields } from 'better-auth/client/plugins';

export const authClient = createAuthClient({
  baseURL: process.env.BETTER_AUTH_URL || 'http://localhost:3000',
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
```

#### 6.2 Create Auth Provider

Create `book/src/auth/AuthProvider.tsx`:
```typescript
import React, { createContext, useContext } from 'react';
import { useSession } from './client';

const AuthContext = createContext(null);

export function AuthProvider({ children }) {
  const { data: session, isLoading } = useSession();

  const value = {
    session,
    isLoading,
    isAuthenticated: !!session?.user,
  };

  return <AuthContext.Provider value={value}>{children}</AuthContext.Provider>;
}

export function useAuth() {
  const context = useContext(AuthContext);
  if (!context) throw new Error('useAuth must be used within AuthProvider');
  return context;
}
```

#### 6.3 Swizzle Root Component

```bash
npm run swizzle @docusaurus/theme-classic Root -- --wrap
```

Update `book/src/theme/Root.tsx`:
```typescript
import React from 'react';
import { AuthProvider } from '../auth/AuthProvider';

export default function Root({ children }) {
  return <AuthProvider>{children}</AuthProvider>;
}
```

### Phase 7: Test Installation

```bash
# Start Docusaurus
npm run start

# In browser, visit:
# - http://localhost:3000/api/auth/session (should return null)
# - Check console for "✓ Authentication API enabled"
```

## Verification Checklist

- [ ] NeonDB database created and connection string set
- [ ] Environment variables configured in `.env.local`
- [ ] Dependencies installed (better-auth, @neondatabase/serverless, drizzle-orm)
- [ ] Database schema created and migrations run
- [ ] Better-auth instance configured with custom fields
- [ ] Auth plugin registered in docusaurus.config.js
- [ ] API handler created at `/api/auth/*`
- [ ] Auth client and provider created
- [ ] Root component swizzled and wrapped with AuthProvider
- [ ] Dev server starts without errors
- [ ] API endpoint `/api/auth/session` accessible

## Common Issues & Solutions

### Issue: "Cannot find module 'ws'"
```bash
npm install ws @types/ws
```

### Issue: Database connection fails
- Check DATABASE_URL in .env.local
- Ensure NeonDB project is active (not paused)
- Verify SSL mode is set: `?sslmode=require`

### Issue: "BETTER_AUTH_SECRET is required"
- Generate new secret: `node -e "console.log(require('crypto').randomBytes(32).toString('base64'))"`
- Add to .env.local
- Restart dev server

### Issue: Migrations fail
```bash
# Reset and try again
npx drizzle-kit drop
npx drizzle-kit generate
npx drizzle-kit push
```

## Next Steps

Once setup is complete:
1. Implement UI components (SignUpForm, SignInForm, UserMenu)
2. Create authentication pages (/signup, /signin, /profile)
3. Integrate Urdu translation toggle with auth check
4. Add tests for authentication flows
5. Deploy to production with updated environment variables

## Useful Commands

```bash
# Generate migration after schema changes
npx drizzle-kit generate

# Apply migrations
npx drizzle-kit push

# Inspect database
npx drizzle-kit studio

# Run tests
npm run test

# Build for production
npm run build

# Check types
npx tsc --noEmit
```

## Resources

- Better-Auth Docs: https://better-auth.com/docs
- NeonDB Docs: https://neon.tech/docs
- Drizzle ORM Docs: https://orm.drizzle.team/docs
- Docusaurus Docs: https://docusaurus.io/docs

## Support

If you encounter issues:
1. Check the troubleshooting section above
2. Review logs in terminal for specific errors
3. Verify all environment variables are set correctly
4. Ensure database migrations completed successfully
5. Check browser console for client-side errors
