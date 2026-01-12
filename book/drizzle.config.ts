import { defineConfig } from 'drizzle-kit';

export default defineConfig({
  schema: './src/auth/db/schema.ts',
  out: './src/auth/db/migrations',
  dialect: 'postgresql',
  dbCredentials: {
    url: process.env.DATABASE_URL || '',
  },
});