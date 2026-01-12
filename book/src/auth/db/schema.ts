import { pgTable, pgEnum, uuid, varchar, timestamp, boolean } from 'drizzle-orm/pg-core';

// Define enums for software and hardware background
export const softwareBackgroundEnum = pgEnum('software_background', [
  'Beginner',
  'Intermediate',
  'Advanced',
  'Expert'
]);

export const hardwareBackgroundEnum = pgEnum('hardware_background', [
  'No Experience',
  'Basic Knowledge',
  'Hands-on Experience',
  'Professional Experience'
]);

// Users table
export const users = pgTable('users', {
  id: uuid('id').primaryKey().defaultRandom(),
  email: varchar('email', { length: 255 }).notNull().unique(),
  password_hash: varchar('password_hash', { length: 255 }).notNull(),
  software_background: softwareBackgroundEnum('software_background'),
  hardware_background: hardwareBackgroundEnum('hardware_background'),
  created_at: timestamp('created_at').notNull().defaultNow(),
  last_login_at: timestamp('last_login_at'),
});

// Sessions table
export const sessions = pgTable('sessions', {
  id: uuid('id').primaryKey().defaultRandom(),
  user_id: uuid('user_id').notNull().references(() => users.id, { onDelete: 'cascade' }),
  token_hash: varchar('token_hash', { length: 255 }).notNull().unique(),
  expires_at: timestamp('expires_at').notNull(),
  created_at: timestamp('created_at').notNull().defaultNow(),
  ip_address: varchar('ip_address', { length: 45 }),
  user_agent: varchar('user_agent', { length: 1000 }),
});