import { Pool } from 'pg';
import { drizzle } from 'drizzle-orm/node-postgres';
import * as schema from './schema';
import dotenv from 'dotenv';

dotenv.config({ path: '../../.env' }); // Load from root or book root
if (!process.env.DATABASE_URL) {
  dotenv.config();
}

const connectionString = process.env.DATABASE_URL || 'postgres://postgres:postgres@localhost:5432/chatbot_db';

const pool = new Pool({ connectionString });
export const db = drizzle(pool, { schema });