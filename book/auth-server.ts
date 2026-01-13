import express from 'express';
import cors from 'cors';
import { toNodeHandler } from 'better-auth/node';
import { auth } from './src/auth/auth';
import dotenv from 'dotenv';

dotenv.config();

const app = express();
const PORT = 3001; // Auth server port

const allowedOrigins = process.env.TRUSTED_ORIGINS 
    ? process.env.TRUSTED_ORIGINS.split(',') 
    : ['http://localhost:3000'];

app.use(cors({
    origin: allowedOrigins,
    credentials: true,
    methods: ['GET', 'POST', 'PUT', 'DELETE', 'OPTIONS'],
    allowedHeaders: ['Content-Type', 'Authorization', 'Cookie']
}));

// Better Auth Handler
app.all('/api/auth/*', toNodeHandler(auth));

app.listen(PORT, () => {
    console.log(`Auth server running on http://localhost:${PORT}`);
    console.log(`Better Auth Base URL: ${auth.options.baseURL}`);
});
