# Deployment Instructions

## Overview

The application consists of two main parts that need to be deployed:

1.  **Frontend (Docusaurus):** A static website.
2.  **Auth Server (Node.js/Express):** Handles authentication API requests.

(Optional) **Backend (Python/FastAPI):** Handles AI/RAG tasks (see `backend/README.md`).

## Prerequisites

-   **PostgreSQL Database:** You need a Postgres database (e.g., Neon, AWS RDS, Supabase, or self-hosted).
-   **Environment Variables:** Configure the following secrets in your deployment environment.

## 1. Database Setup

Ensure your Postgres database is ready. Get the connection string.

Run migrations from the `book` directory to set up the database schema:

```bash
cd book
npm run db:push
```

*Note: This requires `DATABASE_URL` to be set in `.env` or passed to the command.*

## 2. Auth Server Deployment

The Auth Server is located in `book/auth-server.ts`. It acts as the API for authentication.

**Deployment Steps:**

1.  **Build/Prepare:** Ensure `package.json` dependencies are installed (`better-auth`, `express`, `pg`, `drizzle-orm`, etc.).
2.  **Start Command:** The server runs via `tsx` (TypeScript Execute) or can be transpiled.
    *   Development/Simple: `npx tsx auth-server.ts`
    *   Production: You may want to compile it or use `ts-node`/`tsx` directly if supported by your host.
    *   Example `start` script for Auth Server: `tsx auth-server.ts`
3.  **Environment Variables:**

    | Variable | Description | Example |
    | :--- | :--- | :--- |
    | `DATABASE_URL` | Postgres connection string | `postgres://user:pass@host:5432/db` |
    | `BETTER_AUTH_SECRET` | Secret key for encryption (min 32 chars) | `your-super-secret-key...` |
    | `BETTER_AUTH_URL` | Public URL of this Auth Server | `https://auth.yourdomain.com` |
    | `PORT` | Port to run on (default 3001) | `3001` |

## 3. Frontend Deployment (Docusaurus)

The frontend is a static site generated in `book/build`.

**Deployment Steps:**

1.  **Build:**
    ```bash
    cd book
    npm run build
    ```
    *Important: During build time, `BETTER_AUTH_URL` must be available in the environment to point the frontend to the Auth Server.*

2.  **Environment Variables (Build Time):**

    | Variable | Description |
    | :--- | :--- |
    | `BETTER_AUTH_URL` | The URL where your Auth Server is deployed (e.g., `https://auth.yourdomain.com`). This is baked into the static files. |

3.  **Deploy:** Upload the `book/build` directory to any static hosting service (Vercel, Netlify, GitHub Pages, AWS S3, etc.).

    *   **Vercel:** Vercel can handle the build automatically. Set `npm run build` as the build command and `build` as the output directory. Add `BETTER_AUTH_URL` to Vercel project settings.

## Summary of URLs

-   **Frontend:** `https://your-site.com` (User visits this)
-   **Auth Server:** `https://auth.your-site.com` (Frontend calls this API)
-   **Database:** `postgres://...` (Both Auth Server and Backend connect here)

## Verification

1.  Visit the Frontend URL.
2.  Click "Sign In".
3.  The request should go to `https://auth.your-site.com/api/auth/...`.
4.  If successful, you should be logged in.
