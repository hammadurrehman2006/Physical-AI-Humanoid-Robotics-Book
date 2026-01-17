# Deployment Instructions

## Overview

The application consists of two main parts that need to be deployed:

1.  **Frontend (Docusaurus):** A static website.
2.  **Backend (Python/FastAPI):** Handles authentication and AI/RAG tasks (see `backend/README.md`).

## Prerequisites

-   **PostgreSQL Database:** You need a Postgres database (e.g., Neon, AWS RDS, Supabase, or self-hosted).
-   **Environment Variables:** Configure the following secrets in your deployment environment.

## 1. Database Setup

Ensure your Postgres database is ready. Get the connection string.

Run migrations from the `backend` directory to set up the database schema:

```bash
cd backend
alembic upgrade head
```

*Note: This requires `DATABASE_URL` to be set in `.env` or passed to the command.*

## 2. Backend Deployment (Python/FastAPI)

The Backend is located in the `backend/` directory.

**Deployment Steps:**

1.  **Build/Prepare:** Ensure `requirements.txt` dependencies are installed.
2.  **Start Command:** The server runs via `uvicorn`.
    *   Command: `uvicorn app.main:app --host 0.0.0.0 --port 8000`
3.  **Environment Variables:**

    | Variable | Description | Example |
    | :--- | :--- | :--- |
    | `DATABASE_URL` | Postgres connection string | `postgres://user:pass@host:5432/db` |
    | `SECRET_KEY` | Secret key for JWT/Cookies | `your-super-secret-key...` |

## 3. Frontend Deployment (Docusaurus)

The frontend is a static site generated in `book/build`.

**Deployment Steps:**

1.  **Build:**
    ```bash
    cd book
    npm run build
    ```
2.  **Deploy:** Upload the `book/build` directory to any static hosting service (Vercel, Netlify, GitHub Pages, AWS S3, etc.).

## Summary of URLs

-   **Frontend:** `https://your-site.com` (User visits this)
-   **Backend:** `https://api.your-site.com` (Frontend calls this API)
-   **Database:** `postgres://...` (Backend connects here)

## Verification

1.  Visit the Frontend URL.
2.  Click "Sign In".
3.  The request should go to `https://auth.your-site.com/api/auth/...`.
4.  If successful, you should be logged in.
