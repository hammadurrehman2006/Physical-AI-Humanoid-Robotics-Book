# 🚀 Direct Production Deployment (Vercel)

The backend is configured for immediate deployment to Vercel.

## 1. Prerequisites
- A Vercel account.
- A **PostgreSQL Database** URL (e.g., from [Neon](https://neon.tech), [Supabase](https://supabase.com), or [Render](https://render.com)).
  - *Note: SQLite (`.db` files) will NOT work on Vercel.*

## 2. Deploying via Vercel Dashboard (Recommended)

1.  **Push** your code to a Git repository (GitHub/GitLab/Bitbucket).
2.  Import the project in Vercel.
3.  **Root Directory**: Select `backend` as the root directory.
4.  **Environment Variables**: Add the following in the deployment settings:
    - `DATABASE_URL`: Your full PostgreSQL connection string (e.g., `postgresql://user:pass@host/db?sslmode=require`).
    - `SECRET_KEY`: A long, random string for JWT/Cookie security.
    - `BACKEND_CORS_ORIGINS`: `["https://your-frontend-domain.com"]` (or `*` for testing).
5.  **Deploy**.

## 3. Deploying via CLI (Alternative)

If you have the `vercel` CLI installed:

```bash
cd backend
vercel --prod
```

When prompted:
- **Set up and deploy?**: Yes
- **Link to existing project?**: No
- **Link to existing project settings?**: No
- **Environment Variables**: Go to the Vercel dashboard after the first build fails (or is created) to set `DATABASE_URL`, then redeploy.

## 4. Post-Deployment

After deployment, the API will be available at `https://your-project.vercel.app`.
- Docs: `https://your-project.vercel.app/docs`
- Health Check: `https://your-project.vercel.app/` (Returns "Welcome to Physical AI Book Backend")