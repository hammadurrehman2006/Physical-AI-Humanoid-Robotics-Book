# Backend Setup

## 1. Environment Variables

Copy `.env.example` to `.env` and fill in the values.

```bash
cp .env.example .env
```

**Crucial**: Use your NeonDB Connection String for `DATABASE_URL`. It usually looks like:
`postgresql://neondb_owner:.......@ep-....aws.neon.tech/neondb?sslmode=require`

## 2. Install Dependencies

```bash
pip install -r requirements.txt
```

## 3. Run Development Server

```bash
fastapi dev app/main.py
# OR
uvicorn app.main:app --reload
```

The API will be available at `http://localhost:8000`.
Docs at `http://localhost:8000/docs`.

## 4. Database & Migrations

We use **Alembic** for migrations. See [DEPLOYMENT.md](DEPLOYMENT.md) for detailed instructions.

```bash
# Apply migrations
alembic upgrade head
```

## 5. Auth Integration

The backend is configured to share the database with the `book` (Drizzle/BetterAuth) project.
It reads the `users` and `sessions` tables directly.

- Ensure the database schema is up to date (using Alembic or Drizzle).
- The backend verifies the `better-auth.session_token` cookie or `Authorization: Bearer <token>` header.
