import pytest
from httpx import AsyncClient
from sqlmodel.ext.asyncio.session import AsyncSession
from app.models.auth import User, Session
from datetime import datetime, timedelta

@pytest.mark.asyncio
async def test_full_auth_flow_mock(client: AsyncClient, session: AsyncSession):
    # 1. Create a user (simulating better-auth signup)
    user = User(
        email="integrated@example.com",
        password_hash="argon2_hashed_password",
        software_background="Intermediate",
        hardware_background="Basic Knowledge"
    )
    session.add(user)
    await session.commit()
    await session.refresh(user)

    # 2. Create a session (simulating better-auth login)
    token = "session_token_123"
    expires_at = datetime.utcnow() + timedelta(days=7)
    
    db_session = Session(
        user_id=user.id,
        token_hash=token,
        expires_at=expires_at,
        ip_address="127.0.0.1",
        user_agent="Pytest"
    )
    session.add(db_session)
    await session.commit()

    # 3. Access FastAPI protected route with the token
    response = await client.get(
        "/api/v1/auth/me",
        cookies={"better-auth.session_token": token}
    )

    assert response.status_code == 200
    data = response.json()
    assert data["email"] == "integrated@example.com"
    assert data["software_background"] == "Intermediate"
    assert data["hardware_background"] == "Basic Knowledge"

@pytest.mark.asyncio
async def test_expired_session(client: AsyncClient, session: AsyncSession):
    user = User(email="expired@example.com", password_hash="hash")
    session.add(user)
    await session.commit()
    await session.refresh(user)

    token = "expired_token"
    # Set expiration in the past
    expires_at = datetime.utcnow() - timedelta(hours=1)
    
    db_session = Session(
        user_id=user.id,
        token_hash=token,
        expires_at=expires_at
    )
    session.add(db_session)
    await session.commit()

    response = await client.get(
        "/api/v1/auth/me",
        cookies={"better-auth.session_token": token}
    )

    assert response.status_code == 401
    assert "expired" in response.json()["detail"].lower()
