import pytest
from httpx import AsyncClient
from sqlmodel.ext.asyncio.session import AsyncSession
from app.models.auth import User, Session
from datetime import datetime, timedelta
import uuid

@pytest.mark.asyncio
async def test_read_users_me_unauthorized(client: AsyncClient):
    response = await client.get("/api/v1/auth/me")
    assert response.status_code == 401
    assert response.json()["detail"] == "Not authenticated"

@pytest.mark.asyncio
async def test_read_users_me_success(client: AsyncClient, session: AsyncSession):
    # Create user
    user = User(
        email="test@example.com",
        password_hash="hashed_secret",
        software_background="Expert"
    )
    session.add(user)
    await session.commit()
    await session.refresh(user)

    # Create session
    token = "valid_token"
    
    session_obj = Session(
        user_id=user.id,
        token_hash=token, 
        expires_at=datetime.utcnow() + timedelta(days=1)
    )
    session.add(session_obj)
    await session.commit()

    # Test with cookie
    cookies = {"better-auth.session_token": token}
    response = await client.get("/api/v1/auth/me", cookies=cookies)
    
    # Debug info if fails
    if response.status_code != 200:
        print(f"Response: {response.text}")

    assert response.status_code == 200
    data = response.json()
    assert data["email"] == "test@example.com"
    assert data["id"] == str(user.id)
    
    # Test with header
    headers = {"Authorization": f"Bearer {token}"}
    response_header = await client.get("/api/v1/auth/me", headers=headers)
    assert response_header.status_code == 200
    assert response_header.json()["email"] == "test@example.com"
