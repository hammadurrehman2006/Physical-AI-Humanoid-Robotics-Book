from typing import Annotated, Optional
from fastapi import Depends, HTTPException, status, Cookie, Request
from sqlmodel import select
from sqlmodel.ext.asyncio.session import AsyncSession
from datetime import datetime

from app.core.db import get_session
from app.models.auth import User, Session

async def get_session_token(
    request: Request,
    # Try to get from cookie first (standard better-auth)
    better_auth_session_token: Annotated[Optional[str], Cookie(alias="better-auth.session_token")] = None,
    # Fallback to Authorization header if needed (Bearer token)
    authorization: Optional[str] = None
) -> str:
    if better_auth_session_token:
        return better_auth_session_token
    
    # Check header
    auth_header = request.headers.get("Authorization")
    if auth_header and auth_header.startswith("Bearer "):
        return auth_header.split(" ")[1]
        
    return ""

async def get_current_user(
    token: Annotated[str, Depends(get_session_token)],
    db: Annotated[AsyncSession, Depends(get_session)]
) -> User:
    if not token:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Not authenticated"
        )

    # Note: better-auth might hash the token in the DB. 
    # If the column is 'token_hash', we might need to hash 'token' before querying.
    # However, without knowing the specific hashing algo/salt used by better-auth,
    # we first assume direct storage or standard verify.
    # If direct lookup fails, we might need to investigate better-auth's hashing.
    
    # Query for the session
    statement = select(Session).where(Session.token_hash == token)
    result = await db.exec(statement)
    session_record = result.first()

    if not session_record:
        # Debugging tip: If token exists but not found, it might be hashed in DB.
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid session"
        )

    if session_record.expires_at < datetime.utcnow():
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Session expired"
        )

    # Get the user
    user_statement = select(User).where(User.id == session_record.user_id)
    user_result = await db.exec(user_statement)
    user = user_result.first()
    
    if not user:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="User not found"
        )
        
    return user

CurrentUser = Annotated[User, Depends(get_current_user)]
