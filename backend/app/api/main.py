from fastapi import APIRouter, Depends
from app.api.deps import CurrentUser
from app.models.auth import User

api_router = APIRouter()

@api_router.get("/auth/me", response_model=User)
async def read_users_me(current_user: CurrentUser):
    """
    Get current user.
    """
    return current_user
