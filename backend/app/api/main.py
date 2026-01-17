from fastapi import APIRouter
from app.api import chat
from app.core.auth import auth_backend
from app.core.security import fastapi_users
from app.schemas.users import UserCreate, UserRead, UserUpdate

api_router = APIRouter()

api_router.include_router(chat.router, prefix="/chat", tags=["chat"])

api_router.include_router(
    fastapi_users.get_auth_router(auth_backend),
    prefix="/auth",
    tags=["auth"],
)

api_router.include_router(
    fastapi_users.get_register_router(UserRead, UserCreate),
    prefix="/auth",
    tags=["auth"],
)

api_router.include_router(
    fastapi_users.get_users_router(UserRead, UserUpdate),
    prefix="/users",
    tags=["users"],
)
