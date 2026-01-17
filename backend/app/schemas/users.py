import uuid
from typing import Optional

from fastapi_users import schemas
from pydantic import BaseModel

class UserRead(schemas.BaseUser[uuid.UUID]):
    software_background: Optional[str] = None
    hardware_background: Optional[str] = None

class UserCreate(schemas.BaseUserCreate):
    software_background: Optional[str] = None
    hardware_background: Optional[str] = None

class UserUpdate(schemas.BaseUserUpdate):
    software_background: Optional[str] = None
    hardware_background: Optional[str] = None
