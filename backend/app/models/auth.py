import uuid
from datetime import datetime
from typing import Optional

from fastapi_users.db import SQLAlchemyBaseAccessTokenTableUUID, SQLAlchemyBaseUserTableUUID
from sqlmodel import Field, SQLModel

class User(SQLAlchemyBaseUserTableUUID, SQLModel, table=True):
    __tablename__ = "users"
    
    # Custom fields
    software_background: Optional[str] = Field(default=None)
    hardware_background: Optional[str] = Field(default=None)
    created_at: datetime = Field(default_factory=datetime.utcnow)
    last_login_at: Optional[datetime] = Field(default=None)

class AccessToken(SQLAlchemyBaseAccessTokenTableUUID, SQLModel, table=True):
    __tablename__ = "access_tokens"
