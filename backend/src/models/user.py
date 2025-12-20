from datetime import datetime
from typing import Optional
from pydantic import BaseModel, Field
from sqlalchemy import Column, String, DateTime, Boolean, JSON
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.sql import func
import uuid


class UserBase(BaseModel):
    preferences: Optional[dict] = None


class UserCreate(UserBase):
    pass


class UserUpdate(UserBase):
    last_active: Optional[datetime] = None


class User(UserBase):
    id: str
    created_at: datetime
    last_active: Optional[datetime] = None

    class Config:
        from_attributes = True


# SQLAlchemy model
from sqlalchemy.orm import declarative_base
from sqlalchemy import create_engine, ForeignKey

Base = declarative_base()


class UserDB(Base):
    __tablename__ = "users"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    preferences = Column(JSON)
    last_active = Column(DateTime(timezone=True), onupdate=func.now())