from datetime import datetime
from typing import Optional
from pydantic import BaseModel
from sqlalchemy import Column, String, DateTime, Boolean, JSON
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.sql import func
import uuid


class SessionBase(BaseModel):
    user_id: str
    is_active: bool = True
    metadata: Optional[dict] = None


class SessionCreate(SessionBase):
    pass


class SessionUpdate(BaseModel):
    is_active: Optional[bool] = None
    metadata: Optional[dict] = None


class Session(SessionBase):
    id: str
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True


# SQLAlchemy model
from sqlalchemy import ForeignKey


class SessionDB(Base):
    __tablename__ = "sessions"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    user_id = Column(UUID(as_uuid=True), ForeignKey("users.id"), nullable=False)
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), server_default=func.now(), onupdate=func.now())
    is_active = Column(Boolean, default=True)
    metadata = Column(JSON)