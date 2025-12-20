from datetime import datetime
from typing import Optional, List
from pydantic import BaseModel
from sqlalchemy import Column, String, DateTime, Text, Enum as SQLEnum
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.sql import func
import uuid


class SenderType:
    USER = "USER"
    ASSISTANT = "ASSISTANT"


class QueryType:
    FULL_BOOK = "FULL_BOOK"
    SELECTED_TEXT = "SELECTED_TEXT"


class MessageBase(BaseModel):
    conversation_id: str
    sender_type: str  # "USER" or "ASSISTANT"
    content: str
    source_citations: Optional[List[dict]] = []
    query_type: str  # "FULL_BOOK" or "SELECTED_TEXT"
    selected_text: Optional[str] = None


class MessageCreate(MessageBase):
    pass


class MessageUpdate(BaseModel):
    content: Optional[str] = None
    source_citations: Optional[List[dict]] = None


class Message(MessageBase):
    id: str
    created_at: datetime

    class Config:
        from_attributes = True


# SQLAlchemy model
from sqlalchemy import ForeignKey


class MessageDB(Base):
    __tablename__ = "messages"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    conversation_id = Column(UUID(as_uuid=True), ForeignKey("conversations.id"), nullable=False)
    sender_type = Column(SQLEnum(SenderType.USER, SenderType.ASSISTANT, name="sender_type"))
    content = Column(Text, nullable=False)
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    source_citations = Column(Text)  # Store as JSON string
    query_type = Column(SQLEnum(QueryType.FULL_BOOK, QueryType.SELECTED_TEXT, name="query_type"))
    selected_text = Column(Text)