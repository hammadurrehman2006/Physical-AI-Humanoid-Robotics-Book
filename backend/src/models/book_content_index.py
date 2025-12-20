from datetime import datetime
from typing import Optional
from pydantic import BaseModel
from sqlalchemy import Column, String, DateTime, Text
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.sql import func
import uuid


class BookContentIndexBase(BaseModel):
    source_file: str
    section_title: str
    chapter: str
    lesson: str
    content_text: str
    # content_embedding will be handled separately in Qdrant
    created_at: Optional[datetime] = None
    updated_at: Optional[datetime] = None


class BookContentIndexCreate(BookContentIndexBase):
    pass


class BookContentIndexUpdate(BaseModel):
    section_title: Optional[str] = None
    chapter: Optional[str] = None
    lesson: Optional[str] = None
    content_text: Optional[str] = None
    updated_at: Optional[datetime] = None


class BookContentIndex(BookContentIndexBase):
    id: str
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True


# Note: The actual content embeddings will be stored in Qdrant Cloud
# This model represents the metadata that will be stored with each vector

from sqlalchemy import ForeignKey
from .user import Base
from sqlalchemy.dialects.postgresql import UUID


class BookContentIndexDB(Base):
    __tablename__ = "book_content_index"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    source_file = Column(String, nullable=False)
    section_title = Column(String, nullable=False)
    chapter = Column(String, nullable=False)
    lesson = Column(String, nullable=False)
    content_text = Column(Text, nullable=False)
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), server_default=func.now(), onupdate=func.now())