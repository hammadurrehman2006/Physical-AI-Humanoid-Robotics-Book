import os
from sqlalchemy import create_engine
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker
from sqlalchemy.pool import QueuePool
from typing import Generator


class DatabaseConfig:
    """
    Configuration for database connection and session management
    """

    # Database URL - defaults to Neon Postgres
    DATABASE_URL = os.getenv(
        "DATABASE_URL",
        "postgresql+asyncpg://user:password@localhost/dbname"
    )

    # Connection pool settings
    POOL_SIZE = int(os.getenv("DB_POOL_SIZE", "5"))
    MAX_OVERFLOW = int(os.getenv("DB_MAX_OVERFLOW", "10"))
    POOL_TIMEOUT = int(os.getenv("DB_POOL_TIMEOUT", "30"))
    POOL_RECYCLE = int(os.getenv("DB_POOL_RECYCLE", "3600"))  # 1 hour

    # Engine and session setup
    engine = None
    SessionLocal = None

    @classmethod
    def initialize_engine(cls):
        """Initialize the database engine with connection pooling"""
        cls.engine = create_engine(
            cls.DATABASE_URL,
            poolclass=QueuePool,
            pool_size=cls.POOL_SIZE,
            max_overflow=cls.MAX_OVERFLOW,
            pool_timeout=cls.POOL_TIMEOUT,
            pool_recycle=cls.POOL_RECYCLE,
            echo=False  # Set to True for SQL query logging
        )

        cls.SessionLocal = sessionmaker(
            autocommit=False,
            autoflush=False,
            bind=cls.engine
        )

    @classmethod
    def get_db_session(cls) -> Generator:
        """Dependency to get database session"""
        if cls.SessionLocal is None:
            cls.initialize_engine()

        db = cls.SessionLocal()
        try:
            yield db
        finally:
            db.close()

    @classmethod
    def create_all_tables(cls):
        """Create all tables in the database"""
        from ..models import Base
        Base.metadata.create_all(bind=cls.engine)

    @classmethod
    def get_engine(cls):
        """Get the initialized database engine"""
        if cls.engine is None:
            cls.initialize_engine()
        return cls.engine


# Initialize engine on import
DatabaseConfig.initialize_engine()