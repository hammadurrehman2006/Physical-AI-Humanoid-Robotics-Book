from collections.abc import AsyncGenerator

from sqlalchemy.ext.asyncio import create_async_engine
from sqlalchemy.orm import sessionmaker
from sqlmodel.ext.asyncio.session import AsyncSession

from app.core.config import settings

# Create async engine
# We replace postgresql:// with postgresql+asyncpg:// to use the async driver
# if the user provides a standard postgres connection string.
connection_str = str(settings.DATABASE_URL).replace(
    "postgresql://", "postgresql+asyncpg://"
)

engine = create_async_engine(
    connection_str, 
    echo=True, # Set to False in production
    future=True
)

async def get_session() -> AsyncGenerator[AsyncSession, None]:
    async_session = sessionmaker(
        engine, class_=AsyncSession, expire_on_commit=False
    )
    async with async_session() as session:
        yield session
