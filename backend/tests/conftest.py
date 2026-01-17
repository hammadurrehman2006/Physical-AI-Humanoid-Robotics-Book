import pytest
from typing import AsyncGenerator
from httpx import AsyncClient, ASGITransport
from sqlmodel import SQLModel
from sqlmodel.ext.asyncio.session import AsyncSession
from sqlalchemy.ext.asyncio import create_async_engine
from sqlalchemy.orm import sessionmaker
import asyncio

from app.main import app
from app.core.db import get_session
from app.core.config import settings

# Use the configured DATABASE_URL (Neon DB)
DATABASE_URL = str(settings.DATABASE_URL)

engine = create_async_engine(
    DATABASE_URL,
    echo=False,
    future=True,
)

async def override_get_session() -> AsyncGenerator[AsyncSession, None]:
    async_session = sessionmaker(
        engine, class_=AsyncSession, expire_on_commit=False
    )
    async with async_session() as session:
        yield session

app.dependency_overrides[get_session] = override_get_session

@pytest.fixture(name="session")
async def session_fixture():
    async with engine.begin() as conn:
        await conn.run_sync(SQLModel.metadata.create_all)

    async_session = sessionmaker(
        engine, class_=AsyncSession, expire_on_commit=False
    )
    async with async_session() as session:
        yield session

    async with engine.begin() as conn:
        await conn.run_sync(SQLModel.metadata.drop_all)

@pytest.fixture(name="client")
async def client_fixture():
    transport = ASGITransport(app=app)
    async with AsyncClient(transport=transport, base_url="http://test") as client:
        yield client
