from fastapi import APIRouter
from typing import Dict


router = APIRouter()


@router.get("/health")
async def health_check() -> Dict[str, str]:
    """
    Health check endpoint to verify API is running
    """
    return {
        "status": "healthy",
        "service": "rag-chatbot-api",
        "version": "1.0.0"
    }


@router.get("/ready")
async def readiness_check() -> Dict[str, str]:
    """
    Readiness check endpoint
    """
    # Here you would typically check if all dependencies are ready
    # (database connections, external APIs, etc.)
    return {
        "status": "ready",
        "service": "rag-chatbot-api"
    }


@router.get("/live")
async def liveness_check() -> Dict[str, str]:
    """
    Liveness check endpoint
    """
    return {
        "status": "alive",
        "service": "rag-chatbot-api"
    }