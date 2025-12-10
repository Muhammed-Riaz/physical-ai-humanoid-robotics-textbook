from fastapi import APIRouter
from src.models.schemas import HealthResponse
from src.config.settings import settings


router = APIRouter()


@router.get("/health", response_model=HealthResponse)
async def health_check():
    """
    Health check endpoint
    """
    return HealthResponse(
        status="healthy",
        version=settings.app_version
    )