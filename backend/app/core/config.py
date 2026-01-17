
from typing import List, Union
from pydantic import AnyHttpUrl, PostgresDsn, computed_field
from pydantic_settings import BaseSettings, SettingsConfigDict

class Settings(BaseSettings):
    PROJECT_NAME: str = "Physical AI Book Backend"
    API_V1_STR: str = "/api/v1"
    
    # Database
    DATABASE_URL: str
    SECRET_KEY: str = "supersecretkey"
    
    # CORS
    BACKEND_CORS_ORIGINS: List[AnyHttpUrl] = []

    model_config = SettingsConfigDict(
        env_file=".env", 
        env_ignore_empty=True, 
        extra="ignore"
    )

settings = Settings()
