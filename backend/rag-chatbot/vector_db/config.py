from pydantic_settings import BaseSettings
from typing import Optional

class QdrantConfig(BaseSettings):
    """Configuration for Qdrant vector database"""

    # Qdrant connection settings
    QDRANT_HOST: str = "localhost"
    QDRANT_PORT: int = 6333
    QDRANT_API_KEY: Optional[str] = None
    QDRANT_COLLECTION_NAME: str = "robotics_book"

    # For cloud-hosted Qdrant
    QDRANT_URL: Optional[str] = None

    class Config:
        env_prefix = "QDRANT_"
        case_sensitive = True
        env_file = ".env"
        extra = "ignore"

# Global instance
qdrant_config = QdrantConfig()