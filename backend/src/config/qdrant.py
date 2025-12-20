import os
from qdrant_client import QdrantClient
from qdrant_client.http import models
from typing import Optional


class QdrantConfig:
    """
    Configuration for Qdrant Cloud vector database
    Gemini embedding-001 model produces 768-dimensional vectors
    """

    # Qdrant connection parameters
    QDRANT_URL = os.getenv("QDRANT_URL", "http://localhost:6333")
    QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
    COLLECTION_NAME = os.getenv("QDRANT_COLLECTION_NAME", "book_content_embeddings")

    # Vector dimensions for Gemini embedding-001 model
    VECTOR_DIMENSION = 768

    @classmethod
    def get_client(cls) -> QdrantClient:
        """Initialize and return Qdrant client"""
        if cls.QDRANT_API_KEY:
            return QdrantClient(
                url=cls.QDRANT_URL,
                api_key=cls.QDRANT_API_KEY
            )
        else:
            return QdrantClient(url=cls.QDRANT_URL)

    @classmethod
    def initialize_collection(cls) -> None:
        """Initialize the collection with proper vector configuration"""
        client = cls.get_client()

        # Check if collection already exists
        collection_exists = False
        try:
            client.get_collection(cls.COLLECTION_NAME)
            collection_exists = True
        except:
            pass  # Collection doesn't exist

        if not collection_exists:
            client.create_collection(
                collection_name=cls.COLLECTION_NAME,
                vectors_config=models.VectorParams(
                    size=cls.VECTOR_DIMENSION,
                    distance=models.Distance.COSINE
                )
            )

            print(f"Created Qdrant collection: {cls.COLLECTION_NAME} with {cls.VECTOR_DIMENSION} dimensions")


# Initialize collection on import
# Uncomment the following line if you want to auto-initialize on import
# QdrantConfig.initialize_collection()