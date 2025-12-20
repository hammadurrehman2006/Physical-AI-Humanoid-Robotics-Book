import os
import google.generativeai as genai
from typing import Optional


class GeminiConfig:
    """
    Configuration for Google Gemini API
    """

    # API Configuration
    GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")
    DEFAULT_MODEL = os.getenv("GEMINI_MODEL", "gemini-pro")
    EMBEDDING_MODEL = os.getenv("GEMINI_EMBEDDING_MODEL", "embedding-001")

    # Request configuration
    MAX_OUTPUT_TOKENS = int(os.getenv("MAX_OUTPUT_TOKENS", "2048"))
    TEMPERATURE = float(os.getenv("TEMPERATURE", "0.7"))
    TOP_P = float(os.getenv("TOP_P", "0.9"))
    TOP_K = int(os.getenv("TOP_K", "40"))

    @classmethod
    def initialize_client(cls) -> None:
        """Initialize the Gemini API client"""
        if not cls.GEMINI_API_KEY:
            raise ValueError("GEMINI_API_KEY environment variable must be set")

        genai.configure(api_key=cls.GEMINI_API_KEY)

    @classmethod
    def get_generative_model(cls):
        """Get configured generative model instance"""
        if not cls.GEMINI_API_KEY:
            raise ValueError("GEMINI_API_KEY environment variable must be set")

        return genai.GenerativeModel(
            model_name=cls.DEFAULT_MODEL,
            generation_config={
                "max_output_tokens": cls.MAX_OUTPUT_TOKENS,
                "temperature": cls.TEMPERATURE,
                "top_p": cls.TOP_P,
                "top_k": cls.TOP_K
            }
        )

    @classmethod
    def get_embedding_model(cls):
        """Get configured embedding model instance"""
        if not cls.GEMINI_API_KEY:
            raise ValueError("GEMINI_API_KEY environment variable must be set")

        return genai.EmbeddingModel(cls.EMBEDDING_MODEL)

    @classmethod
    def embed_content(cls, content: str) -> list:
        """Generate embedding for the given content"""
        if not cls.GEMINI_API_KEY:
            raise ValueError("GEMINI_API_KEY environment variable must be set")

        embedding_model = cls.get_embedding_model()
        response = embedding_model.embed_content(
            content=content,
            task_type="RETRIEVAL_DOCUMENT"  # Optimize for document retrieval
        )

        return response.embedding

    @classmethod
    def embed_batch(cls, contents: list) -> list:
        """Generate embeddings for a batch of content"""
        if not cls.GEMINI_API_KEY:
            raise ValueError("GEMINI_API_KEY environment variable must be set")

        embedding_model = cls.get_embedding_model()
        response = embedding_model.embed_content(
            content=contents,
            task_type="RETRIEVAL_DOCUMENT"
        )

        return response.embeddings