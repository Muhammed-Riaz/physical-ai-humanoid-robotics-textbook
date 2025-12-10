from pydantic_settings import BaseSettings
from typing import List


class Settings(BaseSettings):
    # Gemini API configuration
    gemini_api_key: str
    
    # Qdrant configuration
    qdrant_url: str
    qdrant_api_key: str
    
    # CORS configuration
    cors_origins: str = "http://localhost:3000,https://naimalarain13.github.io"
    
    # Application settings
    app_title: str = "Physical AI Chatbot API"
    app_description: str = "RAG-based chatbot backend for the Physical AI & Humanoid Robotics textbook"
    app_version: str = "1.0.0"
    
    # Vector database settings
    vector_collection_name: str = "textbook_content"
    
    # Model settings
    embedding_model: str = "BAAI/bge-small-en-v1.5"
    max_chunk_size: int = 500
    num_relevant_chunks: int = 5
    
    class Config:
        env_file = ".env"


settings = Settings()