from fastembed import TextEmbedding
from typing import List
import numpy as np
from src.config.settings import settings


class EmbeddingService:
    def __init__(self):
        self.model = TextEmbedding(
            model_name=settings.embedding_model,
            cache_dir="./cache/models"
        )
        
    def generate_embeddings(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for a list of texts
        """
        embeddings = []
        for embedding in self.model.embed(texts):
            embeddings.append(embedding.tolist())
        return embeddings
    
    def generate_embedding(self, text: str) -> List[float]:
        """
        Generate embedding for a single text
        """
        return self.generate_embeddings([text])[0]
    

# Global embedding service instance
embedding_service = EmbeddingService()