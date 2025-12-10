from qdrant_client import QdrantClient
from qdrant_client.http import models
from typing import List, Dict, Any
from src.config.settings import settings
from src.services.embeddings import embedding_service
import logging


logger = logging.getLogger(__name__)


class IndexerService:
    def __init__(self):
        self.client = QdrantClient(
            url=settings.qdrant_url,
            api_key=settings.qdrant_api_key,
            https=True
        )
        self.collection_name = settings.vector_collection_name
    
    def create_collection(self):
        """
        Create the vector collection if it doesn't exist
        """
        try:
            # Check if collection exists
            self.client.get_collection(self.collection_name)
            logger.info(f"Collection {self.collection_name} already exists")
        except:
            # Create collection
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=models.VectorParams(
                    size=384,  # Size for bge-small-en-v1.5 embeddings
                    distance=models.Distance.COSINE
                )
            )
            logger.info(f"Created collection {self.collection_name}")
    
    def index_textbook_content(self, chunks: List[Dict[str, Any]]):
        """
        Index textbook content chunks into Qdrant
        """
        self.create_collection()
        
        # Prepare points for insertion
        points = []
        for i, chunk in enumerate(chunks):
            text = chunk['text']
            metadata = {
                'chapter': chunk.get('chapter', ''),
                'lesson': chunk.get('lesson', ''),
                'section': chunk.get('section', ''),
                'url': chunk.get('url', ''),
                'original_text': text
            }
            
            # Generate embedding
            embedding = embedding_service.generate_embedding(text)
            
            # Create point
            point = models.PointStruct(
                id=i,
                vector=embedding,
                payload=metadata
            )
            points.append(point)
        
        # Upload points to Qdrant
        self.client.upload_points(
            collection_name=self.collection_name,
            points=points
        )
        
        logger.info(f"Indexed {len(points)} chunks into {self.collection_name}")
    
    def search_relevant_content(self, query: str, top_k: int = settings.num_relevant_chunks) -> List[Dict[str, Any]]:
        """
        Search for relevant content based on the query
        """
        query_embedding = embedding_service.generate_embedding(query)
        
        results = self.client.search(
            collection_name=self.collection_name,
            query_vector=query_embedding,
            limit=top_k
        )
        
        # Format results
        formatted_results = []
        for result in results:
            formatted_results.append({
                'text': result.payload['original_text'],
                'chapter': result.payload['chapter'],
                'lesson': result.payload['lesson'],
                'section': result.payload['section'],
                'url': result.payload['url'],
                'score': result.score
            })
        
        return formatted_results


# Global indexer service instance
indexer_service = IndexerService()