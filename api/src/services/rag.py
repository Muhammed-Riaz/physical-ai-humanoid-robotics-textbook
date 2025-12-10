from typing import List, Dict, Any
from src.services.indexer import indexer_service
from src.models.schemas import Source
import logging


logger = logging.getLogger(__name__)


class RAGService:
    def __init__(self):
        self.indexer = indexer_service
    
    def retrieve_context(self, query: str, selected_text: str = None, top_k: int = 5) -> tuple:
        """
        Retrieve relevant context from the textbook
        
        Args:
            query: The user's question
            selected_text: Optional selected text for additional context
            top_k: Number of relevant chunks to retrieve
        
        Returns:
            Tuple of (retrieved_context, sources)
        """
        search_query = query
        if selected_text:
            # Combine the query with selected text for better search
            search_query = f"{query} Context: {selected_text}"
        
        # Search for relevant content
        results = self.indexer.search_relevant_content(search_query, top_k=top_k)
        
        # Extract context and sources
        context_list = [result['text'] for result in results]
        context = "\n\n".join(context_list)
        
        sources = []
        for result in results:
            source = Source(
                chapter=int(result.get('chapter', 0)) if result.get('chapter', '0').isdigit() else 0,
                lesson=int(result.get('lesson', 0)) if result.get('lesson', '0').isdigit() else 0,
                section=result.get('section', 'Unknown'),
                url=result.get('url', '')
            )
            sources.append(source)
        
        return context, sources


# Global RAG service instance
rag_service = RAGService()