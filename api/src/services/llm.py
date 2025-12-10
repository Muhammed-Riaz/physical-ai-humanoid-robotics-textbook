import google.generativeai as genai
from typing import List, Generator
from src.config.settings import settings
from src.models.schemas import Message
import logging


logger = logging.getLogger(__name__)


class LLMService:
    def __init__(self):
        # Configure the Gemini API
        genai.configure(api_key=settings.gemini_api_key)
        
        # Initialize the model
        self.model = genai.GenerativeModel(
            model_name="gemini-1.5-flash",
            system_instruction="You are an expert assistant for the Physical AI & Humanoid Robotics textbook. Answer questions based only on the provided textbook content. Always cite the relevant chapters and lessons. If the information is not available in the provided context, clearly state that the answer is not in the textbook content."
        )
    
    def generate_response(self, prompt: str, context: str = None, conversation_history: List[Message] = None) -> str:
        """
        Generate a response using the LLM
        """
        # Build the full prompt
        full_prompt = prompt
        
        if context:
            full_prompt = f"Context from textbook:\n{context}\n\nQuestion: {prompt}"
        
        try:
            # Generate response
            response = self.model.generate_content(full_prompt)
            return response.text
        except Exception as e:
            logger.error(f"Error generating response: {e}")
            return "Sorry, I encountered an error while processing your request."
    
    def generate_streaming_response(self, prompt: str, context: str = None, conversation_history: List[Message] = None) -> Generator[str, None, None]:
        """
        Generate a streaming response using the LLM
        """
        # Build the full prompt
        full_prompt = prompt
        
        if context:
            full_prompt = f"Context from textbook:\n{context}\n\nQuestion: {prompt}"
        
        try:
            # Generate response
            response = self.model.generate_content(full_prompt, stream=True)
            
            for chunk in response:
                if chunk.text:
                    yield chunk.text
        except Exception as e:
            logger.error(f"Error generating streaming response: {e}")
            yield "Sorry, I encountered an error while processing your request."


# Global LLM service instance
llm_service = LLMService()