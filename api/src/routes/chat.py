from fastapi import APIRouter, HTTPException
from fastapi.responses import StreamingResponse
from src.models.schemas import ChatRequest, ChatResponse, ChatEvent
from src.services.rag import rag_service
from src.services.llm import llm_service
from typing import AsyncGenerator
import json


router = APIRouter()


@router.post("/chat/sync", response_model=ChatResponse)
async def chat_sync(request: ChatRequest):
    """
    Synchronous chat endpoint for testing
    """
    try:
        # Retrieve relevant context
        context, sources = rag_service.retrieve_context(
            query=request.message,
            selected_text=request.selected_text
        )
        
        # Generate response using LLM
        response_text = llm_service.generate_response(
            prompt=request.message,
            context=context,
            conversation_history=request.conversation_history
        )
        
        return ChatResponse(
            content=response_text,
            sources=sources
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/chat")
async def chat_streaming(request: ChatRequest):
    """
    Streaming chat endpoint using Server-Sent Events
    """
    async def event_generator():
        try:
            # Retrieve relevant context
            context, sources = rag_service.retrieve_context(
                query=request.message,
                selected_text=request.selected_text
            )
            
            # Send sources as the first event
            sources_event = ChatEvent(type="sources", sources=sources)
            yield f"data: {sources_event.model_dump_json()}\n\n"
            
            # Generate streaming response
            response_generator = llm_service.generate_streaming_response(
                prompt=request.message,
                context=context,
                conversation_history=request.conversation_history
            )
            
            for chunk in response_generator:
                if chunk:
                    content_event = ChatEvent(type="content", chunk=chunk)
                    yield f"data: {content_event.model_dump_json()}\n\n"
            
            # Send done event
            done_event = ChatEvent(type="done")
            yield f"data: {done_event.model_dump_json()}\n\n"
            
        except Exception as e:
            error_event = ChatEvent(type="error", content=f"Error: {str(e)}")
            yield f"data: {error_event.model_dump_json()}\n\n"
    
    return StreamingResponse(event_generator(), media_type="text/plain")