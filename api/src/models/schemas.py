from pydantic import BaseModel, Field
from typing import List, Optional
from enum import Enum


class MessageRole(str, Enum):
    user = "user"
    assistant = "assistant"


class Message(BaseModel):
    role: MessageRole
    content: str


class ChatRequest(BaseModel):
    message: str
    selected_text: Optional[str] = None
    current_page: Optional[str] = None
    conversation_history: List[Message] = Field(default_factory=list)


class Source(BaseModel):
    chapter: int
    lesson: int
    section: str
    url: str


class ChatResponse(BaseModel):
    content: str
    sources: List[Source]


class ChatEvent(BaseModel):
    type: str
    content: Optional[str] = None
    sources: Optional[List[Source]] = None
    chunk: Optional[str] = None


class HealthResponse(BaseModel):
    status: str
    version: str