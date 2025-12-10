from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from src.routes import health, chat
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Initialize the FastAPI app
app = FastAPI(
    title="Physical AI Chatbot API",
    description="RAG-based chatbot backend for the Physical AI & Humanoid Robotics textbook",
    version="1.0.0"
)

# CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=os.getenv("CORS_ORIGINS", "http://localhost:3000,https://naimalarain13.github.io").split(","),
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include routers
app.include_router(health.router, prefix="/api", tags=["health"])
app.include_router(chat.router, prefix="/api", tags=["chat"])

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)