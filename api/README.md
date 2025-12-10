Physical AI Chatbot API

RAG-based chatbot backend for the Physical AI & Humanoid Robotics textbook.
Architecture

    Framework: FastAPI with uv package manager
    LLM: Gemini 1.5 Flash via OpenAI Agents SDK (free tier)
    Embeddings: Qdrant FastEmbed - BAAI/bge-small-en-v1.5 (384-dim, local, free)
    Vector DB: Qdrant Cloud (free tier)
    Cost: $0 (all free services)

Setup
1. Install dependencies (in separate terminal)

cd api
export PATH="$HOME/.local/bin:$PATH"

# Install dependencies
uv add fastapi uvicorn fastembed agents qdrant-client pydantic pydantic-settings python-dotenv httpx tiktoken sse-starlette

# Install dev dependencies  
uv add --dev pytest pytest-asyncio httpx

2. Test Qdrant connection

uv run python scripts/test_qdrant.py

3. Index textbook content

uv run python scripts/index_content.py

4. Start API server

uv run uvicorn src.main:app --reload --port 8000

API Endpoints

    GET /api/health - Health check
    POST /api/chat - Chat with streaming (SSE)
    POST /api/chat/sync - Chat without streaming

Features Implemented

✅ All backend Phase 0-3 complete!

See full documentation in this file for usage details.

title 	emoji 	colorFrom 	colorTo 	sdk 	sdk_version 	app_file 	pinned 	license
Physical AI Chatbot API
	
🤖
	
purple
	
blue
	
docker
	
3.11
	
app.py
	
false
	
mit
Physical AI Chatbot API 🤖

RAG-based chatbot for the Physical AI & Humanoid Robotics textbook.
Features

    ✅ Answer questions from textbook content exclusively
    ✅ Text selection queries - Ask about highlighted text
    ✅ Navigation guidance - Get links to relevant lessons
    ✅ Study companion - Recommendations for learning path
    ✅ Streaming responses via Server-Sent Events (SSE)
    ✅ Source citations - Every answer includes chapter/lesson references

Technology Stack

    Framework: FastAPI
    LLM: Google Gemini 1.5 Flash (free tier)
    Embeddings: Qdrant FastEmbed (BAAI/bge-small-en-v1.5)
    Vector DB: Qdrant Cloud (free tier)
    Streaming: SSE (Server-Sent Events)

API Endpoints
Health Check

GET /api/health

Chat

POST /api/chat
Content-Type: application/json

{
  "message": "What is embodied intelligence?",
  "selected_text": null,
  "current_page": null,
  "conversation_history": []
}

Interactive API Documentation

    Swagger UI: /docs
    ReDoc: /redoc

Environment Variables

Required secrets (configure in Hugging Face Space settings):

    GEMINI_API_KEY - Get from https://aistudio.google.com/app/apikey
    QDRANT_URL - Qdrant Cloud cluster URL
    QDRANT_API_KEY - Qdrant Cloud API key
    CORS_ORIGINS - Allowed CORS origins (e.g., https://yoursite.com)

Local Development

# Install dependencies
pip install -r requirements.txt

# Set environment variables
export GEMINI_API_KEY=your_key
export QDRANT_URL=your_qdrant_url
export QDRANT_API_KEY=your_qdrant_key

# Run server
python app.py

Frontend Integration

This API is designed to work with the Physical AI textbook frontend:

    Frontend: https://naimalarain13.github.io/physical-ai-and-humaniod-robotics/
    Repository: https://github.com/NaimalArain13/physical-ai-and-humaniod-robotics

Cost

💰 Total Cost: $0/month

    Gemini API: Free tier (15 requests/minute)
    Qdrant FastEmbed: Local, free
    Qdrant Cloud: Free tier (1GB storage)

License

MIT License
Support
For issues and questions, please visit the GitHub repository.

Physical AI Chatbot API

RAG-based chatbot backend for the Physical AI & Humanoid Robotics textbook.
Architecture

    Framework: FastAPI with uv package manager
    LLM: Gemini 1.5 Flash via OpenAI Agents SDK (free tier)
    Embeddings: Qdrant FastEmbed - BAAI/bge-small-en-v1.5 (384-dim, local, free)
    Vector DB: Qdrant Cloud (free tier)
    Cost: $0 (all free services)

Setup
1. Install dependencies (in separate terminal)

cd api
export PATH="$HOME/.local/bin:$PATH"

# Install dependencies
uv add fastapi uvicorn fastembed agents qdrant-client pydantic pydantic-settings python-dotenv httpx tiktoken sse-starlette

# Install dev dependencies  
uv add --dev pytest pytest-asyncio httpx

2. Test Qdrant connection

uv run python scripts/test_qdrant.py

3. Index textbook content

uv run python scripts/index_content.py

4. Start API server

uv run uvicorn src.main:app --reload --port 8000

API Endpoints

    GET /api/health - Health check
    POST /api/chat - Chat with streaming (SSE)
    POST /api/chat/sync - Chat without streaming

Features Implemented

✅ All backend Phase 0-3 complete!

See full documentation in this file for usage details.