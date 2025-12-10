# Physical AI & Humanoid Robotics Textbook

A comprehensive textbook covering Physical AI and Humanoid Robotics with an integrated RAG chatbot for enhanced learning.

## Features

- 🤖 **Integrated RAG Chatbot**: Ask questions about the textbook content and get AI-powered answers
- 📚 **Comprehensive Content**: Covering ROS2, Digital Twins, Isaac Sim, VLA (Vision-Language-Action), and more
- 🌐 **Multilingual Support**: Content available in English and Urdu
- 🔍 **Searchable Knowledge**: Vector-searchable textbook content for quick answers

## Architecture

- **Frontend**: Docusaurus-based documentation site
- **Backend**: FastAPI with RAG capabilities
- **Vector DB**: Qdrant Cloud for content indexing
- **LLM**: Google Gemini 1.5 Flash
- **Embeddings**: FastEmbed with BAAI/bge-small-en-v1.5 model

## Setup

1. Install dependencies:
   ```bash
   cd api
   pip install -e .
   ```

2. Configure environment variables:
   ```bash
   # Copy the example environment file
   cp api/.env.example api/.env
   # Edit api/.env with your API keys
   ```

3. Index textbook content:
   ```bash
   cd api
   python -m scripts.index_content
   ```

4. Start the API server:
   ```bash
   cd api
   uvicorn src.main:app --reload --port 8000
   ```

## API Endpoints

- Health Check: `GET /api/health`
- Chat (Streaming): `POST /api/chat`
- Chat (Sync): `POST /api/chat/sync`
- API Documentation: `GET /docs`

## License

MIT License