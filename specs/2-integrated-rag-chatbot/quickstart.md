# Quickstart Guide: Integrated RAG Chatbot for AI-Native Physical AI & Humanoid Robotics Textbook

**Feature**: 2-integrated-rag-chatbot
**Date**: 2025-12-17

## Overview

This guide provides the essential steps to set up and run the integrated RAG chatbot system for the Physical AI & Humanoid Robotics textbook.

## Prerequisites

- Python 3.11+
- Node.js 18+ (for Docusaurus frontend)
- Access to Google Gemini API
- Qdrant Cloud account (Free Tier)
- Neon Serverless Postgres account

## Environment Setup

1. **Clone the repository**
   ```bash
   git clone <repository-url>
   cd Physical_AI_Humanoid_Robotics
   ```

2. **Set up backend environment**
   ```bash
   cd backend
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   pip install -r requirements.txt
   ```

3. **Create environment file**
   ```bash
   cp .env.example .env
   ```

4. **Configure environment variables**
   Edit the `.env` file with your API keys and service configurations:
   ```env
   GEMINI_API_KEY=your_gemini_api_key_here
   QDRANT_URL=your_qdrant_cluster_url
   QDRANT_API_KEY=your_qdrant_api_key
   DATABASE_URL=your_neon_postgres_connection_string
   ```

## Backend Setup

1. **Install dependencies**
   ```bash
   cd backend
   pip install -r requirements.txt
   ```

2. **Initialize the database**
   ```bash
   python -m src.utils.init_db
   ```

3. **Index the textbook content**
   ```bash
   python -m src.services.ingestion_service --source-path path/to/textbook/content
   ```

4. **Run the backend server**
   ```bash
   uvicorn api.main:app --host 0.0.0.0 --port 8000
   ```

## Frontend Integration

1. **Navigate to docs directory**
   ```bash
   cd docs
   ```

2. **Install dependencies**
   ```bash
   npm install
   ```

3. **Add the chatbot component**
   The chatbot is integrated via a React component that connects to the backend API. The component is already configured in the Docusaurus setup.

4. **Start the documentation site**
   ```bash
   npm start
   ```

## API Endpoints

The backend provides the following key endpoints:

- `POST /chat/full-book` - Submit a question for full-book mode
- `POST /chat/selected-text` - Submit a question with selected text
- `GET /health` - Check system health
- `POST /ingestion` - Ingest new textbook content

## Configuration Options

- **Response timeout**: Configure maximum time to wait for responses in `config.py`
- **Relevance threshold**: Adjust the minimum relevance score for retrieved content
- **Chunk size**: Modify the size of content chunks during ingestion
- **Rate limiting**: Configure request limits to respect API quotas

## Testing the System

1. **Start both backend and frontend**
2. **Open the textbook in your browser** (usually http://localhost:3000)
3. **Use the embedded chat interface** to ask questions about the textbook content
4. **Verify that responses include proper chapter/section references**

## Troubleshooting

- **No responses**: Check that the Qdrant vector database is properly configured and contains indexed content
- **API errors**: Verify that your GEMINI_API_KEY is correctly set in the environment
- **Slow responses**: Monitor your API usage to ensure you're within free tier limits
- **Missing references**: Ensure the textbook content was properly indexed with metadata

## Next Steps

1. Customize the chatbot UI to match your documentation theme
2. Add additional textbook content through the ingestion service
3. Monitor usage metrics and performance
4. Implement additional features as needed