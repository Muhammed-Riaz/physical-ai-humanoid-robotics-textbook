# Research: Integrated RAG Chatbot for AI-Native Physical AI & Humanoid Robotics Textbook

**Feature**: 2-integrated-rag-chatbot
**Date**: 2025-12-17

## Overview

This research document addresses the technical decisions required for implementing the RAG chatbot system. It covers key architectural decisions with their alternatives, trade-offs, and final justifications.

## Decision: RAG Architecture Pattern

**Rationale**: For the RAG system, we'll use a standard architecture with ingestion pipeline, vector store, retriever, and reasoning component. This follows the well-established pattern for RAG systems.

**Alternatives considered**:
1. Simple embedding + retrieval approach
2. Multi-modal RAG (text + images)
3. Graph RAG for complex relationships

**Final choice**: Standard RAG pipeline with semantic search, as it's most appropriate for textbook content and aligns with the requirements.

## Decision: Embedding Model Selection

**Rationale**: Since the constitution specifies using Google Gemini 2.0-flash, we'll use the associated embedding model for consistency and to stay within the free tier limitations.

**Alternatives considered**:
1. OpenAI embeddings (would require different reasoning model)
2. Sentence Transformers (local, but larger infrastructure needs)
3. Cohere embeddings (different provider ecosystem)

**Final choice**: Gemini embedding model to maintain consistency with the Gemini 2.0-flash reasoning model.

## Decision: Vector Database

**Rationale**: The constitution specifies Qdrant Cloud (Free Tier) as the vector store, which is suitable for this implementation.

**Alternatives considered**:
1. Pinecone (would require different configuration)
2. Weaviate (different API structure)
3. FAISS (local option, but doesn't meet cloud requirement)

**Final choice**: Qdrant Cloud as specified in the constitution.

## Decision: Chunking Strategy

**Rationale**: To maintain context while enabling effective retrieval, we'll implement hierarchical chunking that respects document structure (chapters, sections, subsections).

**Alternatives considered**:
1. Fixed-size chunks (may break context)
2. Semantic chunking (more complex but potentially better results)
3. Sentence-level chunks (too granular for textbook content)

**Final choice**: Hierarchical chunking that respects heading hierarchy, with metadata including chapter, section, and source file as required by the constitution.

## Decision: Retrieval Strategy

**Rationale**: For accurate retrieval with textbook content, we'll implement cosine similarity search with configurable relevance thresholds.

**Alternatives considered**:
1. Maximal Marginal Relevance (MMR) for diversity
2. Hybrid search (keyword + semantic)
3. Multi-query retrieval

**Final choice**: Cosine similarity search as specified in the constitution, with relevance scoring to rank retrieved chunks.

## Decision: Frontend Integration

**Rationale**: The chatbot needs to be embedded in the Docusaurus textbook, so we'll create a React component that integrates with the existing documentation system.

**Alternatives considered**:
1. Standalone web application (would require leaving the textbook)
2. Browser extension (more complex distribution)
3. IFrame embedding (less seamless integration)

**Final choice**: React component integrated into Docusaurus as specified in the constitution.

## Decision: Selected-Text Isolation

**Rationale**: To comply with the constitution's "Selected-Text Isolation" principle, we'll implement a mode where only the selected text is used for reasoning, ignoring the vector database and other book content.

**Implementation approach**:
1. When in selected-text mode, bypass vector retrieval entirely
2. Pass only the selected text to the reasoning component
3. Ensure no other content influences the response

## Decision: Response Validation

**Rationale**: To ensure compliance with the "Book-Only Intelligence" principle, we'll implement validation to ensure responses are based only on textbook content and include proper references.

**Implementation approach**:
1. Track source chunks for each response
2. Verify content alignment between response and sources
3. Include chapter/section references in all responses
4. Handle cases where no relevant content exists by returning "This is not covered in this book."

## Decision: API Design

**Rationale**: The API should support both question-answering modes while maintaining clear separation of concerns.

**Endpoints to implement**:
1. `/chat/full-book` - Full-book question answering
2. `/chat/selected-text` - Selected-text-only question answering
3. `/health` - Health check endpoint
4. `/ingestion` - Content ingestion endpoints (for initial setup and updates)

## Decision: Error Handling and Monitoring

**Rationale**: Production-ready systems require comprehensive error handling and monitoring.

**Implementation approach**:
1. Comprehensive logging at all service levels
2. Proper error responses for different failure modes
3. Performance monitoring for response times
4. Usage tracking to monitor against free tier limits