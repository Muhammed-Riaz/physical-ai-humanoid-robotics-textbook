# Implementation Plan: Integrated RAG Chatbot for AI-Native Physical AI & Humanoid Robotics Textbook

**Branch**: `2-integrated-rag-chatbot` | **Date**: 2025-12-17 | **Spec**: [link to spec.md](../2-integrated-rag-chatbot/spec.md)
**Input**: Feature specification from `/specs/[###-feature-name]/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Implementation of an integrated RAG chatbot that allows students to ask questions about Physical AI & Humanoid Robotics textbook content and receive accurate answers sourced only from the textbook. The system supports both full-book question answering and selected-text-only question answering, with responses including chapter/section references. Built with FastAPI backend, Qdrant vector database, and Google Gemini 2.0-flash for reasoning.

## Technical Context

<!--
  ACTION REQUIRED: Replace the content in this section with the technical details
  for the project. The structure here is presented in advisory capacity to guide
  the iteration process.
-->

**Language/Version**: Python 3.11, JavaScript/TypeScript for frontend integration
**Primary Dependencies**: FastAPI, Qdrant, Google Gemini API, Docusaurus, ChatKit UI
**Storage**: Qdrant Cloud (vector database), Neon Serverless Postgres (relational DB)
**Testing**: pytest, integration tests for RAG pipeline
**Target Platform**: Linux server (backend), Web browser (frontend)
**Project Type**: Web application (backend API + frontend integration)
**Performance Goals**: Sub-second response times for typical queries, handle 100 concurrent users
**Constraints**: Must respect Gemini 2.0-flash free tier limitations, <5s response time for queries, no hallucination of information
**Scale/Scope**: Support all textbook content, handle multiple students simultaneously during peak usage

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

Based on the constitution, the implementation must:
1. Follow Spec-Kit Plus workflow: Specify → Plan → Tasks → Implement
2. Only answer using textbook content (no external knowledge)
3. Be traceable to retrieved chunks with chapter/section references
4. When user provides highlighted text, ignore vector database and reason only on selected text
5. Use modular, testable, async-first backend with clear separation of concerns
6. Use Google Gemini 2.0-flash as the reasoning model
7. Store API keys in .env file with GEMINI_API_KEY environment variable
8. Implement proper error handling throughout the system

## Project Structure

### Documentation (this feature)

```text
specs/2-integrated-rag-chatbot/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── src/
│   ├── models/
│   │   ├── question.py
│   │   ├── response.py
│   │   ├── textbook_content.py
│   │   └── user_session.py
│   ├── services/
│   │   ├── ingestion_service.py
│   │   ├── retrieval_service.py
│   │   ├── reasoning_service.py
│   │   ├── embedding_service.py
│   │   └── validation_service.py
│   ├── api/
│   │   ├── chat_endpoints.py
│   │   ├── ingestion_endpoints.py
│   │   └── health_endpoints.py
│   └── utils/
│       ├── chunking_utils.py
│       ├── metadata_utils.py
│       └── reference_utils.py
└── tests/
    ├── unit/
    ├── integration/
    └── contract/

api/
├── main.py
├── config.py
├── requirements.txt
└── .env.example

docs/
├── src/
│   └── components/
│       └── ChatbotComponent.js  # Embedded chat interface
└── docusaurus.config.js
```

**Structure Decision**: Web application with separate backend API and frontend integration. Backend handles all RAG processing, API endpoints, and business logic using FastAPI. Frontend integrates the chatbot into the Docusaurus textbook via a React component that communicates with the backend API.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [e.g., 4th project] | [current need] | [why 3 projects insufficient] |
| [e.g., Repository pattern] | [specific problem] | [why direct DB access insufficient] |