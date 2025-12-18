---
id: 1
title: update-constitution-rag-chatbot
stage: constitution
date_iso: 2025-12-17
surface: agent
model: claude-opus-4-5-20251101
feature: none
branch: main
user: user
command: /sp.constitution
labels: ["constitution", "rag", "chatbot", "spec-kit-plus"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files_yaml:
  - .specify/memory/constitution.md
tests_yaml: []
---

# Update Constitution for RAG Chatbot System

## Prompt Text

```
/sp.constitution

Project: Integrated RAG Chatbot for an AI-Native Physical AI & Humanoid Robotics Textbook

Purpose:
Define a binding constitution that governs how all AI agents (Claude Code, subagents, skills) design, implement, and validate the RAG chatbot system for this project.

Core Principles:
1. Spec-Driven Development First
   - All work must strictly follow Spec-Kit Plus workflow:
     Specify → Plan → Tasks → Implement
   - No implementation without an approved task in /spec/tasks.md

2. Single Source of Truth
   - Only Spec-Kit artifacts are authoritative:
     /spec/specify.md
     /spec/plan.md
     /spec/tasks.md
   - Prompts, chat messages, or assumptions must never override specs

3. Book-Only Intelligence
   - The chatbot may ONLY answer using the textbook content
   - No web search, no external knowledge, no hallucinations
   - If the answer is not present, respond:
     "This is not covered in this book."

4. Deterministic & Verifiable RAG
   - All answers must be traceable to retrieved chunks
   - Each response must reference chapter and section
   - Retrieval logic must be transparent and debuggable

5. Selected-Text Isolation
   - When user provides highlighted text:
     - Ignore vector database
     - Ignore all other book content
     - Reason ONLY on the selected text
   - This rule is non-negotiable

6. Engineering Rigor
   - Modular, testable, async-first backend
   - Clear separation:
     ingestion / retrieval / reasoning / API / UI
   - Production-grade error handling

Key Standards:
- Backend Framework: FastAPI (Python)
- AI Reasoning: OpenAI Agents SDK
- Embeddings: OpenAI embeddings only
- Vector Store: Qdrant Cloud (Free Tier)
- Relational DB: Neon Serverless Postgres
- Frontend: Docusaurus + ChatKit UI
- Secrets: Environment variables only

RAG Quality Rules:
- Chunking must respect heading hierarchy
- Metadata must include:
  chapter, section, source_file
- Similarity search must use cosine distance
- Max context window must be enforced

Security & Safety:
- No har
```

## Response Text

Updated the project constitution to focus specifically on the RAG chatbot system for the Physical AI & Humanoid Robotics textbook. The new constitution includes six core principles (Spec-Driven Development First, Single Source of Truth, Book-Only Intelligence, Deterministic & Verifiable RAG, Selected-Text Isolation, and Engineering Rigor) along with detailed standards for backend, frontend, RAG quality, security, and data management. The constitution version was updated from 1.0.0 to 2.0.0 to reflect the significant changes in focus and content.

## Outcome

The constitution has been successfully updated to govern the development of the RAG chatbot system. It establishes clear principles and standards that will guide all future development work on the project.