<!--
Sync Impact Report:
Version change: 2.0.0 -> 2.1.0
List of modified principles:
- AI Reasoning Model: Updated to use Gemini 2.0-flash
Added sections:
- Environment Configuration
Removed sections:
- None
Templates requiring updates:
- .specify/templates/plan-template.md: ⚠ pending (Constitution Check alignment)
- .specify/templates/spec-template.md: ⚠ pending (scope/requirements alignment)
- .specify/templates/tasks-template.md: ⚠ pending (task categorization alignment)
Follow-up TODOs: None
-->
# Integrated RAG Chatbot for AI-Native Physical AI & Humanoid Robotics Textbook Constitution

## Core Principles

### Spec-Driven Development First
All work must strictly follow Spec-Kit Plus workflow: Specify → Plan → Tasks → Implement. No implementation without an approved task in /spec/tasks.md. This ensures all development is intentional, traceable, and aligned with project goals.

### Single Source of Truth
Only Spec-Kit artifacts are authoritative: /spec/specify.md, /spec/plan.md, /spec/tasks.md. Prompts, chat messages, or assumptions must never override specs. This creates consistency across all development activities and prevents conflicting implementations.

### Book-Only Intelligence
The chatbot may ONLY answer using the textbook content. No web search, no external knowledge, no hallucinations. If the answer is not present, respond: "This is not covered in this book." This ensures accuracy and reliability of the educational content provided to students.

### Deterministic & Verifiable RAG
All answers must be traceable to retrieved chunks. Each response must reference chapter and section. Retrieval logic must be transparent and debuggable. This enables educators and students to verify the source of information and understand how answers are derived.

### Selected-Text Isolation
When user provides highlighted text: ignore vector database, ignore all other book content, reason ONLY on the selected text. This rule is non-negotiable and ensures focused analysis of user-specified content without contamination from broader knowledge sources.

### Engineering Rigor
Modular, testable, async-first backend with clear separation: ingestion / retrieval / reasoning / API / UI. Production-grade error handling must be implemented throughout the system to ensure reliability and maintainability of the chatbot service.

## Key Standards

### Backend Framework
- Framework: FastAPI (Python)
- AI Reasoning: Google Gemini 2.0-flash (free model)
- Embeddings: Gemini embedding model
- Vector Store: Qdrant Cloud (Free Tier)
- Relational DB: Neon Serverless Postgres
- Async-first design for scalability
- Comprehensive logging and monitoring

### Frontend Framework
- Platform: Docusaurus + ChatKit UI
- Responsive design for all device sizes
- Accessible interface following WCAG guidelines
- Offline-capable where possible
- Real-time chat interface with message history
- Loading states and error handling

### RAG Quality Rules
- Chunking must respect heading hierarchy to maintain context
- Metadata must include: chapter, section, source_file for traceability
- Similarity search must use cosine distance for consistent results
- Max context window must be enforced to prevent token overflow
- Relevance scoring must be implemented to rank retrieved chunks
- Fallback responses must be available when no relevant content found

### Security & Safety
- No hallucination: Strict adherence to source material only
- Input sanitization: All user inputs must be validated and sanitized
- Rate limiting: Prevent abuse and ensure fair usage
- Authentication: Secure access controls where required
- Privacy: No personal data retention beyond session requirements
- Content filtering: Prevent inappropriate use of the system

### Data Management
- Vector storage: Qdrant Cloud with proper indexing for fast retrieval
- Metadata preservation: Maintain chapter/section references in all chunks
- Backup procedures: Regular backups of vector and relational databases
- Data retention: Clear policies for temporary data lifecycle
- Content updates: Mechanism for refreshing textbook content in vectors

### Environment Configuration
- All API keys must be stored in .env file
- Gemini API key must be added as GEMINI_API_KEY environment variable
- Environment variables must be loaded securely at application startup
- No hardcoding of API keys in source code

## Project Requirements & Goals

### Technical Requirements
- Backend: FastAPI with async support and comprehensive API documentation
- RAG Pipeline: Properly configured ingestion, embedding, and retrieval pipeline
- Vector Database: Qdrant Cloud with optimized similarity search
- Frontend Integration: Seamless chat interface within Docusaurus documentation
- Search Quality: High precision and recall for textbook content retrieval
- Performance: Sub-second response times for typical queries
- Scalability: Support for concurrent users during peak usage
- API Integration: Google Gemini 2.0-flash integration with proper error handling

### Constraints
- Content Scope: Limited exclusively to textbook content
- External Dependencies: Minimal dependencies to ensure maintainability
- Token Usage: Efficient use of Gemini API to minimize costs
- Context Window: Strict management of token limits to avoid truncation
- Compliance: Adherence to educational content guidelines and policies
- API Limits: Respect free tier limitations of Gemini 2.0-flash model

### Success Criteria
- Accuracy: High precision in retrieving and responding to textbook queries
- Usability: Intuitive chat interface that enhances learning experience
- Reliability: Consistent availability and performance during student use
- Traceability: All responses clearly linked to source chapters and sections
- Integration: Seamless incorporation into existing Docusaurus documentation
- Documentation: Comprehensive setup and maintenance guides

## Governance

The Constitution supersedes all other practices and serves as the ultimate source of truth for project principles and standards. Amendments to this constitution require thorough documentation of the changes, explicit approval from project stakeholders, and a clear migration plan for any affected systems or processes. All Pull Requests and code reviews MUST explicitly verify compliance with the principles and standards outlined in this constitution. Any proposed increase in complexity within the project MUST be thoroughly justified against the core principles and overall project goals. Regular compliance reviews must be conducted to ensure ongoing adherence to constitutional principles throughout the development lifecycle.

**Version**: 2.1.0 | **Ratified**: 2025-12-17 | **Last Amended**: 2025-12-17