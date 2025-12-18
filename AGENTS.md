# AGENTS.md

root directory api folder is backend 
docs is my frontend docsours 
i already do all work on my hands on so check all requremnet if fullfill if not correct and update it 

## Purpose
This repository uses **Spec‑Kit Plus** for all development.
All AI agents (Claude Code, sub‑agents, skills) MUST follow this workflow:


**Specify → Plan → Tasks → Implement**


No agent is allowed to:
- Skip Spec‑Kit artifacts
- Invent requirements
- Implement without a task ID


---


## Source of Truth


The following files are authoritative:


- `/spec/specify.md` – WHAT we are building
- `/spec/plan.md` – HOW the system is designed
- `/spec/tasks.md` – STEP‑BY‑STEP implementation tasks


If there is a conflict between:
- A prompt
- A chat message
- An agent instruction


👉 **Spec‑Kit files always win**.


---


## Stack (Locked)


Agents MUST use only the following technologies:


### Frontend
- Docusaurus (Book)
- ChatKit UI (embedded widget)


### Backend
- FastAPI (Python)
- OpenAI Agents SDK


### Data
- Qdrant Cloud (Free Tier) – vector store
- Neon Serverless Postgres – users, chats, metadata


### AI
- OpenAI Embeddings
- OpenAI Reasoning Models (via Agents SDK)


---
If unclear → update Spec files first.