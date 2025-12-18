---
id: 2
title: update-constitution-gemini-model
stage: constitution
date_iso: 2025-12-17
surface: agent
model: claude-opus-4-5-20251101
feature: none
branch: main
user: user
command: /sp.constitution
labels: ["constitution", "rag", "chatbot", "gemini", "spec-kit-plus"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files_yaml:
  - .specify/memory/constitution.md
tests_yaml: []
---

# Update Constitution to Use Gemini 2.0-flash

## Prompt Text

```
update that i will use free model gemini 2.0-flash so also add my gemini api key in .env file so be update this
```

## Response Text

Updated the project constitution to change from OpenAI to Google Gemini 2.0-flash (free model) as the AI reasoning engine. Updated the backend framework section to specify "AI Reasoning: Google Gemini 2.0-flash (free model)" and "Embeddings: Gemini embedding model". Added a new Environment Configuration section that specifies API keys must be stored in .env file with GEMINI_API_KEY environment variable. Updated constraints to reference "Efficient use of Gemini API" and added a constraint about respecting free tier limitations of the Gemini 2.0-flash model. The constitution version was updated from 2.0.0 to 2.1.0 to reflect these changes.

## Outcome

The constitution has been successfully updated to specify the use of Google Gemini 2.0-flash as the AI reasoning model and to require the Gemini API key to be stored in a .env file.