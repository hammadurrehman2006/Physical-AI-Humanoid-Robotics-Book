# Implementation Plan: [FEATURE]

**Branch**: `[###-feature-name]` | **Date**: [DATE] | **Spec**: [link]
**Input**: Feature specification from `/specs/[###-feature-name]/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Implementation of an integrated RAG chatbot for the Physical AI & Humanoid Robotics book using OpenAI Agents SDK. The system will provide learners with immediate access to book content through a floating chat interface positioned in the bottom-left corner, with an additional modal popup option. The chatbot supports both full-book Q&A and selected-text Q&A modes, maintains conversation history, and provides proper source citations to specific lessons/chapters. The frontend components are integrated directly within the book/src/components/ directory as required, with authentication implemented using better-auth framework, and data stored in Neon Serverless Postgres and vector embeddings in Qdrant Cloud.

## Technical Context

**Language/Version**: Python 3.10+ for backend services, JavaScript/TypeScript (Node.js 18+) for frontend integration with Docusaurus
**Primary Dependencies**: OpenAI Agents SDK for RAG functionality, Neon Serverless Postgres for user/session data, Qdrant Cloud for vector storage, better-auth for authentication, FastAPI for backend API, React for Docusaurus integration
**Storage**: Neon Serverless Postgres for conversation history and user sessions, Qdrant Cloud for book content embeddings
**Testing**: pytest for backend services, Jest/React Testing Library for frontend components, Playwright for end-to-end tests
**Target Platform**: Web application integrated with Docusaurus documentation framework, responsive across desktop and mobile devices
**Project Type**: Web application (frontend components in book/src/ + backend services)
**Performance Goals**: <3 second response time for queries, 90% accuracy for book content retrieval, 99% uptime during peak usage
**Constraints**: Must integrate seamlessly with Docusaurus 3.x, maintain accessibility compliance (95%), handle 100+ concurrent users, all frontend components must be located in book/src/components/
**Scale/Scope**: Support all book content for RAG, handle multiple simultaneous user conversations, provide persistent user sessions

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Compliance Check

- ✅ **Learning Philosophy: Hands-On Mastery**: The RAG chatbot provides practical, implementable examples that readers can interact with directly in the book content
- ✅ **Content Quality Standards**: Technical accuracy maintained through OpenAI Agents SDK and proper source citations to specific book lessons/chapters
- ✅ **Project-Based Learning Approach**: The chatbot serves as a practical tool that demonstrates AI integration concepts in the context of educational content
- ✅ **Accessible and Practical Tone**: The floating UI and selected-text Q&A modes make the chatbot accessible and practical for learners
- ✅ **Progressive Complexity**: The chatbot can provide explanations at different levels of complexity based on user queries
- ✅ **Technical Constraints**: Solution uses standard web technologies compatible with Docusaurus and accessible to learners
- ✅ **Resource Constraints**: Uses cloud services (Neon, Qdrant) that are accessible and cost-effective
- ✅ **Content Constraints**: Integrates directly with book content without requiring additional resources from learners
- ✅ **Docusaurus Integration**: All frontend components are located within book/src/components/ as required

### Potential Violations and Justifications

- **None identified**: All aspects of the RAG chatbot implementation align with the project constitution.

## Project Structure

### Documentation (this feature)

```text
specs/[###-feature]/
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
│   │   ├── user.py
│   │   ├── conversation.py
│   │   ├── message.py
│   │   └── content_index.py
│   ├── services/
│   │   ├── rag_service.py
│   │   ├── auth_service.py
│   │   ├── content_service.py
│   │   └── embedding_service.py
│   ├── api/
│   │   ├── main.py
│   │   ├── routes/
│   │   │   ├── chat.py
│   │   │   ├── auth.py
│   │   │   └── content.py
│   │   └── middleware/
│   │       └── auth.py
│   └── core/
│       ├── config.py
│       ├── database.py
│       └── vector_store.py
└── tests/

book/
├── src/
│   ├── components/
│   │   └── Chatbot/
│   │       ├── ChatWindow.tsx
│   │       ├── FloatingButton.tsx
│   │       ├── ModalPopup.tsx
│   │       └── MessageBubble.tsx
│   ├── hooks/
│   │   └── useChatbot.ts
│   ├── services/
│   │   ├── api.ts
│   │   └── chatbot.ts
│   └── types/
│       └── chatbot.ts
└── tests/
```

**Structure Decision**: Web application with separate backend services and frontend components integrated directly into the book directory. The backend uses FastAPI to provide API endpoints for the RAG functionality, while the frontend components are located within book/src/components/ as required by the specification. The floating chat button and modal popup will be implemented as React components that integrate seamlessly with the Docusaurus documentation framework.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [e.g., 4th project] | [current need] | [why 3 projects insufficient] |
| [e.g., Repository pattern] | [specific problem] | [why direct DB access insufficient] |
