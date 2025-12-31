# Physical AI & Humanoid Robotics RAG Chatbot

An integrated chatbot for the Physical AI & Humanoid Robotics book that enables learners to ask questions about book content and receive accurate responses with citations.

## Features

- **Full Book Q&A**: Ask questions about the entire book content
- **Selected Text Q&A**: Get clarifications on specific text selections
- **Conversation History**: Maintain context across sessions
- **Source Citations**: Responses include references to specific chapters and lessons
- **Floating UI**: Unobtrusive chat interface positioned in the bottom-left corner
- **Responsive Design**: Works on desktop and mobile devices

## Architecture

The application consists of:
- **Backend**: FastAPI service with RAG (Retrieval Augmented Generation) capabilities
- **Database**: Neon Serverless Postgres for conversation history and user sessions
- **Vector Store**: Qdrant Cloud for book content embeddings
- **AI Model**: Google Gemini for content understanding and response generation
- **Frontend**: React components integrated with Docusaurus

## Prerequisites

- Docker and Docker Compose
- Python 3.10+
- Node.js 18+ (for Docusaurus integration)
- Google Gemini API key
- Qdrant Cloud account
- Neon Postgres account

## Installation

### Backend Setup

1. Clone the repository:
```bash
git clone <repository-url>
cd physical-ai-and-humanoid-robotics-book
```

2. Set up environment variables by creating a `.env` file:
```bash
cp .env.example .env
```

3. Update the `.env` file with your credentials:
```env
DATABASE_URL=postgresql://user:password@localhost:5432/chatbot_db
QDRANT_HOST=localhost
QDRANT_PORT=6333
QDRANT_API_KEY=your-qdrant-api-key
GEMINI_API_KEY=your-gemini-api-key
SECRET_KEY=your-secret-key
```

4. Build and start the services:
```bash
docker-compose up --build
```

### Frontend Integration

The chatbot components are designed to integrate with Docusaurus. To use the chatbot in your Docusaurus site:

1. Copy the components from `book/src/components/Chatbot/` to your Docusaurus `src/components/` directory
2. Import and use the `ChatbotWidget` component in your layout

## API Endpoints

### Chat Endpoints

- `POST /api/v1/chat` - General book content Q&A
- `POST /api/v1/chat/selected-text` - Q&A based on selected text
- `GET /api/v1/conversations/{conversation_id}` - Get specific conversation
- `GET /api/v1/conversations?session_id={session_id}` - Get conversations for a session
- `DELETE /api/v1/conversations/{conversation_id}` - Delete a conversation
- `GET /api/v1/health` - Health check endpoint

### Session Endpoints

- `GET /api/v1/sessions/{session_id}` - Get session details
- `POST /api/v1/sessions` - Create a new session
- `PUT /api/v1/sessions/{session_id}` - Update session details

## Environment Variables

- `DATABASE_URL`: Connection string for Postgres database
- `QDRANT_HOST`: Host for Qdrant vector database
- `QDRANT_PORT`: Port for Qdrant vector database
- `QDRANT_API_KEY`: API key for Qdrant Cloud (if using cloud version)
- `QDRANT_COLLECTION_NAME`: Name of the collection to store embeddings (default: book_content_embeddings)
- `GEMINI_API_KEY`: API key for Google Gemini
- `GEMINI_MODEL`: Gemini model to use (default: gemini-pro)
- `GEMINI_EMBEDDING_MODEL`: Gemini embedding model (default: embedding-001)
- `SECRET_KEY`: Secret key for security purposes

## Development

### Running Tests

```bash
# Run backend tests
cd backend
python -m pytest tests/

# Run specific test file
python -m pytest tests/unit/test_chat_routes.py
```

### Content Indexing

To index book content for RAG:

1. Ensure your book content is in markdown format
2. Run the content indexing script:
```bash
python -m src.scripts.index_content
```

## Deployment

### Production Deployment

1. Update environment variables in production
2. Build and deploy Docker containers
3. Ensure Qdrant Cloud and Neon Postgres are properly configured
4. Set up reverse proxy (nginx, etc.) if needed

### Configuration for Production

- Use secure environment variables
- Set up proper logging
- Configure monitoring and alerting
- Set up SSL certificates
- Configure rate limiting appropriately

## Performance Considerations

- Response time target: <3 seconds
- Support for 100+ concurrent users
- Rate limits: 100 requests/min for authenticated users, 10 requests/min for unauthenticated
- Vector search optimized for semantic similarity

## Security

- API endpoints protected with rate limiting
- Environment variables for sensitive data
- Input validation on all endpoints
- CORS configured appropriately

## Troubleshooting

### Common Issues

1. **API Keys Not Working**: Verify that GEMINI_API_KEY is set correctly
2. **Database Connection Issues**: Check DATABASE_URL format and credentials
3. **Vector Store Issues**: Ensure QDRANT_HOST and QDRANT_PORT are accessible
4. **Rate Limiting**: Check rate limit logs if requests are being rejected

### Logs

Check Docker logs for issues:
```bash
docker-compose logs backend
docker-compose logs postgres
docker-compose logs qdrant
```

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests if applicable
5. Submit a pull request

## License

[Specify your license here]