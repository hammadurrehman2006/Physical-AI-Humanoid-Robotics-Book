# Quickstart Guide: Integrated Chatbot for Physical AI & Humanoid Robotics Book

## Overview
This guide provides a quick start for setting up and running the RAG chatbot system that integrates with the Physical AI & Humanoid Robotics Book.

## Prerequisites
- Python 3.10+ installed
- Node.js 18+ installed (for Docusaurus integration)
- Docker and Docker Compose (for local development)
- Access to gemini API (API key required)
- Access to Neon Serverless Postgres (connection details)
- Access to Qdrant Cloud (connection details)

## Local Development Setup

### 1. Clone and Navigate to Repository
```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Backend Setup
```bash
# Navigate to backend directory
cd backend

# Create virtual environment
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt

# Set environment variables
cp .env.example .env
# Edit .env with your actual API keys and connection strings
```

### 3. Frontend Setup
```bash
# Navigate to frontend directory
cd frontend

# Install dependencies
npm install

# Set environment variables (if needed)
cp .env.example .env
```

## Environment Variables

### Backend (.env)
```env
# Database Configuration
NEON_DATABASE_URL=your_neon_database_url

# Qdrant Configuration
QDRANT_HOST=your_qdrant_host
QDRANT_API_KEY=your_qdrant_api_key
QDRANT_PORT=6333

# gemini API Configuration
GEMINI_API_KEY=your_gemini_api_key

# Application Settings
SECRET_KEY=your_secret_key
DEBUG=true
```

### Frontend (.env)
```env
# API Configuration
REACT_APP_API_BASE_URL=http://localhost:8000
```

## Running the Application

### 1. Start Backend Service
```bash
cd backend
source venv/bin/activate
uvicorn src.api.main:app --reload --port 8000
```

### 2. Start Frontend (for development)
```bash
cd frontend
npm start
```

## Content Ingestion

### 1. Index Book Content
```bash
cd backend
source venv/bin/activate
python -m src.services.content_index_service --ingest-all
```

### 2. Update Index for New Content
```bash
cd backend
source venv/bin/activate
python -m src.services.content_index_service --update-changed
```

## API Usage Examples

### General Book Content Q&A
```bash
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{
    "query": "Explain how inverse kinematics works in humanoid robots",
    "session_id": "session123"
  }'
```

### Selected Text Q&A
```bash
curl -X POST http://localhost:8000/chat/selected-text \
  -H "Content-Type: application/json" \
  -d '{
    "query": "Can you explain this algorithm in more detail?",
    "selected_text": "The inverse kinematics algorithm calculates joint angles...",
    "session_id": "session123"
  }'
```

### Get Conversation History
```bash
curl -X GET http://localhost:8000/conversations/conversation123
```

## Docusaurus Integration

### 1. Install the Chatbot Component
```bash
# In your Docusaurus project
npm install path/to/chatbot-frontend
```

### 2. Add to Layout
```jsx
// In your Docusaurus layout
import ChatbotWidget from '@site/src/components/ChatbotWidget';

function Layout({children}) {
  return (
    <>
      <ChatbotWidget />
      {children}
    </>
  );
}
```

## Testing

### Backend Tests
```bash
cd backend
source venv/bin/activate
pytest tests/
```

### Frontend Tests
```bash
cd frontend
npm test
```

## Docker Deployment

### Build and Run with Docker
```bash
# Build the images
docker-compose build

# Run the services
docker-compose up -d
```

## Key Endpoints

- `GET /health` - Health check
- `POST /chat` - General book content Q&A
- `POST /chat/selected-text` - Selected text Q&A
- `GET /conversations/{id}` - Get conversation history
- `DELETE /conversations/{id}` - Delete conversation

## Troubleshooting

### Common Issues

1. **API Connection Errors**: Verify your API keys and connection strings in the environment variables
2. **Database Connection Issues**: Check your Neon database connection string and credentials
3. **Vector Search Issues**: Ensure Qdrant is running and accessible
4. **Content Not Found**: Run the content ingestion process to index book content

### Logging
Check the application logs for detailed error information:
- Backend logs: Check console output when running uvicorn
- Frontend logs: Check browser console for client-side errors