# API Contract: RAG Chatbot for Physical AI & Humanoid Robotics Book

## Base URL
`https://api.book-domain.com/v1` (production)
`http://localhost:8000` (development)

## Authentication
All endpoints require authentication via Bearer token in the Authorization header:
```
Authorization: Bearer {token}
```

## Common Headers
- `Content-Type: application/json`
- `Accept: application/json`

## Error Response Format
```json
{
  "error": {
    "code": "ERROR_CODE",
    "message": "Human-readable error message",
    "details": "Additional error details (optional)"
  }
}
```

## Endpoints

### POST /chat
General book content Q&A

#### Request
```json
{
  "query": "string (required) - The question about book content",
  "session_id": "string (required) - Session identifier",
  "conversation_id": "string (optional) - Existing conversation identifier",
  "include_sources": "boolean (optional, default: true) - Whether to include source citations"
}
```

#### Response (200 OK)
```json
{
  "response": "string - The answer to the query",
  "sources": [
    {
      "title": "string - Title of the source",
      "chapter": "string - Chapter name/number",
      "lesson": "string - Lesson name/number",
      "url": "string - URL to the source content",
      "relevance_score": "number - 0.0 to 1.0 relevance score"
    }
  ],
  "conversation_id": "string - Identifier for the conversation",
  "message_id": "string - Identifier for this message"
}
```

### POST /chat/selected-text
Q&A based on selected text

#### Request
```json
{
  "query": "string (required) - The question about the selected text",
  "selected_text": "string (required) - The text that was selected by the user",
  "session_id": "string (required) - Session identifier",
  "conversation_id": "string (optional) - Existing conversation identifier",
  "include_sources": "boolean (optional, default: true) - Whether to include source citations"
}
```

#### Response (200 OK)
```json
{
  "response": "string - The answer to the query",
  "sources": [
    {
      "title": "string - Title of the source",
      "chapter": "string - Chapter name/number",
      "lesson": "string - Lesson name/number",
      "url": "string - URL to the source content",
      "relevance_score": "number - 0.0 to 1.0 relevance score"
    }
  ],
  "conversation_id": "string - Identifier for the conversation",
  "message_id": "string - Identifier for this message"
}
```

### GET /conversations/{conversation_id}
Get conversation history

#### Path Parameters
- `conversation_id`: string - The conversation identifier

#### Response (200 OK)
```json
{
  "conversation": {
    "id": "string",
    "title": "string",
    "created_at": "string (ISO 8601 datetime)",
    "updated_at": "string (ISO 8601 datetime)",
    "messages": [
      {
        "id": "string",
        "role": "string (user|assistant)",
        "content": "string",
        "created_at": "string (ISO 8601 datetime)",
        "sources": [
          {
            "title": "string",
            "chapter": "string",
            "lesson": "string",
            "url": "string",
            "relevance_score": "number"
          }
        ]
      }
    ]
  }
}
```

### GET /conversations
Get all conversations for a user

#### Query Parameters
- `user_id`: string (required) - The user identifier
- `limit`: number (optional, default: 20) - Number of conversations to return
- `offset`: number (optional, default: 0) - Number of conversations to skip

#### Response (200 OK)
```json
{
  "conversations": [
    {
      "id": "string",
      "title": "string",
      "created_at": "string (ISO 8601 datetime)",
      "updated_at": "string (ISO 8601 datetime)",
      "message_count": "number"
    }
  ],
  "total_count": "number"
}
```

### GET /health
Health check endpoint

#### Response (200 OK)
```json
{
  "status": "ok",
  "timestamp": "string (ISO 8601 datetime)",
  "version": "string"
}
```

## Rate Limiting
- Authenticated users: 100 requests per minute
- Unauthenticated users: 10 requests per minute
- Exceeded requests return 429 Too Many Requests

## Response Codes
- `200 OK`: Request successful
- `400 Bad Request`: Invalid request parameters
- `401 Unauthorized`: Missing or invalid authentication
- `403 Forbidden`: Insufficient permissions
- `404 Not Found`: Resource not found
- `429 Too Many Requests`: Rate limit exceeded
- `500 Internal Server Error`: Server error