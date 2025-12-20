from fastapi import APIRouter, HTTPException, Depends
from pydantic import BaseModel
from typing import List, Optional, Dict, Any
from ..services.rag_service import RAGService
from ..services.content_index_service import ContentIndexService
from ..models.message import MessageCreate, Message, SenderType, QueryType
from ..models.conversation import Conversation, ConversationCreate
from ..config.database import DatabaseConfig
from sqlalchemy.orm import Session
from ..models import SessionDB, ConversationDB, MessageDB
import uuid


router = APIRouter()


class ChatRequest(BaseModel):
    query: str
    session_id: Optional[str] = None
    conversation_id: Optional[str] = None
    query_type: str = "FULL_BOOK"  # "FULL_BOOK" or "SELECTED_TEXT"
    selected_text: Optional[str] = None


class ChatResponse(BaseModel):
    response: str
    sources: List[Dict[str, Any]]
    session_id: str
    conversation_id: str


# Initialize services
rag_service = RAGService()
content_index_service = ContentIndexService()


def get_db():
    for db in DatabaseConfig.get_db_session():
        yield db


def store_chat_interaction(
    db: Session,
    query: str,
    response: str,
    session_id: str,
    conversation_id: str,
    query_type: str,
    selected_text: Optional[str] = None,
    sources: Optional[List[Dict[str, Any]]] = None
):
    """
    Store the chat interaction in the database
    """
    # Get or create session
    session_db = db.query(SessionDB).filter(SessionDB.id == session_id).first()
    if not session_db:
        session_db = SessionDB(id=session_id, user_id=None)  # Anonymous user
        db.add(session_db)
        db.commit()

    # Get or create conversation
    conversation_db = db.query(ConversationDB).filter(ConversationDB.id == conversation_id).first()
    if not conversation_db:
        conversation_db = ConversationDB(
            id=conversation_id,
            session_id=session_id,
            title=query[:50] + "..." if len(query) > 50 else query  # Auto-generate title
        )
        db.add(conversation_db)
        db.commit()

    # Create user message
    user_message = MessageDB(
        conversation_id=conversation_id,
        sender_type=SenderType.USER,
        content=query,
        source_citations=str(sources) if sources else "[]",  # Store as JSON string
        query_type=query_type,
        selected_text=selected_text
    )
    db.add(user_message)

    # Create assistant message
    assistant_message = MessageDB(
        conversation_id=conversation_id,
        sender_type=SenderType.ASSISTANT,
        content=response,
        source_citations=str(sources) if sources else "[]",  # Store as JSON string
        query_type=query_type,
        selected_text=selected_text
    )
    db.add(assistant_message)

    db.commit()


@router.post("/chat", response_model=ChatResponse)
async def chat_endpoint(chat_request: ChatRequest, db: Session = Depends(get_db)):
    """
    Main chat endpoint for full-book queries and selected-text queries
    """
    try:
        # Use existing session or create a new one
        session_id = chat_request.session_id or str(uuid.uuid4())

        # Use existing conversation or create a new one
        conversation_id = chat_request.conversation_id or str(uuid.uuid4())

        # Determine query type
        query_type = chat_request.query_type
        if query_type not in [QueryType.FULL_BOOK, QueryType.SELECTED_TEXT]:
            query_type = QueryType.FULL_BOOK

        # Get response from RAG service
        result = rag_service.answer_query(
            query=chat_request.query,
            query_type=query_type,
            selected_text=chat_request.selected_text if query_type == QueryType.SELECTED_TEXT else None
        )

        # Store the interaction in the database
        store_chat_interaction(
            db=db,
            query=chat_request.query,
            response=result["response"],
            session_id=session_id,
            conversation_id=conversation_id,
            query_type=query_type,
            selected_text=chat_request.selected_text,
            sources=result["sources"]
        )

        # Create response
        response = ChatResponse(
            response=result["response"],
            sources=result["sources"],
            session_id=session_id,
            conversation_id=conversation_id
        )

        return response

    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error processing chat request: {str(e)}")


@router.post("/chat/selected-text", response_model=ChatResponse)
async def chat_selected_text_endpoint(chat_request: ChatRequest):
    """
    Chat endpoint specifically for selected-text queries
    """
    # Force the query type to be selected text
    chat_request.query_type = QueryType.SELECTED_TEXT

    # Call the main chat endpoint
    return await chat_endpoint(chat_request)


@router.get("/health")
async def health_check():
    """
    Health check endpoint
    """
    return {"status": "healthy", "service": "chat-api"}