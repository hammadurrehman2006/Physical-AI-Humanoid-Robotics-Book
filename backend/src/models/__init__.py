from .user import UserDB, User, UserCreate, UserUpdate
from .session import SessionDB, Session, SessionCreate, SessionUpdate
from .conversation import ConversationDB, Conversation, ConversationCreate, ConversationUpdate
from .message import MessageDB, Message, MessageCreate, MessageUpdate, SenderType, QueryType
from .book_content_index import BookContentIndexDB, BookContentIndex, BookContentIndexCreate, BookContentIndexUpdate

# Import the Base class for metadata
from .user import Base

__all__ = [
    "Base",
    "UserDB", "User", "UserCreate", "UserUpdate",
    "SessionDB", "Session", "SessionCreate", "SessionUpdate",
    "ConversationDB", "Conversation", "ConversationCreate", "ConversationUpdate",
    "MessageDB", "Message", "MessageCreate", "MessageUpdate", "SenderType", "QueryType",
    "BookContentIndexDB", "BookContentIndex", "BookContentIndexCreate", "BookContentIndexUpdate"
]