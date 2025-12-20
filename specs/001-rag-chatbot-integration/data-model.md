# Data Model: Integrated Chatbot for Physical AI & Humanoid Robotics Book

## Entity: User
- **id**: string (UUID) - unique identifier for the user
- **created_at**: datetime - timestamp when user record was created
- **preferences**: JSON object - user preferences for the chatbot (e.g., response length, technical depth)
- **last_active**: datetime - timestamp of last interaction
- **Validation**: id must be valid UUID, created_at must be in past
- **Relationships**: One-to-many with Session

## Entity: Session
- **id**: string (UUID) - unique identifier for the session
- **user_id**: string (UUID) - reference to User
- **created_at**: datetime - timestamp when session started
- **updated_at**: datetime - timestamp of last activity in session
- **is_active**: boolean - whether session is currently active
- **metadata**: JSON object - additional session information
- **Validation**: user_id must reference existing User, created_at must be before updated_at
- **Relationships**: Many-to-one with User, One-to-many with Conversation

## Entity: Conversation
- **id**: string (UUID) - unique identifier for the conversation
- **session_id**: string (UUID) - reference to Session
- **title**: string (optional) - auto-generated title based on first query
- **created_at**: datetime - timestamp when conversation started
- **updated_at**: datetime - timestamp of last message
- **is_active**: boolean - whether conversation is currently active
- **Validation**: session_id must reference existing Session
- **Relationships**: Many-to-one with Session, One-to-many with Message

## Entity: Message
- **id**: string (UUID) - unique identifier for the message
- **conversation_id**: string (UUID) - reference to Conversation
- **sender_type**: enum (USER, ASSISTANT) - who sent the message
- **content**: string - the message content
- **created_at**: datetime - timestamp when message was created
- **source_citations**: array of objects - list of sources used in response
- **query_type**: enum (FULL_BOOK, SELECTED_TEXT) - type of query made
- **selected_text**: string (optional) - text that was selected when query_type is SELECTED_TEXT
- **Validation**: conversation_id must reference existing Conversation, sender_type must be valid enum value
- **Relationships**: Many-to-one with Conversation

## Entity: BookContentIndex
- **id**: string (UUID) - unique identifier for the indexed content
- **source_file**: string - path to the original book content file
- **section_title**: string - title of the section
- **chapter**: string - chapter name/number
- **lesson**: string - lesson name/number
- **content_text**: string - the actual content text
- **content_embedding**: vector - embedding vector for semantic search
- **created_at**: datetime - timestamp when content was indexed
- **updated_at**: datetime - timestamp when content was last updated
- **Validation**: content_embedding must be proper vector format, source_file must exist
- **Relationships**: No direct relationships, referenced by search results

## Entity: SearchResult
- **id**: string (UUID) - unique identifier for the search result
- **query_id**: string (UUID) - reference to the query that generated this result
- **content_id**: string (UUID) - reference to BookContentIndex
- **relevance_score**: float - similarity score from vector search
- **matched_content**: string - the specific content that matched
- **source_metadata**: JSON object - metadata about the source (file, section, chapter, etc.)
- **created_at**: datetime - timestamp when search was performed
- **Validation**: content_id must reference existing BookContentIndex, relevance_score must be between 0 and 1
- **Relationships**: Many-to-one with BookContentIndex (conceptual, as search happens at runtime)

## State Transitions

### Session State Transitions
- **Created** → **Active**: When user starts first interaction in session
- **Active** → **Inactive**: When session times out or user explicitly ends session
- **Inactive** → **Active**: When user returns to the same session

### Conversation State Transitions
- **Created** → **Active**: When first message is added to conversation
- **Active** → **Paused**: When user stops interacting but session remains active
- **Paused** → **Active**: When user continues conversation
- **Paused** → **Archived**: When conversation is old and inactive

## Validation Rules from Requirements

1. **FR-004**: Conversation history must be maintained across sessions
   - Implemented through Session and Conversation entities with proper relationships

2. **FR-011**: User session data must be stored persistently
   - Implemented through User and Session entities with created_at/updated_at timestamps

3. **FR-013**: Source attribution must be provided for answers
   - Implemented through Message.source_citations and SearchResult.source_metadata

4. **FR-003**: Responses must include citations to specific lessons/chapters
   - Implemented through BookContentIndex.chapter, lesson fields and SearchResult.source_metadata