from typing import List, Dict, Any
from ..config.gemini import GeminiConfig
from ..config.qdrant import QdrantConfig
from .embedding_service import EmbeddingService


class RAGService:
    """
    Retrieval Augmented Generation service that combines content retrieval with Gemini generation
    """

    def __init__(self):
        self.embedding_service = EmbeddingService()
        self.qdrant_client = QdrantConfig.get_client()
        self.collection_name = QdrantConfig.COLLECTION_NAME
        self.gemini_model = GeminiConfig.get_generative_model()

    def retrieve_relevant_content(self, query: str, limit: int = 5) -> List[Dict[str, Any]]:
        """
        Retrieve relevant content based on the query
        """
        return self.embedding_service.search_similar_content(query, limit)

    def generate_response(self, query: str, context: List[Dict[str, Any]], selected_text: str = None) -> Dict[str, Any]:
        """
        Generate a response using Gemini with retrieved context
        """
        # Format the context for the prompt
        context_str = ""
        sources = []

        for item in context:
            context_str += f"Source: {item['source_file']} - {item['section_title']}\n"
            context_str += f"Content: {item['content']}\n\n"

            sources.append({
                "source_file": item['source_file'],
                "section_title": item['section_title'],
                "chapter": item['chapter'],
                "lesson": item['lesson'],
                "relevance_score": item['relevance_score']
            })

        # Create the prompt based on whether it's a full-book query or selected-text query
        if selected_text:
            prompt = f"""
            You are an expert assistant for the Physical AI & Humanoid Robotics Book. The user has selected specific text and asked a question about it.

            Selected text: {selected_text}

            User's question: {query}

            Here is relevant context from the book that may help answer the question:

            {context_str}

            Please provide a helpful and accurate answer based on the selected text and the provided context. If the context doesn't directly answer the question, say so and provide the most relevant information you can find in the context. Always cite the specific sources when referencing information from the context.
            """
        else:
            prompt = f"""
            You are an expert assistant for the Physical AI & Humanoid Robotics Book. Answer the user's question based on the following context from the book:

            User's question: {query}

            Relevant context from the book:

            {context_str}

            Please provide a helpful and accurate answer based on the context. Be thorough but concise. Always cite the specific sources when referencing information from the context, including the chapter and lesson names. If the context doesn't contain the information needed to answer the question, acknowledge this and provide the best possible response based on the available information.
            """

        # Generate response using Gemini
        response = self.gemini_model.generate_content(prompt)

        return {
            "response": response.text if response.text else "I couldn't generate a response based on the provided context.",
            "sources": sources
        }

    def answer_query(self, query: str, query_type: str = "FULL_BOOK", selected_text: str = None, top_k: int = 5) -> Dict[str, Any]:
        """
        Complete RAG pipeline: retrieve relevant content and generate response
        """
        # Retrieve relevant content
        relevant_content = self.retrieve_relevant_content(query, top_k)

        # Generate response with context
        result = self.generate_response(query, relevant_content, selected_text)

        return result

    def get_conversation_context(self, conversation_history: List[Dict[str, str]], query: str) -> str:
        """
        Format conversation history to provide context for the current query
        """
        if not conversation_history:
            return query

        context = "Previous conversation context:\n"
        for msg in conversation_history[-5:]:  # Use last 5 messages as context
            sender = msg.get('sender_type', 'USER')
            content = msg.get('content', '')
            context += f"{sender}: {content}\n"

        context += f"\nCurrent query: {query}"

        return context