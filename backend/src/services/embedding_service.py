import uuid
from typing import List, Dict, Any
from qdrant_client.http import models
from ..config.gemini import GeminiConfig
from ..config.qdrant import QdrantConfig
from .chunking_service import ContentChunk


class EmbeddingService:
    """
    Service to generate embeddings using Google Gemini and store in Qdrant
    """

    def __init__(self):
        # Initialize configurations
        GeminiConfig.initialize_client()
        QdrantConfig.initialize_collection()

        self.qdrant_client = QdrantConfig.get_client()
        self.collection_name = QdrantConfig.COLLECTION_NAME

    def generate_and_store_embeddings(self, chunks: List[ContentChunk]) -> List[str]:
        """
        Generate embeddings for content chunks and store in Qdrant
        """
        # Extract content texts for batch processing
        contents = [chunk.content for chunk in chunks]

        # Generate embeddings in batch
        embeddings = GeminiConfig.embed_batch(contents)

        # Prepare points for Qdrant
        points = []
        for i, chunk in enumerate(chunks):
            point = models.PointStruct(
                id=str(uuid.uuid4()),
                vector=embeddings[i],
                payload={
                    "source_file": chunk.source_file,
                    "section_title": chunk.section_title,
                    "chapter": chunk.chapter,
                    "lesson": chunk.lesson,
                    "content": chunk.content,
                    "chunk_index": chunk.chunk_index,
                    "total_chunks": chunk.total_chunks,
                    "metadata": chunk.metadata
                }
            )
            points.append(point)

        # Upload to Qdrant in batches (Qdrant has limits)
        batch_size = 100  # Typical batch size for Qdrant
        successful_ids = []

        for i in range(0, len(points), batch_size):
            batch = points[i:i + batch_size]
            self.qdrant_client.upsert(
                collection_name=self.collection_name,
                points=batch
            )
            # Collect the IDs of the inserted points
            for point in batch:
                successful_ids.append(point.id)

        return successful_ids

    def generate_single_embedding(self, content: str) -> List[float]:
        """
        Generate a single embedding for content
        """
        return GeminiConfig.embed_content(content)

    async def search_similar_content(self, query: str, limit: int = 5) -> List[Dict[str, Any]]:
        """
        Search for similar content based on query
        """
        # Generate embedding for the query
        query_embedding = self.generate_single_embedding(query)

        # Search in Qdrant
        search_results = self.qdrant_client.search(
            collection_name=self.collection_name,
            query_vector=query_embedding,
            limit=limit
        )

        # Format results
        results = []
        for result in search_results:
            results.append({
                "id": result.id,
                "content": result.payload.get("content", ""),
                "source_file": result.payload.get("source_file", ""),
                "section_title": result.payload.get("section_title", ""),
                "chapter": result.payload.get("chapter", ""),
                "lesson": result.payload.get("lesson", ""),
                "relevance_score": result.score,
                "metadata": result.payload.get("metadata", {})
            })

        return results

    def delete_content_by_source(self, source_file: str) -> bool:
        """
        Delete all embeddings associated with a source file
        """
        try:
            # Find points with the source file
            search_results = self.qdrant_client.scroll(
                collection_name=self.collection_name,
                scroll_filter=models.Filter(
                    must=[
                        models.FieldCondition(
                            key="source_file",
                            match=models.MatchValue(value=source_file)
                        )
                    ]
                ),
                limit=10000  # Assuming we won't have more than 10k chunks per file
            )

            # Extract IDs
            ids_to_delete = [result.id for result in search_results[0]]

            if ids_to_delete:
                # Delete points
                self.qdrant_client.delete(
                    collection_name=self.collection_name,
                    points_selector=models.PointIdsList(
                        points=ids_to_delete
                    )
                )

            return True
        except Exception as e:
            print(f"Error deleting content for {source_file}: {str(e)}")
            return False

    def update_content_embeddings(self, source_file: str, new_chunks: List[ContentChunk]) -> bool:
        """
        Update embeddings for a specific source file by deleting old ones and adding new ones
        """
        # Delete old embeddings
        success = self.delete_content_by_source(source_file)
        if not success:
            return False

        # Add new embeddings
        try:
            asyncio.run(self.generate_and_store_embeddings(new_chunks))
            return True
        except Exception as e:
            print(f"Error updating content for {source_file}: {str(e)}")
            return False