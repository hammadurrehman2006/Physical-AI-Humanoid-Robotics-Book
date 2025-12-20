from typing import List
from .content_parser import DocusaurusContentParser, ParsedContent
from .chunking_service import ContentChunkingService, ContentChunk
from .embedding_service import EmbeddingService


class ContentIndexService:
    """
    Service to manage the full content indexing pipeline:
    Parse -> Chunk -> Embed -> Store in Qdrant
    """

    def __init__(self, base_path: str = "book/docs"):
        self.parser = DocusaurusContentParser(base_path)
        self.chunking_service = ContentChunkingService()
        self.embedding_service = EmbeddingService()

    def index_single_file(self, file_path: str) -> bool:
        """
        Index a single file: parse, chunk, embed, and store
        """
        try:
            # Parse the file
            parsed_content = self.parser.parse_file(file_path)

            # Chunk the content
            chunks = self.chunking_service.chunk_content(parsed_content)

            # Generate embeddings and store in Qdrant
            embedding_ids = self.embedding_service.generate_and_store_embeddings(chunks)

            print(f"Successfully indexed {file_path}: {len(chunks)} chunks, {len(embedding_ids)} embeddings stored")
            return True
        except Exception as e:
            print(f"Error indexing {file_path}: {str(e)}")
            return False

    def index_directory(self, directory_path: str = None) -> bool:
        """
        Index all markdown files in a directory
        """
        try:
            # Parse all files in directory
            parsed_contents = self.parser.parse_directory(directory_path)

            # Process each parsed content
            total_chunks = 0
            for parsed_content in parsed_contents:
                chunks = self.chunking_service.chunk_content(parsed_content)
                embedding_ids = self.embedding_service.generate_and_store_embeddings(chunks)
                total_chunks += len(chunks)

                print(f"Indexed {parsed_content.source_file}: {len(chunks)} chunks")

            print(f"Successfully indexed directory: {len(parsed_contents)} files, {total_chunks} total chunks")
            return True
        except Exception as e:
            print(f"Error indexing directory: {str(e)}")
            return False

    def update_file_index(self, file_path: str) -> bool:
        """
        Update index for a specific file by removing old embeddings and adding new ones
        """
        try:
            # Parse the file
            parsed_content = self.parser.parse_file(file_path)

            # Chunk the content
            chunks = self.chunking_service.chunk_content(parsed_content)

            # Update embeddings in Qdrant
            success = self.embedding_service.update_content_embeddings(file_path, chunks)

            if success:
                print(f"Successfully updated index for {file_path}")
            else:
                print(f"Failed to update index for {file_path}")

            return success
        except Exception as e:
            print(f"Error updating index for {file_path}: {str(e)}")
            return False

    def search_content(self, query: str, limit: int = 5) -> List[dict]:
        """
        Search for content similar to the query
        """
        return self.embedding_service.search_similar_content(query, limit)

    def delete_file_index(self, file_path: str) -> bool:
        """
        Remove all embeddings associated with a specific file
        """
        return self.embedding_service.delete_content_by_source(file_path)