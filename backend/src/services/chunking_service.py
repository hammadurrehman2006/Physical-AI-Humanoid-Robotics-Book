from typing import List, Dict, Any
from dataclasses import dataclass
from .content_parser import ParsedContent


@dataclass
class ContentChunk:
    """Data class to represent a content chunk"""
    id: str
    source_file: str
    section_title: str
    chapter: str
    lesson: str
    content: str
    metadata: Dict[str, Any]
    chunk_index: int
    total_chunks: int


class ContentChunkingService:
    """
    Service to chunk parsed content into smaller pieces for embedding
    """

    def __init__(self, max_chunk_size: int = 1000, overlap_size: int = 100):
        self.max_chunk_size = max_chunk_size
        self.overlap_size = overlap_size

    def chunk_content(self, parsed_content: ParsedContent) -> List[ContentChunk]:
        """
        Split parsed content into chunks while preserving context
        """
        content_text = parsed_content.content_text
        if len(content_text) <= self.max_chunk_size:
            # Content fits in one chunk
            chunk = ContentChunk(
                id=f"{parsed_content.source_file}_0",
                source_file=parsed_content.source_file,
                section_title=parsed_content.section_title,
                chapter=parsed_content.chapter,
                lesson=parsed_content.lesson,
                content=content_text,
                metadata=parsed_content.metadata,
                chunk_index=0,
                total_chunks=1
            )
            return [chunk]

        # Split content into chunks
        chunks = []
        start = 0
        chunk_index = 0

        while start < len(content_text):
            # Determine the end position
            end = start + self.max_chunk_size

            # If we're at the end, just take the remaining content
            if end >= len(content_text):
                end = len(content_text)
            else:
                # Try to break at sentence boundary
                original_end = end
                while end < len(content_text) and content_text[end] not in '.!?':
                    end += 1
                if end >= len(content_text):
                    end = original_end

            # Create chunk
            chunk_text = content_text[start:end].strip()
            if chunk_text:  # Only add non-empty chunks
                chunk = ContentChunk(
                    id=f"{parsed_content.source_file}_{chunk_index}",
                    source_file=parsed_content.source_file,
                    section_title=parsed_content.section_title,
                    chapter=parsed_content.chapter,
                    lesson=parsed_content.lesson,
                    content=chunk_text,
                    metadata=parsed_content.metadata,
                    chunk_index=chunk_index,
                    total_chunks=0  # Will update after all chunks are created
                )
                chunks.append(chunk)

            # Move start position with overlap
            start = end - self.overlap_size if self.overlap_size < end else end
            chunk_index += 1

        # Update total chunks count
        for i, chunk in enumerate(chunks):
            chunks[i].total_chunks = len(chunks)

        return chunks

    def chunk_multiple_contents(self, parsed_contents: List[ParsedContent]) -> List[ContentChunk]:
        """
        Chunk multiple parsed contents
        """
        all_chunks = []
        for parsed_content in parsed_contents:
            chunks = self.chunk_content(parsed_content)
            all_chunks.extend(chunks)
        return all_chunks