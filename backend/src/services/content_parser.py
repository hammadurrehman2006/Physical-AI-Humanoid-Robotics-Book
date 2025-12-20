import os
import re
from pathlib import Path
from typing import List, Dict, Any
from dataclasses import dataclass
import markdown
from bs4 import BeautifulSoup
import frontmatter


@dataclass
class ParsedContent:
    """Data class to represent parsed content"""
    source_file: str
    section_title: str
    chapter: str
    lesson: str
    content_text: str
    metadata: Dict[str, Any]


class DocusaurusContentParser:
    """
    Service to parse Docusaurus markdown content and extract structured information
    """

    def __init__(self, base_path: str = None):
        self.base_path = base_path or "book/docs"

    def parse_file(self, file_path: str) -> ParsedContent:
        """
        Parse a single markdown file and extract content with metadata
        """
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()

        # Parse frontmatter and content
        try:
            post = frontmatter.loads(content)
            markdown_content = post.content
            metadata = post.metadata
        except:
            # If no frontmatter, treat entire content as markdown
            markdown_content = content
            metadata = {}

        # Extract section title from the first heading
        section_title = self._extract_title(markdown_content)

        # Extract chapter and lesson from file path
        path_parts = Path(file_path).relative_to(self.base_path).parts
        chapter = path_parts[0] if len(path_parts) > 1 else "unknown"
        lesson = Path(file_path).stem  # filename without extension

        # Convert markdown to plain text for embedding
        plain_text = self._markdown_to_text(markdown_content)

        return ParsedContent(
            source_file=file_path,
            section_title=section_title,
            chapter=chapter,
            lesson=lesson,
            content_text=plain_text,
            metadata=metadata
        )

    def parse_directory(self, directory_path: str = None) -> List[ParsedContent]:
        """
        Parse all markdown files in a directory recursively
        """
        directory_path = directory_path or self.base_path
        parsed_contents = []

        for root, dirs, files in os.walk(directory_path):
            for file in files:
                if file.endswith(('.md', '.mdx')):
                    file_path = os.path.join(root, file)
                    try:
                        parsed_content = self.parse_file(file_path)
                        parsed_contents.append(parsed_content)
                    except Exception as e:
                        print(f"Error parsing {file_path}: {str(e)}")

        return parsed_contents

    def _extract_title(self, markdown_content: str) -> str:
        """
        Extract the first heading as the section title
        """
        lines = markdown_content.split('\n')
        for line in lines:
            # Look for markdown heading patterns
            match = re.match(r'^#{1,6}\s+(.+)', line)
            if match:
                return match.group(1).strip()
        return "Untitled Section"

    def _markdown_to_text(self, markdown_content: str) -> str:
        """
        Convert markdown to plain text, preserving important content
        """
        # Convert markdown to HTML first
        html = markdown.markdown(markdown_content)

        # Parse HTML and extract text
        soup = BeautifulSoup(html, 'html.parser')

        # Remove code blocks and other elements that might not be useful for embeddings
        for code_block in soup.find_all(['code', 'pre']):
            code_block.decompose()

        # Get text content
        text = soup.get_text()

        # Clean up extra whitespace
        text = re.sub(r'\s+', ' ', text).strip()

        return text


# Example usage
if __name__ == "__main__":
    parser = DocusaurusContentParser()
    contents = parser.parse_directory()

    for content in contents[:5]:  # Print first 5 parsed contents
        print(f"File: {content.source_file}")
        print(f"Title: {content.section_title}")
        print(f"Chapter: {content.chapter}")
        print(f"Lesson: {content.lesson}")
        print(f"Content preview: {content.content_text[:100]}...")
        print("-" * 50)