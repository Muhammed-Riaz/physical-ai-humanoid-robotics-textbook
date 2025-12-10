import re
from typing import List, Dict, Any


def chunk_markdown_content(content: str, max_chunk_size: int = 500) -> List[Dict[str, Any]]:
    """
    Chunk markdown content by headings
    """
    # Split content by level 2 headings (##)
    sections = re.split(r'\n##\s+(.+?)(?=\n##\s+|\Z)', content)
    
    # The first element might be content before any heading
    chunks = []
    if sections[0].strip():
        chunks.append({
            'section': 'Introduction',
            'text': sections[0].strip()
        })
    
    # Process pairs of heading and content
    for i in range(1, len(sections), 2):
        if i + 1 < len(sections):
            heading = sections[i].strip()
            content_text = sections[i + 1].strip()
            
            # If content is too large, split it further
            if len(content_text.split()) > max_chunk_size:
                sub_chunks = split_large_content(content_text, max_chunk_size)
                for j, sub_chunk in enumerate(sub_chunks):
                    chunks.append({
                        'section': f"{heading} (Part {j+1})",
                        'text': sub_chunk
                    })
            else:
                chunks.append({
                    'section': heading,
                    'text': content_text
                })
    
    return chunks


def split_large_content(content: str, max_chunk_size: int) -> List[str]:
    """
    Split large content into smaller chunks based on sentences
    """
    sentences = re.split(r'(?<=[.!?])\s+', content)
    chunks = []
    current_chunk = ""
    
    for sentence in sentences:
        if len((current_chunk + " " + sentence).split()) <= max_chunk_size:
            current_chunk += " " + sentence if current_chunk else sentence
        else:
            if current_chunk:
                chunks.append(current_chunk.strip())
            current_chunk = sentence
    
    if current_chunk:
        chunks.append(current_chunk.strip())
    
    return chunks


def extract_lesson_info_from_path(file_path: str) -> Dict[str, Any]:
    """
    Extract chapter and lesson info from file path
    """
    import os
    path_parts = os.path.normpath(file_path).split(os.sep)
    
    chapter = 0
    lesson = 0
    
    for part in path_parts:
        if part.startswith('chapter-'):
            # Extract chapter number from chapter-XX-name format
            try:
                chapter = int(part.split('-')[1])
            except (IndexError, ValueError):
                pass
        elif part.startswith('lesson-'):
            # Extract lesson number from lesson-XX-name format
            try:
                lesson = int(part.split('-')[1])
            except (IndexError, ValueError):
                pass
    
    return {
        'chapter': chapter,
        'lesson': lesson
    }