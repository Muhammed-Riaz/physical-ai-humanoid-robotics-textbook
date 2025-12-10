import os
import glob
from pathlib import Path
from typing import List, Dict, Any
from src.utils.chunker import chunk_markdown_content, extract_lesson_info_from_path
from src.services.indexer import indexer_service
from src.config.settings import settings


def index_textbook_content():
    """
    Index all textbook content from docs directory
    """
    # Find all markdown files in the docs directory
    # Assuming the docs directory is one level up from api
    docs_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), "..", "docs", "docs")
    docs_path = os.path.abspath(docs_path)
    
    markdown_files = []
    for root, dirs, files in os.walk(docs_path):
        for file in files:
            if file.endswith('.md'):
                markdown_files.append(os.path.join(root, file))
    
    if not markdown_files:
        print("No markdown files found in docs/docs/")
        print(f"Looking in: {docs_path}")
        return
    
    print(f"Found {len(markdown_files)} markdown files to index")
    
    all_chunks = []
    
    for file_path in markdown_files:
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read()
            
            # Extract lesson info from file path
            lesson_info = extract_lesson_info_from_path(file_path)
            
            # Get relative path for URL construction
            rel_path = os.path.relpath(file_path, docs_path)
            url = rel_path.replace('\\', '/').replace('.md', '').lower()
            if not url.startswith('/'):
                url = '/' + url
            
            # Chunk the content
            chunks = chunk_markdown_content(content)
            
            for chunk in chunks:
                chunk_data = {
                    'text': chunk['text'],
                    'chapter': lesson_info.get('chapter', 0),
                    'lesson': lesson_info.get('lesson', 0),
                    'section': chunk['section'],
                    'url': url
                }
                all_chunks.append(chunk_data)
                
            print(f"Processed {file_path}: {len(chunks)} chunks")
        
        except Exception as e:
            print(f"Error processing {file_path}: {e}")
    
    # Index all chunks
    print(f"\nIndexing {len(all_chunks)} total chunks...")
    indexer_service.index_textbook_content(all_chunks)
    print("Indexing completed!")


if __name__ == "__main__":
    index_textbook_content()