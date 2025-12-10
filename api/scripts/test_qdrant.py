import asyncio
from qdrant_client import QdrantClient
from src.config.settings import settings


def test_qdrant_connection():
    """
    Test the Qdrant connection
    """
    try:
        client = QdrantClient(
            url=settings.qdrant_url,
            api_key=settings.qdrant_api_key,
            https=True
        )
        
        # Test connection by getting collections
        collections = client.get_collections()
        
        print("V Qdrant connection successful")
        print(f"Collections: {[col.name for col in collections.collections]}")

        return True
    except Exception as e:
        print(f"X Qdrant connection failed: {e}")
        return False


if __name__ == "__main__":
    test_qdrant_connection()