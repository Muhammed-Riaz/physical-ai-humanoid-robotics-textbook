# This file is kept for compatibility, but the main app is now in src/main.py
# To run the application, use: uv run uvicorn src.main:app --reload --port 8000
from src.main import app


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
