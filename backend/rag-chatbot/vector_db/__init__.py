"""
Vector database module for the RAG Chatbot API
Handles vector storage and retrieval using Qdrant
"""

from .qdrant_service import QdrantService, DocumentPayload
from .config import qdrant_config

__all__ = ["QdrantService", "DocumentPayload", "qdrant_config"]