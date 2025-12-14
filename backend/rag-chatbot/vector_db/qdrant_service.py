from qdrant_client import QdrantClient
from qdrant_client.http import models
from typing import List, Dict, Any
import logging
from pydantic import BaseModel

logger = logging.getLogger(__name__)

class DocumentPayload(BaseModel):
    """Structure for document payload in Qdrant"""
    content: str
    module: str
    chapter: str
    page_reference: str
    metadata: Dict[str, Any] = {}

class QdrantService:
    """Service class for handling Qdrant vector database operations"""

    def __init__(self, host: str = "localhost", port: int = 6333, collection_name: str = "robotics_book"):
        """
        Initialize Qdrant service

        Args:
            host: Qdrant server host
            port: Qdrant server port
            collection_name: Name of the collection to use
        """
        self.client = QdrantClient(host=host, port=port)
        self.collection_name = collection_name
        self._ensure_collection_exists()

    def _ensure_collection_exists(self):
        """Ensure the collection exists with proper configuration"""
        try:
            # Check if collection exists
            collections = self.client.get_collections()
            collection_names = [col.name for col in collections.collections]

            if self.collection_name not in collection_names:
                # Create collection with vector configuration
                self.client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=models.VectorParams(
                        size=1536,  # Standard OpenAI embedding size
                        distance=models.Distance.COSINE
                    )
                )
                logger.info(f"Created Qdrant collection: {self.collection_name}")
            else:
                logger.info(f"Qdrant collection {self.collection_name} already exists")
        except Exception as e:
            logger.error(f"Error ensuring collection exists: {e}")
            raise

    def add_document(self, doc_id: str, content: str, module: str, chapter: str, page_reference: str,
                     metadata: Dict[str, Any] = None, embedding: List[float] = None):
        """
        Add a document to the Qdrant collection

        Args:
            doc_id: Unique identifier for the document
            content: Text content of the document
            module: Module identifier
            chapter: Chapter identifier
            page_reference: Page reference
            metadata: Additional metadata
            embedding: Pre-computed embedding vector (if None, will need to be computed separately)
        """
        if metadata is None:
            metadata = {}

        payload = DocumentPayload(
            content=content,
            module=module,
            chapter=chapter,
            page_reference=page_reference,
            metadata=metadata
        )

        # If no embedding is provided, we'll need to compute it separately
        # For now, we'll use a placeholder if none is provided
        if embedding is None:
            # In a real implementation, this would be computed using an embedding model
            embedding = [0.0] * 1536  # Placeholder for OpenAI embedding size

        point = models.PointStruct(
            id=doc_id,
            vector=embedding,
            payload=payload.dict()
        )

        try:
            self.client.upsert(
                collection_name=self.collection_name,
                points=[point]
            )
            logger.info(f"Added document {doc_id} to collection {self.collection_name}")
        except Exception as e:
            logger.error(f"Error adding document {doc_id}: {e}")
            raise

    def search_documents(self, query_embedding: List[float], limit: int = 5, module_filter: str = None) -> List[Dict[str, Any]]:
        """
        Search for documents similar to the query embedding

        Args:
            query_embedding: Embedding vector to search for
            limit: Maximum number of results to return
            module_filter: Optional filter to search within a specific module

        Returns:
            List of documents with similarity scores
        """
        try:
            # Build filters if needed
            filters = None
            if module_filter:
                filters = models.Filter(
                    must=[
                        models.FieldCondition(
                            key="module",
                            match=models.MatchValue(value=module_filter)
                        )
                    ]
                )

            search_results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                query_filter=filters,
                limit=limit
            )

            results = []
            for hit in search_results:
                result = {
                    "id": hit.id,
                    "content": hit.payload["content"],
                    "module": hit.payload["module"],
                    "chapter": hit.payload["chapter"],
                    "page_reference": hit.payload["page_reference"],
                    "metadata": hit.payload["metadata"],
                    "score": hit.score
                }
                results.append(result)

            logger.info(f"Search returned {len(results)} results")
            return results

        except Exception as e:
            logger.error(f"Error searching documents: {e}")
            raise

    def batch_add_documents(self, documents: List[Dict[str, Any]]):
        """
        Add multiple documents in a batch operation

        Args:
            documents: List of document dictionaries with required fields
        """
        points = []

        for i, doc in enumerate(documents):
            doc_id = doc.get("doc_id", f"doc_{i}")
            content = doc["content"]
            module = doc["module"]
            chapter = doc["chapter"]
            page_reference = doc["page_reference"]
            metadata = doc.get("metadata", {})
            embedding = doc.get("embedding", [0.0] * 1536)  # Placeholder

            payload = DocumentPayload(
                content=content,
                module=module,
                chapter=chapter,
                page_reference=page_reference,
                metadata=metadata
            )

            point = models.PointStruct(
                id=doc_id,
                vector=embedding,
                payload=payload.dict()
            )

            points.append(point)

        try:
            self.client.upsert(
                collection_name=self.collection_name,
                points=points
            )
            logger.info(f"Batch added {len(documents)} documents to collection {self.collection_name}")
        except Exception as e:
            logger.error(f"Error batch adding documents: {e}")
            raise

    def delete_collection(self):
        """Delete the entire collection (use with caution!)"""
        try:
            self.client.delete_collection(collection_name=self.collection_name)
            logger.info(f"Deleted collection {self.collection_name}")
        except Exception as e:
            logger.error(f"Error deleting collection: {e}")
            raise