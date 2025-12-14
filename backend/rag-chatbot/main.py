from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from typing import List, Optional
import os
from dotenv import load_dotenv
from openai import OpenAI
import logging

# Load environment variables
load_dotenv()

# Initialize logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Initialize services
from vector_db import QdrantService, qdrant_config

try:
    qdrant_service = QdrantService(
        host=qdrant_config.QDRANT_HOST,
        port=qdrant_config.QDRANT_PORT,
        collection_name=qdrant_config.QDRANT_COLLECTION_NAME
    )
    logger.info("Qdrant service initialized successfully")
except Exception as e:
    logger.error(f"Failed to initialize Qdrant service: {e}")
    qdrant_service = None

# Initialize OpenAI client
try:
    openai_client = OpenAI()
    logger.info("OpenAI client initialized successfully")
except Exception as e:
    logger.error(f"Failed to initialize OpenAI client: {e}")
    openai_client = None

app = FastAPI(
    title="RAG Chatbot API for Physical AI & Humanoid Robotics Book",
    description="API for querying the robotics book content using Retrieval-Augmented Generation",
    version="1.0.0"
)

# Pydantic models
class QueryRequest(BaseModel):
    question: str
    context_module: Optional[str] = None

class Source(BaseModel):
    chapter: str
    module: str
    page_reference: str

class QueryResponse(BaseModel):
    answer: str
    confidence: float
    sources: List[Source]

class HealthResponse(BaseModel):
    status: str
    timestamp: str

def get_embedding(text: str) -> List[float]:
    """
    Get embedding for text using OpenAI API
    In a real implementation, this would call the OpenAI embeddings API
    """
    global openai_client
    if not openai_client:
        # Return a dummy embedding for demonstration
        return [0.0] * 1536

    try:
        # Use OpenAI's text-embedding-ada-002 model
        response = openai_client.embeddings.create(
            input=text,
            model="text-embedding-ada-002"
        )
        return response.data[0].embedding
    except Exception as e:
        logger.error(f"Error getting embedding: {e}")
        # Return a dummy embedding as fallback
        return [0.0] * 1536


def generate_answer_with_context(question: str, context_chunks: List[Dict]) -> str:
    """
    Generate an answer using OpenAI API based on the context chunks
    """
    global openai_client
    if not openai_client:
        return "I'm sorry, but the OpenAI service is not available."

    try:
        # Format context for the LLM
        context_text = "\n\n".join([chunk["content"] for chunk in context_chunks])

        # Create a prompt for the LLM
        prompt = f"""
        You are an expert assistant for the Physical AI & Humanoid Robotics book.
        Answer the following question based on the provided context.

        Question: {question}

        Context: {context_text}

        Please provide a detailed answer with references to the relevant modules and chapters.
        If the context doesn't contain enough information to answer the question, say so.
        """

        response = openai_client.chat.completions.create(
            model="gpt-3.5-turbo",
            messages=[{"role": "user", "content": prompt}],
            max_tokens=500,
            temperature=0.7
        )

        return response.choices[0].message.content.strip()
    except Exception as e:
        logger.error(f"Error generating answer: {e}")
        return "I encountered an error while generating the answer. Please try again."

@app.get("/health", response_model=HealthResponse)
async def health_check():
    """
    Health check endpoint to verify the service is running
    """
    from datetime import datetime
    return HealthResponse(
        status="healthy",
        timestamp=datetime.now().isoformat()
    )

@app.post("/query", response_model=QueryResponse)
async def query_book(request: QueryRequest):
    """
    Query the book content using natural language
    """
    global qdrant_service
    if not qdrant_service:
        raise HTTPException(status_code=500, detail="Qdrant service is not available")

    try:
        # Get embedding for the question
        question_embedding = get_embedding(request.question)

        # Search for relevant documents in Qdrant
        search_results = qdrant_service.search_documents(
            query_embedding=question_embedding,
            limit=5,
            module_filter=request.context_module
        )

        if not search_results:
            return QueryResponse(
                answer="I couldn't find any relevant information in the book to answer your question.",
                confidence=0.0,
                sources=[]
            )

        # Generate answer using context from search results
        answer = generate_answer_with_context(request.question, search_results)

        # Extract sources from search results
        sources = []
        for result in search_results:
            source = Source(
                chapter=result["chapter"],
                module=result["module"],
                page_reference=result["page_reference"]
            )
            sources.append(source)

        # Calculate a confidence score based on the highest similarity score
        max_score = max([result["score"] for result in search_results])
        confidence = min(max_score, 1.0)  # Ensure confidence is between 0 and 1

        return QueryResponse(
            answer=answer,
            confidence=confidence,
            sources=sources
        )
    except Exception as e:
        logger.error(f"Error processing query: {e}")
        raise HTTPException(status_code=500, detail="Error processing query")

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)