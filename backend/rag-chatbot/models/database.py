from sqlalchemy import create_engine, Column, Integer, String, DateTime, Text, ForeignKey
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker, relationship
from sqlalchemy.sql import func
from pydantic_settings import BaseSettings
from typing import Optional
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Configuration for Neon Postgres
class DatabaseConfig(BaseSettings):
    """Configuration for Neon Postgres database"""

    NEON_DATABASE_URL: Optional[str] = os.getenv("NEON_DATABASE_URL")
    DATABASE_URL: str = "postgresql://default:your-password@ep-xxx.us-east-1.aws.neon.tech/your-db-name?sslmode=require"

    class Config:
        env_prefix = "NEON_"
        case_sensitive = True
        env_file = ".env"
        extra = "ignore"

# Initialize configuration
db_config = DatabaseConfig()

# Use the configured database URL or default to a placeholder
DATABASE_URL = db_config.NEON_DATABASE_URL or db_config.DATABASE_URL

# Create engine
engine = create_engine(
    DATABASE_URL,
    pool_pre_ping=True,  # Verify connections before use
    pool_recycle=300,    # Recycle connections after 5 minutes
    echo=False           # Set to True for SQL debugging
)

# Create session factory
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)

# Base class for declarative models
Base = declarative_base()

# Define models for the robotics book data
class Module(Base):
    """Model for book modules"""
    __tablename__ = "modules"

    id = Column(Integer, primary_key=True, index=True)
    name = Column(String, index=True, nullable=False)
    description = Column(Text)
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now())

    # Relationship to chapters
    chapters = relationship("Chapter", back_populates="module")


class Chapter(Base):
    """Model for book chapters"""
    __tablename__ = "chapters"

    id = Column(Integer, primary_key=True, index=True)
    module_id = Column(Integer, ForeignKey("modules.id"), nullable=False)
    title = Column(String, nullable=False)
    content = Column(Text)
    page_reference = Column(String)
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now())

    # Relationship to module
    module = relationship("Module", back_populates="chapters")


class UserQuery(Base):
    """Model for tracking user queries and responses"""
    __tablename__ = "user_queries"

    id = Column(Integer, primary_key=True, index=True)
    question = Column(Text, nullable=False)
    answer = Column(Text, nullable=False)
    confidence_score = Column(Integer)  # Confidence score from 0-100
    sources = Column(Text)  # JSON string of sources
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    user_id = Column(String, index=True)  # Optional user identifier


class GlossaryTerm(Base):
    """Model for glossary terms"""
    __tablename__ = "glossary_terms"

    id = Column(Integer, primary_key=True, index=True)
    term = Column(String, unique=True, index=True, nullable=False)
    definition = Column(Text, nullable=False)
    module = Column(String)  # Associated module
    category = Column(String)  # Category like 'hardware', 'software', 'algorithm', etc.
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now())


def get_db():
    """Dependency to get database session"""
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()


# Create all tables
def init_db():
    """Initialize the database by creating all tables"""
    Base.metadata.create_all(bind=engine)


if __name__ == "__main__":
    init_db()
    print("Database tables created successfully!")