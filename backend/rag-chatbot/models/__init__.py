"""
Models module for the RAG Chatbot API
Contains database models and configuration for Neon Postgres
"""

from .database import (
    Base,
    SessionLocal,
    engine,
    Module,
    Chapter,
    UserQuery,
    GlossaryTerm,
    get_db,
    init_db,
    db_config
)

__all__ = [
    "Base",
    "SessionLocal",
    "engine",
    "Module",
    "Chapter",
    "UserQuery",
    "GlossaryTerm",
    "get_db",
    "init_db",
    "db_config"
]