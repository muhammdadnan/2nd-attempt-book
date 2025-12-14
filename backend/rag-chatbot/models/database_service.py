"""
Database service layer for the RAG Chatbot API
Provides methods to interact with Neon Postgres database
"""

from sqlalchemy.orm import Session
from sqlalchemy import and_, or_
from typing import List, Optional
from ..models.database import Module, Chapter, UserQuery, GlossaryTerm


class DatabaseService:
    """Service class for database operations"""

    def __init__(self, db: Session):
        self.db = db

    def get_modules(self) -> List[Module]:
        """Get all modules"""
        return self.db.query(Module).all()

    def get_module_by_name(self, name: str) -> Optional[Module]:
        """Get a module by name"""
        return self.db.query(Module).filter(Module.name == name).first()

    def get_chapters_by_module(self, module_id: int) -> List[Chapter]:
        """Get all chapters for a specific module"""
        return self.db.query(Chapter).filter(Chapter.module_id == module_id).all()

    def get_chapter_by_title(self, title: str) -> Optional[Chapter]:
        """Get a chapter by title"""
        return self.db.query(Chapter).filter(Chapter.title == title).first()

    def create_user_query(self, question: str, answer: str, confidence_score: int = None,
                         sources: str = None, user_id: str = None) -> UserQuery:
        """Create a record of a user query"""
        db_query = UserQuery(
            question=question,
            answer=answer,
            confidence_score=confidence_score,
            sources=sources,
            user_id=user_id
        )
        self.db.add(db_query)
        self.db.commit()
        self.db.refresh(db_query)
        return db_query

    def get_glossary_term(self, term: str) -> Optional[GlossaryTerm]:
        """Get a glossary term by name"""
        return self.db.query(GlossaryTerm).filter(GlossaryTerm.term == term).first()

    def get_glossary_terms_by_category(self, category: str) -> List[GlossaryTerm]:
        """Get glossary terms by category"""
        return self.db.query(GlossaryTerm).filter(GlossaryTerm.category == category).all()

    def get_glossary_terms_by_module(self, module: str) -> List[GlossaryTerm]:
        """Get glossary terms by module"""
        return self.db.query(GlossaryTerm).filter(GlossaryTerm.module == module).all()

    def search_glossary_terms(self, search_term: str) -> List[GlossaryTerm]:
        """Search glossary terms by partial match"""
        return self.db.query(GlossaryTerm).filter(
            or_(
                GlossaryTerm.term.contains(search_term),
                GlossaryTerm.definition.contains(search_term)
            )
        ).all()

    def get_all_glossary_terms(self) -> List[GlossaryTerm]:
        """Get all glossary terms"""
        return self.db.query(GlossaryTerm).all()

    def create_glossary_term(self, term: str, definition: str, module: str = None,
                           category: str = None) -> GlossaryTerm:
        """Create a new glossary term"""
        db_term = GlossaryTerm(
            term=term,
            definition=definition,
            module=module,
            category=category
        )
        self.db.add(db_term)
        self.db.commit()
        self.db.refresh(db_term)
        return db_term

    def update_glossary_term(self, term: str, definition: str = None,
                           module: str = None, category: str = None) -> Optional[GlossaryTerm]:
        """Update an existing glossary term"""
        db_term = self.get_glossary_term(term)
        if not db_term:
            return None

        if definition is not None:
            db_term.definition = definition
        if module is not None:
            db_term.module = module
        if category is not None:
            db_term.category = category

        self.db.commit()
        self.db.refresh(db_term)
        return db_term

    def delete_glossary_term(self, term: str) -> bool:
        """Delete a glossary term"""
        db_term = self.get_glossary_term(term)
        if not db_term:
            return False

        self.db.delete(db_term)
        self.db.commit()
        return True