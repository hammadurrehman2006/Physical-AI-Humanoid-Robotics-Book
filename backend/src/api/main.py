from fastapi import FastAPI, Depends
from fastapi.middleware.cors import CORSMiddleware
from .chat_routes import router as chat_router
from .health_routes import router as health_router
from ..config.database import DatabaseConfig


app = FastAPI(
    title="Physical AI & Humanoid Robotics RAG Chatbot API",
    description="API for the integrated chatbot that enables learners to ask questions about book content",
    version="1.0.0"
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, specify exact origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include routers
app.include_router(chat_router, prefix="/api/v1", tags=["chat"])
app.include_router(health_router, prefix="/api/v1", tags=["health"])

# Create tables on startup
@app.on_event("startup")
def startup_event():
    DatabaseConfig.create_all_tables()

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)