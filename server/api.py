from fastapi import FastAPI, APIRouter
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from server.database.dynamodb import save_key_location

app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

router = APIRouter()

class KeyLocation(BaseModel):
    name: str
    x: float
    y: float
    theta: float

@router.post("/store_key_location")
def store_key_location(loc: KeyLocation):
    save_key_location(loc.name, loc.x, loc.y, loc.theta)
    return {"status": "ok", "saved": loc}

app.include_router(router)
