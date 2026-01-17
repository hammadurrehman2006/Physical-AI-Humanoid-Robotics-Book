from typing import Annotated
from fastapi import Depends
from app.models.auth import User
from app.core.security import current_active_user

CurrentUser = Annotated[User, Depends(current_active_user)]