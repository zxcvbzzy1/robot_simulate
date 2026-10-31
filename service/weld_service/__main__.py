"""python -m weld_service: read .env, then run one API worker."""
import uvicorn
from . import settings


def main():
    uvicorn.run("weld_service.api:app", host=settings.API_HOST, port=settings.API_PORT, workers=1)


if __name__ == "__main__":
    main()
