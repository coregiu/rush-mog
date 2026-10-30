from loguru import logger
import sys

logger.remove()
logger.add(sys.stderr, level="INFO")
logger.add("center_server.log", rotation="200MB")
