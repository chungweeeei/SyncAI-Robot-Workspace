import os

# Temporal server frontend gRPC endpoint. Defaults to the local docker-compose
# service (`temporal:7233` inside the compose network / `127.0.0.1:7233` on host).
TEMPORAL_SERVER_URL = os.getenv("TEMPORAL_ADDRESS", "127.0.0.1:7233")
