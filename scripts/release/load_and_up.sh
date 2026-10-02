#!/bin/bash
# =============================================================================
# IPC-side installer: verify + load the image tar and bring the stack up.
# Run from inside the deploy bundle directory (deploy_<TAG>/):
#   bash load_and_up.sh <TAG>
#
# Idempotent: re-running with the same TAG just re-ups the stack. Rollback:
# keep the previous bundle on disk and run its load_and_up.sh with the old
# TAG (docker compose recreates the containers from the old images).
# =============================================================================
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")"

TAG="${1:?usage: load_and_up.sh <TAG>}"

if [ ! -f .env ]; then
  echo "No .env found — creating from .env.example."
  cp .env.example .env
  echo ">>> EDIT .env NOW (credentials, ROBOT_ID) then re-run this script. <<<"
  exit 1
fi

echo "==> verifying checksum"
sha256sum -c "syncai_${TAG}.tar.sha256"

echo "==> docker load (needs roughly the tar's size in free space on top of the images)"
docker load -i "syncai_${TAG}.tar"

mkdir -p data/postgres map

# Persist TAG so later plain `docker compose -f docker-compose.prod.yml up -d`
# (or restarts) resolve the same image versions.
if grep -q '^TAG=' .env; then
  sed -i "s/^TAG=.*/TAG=${TAG}/" .env
else
  echo "TAG=${TAG}" >> .env
fi

echo "==> starting stack"
docker compose -f docker-compose.prod.yml up -d

echo "==> waiting a moment for healthchecks..."
sleep 15
docker compose -f docker-compose.prod.yml ps

cat <<'EOF'

Done. Quick checks:
  docker compose -f docker-compose.prod.yml ps        # all healthy/running?
  docker logs -f nav                                  # nav stack bringup
  docker logs -f backend                              # DB + temporal connect
  curl http://127.0.0.1:3000/health                   # backend REST
  http://<ipc>:3001                                   # web UI

Debug UIs (temporal-ui :8081, pgadmin :5050):
  docker compose -f docker-compose.prod.yml --profile debug up -d

Cleanup after upgrades:
  docker image prune
EOF
