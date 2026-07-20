#!/bin/bash
# =============================================================================
# Build the frontend production image natively on this machine (arm64, same
# arch as the IPC — no cross-compile).
#
# Usage:
#   bash scripts/release/build_images.sh [TAG]
#
# TAG defaults to `git describe`-based versioning. A "-dirty" tag means the
# working tree has uncommitted changes — treat such a build as NON-RELEASABLE.
#
# NOTE: the nav (syncai-nav) and backend (syncai-backend) images are NOT built
# here — their Dockerfile stages (nav-runtime / backend-runtime) were removed
# while the project is in the dev phase. When they are re-added (see git
# history), restore the two `docker build --target ...` lines, the ldd audit,
# and their entries in save_images.sh / docker-compose.prod.yml.
# =============================================================================
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/../.."

TAG="${1:-$(git describe --tags --always --dirty)-$(date +%Y%m%d)}"

echo "==> Building syncai images with TAG=${TAG}"
case "$TAG" in
  *-dirty-*) echo "WARNING: working tree is dirty — this build is not releasable." ;;
esac

docker build -t "syncai-frontend:${TAG}" src/syncai_frontend

# Convenience tag for local smoke testing.
docker tag "syncai-frontend:${TAG}" syncai-frontend:latest-prod

echo "==> Done. Images:"
docker images --format '{{.Repository}}:{{.Tag}}\t{{.Size}}' | grep -E "syncai-frontend:${TAG}"
echo
echo "Next: bash scripts/release/save_images.sh ${TAG}"
