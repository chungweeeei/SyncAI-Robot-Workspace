#!/bin/bash
# =============================================================================
# Package a release for offline delivery to the IPC:
#   dist/deploy_<TAG>/
#     syncai_<TAG>.tar        one combined docker-save archive (product + infra
#                             images; shared layers are stored ONCE — do not
#                             split into per-image tars, that doubles the size)
#     syncai_<TAG>.tar.sha256
#     docker-compose.prod.yml
#     .env.example
#     config/                 (incl. instances/ and cyclonedds xmls)
#     map/                    (current maps; optional, can be large)
#     load_and_up.sh
#     MANIFEST
#
# Usage:
#   bash scripts/release/save_images.sh <TAG> [--no-map]
#
# The IPC is offline, so the infra images (postgres/temporal/...) must ship
# in the same tar.
# =============================================================================
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/../.."

TAG="${1:?usage: save_images.sh <TAG> [--no-map]}"
INCLUDE_MAP=1
[ "${2:-}" = "--no-map" ] && INCLUDE_MAP=0

INFRA_IMAGES=(
  postgres:16
  temporalio/auto-setup:1.29.7
  temporalio/ui:2.52.1
  dpage/pgadmin4:latest
)
# NOTE: syncai-nav / syncai-backend are omitted — their Dockerfile stages were
# removed during the dev phase (see build_images.sh). Re-add them here when the
# stages come back.
PRODUCT_IMAGES=(
  "syncai-frontend:${TAG}"
)

BUNDLE="dist/deploy_${TAG}"
mkdir -p "$BUNDLE"

echo "==> Verifying images exist locally"
for img in "${PRODUCT_IMAGES[@]}" "${INFRA_IMAGES[@]}"; do
  if ! docker image inspect "$img" > /dev/null 2>&1; then
    echo "==> $img missing locally, pulling..."
    docker pull "$img"
  fi
done

echo "==> docker save (single combined tar — this can take a while)"
docker save "${PRODUCT_IMAGES[@]}" "${INFRA_IMAGES[@]}" -o "${BUNDLE}/syncai_${TAG}.tar"

echo "==> checksum"
( cd "$BUNDLE" && sha256sum "syncai_${TAG}.tar" > "syncai_${TAG}.tar.sha256" )

echo "==> assembling deploy bundle"
cp docker-compose.prod.yml "$BUNDLE/"
cp scripts/release/.env.example "$BUNDLE/"
cp scripts/release/load_and_up.sh "$BUNDLE/"
rsync -a --exclude 'rviz2' config/ "$BUNDLE/config/"
if [ "$INCLUDE_MAP" = 1 ] && [ -d map ]; then
  rsync -a map/ "$BUNDLE/map/"
fi

cat > "$BUNDLE/MANIFEST" <<EOF
tag:        ${TAG}
git_sha:    $(git rev-parse HEAD)
git_status: $(git diff --quiet && git diff --cached --quiet && echo clean || echo DIRTY)
built_on:   $(date -Iseconds) $(uname -m)
images:
$(printf '  %s\n' "${PRODUCT_IMAGES[@]}" "${INFRA_IMAGES[@]}")
EOF

echo "==> Done: $BUNDLE"
du -sh "$BUNDLE" "$BUNDLE/syncai_${TAG}.tar"
echo
echo "Move the whole deploy_${TAG}/ directory to the IPC (USB/scp), then run:"
echo "  bash load_and_up.sh ${TAG}"
