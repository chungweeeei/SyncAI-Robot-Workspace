#!/usr/bin/env bash
# =============================================================================
# Make the directory an engineer carries to a robot that cannot reach the
# registry: the release image as a tar, its checksum, and the script that
# installs it. Run on a machine WITH network access (a laptop; macOS or
# Linux), from a checkout of this repository:
#
#   scripts/release/save_bundle.sh <version> [--with-infra] [--out <dir>]
#   scripts/release/save_bundle.sh 20261119-1530
#
# <version> is the build time CI printed, the part of the image tag after
# `release-` (ghcr.io/<owner>/<repo>:release-<version>). The result is
#
#   dist/syncai_<version>/
#     robot_<version>.tar.gz   the image, `docker save` | gzip
#     infra.tar.gz             postgres + temporal, with --with-infra
#     SHA256SUMS               of the tars; install.sh checks it first
#     install.sh               run this on the robot
#     MANIFEST                 what the image was built from, for a person
#
# Nothing here builds anything: CI is the only place a release image is made
# (.github/workflows/release.yml), and this pulls what it pushed. install.sh
# and MANIFEST are taken out of the image, not out of this checkout, which may
# be weeks ahead of or behind the version being bundled -- the script that
# installs an image is the one that image shipped with.
#
# --with-infra: a robot in a plant cannot pull postgres or temporal either.
# Their tags are pinned in the compose file, so the tar is needed on a robot's
# first install and again only when a pin changes, and is left out by default
# so that an update does not carry two images nobody will load. The names are
# read from the image's own docker-compose.release.yml.
#
# The image is linux/arm64 (the robots are Jetsons). `docker pull --platform`
# fetches it on any machine; nothing in here runs it, so an amd64 laptop needs
# no emulation.
#
# If the package is private on ghcr, `docker login ghcr.io` first. A robot
# that does reach the registry needs no bundle at all:
# `install.sh --image ghcr.io/...:release-<version>` pulls it there.
# =============================================================================
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/../.."

PLATFORM=linux/arm64
IMAGE_WS=/home/syncrobotic/robot_ws

version=""
with_infra=0
out=""
while [ $# -gt 0 ]; do
    case "$1" in
        --with-infra) with_infra=1 ;;
        --out) [ $# -ge 2 ] || { echo "save_bundle.sh: --out needs a directory" >&2; exit 2; }
               out="$2"; shift ;;
        -h|--help) sed -n '2,/^# ====.*$/p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 0 ;;
        -*) echo "save_bundle.sh: unknown option '$1'" >&2; exit 2 ;;
        *) [ -z "${version}" ] || { echo "save_bundle.sh: one version at a time" >&2; exit 2; }
           version="$1" ;;
    esac
    shift
done

die() { printf 'save_bundle.sh: %s\n' "$*" >&2; exit 1; }
step() { printf '\n==> %s\n' "$*"; }

[ -n "${version}" ] || die "which version? e.g. scripts/release/save_bundle.sh 20261119-1530"
version="${version#release-}"
command -v docker >/dev/null || die "docker is not installed."

# ghcr.io/<owner>/<repo>, lowercase, from this checkout's origin unless
# SYNCAI_IMAGE_REPO names it -- the same name release.yml derives from
# GITHUB_REPOSITORY.
repo="${SYNCAI_IMAGE_REPO:-}"
if [ -z "${repo}" ]; then
    url="$(git config --get remote.origin.url 2>/dev/null || true)"
    path="$(printf '%s' "${url}" | sed -E 's#^(git@|https://)github\.com[:/]##; s#\.git$##')"
    [ -n "${path}" ] && [ "${path}" != "${url}" ] || \
        die "cannot tell the image repository from origin ('${url}'); set SYNCAI_IMAGE_REPO=ghcr.io/<owner>/<repo>."
    repo="ghcr.io/$(printf '%s' "${path}" | tr '[:upper:]' '[:lower:]')"
fi
ref="${repo}:release-${version}"

# sha256sum on Linux, shasum on macOS; the file format is the same, and the
# robot checks it with sha256sum.
if command -v sha256sum >/dev/null; then
    sha256() { sha256sum "$@"; }
else
    sha256() { shasum -a 256 "$@"; }
fi

out="${out:-dist/syncai_${version}}"
[ ! -e "${out}" ] || die "${out} already exists -- remove it, or pick another --out."

# A pull that fails is not the end when the image is already here: a laptop
# that pulled yesterday and is offline today still makes the bundle. Said out
# loud, because a local copy can also be one somebody tagged by hand.
step "pulling ${ref} (${PLATFORM})"
if ! docker pull --platform "${PLATFORM}" "${ref}"; then
    docker image inspect "${ref}" >/dev/null 2>&1 || \
        die "cannot pull ${ref}, and there is no local copy. Is the version right (see the release workflow's summary), and are you logged in to ghcr.io?"
    echo "save_bundle.sh: WARNING: could not pull ${ref}; bundling the copy already on this machine." >&2
fi

mkdir -p "${out}"
stage="$(mktemp -d)"
cid=""
cleanup() {
    [ -z "${cid}" ] || docker rm -f "${cid}" >/dev/null 2>&1 || true
    rm -rf "${stage}"
}
trap cleanup EXIT

step "taking install.sh and MANIFEST out of the image"
cid="$(docker create --platform "${PLATFORM}" "${ref}")"
docker cp "${cid}:${IMAGE_WS}/deploy" "${stage}/deploy" || \
    die "${ref} has no ${IMAGE_WS}/deploy -- not a release image."
docker cp "${cid}:${IMAGE_WS}/MANIFEST" "${out}/MANIFEST"
cp "${stage}/deploy/install.sh" "${out}/install.sh"
chmod +x "${out}/install.sh"

step "saving the image (a few minutes)"
docker save "${ref}" | gzip > "${out}/robot_${version}.tar.gz"
tars=("robot_${version}.tar.gz")

if [ "${with_infra}" = 1 ]; then
    # Every `image:` line of the image's compose file but the robot's own,
    # which is a variable.
    infra="$(sed -n 's/^[[:space:]]*image:[[:space:]]*//p' "${stage}/deploy/docker-compose.release.yml" | grep -v '\$' || true)"
    [ -n "${infra}" ] || die "found no infra image in the image's docker-compose.release.yml."
    step "pulling and saving the infra images"
    for img in ${infra}; do
        echo "  ${img}"
        docker pull --platform "${PLATFORM}" "${img}" >/dev/null || die "cannot pull ${img}."
    done
    # shellcheck disable=SC2086
    docker save ${infra} | gzip > "${out}/infra.tar.gz"
    tars+=("infra.tar.gz")
fi

(cd "${out}" && sha256 "${tars[@]}" > SHA256SUMS)

step "done: ${out}"
ls -lh "${out}" | sed 's/^/  /'
cat <<EOF

On the robot, from a copy of this directory:

  ./install.sh              # first install, and every update
  ./install.sh --rollback   # back to the image that ran before

$(sed -n '1,4p' "${out}/MANIFEST" | sed 's/^/  /')
EOF
