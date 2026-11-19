#!/usr/bin/env bash
# =============================================================================
# Step one of a release build: decide the version, refuse a tree that cannot
# be traced, and write the manifest the image carries. Run from anywhere, on
# the machine that then runs `docker build --target release` -- which is the
# CI runner (.github/workflows/release.yml); there is no supported local
# release build.
#
#   scripts/release/prepare.sh [--allow-dirty]
#
# Writes ./.release-manifest (gitignored; the Dockerfile's release stage copies
# it to ~/robot_ws/MANIFEST) and prints `version=` and `revision=` lines, which
# also go to $GITHUB_OUTPUT when that is set.
#
# The version is the build time, YYYYMMDD-HHMM in Asia/Taipei, the image's own
# timezone and the team's: it sorts, it needs no tag to be pushed first (this
# repository has none), and two builds of one commit are still two versions.
# It says nothing about what was built, which is what the manifest is for.
# SYNCAI_VERSION in the environment overrides it.
#
# The manifest matters more here than in most projects because
# dependencies.repos pins nothing: its seven entries follow each repository's
# `dev` branch, so "this workspace at commit X" does not say which
# syncai_common or syncai_mapping went into an image. The SHA of every
# checkout under src/ is read from the checkout itself -- what is on disk is
# what the builder stage compiles, whatever a .repos file says -- with plain
# git, so the runner needs no vcstool for it.
#
# A dirty tree is refused: a manifest that names a commit is a lie if the
# sources differed from it. --allow-dirty builds anyway and writes DIRTY into
# the manifest, for trying the pipeline itself; never for an image that goes
# to a robot.
#
# What the manifest does not give is a reproducible build. The image's apt
# packages are not version-pinned, so the same SHAs built again in three
# months link different system libraries. The artefact to keep is the image.
# =============================================================================
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/../.."

allow_dirty=0
for arg in "$@"; do
    case "${arg}" in
        --allow-dirty) allow_dirty=1 ;;
        *) echo "prepare.sh: unknown argument '${arg}' (only --allow-dirty)" >&2; exit 2 ;;
    esac
done

die() { printf 'prepare.sh: %s\n' "$*" >&2; exit 1; }

command -v git >/dev/null || die "git is not installed"
git rev-parse --git-dir >/dev/null 2>&1 || die "not a git checkout: $(pwd)"

# "clean" or "DIRTY" for the checkout at $1. Untracked files count: the
# builder stage COPYs src/ as it is on disk, tracked or not.
tree_state() {
    if [ -z "$(git -C "$1" status --porcelain 2>/dev/null)" ]; then
        echo clean
    else
        echo DIRTY
    fi
}

dirty=""
root_sha="$(git rev-parse HEAD)"
root_ref="$(git rev-parse --abbrev-ref HEAD)"
root_state="$(tree_state .)"
[ "${root_state}" = clean ] || dirty="${dirty} ."

# Every vcs checkout under src/: the seven of dependencies.repos and the six
# of third-party.repos. Found by their .git rather than read out of the .repos
# files, so a checkout someone added by hand is listed as well -- it is
# compiled as well.
checkouts=""
count=0
for dir in src/*/ src/third-party/*/; do
    dir="${dir%/}"
    [ -e "${dir}/.git" ] || continue
    sha="$(git -C "${dir}" rev-parse HEAD)"
    ref="$(git -C "${dir}" rev-parse --abbrev-ref HEAD)"
    url="$(git -C "${dir}" config --get remote.origin.url || echo '-')"
    state="$(tree_state "${dir}")"
    [ "${state}" = clean ] || dirty="${dirty} ${dir}"
    checkouts="${checkouts}$(printf '  %-52s %s  %-8s %-5s %s' "${dir}" "${sha}" "${ref}" "${state}" "${url}")"$'\n'
    count=$((count + 1))
done

# An unimported workspace has no checkouts at all, and its image would have no
# syncai_common. build.sh says so later, at length; saying so here costs one
# line and saves the image build that precedes it.
[ "${count}" -gt 0 ] || die "no vcs checkout under src/ -- import both .repos files first:" \
    "vcs import --recursive < third-party.repos && vcs import < dependencies.repos"

state=clean
if [ -n "${dirty}" ]; then
    state=DIRTY
    if [ "${allow_dirty}" != 1 ]; then
        printf 'prepare.sh: uncommitted or untracked changes in:\n' >&2
        for d in ${dirty}; do
            printf '  %s\n' "${d}" >&2
            git -C "${d}" status --porcelain | sed -n "1,10p" | sed 's/^/      /' >&2
        done
        die "refusing to release from a tree its manifest could not describe" \
            "(--allow-dirty to build anyway, marked DIRTY)."
    fi
fi

version="${SYNCAI_VERSION:-$(TZ=Asia/Taipei date +%Y%m%d-%H%M)}"
case "${version}" in
    *[!A-Za-z0-9._-]*|'') die "version '${version}' is not usable as an image tag" ;;
esac

{
    echo "version:    ${version}"
    echo "built:      $(TZ=Asia/Taipei date +%Y-%m-%dT%H:%M:%S%z)"
    echo "tree:       ${state}"
    echo "workspace:  ${root_sha}  ${root_ref}  ${root_state}"
    echo "            $(git config --get remote.origin.url || echo '-')"
    echo "not built:  direct_visual_lidar_calibration vizionsdk_ros2 (Dockerfile, builder stage)"
    echo "checkouts:"
    printf '%s' "${checkouts}"
} > .release-manifest

echo "version=${version}"
echo "revision=${root_sha}"
if [ -n "${GITHUB_OUTPUT:-}" ]; then
    {
        echo "version=${version}"
        echo "revision=${root_sha}"
    } >> "${GITHUB_OUTPUT}"
fi
echo "--- .release-manifest ---" >&2
cat .release-manifest >&2
