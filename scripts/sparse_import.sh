#!/usr/bin/env bash
# Materialise the entries of third-party-sparse.repos: third-party repos of
# which only some directories are wanted (that file's header has the keys).
#
#   scripts/sparse_import.sh                 # clone / move to the pin, then the markers
#   scripts/sparse_import.sh --markers-only  # COLCON_IGNOREs only, no network
#
# vcstool clones a repository whole, and DEEP Robotics' sdk_deploy is ~810 MB
# of which two directories are wanted; this is the clone vcstool cannot make:
# partial (--filter=blob:none: commits and trees, no file contents until a
# checkout needs them) with a cone-mode sparse checkout of the `sparse:`
# directories, detached at the pinned SHA.
#
# Called by scripts/sync.sh on the host and by CI after its vcs imports;
# scripts/build.sh runs --markers-only, because a build must not reach the
# network or move a checkout, only make sure colcon skips what it should.
# Plain git, like sync.sh, so it runs on a host without vcstool.
#
# Never destructive, as sync.sh: a checkout on a branch is left alone and
# reported; one at a detached HEAD is moved to the pin (git refuses a move that
# would overwrite a modified file, reported as a failure). An existing whole
# checkout is narrowed to the `sparse:` set. One entry failing does not stop
# the others; the failures are listed and the exit code is 1.
set -uo pipefail
cd "$(dirname "$0")/.."

markers_only=false
case "${1:-}" in
    "") ;;
    --markers-only) markers_only=true ;;
    *) echo "usage: $0 [--markers-only]" >&2; exit 2 ;;
esac

REPOS=third-party-sparse.repos

# `path|url|version|sparse|build` per entry. The two lists are kept as
# space-separated strings, hence the | separator.
entries() {
    awk '/^  src\//{p=$1; sub(":$", "", p); u=v=s=b=""}
         /^    url:/{u=$2}
         /^    version:/{v=$2; gsub(/"/, "", v)}
         /^    sparse:/{s=$0; sub(/^    sparse:[ ]*/, "", s)}
         /^    build:/{b=$0; sub(/^    build:[ ]*/, "", b); print p "|" u "|" v "|" s "|" b}' "$1"
}

# A COLCON_IGNORE in every directory of checkout $1 that has a package.xml and
# is not one of the `build:` directories $2 -- found by package.xml rather than
# named, so a package the vendor adds after a pin bump, or the whole repo
# checked out by hand, is skipped too. A COLCON_IGNORE left in a directory that
# is now in `build:` (one moved there by a commit) is removed.
markers() {
    local path=$1 build=$2 pkg_xml dir rel keep b
    while IFS= read -r pkg_xml; do
        dir=$(dirname "$pkg_xml")
        rel=${dir#"$path"/}
        keep=false
        for b in $build; do [ "$rel" = "$b" ] && keep=true; done
        if $keep; then
            if [ -f "$dir/COLCON_IGNORE" ]; then
                echo "== $dir: building it, COLCON_IGNORE removed"
                rm -f "$dir/COLCON_IGNORE"
            fi
        elif [ ! -f "$dir/COLCON_IGNORE" ]; then
            echo "== $dir: not in build:, COLCON_IGNORE"
            touch "$dir/COLCON_IGNORE"
        fi
    done < <(find "$path" -name .git -prune -o -name package.xml -print)
}

pin_commit() {
    git -C "$1" rev-parse -q --verify "$2^{commit}" 2>/dev/null
}

failed=()

while IFS='|' read -r path url ver sparse build; do
    if ! $markers_only; then
        if [ ! -e "$path/.git" ]; then
            echo "== $path: cloning $ver (sparse: $sparse)"
            # --sparse starts the checkout at the top-level files only; the
            # directories are added below, so nothing else is ever written.
            git clone -q --filter=blob:none --no-checkout --sparse "$url" "$path" \
                || { failed+=("$path"); continue; }
        elif [ "$(git -C "$path" rev-parse --abbrev-ref HEAD)" != HEAD ]; then
            echo "!! $path: on a branch, not at '$ver', left alone"
            markers "$path" "$build"
            continue
        fi
        # Our own COLCON_IGNOREs outside the sparse set go first: git leaves a
        # directory it drops from the checkout in place while an untracked file
        # is in it, so a whole checkout being narrowed would keep a skeleton of
        # every other package, each holding nothing but its marker.
        while IFS= read -r marker; do
            rel=${marker#"$path"/}
            inside=false
            for s in $sparse; do
                case "$rel" in "$s"/*) inside=true ;; esac
            done
            $inside || rm -f "$marker"
        done < <(find "$path" -name .git -prune -o -name COLCON_IGNORE -print)
        # shellcheck disable=SC2086  # the list is meant to split
        git -C "$path" sparse-checkout set --cone $sparse || { failed+=("$path"); continue; }
        if [ -z "$(pin_commit "$path" "$ver")" ]; then
            git -C "$path" fetch -q --filter=blob:none origin "$ver" \
                || { failed+=("$path"); continue; }
        fi
        if [ "$(pin_commit "$path" "$ver")" != "$(git -C "$path" rev-parse -q --verify HEAD 2>/dev/null)" ] \
            || [ -z "$(ls -A "$path" | grep -v '^\.git$')" ]; then
            echo "== $path: checking out $ver"
            git -C "$path" -c advice.detachedHead=false checkout -q "$(pin_commit "$path" "$ver")" \
                || { failed+=("$path"); continue; }
        fi
    fi
    if [ -d "$path" ]; then
        markers "$path" "$build"
    fi
done < <(entries "$REPOS")

if [ ${#failed[@]} -ne 0 ]; then
    echo "failed: ${failed[*]}"
    exit 1
fi
