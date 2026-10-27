#!/usr/bin/env bash
# Pull the workspace AND every checkout dependencies.repos materialises in src/.
#
#   scripts/sync.sh
#
# A `git pull` of this repo never touches those six checkouts: they are
# gitignored, and every entry in dependencies.repos tracks the `dev` branch
# rather than a SHA, so no commit here records where they should be. Keeping
# them current used to mean a `vcs pull src` nobody remembered to run. A
# post-merge hook was rejected: it does not fire when the workspace itself is
# already up to date -- exactly when a package repo has moved on its own --
# and every clone would need `core.hooksPath` set by hand. Submodules were
# rejected too: they pin SHAs, so every push to a package's `dev` would need a
# bump commit here.
#
# Plain git only, no vcstool: the hosts (robot01's Jetson, a developer's Mac)
# are not guaranteed to have it, and the image that does is several GB to pull
# for a `git pull`. dependencies.repos is parsed with awk, anchored on its
# indentation, so a comment that mentions `url:` or `version:` cannot be read
# as an entry.
#
# Never destructive: a checkout on a branch other than its pinned one (a
# feature branch, a detached HEAD) is fetched only and reported, never
# switched. Uncommitted changes do not skip a checkout -- every Rust package's
# Cargo.lock is rewritten by the build, so that rule synced none of them -- the
# fast-forward runs anyway, and git itself refuses one that would touch a
# modified file (reported as a failure, nothing lost). Merges are --ff-only.
# A missing checkout is cloned at its pinned branch, so a package that newly
# moves out arrives with the pull that adds it. One repo failing (offline, diverged)
# does not stop the others; the failures are listed and the exit code is 1.
#
# third-party.repos is out of scope on purpose: its pins are tags and SHAs that
# change only by a commit here, and a re-import needs --recursive and the
# livox_ros_driver2 package.xml restore (README, "Getting started") -- run
# `vcs import --recursive < third-party.repos --force` for that.
set -uo pipefail
cd "$(dirname "$0")/.."

if git rev-parse -q --verify '@{upstream}' >/dev/null; then
    git pull --ff-only || exit 1
else
    echo "!! workspace: branch has no upstream, not pulled"
fi

entries() {
    awk '/^  src\//{p=$1; sub(":$", "", p)}
         /^    url:/{u=$2}
         /^    version:/{print p, u, $2}' dependencies.repos
}

failed=()
while read -r path url ver; do
    if [ ! -d "$path/.git" ]; then
        echo "== $path: cloning $ver"
        git clone -q -b "$ver" "$url" "$path" || failed+=("$path")
        continue
    fi
    git -C "$path" fetch -q || { failed+=("$path"); continue; }
    branch=$(git -C "$path" rev-parse --abbrev-ref HEAD)
    if [ "$branch" != "$ver" ]; then
        echo "!! $path: on '$branch', not '$ver', fetched only"
    else
        git -C "$path" merge -q --ff-only "origin/$ver" || failed+=("$path")
    fi
done < <(entries)

# Where every checkout ended up -- the record `vcs export --exact` gives CI.
echo
while read -r path _ _; do
    printf '%-28s %-14s %s\n' "$path" \
        "$(git -C "$path" rev-parse --abbrev-ref HEAD 2>/dev/null)" \
        "$(git -C "$path" log -1 --format='%h %s' 2>/dev/null)"
done < <(entries)

if [ ${#failed[@]} -ne 0 ]; then
    echo "failed: ${failed[*]}"
    exit 1
fi
