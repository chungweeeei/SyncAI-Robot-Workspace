#!/usr/bin/env bash
# Pull the workspace AND every checkout dependencies.repos and third-party.repos
# materialise in src/.
#
#   scripts/sync.sh
#
# A `git pull` of this repo never touches those checkouts: they are gitignored.
# Every entry in dependencies.repos tracks the `dev` branch rather than a SHA,
# so no commit here records where they should be. Keeping them current used to
# mean a `vcs pull src` nobody remembered to run. A post-merge hook was
# rejected: it does not fire when the workspace itself is already up to date
# -- exactly when a package repo has moved on its own -- and every clone would
# need `core.hooksPath` set by hand. Submodules were rejected too: they pin
# SHAs, so every push to a package's `dev` would need a bump commit here.
#
# third-party.repos is the opposite case: its pins are tags and SHAs that move
# only by a commit here, so the pull that bumps one is what leaves the checkout
# stale -- and a stale livox / BT.CPP / small_gicp builds without complaint.
# Each of those checkouts is moved to its pin (detached, as `vcs import` leaves
# it), with its submodules (direct_visual_lidar_calibration's) updated and the
# livox_ros_driver2 package.xml restored afterwards, the two steps a by-hand
# re-import used to forget (README, "Getting started").
#
# Plain git only, no vcstool: the hosts (robot01's Jetson, a developer's Mac)
# are not guaranteed to have it, and the image that does is several GB to pull
# for a `git pull`. The .repos files are parsed with awk, anchored on their
# indentation, so a comment that mentions `url:` or `version:` cannot be read
# as an entry.
#
# Never destructive. A dependencies.repos checkout on a branch other than its
# pinned one (a feature branch, a detached HEAD) is fetched only and reported,
# never switched; a third-party checkout on a *branch* away from its pin is
# hand work for the same reason and is left alone too -- only a detached HEAD,
# which is what `vcs import` makes, is moved. (`.git` is tested with -e: a
# checkout once made as a submodule has a `.git` file, not a directory.) Uncommitted changes do not skip a checkout --
# every Rust package's Cargo.lock is rewritten by the build, so that rule
# synced none of them -- the fast-forward or checkout runs anyway, and git
# itself refuses one that would touch a modified file (reported as a failure,
# nothing lost). Merges are --ff-only. A missing checkout is cloned at its pin,
# so a package that newly moves out arrives with the pull that adds it. One
# repo failing (offline, diverged) does not stop the others; the failures are
# listed and the exit code is 1.
set -uo pipefail
cd "$(dirname "$0")/.."

if git rev-parse -q --verify '@{upstream}' >/dev/null; then
    git pull --ff-only || exit 1
else
    echo "!! workspace: branch has no upstream, not pulled"
fi

# `path url version` per entry of the .repos file $1. third-party.repos quotes
# its tag pins ("3.8.8"), so the quotes are stripped.
entries() {
    awk '/^  src\//{p=$1; sub(":$", "", p)}
         /^    url:/{u=$2}
         /^    version:/{v=$2; gsub(/"/, "", v); print p, u, v}' "$1"
}

failed=()

while read -r path url ver; do
    if [ ! -e "$path/.git" ]; then
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
done < <(entries dependencies.repos)

# The pin as a commit, or nothing when this clone does not have it yet.
pin_commit() {
    git -C "$1" rev-parse -q --verify "$2^{commit}" 2>/dev/null
}

while read -r path url ver; do
    if [ ! -e "$path/.git" ]; then
        echo "== $path: cloning $ver"
        # Clone the default branch and check the pin out after: `-b` takes a
        # tag but not a SHA, and three of the pins are SHAs.
        if ! git clone -q "$url" "$path"; then
            failed+=("$path")
            continue
        fi
    else
        # Already at the pin, on a branch or not: no fetch, so an offline robot
        # syncs a workspace whose pins did not move without a failure per
        # third-party repo.
        target=$(pin_commit "$path" "$ver")
        if [ -n "$target" ] && [ "$target" = "$(git -C "$path" rev-parse HEAD)" ]; then
            continue
        fi
        branch=$(git -C "$path" rev-parse --abbrev-ref HEAD)
        if [ "$branch" != "HEAD" ]; then
            echo "!! $path: on branch '$branch', not at '$ver', left alone"
            continue
        fi
    fi
    if [ -z "$(pin_commit "$path" "$ver")" ]; then
        # A plain fetch brings the branches and the tags they reach; a tag on no
        # branch or a SHA no branch contains any more needs naming explicitly
        # (GitHub serves a fetch by SHA).
        git -C "$path" fetch -q --tags origin \
            && { [ -n "$(pin_commit "$path" "$ver")" ] || git -C "$path" fetch -q origin "$ver"; } \
            || { failed+=("$path"); continue; }
    fi
    echo "== $path: checking out $ver"
    git -C "$path" -c advice.detachedHead=false checkout -q "$(pin_commit "$path" "$ver")" \
        && git -C "$path" submodule -q update --init --recursive \
        || { failed+=("$path"); continue; }
done < <(entries third-party.repos)

# The livox repo ships only package_ROS1.xml / package_ROS2.xml and gitignores
# package.xml, so a fresh clone has none and every ament package fails to
# configure (CLAUDE.md, "Build"). Copied only when it differs: a rewritten
# package.xml makes colcon reconfigure the driver for nothing.
livox=src/third-party/livox_ros_driver2
if [ -f "$livox/package_ROS2.xml" ] && ! cmp -s "$livox/package_ROS2.xml" "$livox/package.xml"; then
    echo "== $livox: restoring package.xml"
    cp -f "$livox/package_ROS2.xml" "$livox/package.xml"
fi

# Where every checkout ended up -- the record `vcs export --exact` gives CI.
# A detached third-party checkout reads as the tag it is on, or its SHA.
echo
while read -r path _ _; do
    ref=$(git -C "$path" rev-parse --abbrev-ref HEAD 2>/dev/null)
    [ "$ref" = HEAD ] && ref=$(git -C "$path" describe --tags --exact-match 2>/dev/null \
        || git -C "$path" rev-parse --short HEAD)
    printf '%-48s %-14s %s\n' "$path" "$ref" \
        "$(git -C "$path" log -1 --format='%h %s' 2>/dev/null | cut -c1-60)"
done < <(entries dependencies.repos; entries third-party.repos)

if [ ${#failed[@]} -ne 0 ]; then
    echo "failed: ${failed[*]}"
    exit 1
fi
