#!/usr/bin/env bash
# =============================================================================
# Install, update or roll back the release image on a robot. Run ON the robot,
# from the bundle directory scripts/release/save_bundle.sh made (this file is
# in it), as the uid-1000 user or root:
#
#   ./install.sh                 # load the bundle's image and switch to it
#   ./install.sh --image <ref>   # switch to an image docker already has, or
#                                # can pull (a robot that reaches the registry)
#   ./install.sh --rollback      # back to the image that ran before this one
#   ./install.sh --no-start      # everything but `docker compose up`
#
# The same command installs the first time and updates every time after: it
# only ever adds what is missing and replaces what belongs to the image.
#
# The robot's data directory is SYNCAI_HOME (default /opt/syncai) -- the one
# directory the backend's compose mounts as ROBOT_WS. Two kinds of file live
# in it, and the difference is this script's whole job:
#
#   the robot's own, NEVER touched once they exist
#     config/instances/<instance>.ini   identity, active map, lidar
#     config/state/                     the localizer's last pose
#     config/params/                    the settings pages' overlays
#     config/calibration/               the camera's intrinsics
#     map/  log/  data/postgres/  .env
#
#   the image's, REPLACED on every switch from inside the image being started
#     docker-compose.release.yml  attach.sh  MANIFEST
#     config/cyclonedds.xml                  the backend's CYCLONEDDS_URI
#     src/syncai_controller/params/          factory defaults the backend's
#     src/syncai_global_planner/params/      settings pages show and reset to
#
# The second group is copied out of the image rather than shipped beside it,
# so that it cannot be a different version from the image: a rollback puts the
# old compose file and the old defaults back together with the old binaries.
# The two src/ directories look odd on a machine with no sources. They are
# there because the backend's compose mounts exactly those paths read-only;
# their contents are replaced in place and the directories themselves kept,
# since a running backend's bind mount holds the directory's inode.
#
# The first run on a robot stops before starting anything: it writes the
# instance INI and .env from the image's templates and asks for them to be
# filled in. robot_id decides the node namespace and, in the backend, the
# database and task queue; POSTGRES_PASSWORD is fixed in data/postgres at the
# first start. Neither is something to find out was a placeholder afterwards.
#
# Not done here, only checked: docker, the compose plugin, the nvidia runtime,
# and the udev rules behind /dev/syncai/camera0 -- those carry each robot's
# own camera serials (src/syncai_sys_manager/udev/99-syncai-devices.rules is
# robot01's) and are installed by hand.
# =============================================================================
set -euo pipefail

SYNCAI_HOME="${SYNCAI_HOME:-/opt/syncai}"
BUNDLE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COMPOSE_FILE="${SYNCAI_HOME}/docker-compose.release.yml"
ENV_FILE="${SYNCAI_HOME}/.env"
PROJECT=syncai
CONTAINER=robot01
RUN_UID=1000
RUN_GID=1000
# The workspace root inside the image (the Dockerfile's release stage).
IMAGE_WS=/home/syncrobotic/robot_ws

mode=install
image_ref=""
start=1
while [ $# -gt 0 ]; do
    case "$1" in
        --rollback) mode=rollback ;;
        --no-start) start=0 ;;
        --image)    [ $# -ge 2 ] || { echo "install.sh: --image needs a reference" >&2; exit 2; }
                    image_ref="$2"; shift ;;
        -h|--help)  sed -n '2,/^# ====.*$/p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *) echo "install.sh: unknown argument '$1' (--image <ref>, --rollback, --no-start)" >&2; exit 2 ;;
    esac
    shift
done

step() { printf '\n==> %s\n' "$*"; }
warn() { printf 'install.sh: WARNING: %s\n' "$*" >&2; }
die()  { printf 'install.sh: %s\n' "$*" >&2; exit 1; }

if [ "$(id -u)" = 0 ]; then
    as_root() { "$@"; }
else
    as_root() { sudo "$@"; }
fi

compose() {
    docker compose --project-name "${PROJECT}" -f "${COMPOSE_FILE}" --env-file "${ENV_FILE}" "$@"
}

# The value of KEY in .env, empty when the key or the file is absent.
get_env() {
    [ -f "${ENV_FILE}" ] || return 0
    sed -n "s/^$1=//p" "${ENV_FILE}" | tail -n 1
}

# KEY=VALUE in .env, replacing the line in place or appending it. `|` as the
# sed delimiter: an image reference has slashes and colons, never a bar.
set_env() {
    if grep -q "^$1=" "${ENV_FILE}"; then
        sed -i "s|^$1=.*|$1=$2|" "${ENV_FILE}"
    else
        printf '%s=%s\n' "$1" "$2" >> "${ENV_FILE}"
    fi
}

# --- 1. what this machine must already have -----------------------------------
step "checking the host"
[ "$(id -u)" = 0 ] || [ "$(id -u)" = "${RUN_UID}" ] || \
    die "run this as the uid-${RUN_UID} user or as root (you are uid $(id -u))." \
        "The container runs as ${RUN_UID}:${RUN_GID} and ${SYNCAI_HOME} belongs to it."
command -v docker >/dev/null || die "docker is not installed."
docker info >/dev/null 2>&1 || \
    die "cannot talk to the docker daemon -- is it running, and is this user in the docker group?"
docker compose version >/dev/null 2>&1 || \
    die "the docker compose plugin (v2) is not installed; docker-compose v1 will not read this file."
# `runtime: nvidia` in the compose file: without it registered, `up` fails
# with "unknown or invalid runtime name", which does not say what to install.
docker info --format '{{range $name, $_ := .Runtimes}}{{$name}} {{end}}' 2>/dev/null | grep -qw nvidia || \
    die "docker has no 'nvidia' runtime -- install the NVIDIA container toolkit (on a Jetson: nvidia-container, from JetPack)."
getent passwd "${RUN_UID}" >/dev/null || \
    warn "no user with uid ${RUN_UID} on this host; files under ${SYNCAI_HOME} will show a bare number as owner."
for dev in camera0 camera1; do
    [ -e "/dev/syncai/${dev}" ] || \
        warn "/dev/syncai/${dev} does not exist -- compose will refuse to start the robot container." \
             "Install this robot's udev rules (see docs/release-deployment.md) and replug the camera."
done
[ -S /run/dbus/system_bus_socket ] || warn "/run/dbus/system_bus_socket is missing: the wifi services will not work."
[ -d /run/avahi-daemon ] || warn "/run/avahi-daemon is missing (no avahi-daemon?): <robot_id>.local will not be published."

# A container of the same name from another compose project is the development
# stack of a checkout on this machine. Starting over it would fail half-way,
# on a name conflict, with the old robot01 already stopped.
for name in "${CONTAINER}" postgres temporal; do
    owner="$(docker inspect -f '{{ index .Config.Labels "com.docker.compose.project" }}' "${name}" 2>/dev/null || true)"
    if [ -n "${owner}" ] && [ "${owner}" != "${PROJECT}" ]; then
        die "a container named '${name}' already exists, from compose project '${owner}'." \
            "That is a development stack: run \`docker compose down\` in its checkout first" \
            "(docs/release-deployment.md, \"Moving a development robot\")."
    fi
done

# --- 2. which image -------------------------------------------------------------
previous="$(get_env SYNCAI_IMAGE)"

if [ "${mode}" = rollback ]; then
    [ -z "${image_ref}" ] || die "--rollback takes no --image; it goes back to SYNCAI_IMAGE_PREVIOUS in ${ENV_FILE}."
    image_ref="$(get_env SYNCAI_IMAGE_PREVIOUS)"
    [ -n "${image_ref}" ] || die "nothing to roll back to: ${ENV_FILE} has no SYNCAI_IMAGE_PREVIOUS."
    step "rolling back: ${previous:-<none>} -> ${image_ref}"
    docker image inspect "${image_ref}" >/dev/null 2>&1 || \
        die "the previous image ${image_ref} is no longer on this machine (pruned?). Install it from its bundle instead."
elif [ -n "${image_ref}" ]; then
    step "using image ${image_ref}"
    docker image inspect "${image_ref}" >/dev/null 2>&1 || docker pull "${image_ref}" || \
        die "${image_ref} is neither on this machine nor pullable."
else
    step "loading the bundle in ${BUNDLE}"
    [ -f "${BUNDLE}/SHA256SUMS" ] || die "no SHA256SUMS in ${BUNDLE} -- is this a bundle from save_bundle.sh?"
    # Before docker reads a byte of it: a tar cut short on a USB stick loads
    # as far as it goes and fails on the last layer, minutes in.
    (cd "${BUNDLE}" && sha256sum -c SHA256SUMS) || die "checksum mismatch -- the bundle was damaged in transit; copy it again."
    robot_tar="$(find "${BUNDLE}" -maxdepth 1 -name 'robot_*.tar.gz' | sed -n "1,2p")"
    [ -n "${robot_tar}" ] || die "no robot_*.tar.gz in ${BUNDLE}."
    [ "$(printf '%s\n' "${robot_tar}" | wc -l)" -eq 1 ] || die "more than one robot_*.tar.gz in ${BUNDLE}; keep one bundle per directory."
    loaded="$(docker load -i "${robot_tar}")" || die "docker load failed on ${robot_tar}."
    printf '%s\n' "${loaded}"
    image_ref="$(printf '%s\n' "${loaded}" | sed -n 's/^Loaded image: //p' | sed -n "1,1p")"
    [ -n "${image_ref}" ] || die "docker load did not report an image name for ${robot_tar}."
    if [ -f "${BUNDLE}/infra.tar.gz" ]; then
        docker load -i "${BUNDLE}/infra.tar.gz"
    fi
fi

# --- 3. the data directory ------------------------------------------------------
# Made here rather than left to docker: a bind source docker has to create is
# root:root 755, and the container (uid 1000) then cannot write its own logs,
# maps or last pose. An existing directory is left exactly as it is -- its
# owner included: data/postgres belongs to the postgres image's own user once
# that container has started.
step "preparing ${SYNCAI_HOME}"
ensure_dir() {
    [ -d "$1" ] && return 0
    as_root mkdir -p "$1"
    as_root chown "${RUN_UID}:${RUN_GID}" "$1"
    echo "  created $1"
}
for d in "" config config/instances config/state config/params config/calibration \
         map log data data/postgres src src/syncai_controller src/syncai_controller/params \
         src/syncai_global_planner src/syncai_global_planner/params; do
    ensure_dir "${SYNCAI_HOME}${d:+/${d}}"
done

# --- 4. the image's files, out of the image -------------------------------------
# `docker create` + `docker cp`: nothing is run, so this works for an image of
# another architecture too, and a broken image cannot stop its own rollback.
step "taking the image's files out of ${image_ref}"
stage="$(mktemp -d)"
cid=""
cleanup() {
    [ -z "${cid}" ] || docker rm -f "${cid}" >/dev/null 2>&1 || true
    rm -rf "${stage}"
}
trap cleanup EXIT
cid="$(docker create "${image_ref}")"
docker cp "${cid}:${IMAGE_WS}/deploy" "${stage}/deploy" || \
    die "${image_ref} has no ${IMAGE_WS}/deploy -- it is not a release image (the dev image has none)."
docker cp "${cid}:${IMAGE_WS}/MANIFEST" "${stage}/MANIFEST"
docker cp "${cid}:${IMAGE_WS}/config/cyclonedds.xml" "${stage}/cyclonedds.xml"
for pkg in syncai_controller syncai_global_planner; do
    docker cp "${cid}:${IMAGE_WS}/install/${pkg}/share/${pkg}/params" "${stage}/${pkg}_params" || \
        die "${image_ref} has no installed params for ${pkg}."
done

# put <src> <dst>: a file the image owns, replaced whole.
put() {
    as_root cp -f "$1" "$2"
    as_root chown "${RUN_UID}:${RUN_GID}" "$2"
}
put "${stage}/deploy/docker-compose.release.yml" "${COMPOSE_FILE}"
put "${stage}/deploy/attach.sh"                  "${SYNCAI_HOME}/attach.sh"
put "${stage}/MANIFEST"                          "${SYNCAI_HOME}/MANIFEST"
put "${stage}/cyclonedds.xml"                    "${SYNCAI_HOME}/config/cyclonedds.xml"
as_root chmod +x "${SYNCAI_HOME}/attach.sh"
for pkg in syncai_controller syncai_global_planner; do
    dst="${SYNCAI_HOME}/src/${pkg}/params"
    # Empty the directory, never remove it (see the header).
    as_root find "${dst}" -mindepth 1 -delete
    as_root cp -r "${stage}/${pkg}_params/." "${dst}/"
    as_root chown -R "${RUN_UID}:${RUN_GID}" "${dst}"
done
echo "  docker-compose.release.yml, attach.sh, MANIFEST, config/cyclonedds.xml, src/*/params"

# --- 5. the robot's own files, once ---------------------------------------------
pending=0
if [ ! -f "${ENV_FILE}" ]; then
    put "${stage}/deploy/.env.example" "${ENV_FILE}"
    as_root chmod 600 "${ENV_FILE}"
    echo "  created ${ENV_FILE} from the template"
    pending=1
fi
instance="$(get_env ROBOT_INSTANCE)"
instance="${instance:-robot}"
ini="${SYNCAI_HOME}/config/instances/${instance}.ini"
if [ ! -f "${ini}" ]; then
    put "${stage}/deploy/robot.ini.example" "${ini}"
    echo "  created ${ini} from the template"
    pending=1
fi

# --- 6. switch ------------------------------------------------------------------
# PREVIOUS is only moved when the image really changes, so running the same
# bundle twice does not make "the image before this one" this one.
if [ -n "${previous}" ] && [ "${previous}" != "${image_ref}" ]; then
    set_env SYNCAI_IMAGE_PREVIOUS "${previous}"
fi
set_env SYNCAI_IMAGE "${image_ref}"

# Comment lines are skipped: the .env template explains CHANGE_ME in one.
if [ "${pending}" = 1 ] || cat "${ini}" "${ENV_FILE}" | grep -v '^[[:space:]]*#' | grep -q 'CHANGE_ME'; then
    cat <<EOF

install.sh: stopped before starting anything -- this robot is not configured yet.

  ${ini}
      [system] robot_id      this robot's name (node namespace, mDNS name)
      [sensor.lidar] type    mid360 or mid360s -- a wrong type gives no point
                             cloud and no error
      [sensor.lidar] ip      the lidar's address
      [map] name             leave empty: the first map is made from the console

  ${ENV_FILE}
      POSTGRES_PASSWORD      fixed in data/postgres at the first start; the
                             backend's own .env must carry the same one
      VIDEO_GID / AUDIO_GID  \`getent group video audio\` on this host

Replace every CHANGE_ME, then run this script again.
EOF
    exit 3
fi

# Every image the compose file names has to be on this machine already: a
# robot in a plant cannot pull, and `up` would sit on a registry it cannot
# reach before saying so.
missing=""
while IFS= read -r img; do
    [ -n "${img}" ] || continue
    docker image inspect "${img}" >/dev/null 2>&1 || missing="${missing} ${img}"
done < <(compose config --images)
[ -z "${missing}" ] || \
    die "not on this machine:${missing}" \
        "-- bring the infra images too (save_bundle.sh --with-infra makes infra.tar.gz)."

if [ "${start}" != 1 ]; then
    step "--no-start: ${image_ref} is set in ${ENV_FILE}; nothing was started"
    exit 0
fi

# No --remove-orphans: "orphan" means any container of this project name that
# this file does not define, and that is not this script's call to make.
step "docker compose up -d"
compose up -d

# --- 7. did it come up? ---------------------------------------------------------
# sys_manager is the container's main process and answers get_mode as soon as
# it is up, before the session it builds has finished starting; AUTO (2) means
# the nav session exists. A warning, not a failure, when it does not answer:
# the image is switched either way, and what is wrong is in the pane logs.
robot_id="$(sed -n 's/^robot_id[[:space:]]*[:=][[:space:]]*//p' "${ini}" | sed -n "1,1p")"
step "waiting for /${robot_id}/get_mode"
ok=0
for _ in $(seq 1 24); do
    if out="$(docker exec "${CONTAINER}" bash -c "
            source /opt/ros/humble/setup.bash &&
            source /opt/ros2_rust_underlay/install/setup.bash &&
            source install/setup.bash &&
            timeout 10 ros2 service call /${robot_id}/get_mode syncai_common/srv/GetMode" 2>/dev/null)" \
        && printf '%s\n' "${out}" | grep -q 'response:'; then
        printf '%s\n' "${out}" | grep -A2 'response:' | sed 's/^/  /'
        ok=1
        break
    fi
    sleep 5
done
[ "${ok}" = 1 ] || warn "get_mode did not answer within two minutes. Look at:" \
    "docker logs ${CONTAINER}   and   ${SYNCAI_HOME}/log/stack/${robot_id}/"

cat <<EOF

installed: ${image_ref}
previous:  $(get_env SYNCAI_IMAGE_PREVIOUS)
$(sed -n '1,4p' "${SYNCAI_HOME}/MANIFEST" | sed 's/^/  /')

  attach to the session   ${SYNCAI_HOME}/attach.sh
  go back                 ${BUNDLE}/install.sh --rollback
EOF
