#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
# Install the fixed ROS release's deployment files and run its prebuilt image.
set -euo pipefail
umask 022

VERSION=0.1.2
REPOSITORY=ghcr.io/arputhamips/nuvo-depth-ros
IMAGE="$REPOSITORY:jazzy-$VERSION"
INSTALL_DIR=/opt/nuvo-depth-ros
STATE_DIR=/etc/nuvo-depth-ros
CONFIG=
ARGS=()

fail() { printf 'Cannot install Nuvo ROS: %s\n' "$*" >&2; exit 1; }
usage() {
    cat <<'EOF'
Install Nuvo Depth ROS 2 v0.1.2 on the Linux camera host (run with sudo).
Requires working native Nuvo Depth beta 9+, Docker Engine + Compose, Python 3,
curl and tar. No host ROS installation or image build is needed.

Usage: sudo bash install-nuvo-depth-ros.sh [options]
       curl -fsSL <versioned-script-url> | sudo bash -s -- [options]

  --install-dir PATH       Source cache (default: /opt/nuvo-depth-ros)
  --state-dir PATH         Persistent parameters (default: /etc/nuvo-depth-ros)
  --config PATH            Use your existing ROS YAML instead of persistent default
  --outputs LIST           Comma-separated depth,points,left,right,disparity
  --cadence-hz HZ          Requested delivery ceiling (0.001..60)
  --namespace NAME         ROS namespace (default: nuvo/front)
  --ros-domain-id ID       ROS domain (0..232)
  --calibration-file PATH  Explicit native bundle selection; omitted preserves it
  --allow-provisional-calibration  Explicit preview opt-in; authority is unchanged
  --dry-run               Print deployment; no container start/calibration activation
  --help                  Show this help

Unspecified output/rate settings retain the selected YAML. The first install
copies safe defaults into /etc/nuvo-depth-ros/camera.yaml only if it is absent;
later runs preserve it. Pass the same launch options when reconfiguring.
EOF
}

while (($#)); do
    case "$1" in
        --help|-h) usage; exit 0 ;;
        --install-dir|--state-dir|--config)
            (($# >= 2)) && [[ -n "$2" ]] || fail "$1 needs a path"
            case "$1" in
                --install-dir) INSTALL_DIR=$2 ;;
                --state-dir) STATE_DIR=$2 ;;
                --config) CONFIG=$2 ;;
            esac
            shift 2 ;;
        --outputs|--cadence-hz|--namespace|--ros-domain-id|--calibration-file)
            (($# >= 2)) && [[ -n "$2" ]] || fail "$1 needs a value"
            ARGS+=("$1" "$2"); shift 2 ;;
        --allow-provisional-calibration|--dry-run)
            ARGS+=("$1"); shift ;;
        *) fail "Unknown option: $1 (see --help)" ;;
    esac
done

[[ $(uname -s) == Linux ]] || fail 'Run this on the Linux camera host.'
((EUID == 0)) || fail 'Run with sudo to access Docker and native input snapshots.'
for command in python3 curl tar docker; do
    command -v "$command" >/dev/null || fail "Missing $command; install the prerequisites described in README.md."
done
docker version >/dev/null 2>&1 || fail 'Docker Engine is unavailable. Install/start Docker Engine first: https://docs.docker.com/engine/install/debian/'
docker compose version >/dev/null 2>&1 || fail 'Docker Compose plugin is required: https://docs.docker.com/compose/install/linux/'

# The image is the source-of-truth for its deployment revision. Pin the running
# container to its digest, rather than re-resolving a mutable registry tag later.
docker pull "$IMAGE"
METADATA=$(docker image inspect "$IMAGE" | python3 -c '
import json, re, sys
image = json.load(sys.stdin)[0]
labels = image.get("Config", {}).get("Labels") or {}
revision = labels.get("org.opencontainers.image.revision", "")
repository, version = sys.argv[1:]
if labels.get("org.opencontainers.image.version") != version or not re.fullmatch(r"[0-9a-f]{40}", revision):
    sys.exit("Image release version/source revision is missing or does not match the installer")
digests = [value for value in image.get("RepoDigests", [])
           if re.fullmatch(re.escape(repository) + r"@sha256:[0-9a-f]{64}", value)]
if not digests:
    sys.exit("Image has no matching immutable repository digest")
print(revision)
print(digests[0])
' "$REPOSITORY" "$VERSION")
mapfile -t IMAGE_INFO <<< "$METADATA"
REVISION=${IMAGE_INFO[0]}
PINNED_IMAGE=${IMAGE_INFO[1]}

# Downloaded deployment code runs as root. Never reuse it through a cache path
# another user can replace, including a writable ancestor or symbolic link.
secure_cache_path() {
python3 - "$1" <<'PY'
import os
from pathlib import Path
import stat
import sys
path = Path(os.path.abspath(sys.argv[1]))
for parent in (path / 'releases', path, *path.parents):
    try:
        info = parent.lstat()
    except FileNotFoundError:
        continue
    writable = info.st_mode & 0o022
    sticky_parent = parent not in (path, path / 'releases') and info.st_mode & stat.S_ISVTX
    if not stat.S_ISDIR(info.st_mode) or info.st_uid != 0 or (writable and not sticky_parent):
        raise SystemExit(f'Source cache needs root-owned directories without links or unsafe write access: {parent}')
print(path)
PY
}
INSTALL_DIR=$(secure_cache_path "$INSTALL_DIR")
mkdir -p "$INSTALL_DIR/releases"
secure_cache_path "$INSTALL_DIR" >/dev/null
SOURCE_DIR="$INSTALL_DIR/releases/$REVISION"
TEMP_DIR=
cleanup() { if [[ -n "$TEMP_DIR" ]]; then rm -rf -- "$TEMP_DIR"; fi; }
trap cleanup EXIT
if [[ ! -d "$SOURCE_DIR" ]]; then
    TEMP_DIR=$(mktemp -d "$INSTALL_DIR/releases/.download.XXXXXXXX")
    curl --fail --silent --show-error --location --retry 3 --proto '=https' --tlsv1.2 \
        "https://codeload.github.com/arputhamips/nuvo-depth-ros/tar.gz/$REVISION" \
        --output "$TEMP_DIR/source.tar.gz"
    mkdir "$TEMP_DIR/source"
    tar --extract --gzip --file "$TEMP_DIR/source.tar.gz" --directory "$TEMP_DIR/source" --strip-components=1 --no-same-owner --no-same-permissions
    # Validate release metadata before executing any downloaded Python code.
    python3 - "$TEMP_DIR/source" "$VERSION" <<'PY'
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
root, version = Path(sys.argv[1]), sys.argv[2]
for package in ('nuvo_camera', 'nuvo_camera_msgs', 'nuvo_bringup', 'nuvo_depth_client_vendor'):
    if ET.parse(root / package / 'package.xml').findtext('version') != version:
        raise SystemExit('Downloaded deployment package version does not match the image')
for name in ('tools/run_camera.py', 'tools/check_native.py', 'docker/compose.yaml', 'nuvo_bringup/config/camera.yaml'):
    if not (root / name).is_file():
        raise SystemExit(f'Downloaded deployment is incomplete: {name}')
PY
    printf '%s\n' "$REVISION" > "$TEMP_DIR/source/.nuvo-source-revision"
    mv --no-target-directory --no-clobber "$TEMP_DIR/source" "$SOURCE_DIR"
fi
[[ -f "$SOURCE_DIR/.nuvo-source-revision" && $(cat "$SOURCE_DIR/.nuvo-source-revision") == "$REVISION" ]] || \
    fail "Unmanaged source directory exists: $SOURCE_DIR; choose another --install-dir."
python3 - "$INSTALL_DIR/releases" "$SOURCE_DIR" <<'PY'
from pathlib import Path
import stat
import sys
source = Path(sys.argv[2])
for path in (Path(sys.argv[1]), source, *source.rglob('*')):
    info = path.lstat()
    if (not (stat.S_ISREG(info.st_mode) or stat.S_ISDIR(info.st_mode)) or
            info.st_uid != 0 or info.st_mode & 0o022):
        raise SystemExit(f'Refusing unsafe cached deployment file or directory: {path}')
PY

if [[ -z "$CONFIG" ]]; then
    mkdir -p "$STATE_DIR"
    CONFIG="$STATE_DIR/camera.yaml"
    # Complete the copy before exposing the final path. An interrupted/failed
    # write must not leave a partial default that every subsequent run retains.
    python3 - "$SOURCE_DIR/nuvo_bringup/config/camera.yaml" "$CONFIG" <<'PY'
import os
from pathlib import Path
import tempfile
import sys
source, target = map(Path, sys.argv[1:])
if not os.path.lexists(target):
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(dir=target.parent, prefix='.camera.', delete=False) as stream:
            temporary = Path(stream.name)
            stream.write(source.read_bytes())
            stream.flush()
            os.fsync(stream.fileno())
        temporary.chmod(0o644)
        try:
            os.link(temporary, target)
        except FileExistsError:
            pass  # Another run/user created parameters first; preserve them.
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)
PY
fi
printf 'Nuvo ROS %s, source %s\nParameters: %s\n' "$VERSION" "$REVISION" "$CONFIG"
python3 "$SOURCE_DIR/tools/run_camera.py" --image "$PINNED_IMAGE" --config "$CONFIG" "${ARGS[@]}"
