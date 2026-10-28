# Download and run the Docker image

The container image includes Ubuntu 24.04, ROS 2 Jazzy and this driver. It runs
the ROS adapter; the native Nuvo camera service and existing calibration stay on
the camera host. The current release includes output/rate startup controls,
explicit native bundle selection and the idle diagnostic fix:

```text
ghcr.io/arputhamips/nuvo-depth-ros:jazzy-0.1.2
```

Use [release v0.1.2](https://github.com/arputhamips/nuvo-depth-ros/releases/tag/v0.1.2)
for the source, image digest, checksums and validation record. No GitHub login is
needed to pull the public image. The release image selects `linux/amd64` or
`linux/arm64` automatically and has HTTPS enabled alongside local acquisition
and recording replay. No ROS APT package is provided.

`jazzy-0.1.2` identifies this release; the publication workflow refuses to replace
an existing versioned image. Pin the release's manifest digest when you need an
immutable reference. `jazzy-trial` remains a mutable preview label for subsequent
manually published trials. Hosted tests do not establish physical Pi camera
accuracy or sustained performance.

The earlier [Pi 5 retest](validation-pi5-20261027.md) covers v0.1.1, including
idle/resume and native-service restart recovery. The [v0.1.2 validation
record](validation-20261028.md) documents successful hosted checks and public
pulls; the new Pi installation and argument tests remain incomplete because
the test Pi developed storage errors. Release notes record the exact image digest.

## Get the image and matching deployment files

For the same-Pi deployment, the [bootstrap quick start](../README.md#quick-start)
pulls the release, verifies its version, downloads deployment files at its OCI
source revision and runs by image digest. It requires Python 3, curl, tar and
Docker Engine/Compose. Native `v0.1.1-beta.9` must already work on the Pi.

For a manual source checkout, install Git as well, then pull the release and
check out the source revision recorded in it:

```bash
NUVO_DOCKER_IMAGE=ghcr.io/arputhamips/nuvo-depth-ros:jazzy-0.1.2
sudo docker pull "$NUVO_DOCKER_IMAGE"
git clone https://github.com/arputhamips/nuvo-depth-ros.git
cd nuvo-depth-ros
DRIVER_REVISION="$(sudo docker image inspect "$NUVO_DOCKER_IMAGE" --format '{{index .Config.Labels "org.opencontainers.image.revision"}}')"
git checkout --detach "$DRIVER_REVISION"
```

For an existing checkout, use `git fetch origin` instead of cloning and preserve
your local configuration before switching revisions. The checkout supplies small
Compose/YAML files; no compilation is needed. Pinning it to the image avoids
mixing an image with deployment files from a different version.

## Same Pi 5

Install native beta 9 using the
[full](https://github.com/arputhamips/nuvo-depth/releases/download/v0.1.1-beta.9/install-nuvo-depth.sh)
or [minimal](https://github.com/arputhamips/nuvo-depth/releases/download/v0.1.1-beta.9/install-nuvo-depth-minimal.sh)
release installer matching the profile already installed. Then use:

```bash
curl -fsSL https://raw.githubusercontent.com/arputhamips/nuvo-depth-ros/v0.1.2/install-nuvo-depth-ros.sh \
  | sudo bash -s -- --outputs depth,points --cadence-hz 10
```

The bootstrap invokes `tools/run_camera.py`; native group discovery, input
snapshots, launch overrides and Compose startup stay in that helper. It retains
root-owned deployment source under `/opt/nuvo-depth-ros/releases/<revision>` and
creates `/etc/nuvo-depth-ros/camera.yaml` only when absent. Keep user parameters
there or pass `--config /path/to/your.yaml`; do not edit the source cache.
Rerun with the same explicit launch options after editing parameters. The
bootstrap always selects its fixed release, even if an old manual checkout has
`NUVO_IMAGE` in `.env`. Container restarts retain the pinned image and overrides.

No `--calibration-file` keeps the camera's current native calibration. Provisional
publication remains blocked unless you explicitly add
`--allow-provisional-calibration`. See [startup controls](prototyping.md) for
bundle activation and read-only `--dry-run` inspection. Native beta 9 supplies
matching geometry; beta 8 retains its
[compatibility override](reference.md#beta-8-and-older-recordings).

For an existing source checkout instead of the bootstrap, use matching files
as above and let the helper discover the numeric group:

```bash
sudo python3 tools/run_camera.py --outputs depth,points --cadence-hz 10
```

Then start the README's best-effort depth subscriber. The default driver waits
for a measurement subscriber before requesting frames.

For an update, download the new release's bootstrap and supply your existing
`--config` and launch options. It preserves `/etc/nuvo-depth-ros/camera.yaml` and
keeps previous source revisions. It replaces an older container without a
rebuild. Updating ROS does not upgrade native camera packages or require
recalibration. Keep the previous image digest and deployment parameters for
rollback. If a cached download is incomplete, the next run downloads it again;
unsafe or unmanaged cache directories are rejected before executing their code.

For direct Compose commands, you still need its numeric group in the environment:

```bash
export NUVO_VIDEO_GID="$(getent group nuvo-video | cut -d: -f3)"
sudo env NUVO_VIDEO_GID="$NUVO_VIDEO_GID" docker compose -f docker/compose.yaml pull
sudo env NUVO_VIDEO_GID="$NUVO_VIDEO_GID" docker compose -f docker/compose.yaml up -d --no-build
```

Existing `.env` values override Compose defaults. Set or remove an old
`NUVO_IMAGE` before returning to the default release image.

## Optional: build from source

For development, build a separate local image from the matching source checkout
and start it through the same helper. Run from the repository root:

```bash
sudo docker build -f docker/Dockerfile -t nuvo-depth-ros:local .
sudo python3 tools/run_camera.py --image nuvo-depth-ros:local
```

The build runs colcon tests and defaults to two compiler jobs. Using the local
image name keeps the published release name reserved for the downloaded image.
To return to the prebuilt release, rerun the versioned bootstrap with your
configuration and launch options.

## Remote Ethernet or provisioned USB Ethernet

Use a Linux computer with Docker Engine/Compose and network access to the
camera's native gateway. These commands assume a Linux host, including for DDS
host networking; Docker Desktop networking on Windows/macOS needs separate
configuration and is not qualified by this example.

Use the matching checkout above on that computer. Obtain the camera's existing certificate
and token through your native camera setup and save **copies** as:

```text
docker/credentials/nuvo-ca.pem
docker/credentials/nuvo-token
```

Keep the token owned by your normal login user with mode `0600`. The credentials
directory is ignored by Git and excluded from the image build context. Do not
put the device private key there. Prepare the environment from the repository root:

```bash
mkdir -p docker/credentials
chmod 700 docker/credentials
chmod 600 docker/credentials/nuvo-token
printf 'NUVO_UID=%s\nNUVO_GID=%s\nROS_DOMAIN_ID=0\nNUVO_IMAGE=ghcr.io/arputhamips/nuvo-depth-ros:jazzy-0.1.2\n' \
  "$(id -u)" "$(id -g)" > docker/.env
```

Run this as your normal user, not a root login. The remote container uses these
numeric IDs to read the restricted token and writes its logs under `/tmp`.
Change `ROS_DOMAIN_ID` to match your robot. Edit
[`nuvo_bringup/config/remote.yaml`](../nuvo_bringup/config/remote.yaml): replace
the example hostname, set `connect_address` if needed, and supply any required
geometry override. Preserve the camera's certificate hostname in `endpoint`.

```bash
sudo docker compose --env-file docker/.env -f docker/compose.remote.yaml config --quiet
sudo docker compose --env-file docker/.env -f docker/compose.remote.yaml pull
sudo docker compose --env-file docker/.env -f docker/compose.remote.yaml up -d
sudo docker compose --env-file docker/.env -f docker/compose.remote.yaml logs --tail=50 camera
sudo docker compose --env-file docker/.env -f docker/compose.remote.yaml exec camera \
  /nuvo-entrypoint.sh ros2 topic echo /nuvo/front/depth/image_rect --once --field header --qos-reliability best_effort
```

The remote setup needs no `nuvo-video` group or camera socket mount. See
[connection paths](connections.md) for the distinction between HTTPS acquisition
and subscribing to DDS topics from a driver already running on the Pi.

## How images and releases are produced

Maintainers manually run **ROS and client contracts** with `architecture=both`
and `publish_image=true`, selecting the reviewed source branch. For a release,
also set `release_version` to the matching version in all four ROS package
manifests and the Dockerfile's `DRIVER_VERSION` (for example `0.1.2`). This adds
`jazzy-0.1.2` and refuses to overwrite
an existing versioned image. Ordinary pushes,
pull requests and default manual runs only test and upload artifacts. The publish
job runs only after all source and deployment jobs pass, including HTTPS ON/OFF
variants, full-size DDS, local IPC and the remote HTTPS container check.

The workflow publishes the tested HTTPS-enabled images, then the multi-platform
manifest. It records source revision and license labels in the image, and retains
the manifest plus a source archive in `trial-image-provenance`. The complete
vendor source and manifest remain in the public repository at that commit.
It creates no Git tag, GitHub release or APT package.

After the manual run succeeds, maintainers separately create the Git tag and
GitHub release at the exact tested source revision, with explicit release
authorization. Attach the image provenance, matching source archive and
checksums, and link the successful validation run. Verify an anonymous manifest
lookup and pull before publishing the release notes. Each release must retain
the documented hardware and native-runtime compatibility limits.

On the first GHCR publication, a repository administrator may need to set the
package visibility to **Public**. Verify a pull without registry credentials
before advertising the image as publicly downloadable. Public GHCR packages
allow anonymous pulls; see [GitHub's registry documentation](https://docs.github.com/en/packages/working-with-a-github-packages-registry/working-with-the-container-registry).

For comparison, ROS's normal binary package installation uses APT, while Open
Robotics also maintains registry-hosted Docker images. See [ROS Debian package
installation](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
and [Open Robotics' Docker images](https://github.com/osrf/docker_images).
An Actions image archive is useful for offline transfer, but requires a signed-in
GitHub download and a separate `docker load`; it is less convenient for a first trial.
