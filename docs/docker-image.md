# Download and run the Docker image

The container image includes Ubuntu 24.04, ROS 2 Jazzy and this driver. It runs
the ROS adapter; the native Nuvo camera service and existing calibration stay on
the camera host. The current release image includes the idle diagnostic fix:

```text
ghcr.io/arputhamips/nuvo-depth-ros:jazzy-0.1.1
```

Use [release v0.1.1](https://github.com/arputhamips/nuvo-depth-ros/releases/tag/v0.1.1)
for the source, image digest, checksums and validation record. No GitHub login is
needed to pull the public image. The release image selects `linux/amd64` or
`linux/arm64` automatically and has HTTPS enabled alongside local acquisition
and recording replay. No ROS APT package is provided.

`jazzy-0.1.1` identifies this release; the publication workflow refuses to replace
an existing versioned image. Pin the release's manifest digest when you need an
immutable reference. `jazzy-trial` remains a mutable preview label for subsequent
manually published trials. Hosted tests do not establish physical Pi camera
accuracy or sustained performance.

The [Pi 5 retest](validation-pi5-20261027.md) covers the underlying fixes with an
actual camera, including idle/resume and native-service restart recovery. Release
notes record the exact release image digest and subsequent publication checks.

## Get the image and matching deployment files

On the Linux computer that will run the driver, install Docker Engine, Compose
and Git. Then pull the release and check out the source revision recorded in it:

```bash
NUVO_DOCKER_IMAGE=ghcr.io/arputhamips/nuvo-depth-ros:jazzy-0.1.1
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

The v0.1.1 image includes the idle-diagnostic fix. It requires
[working native startup](native-startup.md);
installing ROS does not upgrade an old GPIO daemon to PIO PWM.

Follow the [README prerequisites](../README.md#quick-start) to install working
native Nuvo Depth, use the matching checkout above, and write
`NUVO_VIDEO_GID` and `ROS_DOMAIN_ID` to `docker/.env`. Both Compose examples
default to the release image. To pin it explicitly, add:

```dotenv
NUVO_IMAGE=ghcr.io/arputhamips/nuvo-depth-ros:jazzy-0.1.1
```

Edit `nuvo_bringup/config/camera.yaml` as needed. Public beta 8 still requires the
[geometry override](reference.md#beta-8-and-older-recordings); an image download
does not upgrade the native runtime or change calibration authority.

From the repository root on the Pi:

```bash
sudo docker compose --env-file docker/.env -f docker/compose.yaml pull
sudo docker compose --env-file docker/.env -f docker/compose.yaml up -d --no-build
sudo docker compose --env-file docker/.env -f docker/compose.yaml logs --tail=50 camera
```

Then start the README's best-effort depth subscriber. The default driver waits
for a measurement subscriber before requesting frames.

For an update, preserve your configuration and select the new release's matching
deployment files. Change any existing `NUVO_IMAGE` line in `docker/.env` to
`ghcr.io/arputhamips/nuvo-depth-ros:jazzy-0.1.1`; an explicit old value overrides
the Compose default. Pull and run `up -d --no-build` as above. This also replaces
an older source-built container without rebuilding. Keep a known working digest
so you can return to it. Updating ROS does not upgrade native camera packages.

## Optional: build from source

For development, set `NUVO_IMAGE=nuvo-depth-ros:local` in `docker/.env`, replacing
any release-image setting. Then run from the repository root:

```bash
sudo docker compose --env-file docker/.env -f docker/compose.yaml build
sudo docker compose --env-file docker/.env -f docker/compose.yaml up -d --no-build
```

The build runs colcon tests and defaults to two compiler jobs. Using the local
image name keeps the published release name reserved for the downloaded image.
To return to the prebuilt release, set `NUVO_IMAGE` to the release image and run
the pull/start commands above.

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
printf 'NUVO_UID=%s\nNUVO_GID=%s\nROS_DOMAIN_ID=0\nNUVO_IMAGE=ghcr.io/arputhamips/nuvo-depth-ros:jazzy-0.1.1\n' \
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
manifests and the Dockerfile's `DRIVER_VERSION` (for example `0.1.1`). This adds
`jazzy-0.1.1` and refuses to overwrite
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
