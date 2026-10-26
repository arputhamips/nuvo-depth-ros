# Download and run the Docker image

The container image includes Ubuntu 24.04, ROS 2 Jazzy and this driver. It runs
the ROS adapter; the native Nuvo camera service and existing calibration stay on
the camera host. The initial release image is:

```text
ghcr.io/arputhamips/nuvo-depth-ros:jazzy-0.1.0
```

Use [release v0.1.0](https://github.com/arputhamips/nuvo-depth-ros/releases/tag/v0.1.0)
for the source, image digest, checksums and validation record. No GitHub login is
needed to pull the public image. The release image selects `linux/amd64` or
`linux/arm64` automatically and has HTTPS enabled alongside local acquisition
and recording replay. No ROS APT package is provided.

`jazzy-0.1.0` identifies this release; the publication workflow refuses to replace
an existing versioned image. Pin the release's manifest digest when you need an
immutable reference. `jazzy-trial` remains a mutable preview label for subsequent
manually published trials. Hosted tests do not establish physical Pi camera
accuracy or sustained performance.

## Earlier trial validation

The first public trial was published on **26 October 2026** from
[`c326cfb`](https://github.com/arputhamips/nuvo-depth-ros/commit/c326cfb8e659619aebf3eaf983782ae4c48db0fc).
The [manual validation/publication run](https://github.com/arputhamips/nuvo-depth-ros/actions/runs/40391844367)
passed all six source/deployment jobs before publication. Both architectures
passed the remote HTTPS container check as UID 1001, including session cleanup.
An anonymous AMD64 pull and launch-argument check then passed from a separate
Docker client with an empty credential configuration. Anonymous manifest lookup
confirmed both platforms. No GitHub login is needed to pull this image.

For the exact tested multi-platform image, use:

```bash
docker pull ghcr.io/arputhamips/nuvo-depth-ros@sha256:ee34359e6c9d42a5fdc4fb9227f037ad7259e201575789202690e8d523662027
```

Local evidence is retained under `output/audit/container-trial/` (ignored by Git),
including the anonymous pull, published manifest and hosted HTTPS reports.

## Get the image and matching deployment files

On the Linux computer that will run the driver, install Docker Engine, Compose
and Git. Then pull the release and check out the source revision recorded in it:

```bash
NUVO_DOCKER_IMAGE=ghcr.io/arputhamips/nuvo-depth-ros:jazzy-0.1.0
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

Follow the [README prerequisites](../README.md#quick-start) to install working
native Nuvo Depth, use the matching checkout above, and write
`NUVO_VIDEO_GID` and `ROS_DOMAIN_ID` to `docker/.env`. Add:

```dotenv
NUVO_IMAGE=ghcr.io/arputhamips/nuvo-depth-ros:jazzy-0.1.0
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

For an update, select the new release's image and matching deployment files,
then pull and run `up -d --no-build`. Keep a known working digest so you can
return to it. If building locally instead, remove
`NUVO_IMAGE` from `docker/.env` and use the README's build commands; this avoids
giving a locally built image the registry release name.

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
printf 'NUVO_UID=%s\nNUVO_GID=%s\nROS_DOMAIN_ID=0\nNUVO_IMAGE=ghcr.io/arputhamips/nuvo-depth-ros:jazzy-0.1.0\n' \
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
  ros2 topic echo /nuvo/front/depth/image_rect --once --field header --qos-reliability best_effort
```

The remote setup needs no `nuvo-video` group or camera socket mount. See
[connection paths](connections.md) for the distinction between HTTPS acquisition
and subscribing to DDS topics from a driver already running on the Pi.

## How images and releases are produced

Maintainers manually run **ROS and client contracts** with `architecture=both`
and `publish_image=true`, selecting the reviewed source branch. For a release,
also set `release_version` to the matching version in all four ROS package
manifests (for example `0.1.0`). This adds `jazzy-0.1.0` and refuses to overwrite
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
