# Camera-to-ROS connection paths

The driver always turns associated native Nuvo frames into the same ROS topics.
The connection to the camera depends on **where the driver process runs**.

| Setup | Camera → driver | Driver → ROS applications | Time in ROS headers |
|---|---|---|---|
| ROS container on the camera's Pi 5 | Native Unix socket `/run/nuvo-depth/data.sock`, read-only `memfd` buffers via descriptor passing | ROS 2 DDS, locally or over the robot network | Native capture boottime mapped to host system time |
| Driver on another Linux computer over Ethernet | Authenticated TLS session, then binary WebSocket frames from the Nuvo gateway, normally TCP 7443 | ROS 2 DDS from that computer | Receipt time at the driver; original capture time remains in metadata |
| Driver on another computer over USB-C | **Provisioned USB Ethernet** link, then the same TLS/WebSocket source | ROS 2 DDS from that computer | Same receipt-time policy as Ethernet |

The local socket cannot be shared across computers by mounting a network folder.
Local `memfd` descriptor passing requires the same Linux kernel. HTTPS is the
remote acquisition path; there is no USB camera/UVC, serial, or raw USB driver in
this adapter.

## Same Pi: native socket to ROS container

```text
Camera sensors → native Nuvo daemon → Unix socket + memfd → ROS container → DDS
```

Use `source: local` and the [same-Pi Compose setup](../README.md#quick-start).
Mount the entire `/run/nuvo-depth` directory read-only and pass the host's numeric
`nuvo-video` GID. Neither USB/camera-device passthrough nor privileged mode is
needed. The gateway is unnecessary for this path. The native daemon and driver
must share the native boottime namespace.

Applications on a different computer can subscribe to this Pi-hosted driver's
ROS topics over Ethernet. In that arrangement the camera-to-driver connection
stays local; **DDS** crosses Ethernet. Match `ROS_DOMAIN_ID`, subscriber QoS and
network discovery/firewall settings. No HTTPS camera credentials are needed by
those DDS subscribers. Do not start a second driver merely to subscribe to the
first one's ROS topics.

## Different computer: Ethernet to the gateway

```text
Camera sensors → native Nuvo daemon → Nuvo gateway
    → Ethernet / verified TLS + WebSocket → ROS driver on computer → DDS
```

This moves ROS message construction and optional XYZ work to the computer.
Capture, matching and calibration remain on the camera. Use the full native
installation with the gateway available; a gateway-free minimal installation
cannot serve this remote path.

The [remote Compose file](../docker/compose.remote.yaml) mounts only configuration
and credentials. Follow the [container instructions](docker-image.md#remote-ethernet-or-provisioned-usb-ethernet)
or use the [source-build HTTPS parameters](reference.md#remote-https).

- `endpoint` uses the hostname in the camera's TLS certificate, for example
  `https://camera.example:7443`; replace the example with the actual name.
- `ca_file` trusts the camera's certificate/CA and `credential_file` contains its
  existing token with mode `0600`. The token is not embedded in an image or ROS
  parameter value.
- If DNS/mDNS does not resolve inside the container, set `connect_address` to
  the camera's actual IP while retaining the certificate hostname in `endpoint`.
- The driver verifies TLS, exchanges the token for a session, reads capabilities
  and receives associated raw products over one binary WebSocket. It does not
  change camera settings. Clean shutdown attempts session revocation.

Receipt timestamps do not provide synchronized remote acquisition time. Use the
source clock and identity in metadata for provenance, and do not treat receipt
time as camera exposure time. Ethernet throughput and large-message DDS delivery
still depend on the network and selected products.

## Different computer: USB-C

USB-C is usable here as a **network link**, after the native appliance presents
a USB Ethernet gadget and both computers have working IP addresses. Then use
exactly the Ethernet HTTPS configuration, pointing `connect_address` at the
camera's USB-network IP if needed. The gateway must be reachable on that interface.
Alternatively, keep the driver on the Pi and carry ROS DDS over the USB network.

**Plugging in a USB-C cable alone is not a supported plug-and-play Nuvo setup.**
The native `nuvo-usb-gadget.service` is deliberately not enabled by installation;
it requires saved native provisioning and the product's VID/PID, power/data and
host-compatibility qualification. This ROS package does not enable or reconfigure
USB gadget mode. Use Nuvo's native deployment procedure for approved hardware.

Raspberry Pi documents USB Ethernet gadget support on the Pi 5's USB-C port, but
that does not qualify a particular Nuvo camera, cable, host OS or power arrangement.
See [Raspberry Pi's gadget implementation](https://github.com/raspberrypi/rpi-usb-gadget)
and [Nuvo's native deployment guidance](https://github.com/arputhamips/nuvo-depth/blob/main/deployment/README.md).
Do not assume a computer's USB port supplies enough power for the Pi and cameras.

The synthetic HTTPS tests exercise the protocol shared by Ethernet and USB
Ethernet; they do not test either physical link. USB-C hardware operation remains
unqualified by this repository.
