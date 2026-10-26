"""Verified remote HTTPS/WebSocket -> unprivileged ROS container -> DDS.

Synthetic gateway on loopback; this does not qualify Ethernet or USB hardware.
The remote container has no camera socket mount and does not alter device config.
"""
import base64
import hashlib
import http.server
import json
import os
from pathlib import Path
import ssl
import struct
import subprocess
import tempfile
import threading
import time

import rclpy
from integration import Observer, fixture, packet


class Gateway(fixture.Handler):
    def do_POST(self):
        assert self.path == '/api/v1/session', 'unexpected device write'
        self.server.requests.append(('POST', self.path))
        super().do_POST()

    def do_DELETE(self):
        assert self.path == '/api/v1/session', 'unexpected device write'
        self.server.requests.append(('DELETE', self.path))
        super().do_DELETE()

    def do_GET(self):
        self.server.requests.append(('GET', self.path))
        if self.path == '/api/v1/capabilities':
            return super().do_GET()
        assert self.path.startswith('/api/v1/stream?')
        assert self.headers['Cookie'] == 'nuvo_session=' + 'a' * 64
        accept = base64.b64encode(hashlib.sha1((self.headers['Sec-WebSocket-Key'] +
            '258EAFA5-E914-47DA-95CA-C5AB0DC85B11').encode()).digest()).decode()
        self.send_response(101)
        self.send_header('Upgrade', 'websocket')
        self.send_header('Connection', 'Upgrade')
        self.send_header('Sec-WebSocket-Accept', accept)
        self.end_headers()
        try:
            for sequence in range(1, 301):
                wire = packet(sequence, live=True)
                self.wfile.write(b'\x82\x7e' + struct.pack('>H', len(wire)) + wire)
                self.wfile.flush()
                time.sleep(.1)
        except (OSError, ssl.SSLError):
            pass
        self.close_connection = True


def main():
    image = os.environ.get('NUVO_TEST_IMAGE', 'nuvo-depth-ros:development')
    output = Path(os.environ.get('NUVO_TEST_OUTPUT_DIR', 'output/deployment'))
    output.mkdir(parents=True, exist_ok=True)
    name = 'nuvo-https-contract-' + str(os.getpid())
    rclpy.init()
    observer = Observer('/https_contract')
    process = server = thread = None
    report = dict(image=image, passed=False, camera_socket_mounted=False)
    try:
        with tempfile.TemporaryDirectory(prefix='nuvo-https-') as directory:
            root = Path(directory)
            root.chmod(0o755)
            key, cert, token = (root / name for name in ('key.pem', 'cert.pem', 'token'))
            token.write_text('c' * 64)
            token.chmod(0o600)
            uid = os.getuid() or 10001
            gid = os.getgid() if os.getuid() else 10001
            if os.getuid() == 0:
                os.chown(token, uid, gid)
            subprocess.run(['openssl', 'req', '-x509', '-newkey', 'rsa:2048', '-nodes',
                            '-keyout', str(key), '-out', str(cert), '-days', '1',
                            '-subj', '/CN=camera.test', '-addext', 'subjectAltName=DNS:camera.test'],
                           check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            server = http.server.ThreadingHTTPServer(('127.0.0.1', 0), Gateway)
            server.daemon_threads = True
            server.closing = server.revoked = False
            server.requests = []
            context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
            context.load_cert_chain(cert, key)
            server.socket = context.wrap_socket(server.socket, server_side=True)
            thread = threading.Thread(target=server.serve_forever, daemon=True)
            thread.start()
            (root / 'remote.yaml').write_text(json.dumps({'/**': {'ros__parameters': {
                'source': 'https', 'endpoint': f'https://camera.test:{server.server_port}',
                'connect_address': '127.0.0.1', 'timestamp_mode': 'receipt',
                'ca_file': '/credentials/cert.pem', 'credential_file': '/credentials/token',
                'publish_points': True, 'point_stride': 1}}}))
            with (output / 'https-container.log').open('w+') as log:
                try:
                    process = subprocess.Popen([
                        'docker', 'run', '--rm', '--init', '--name', name,
                        '--network', 'host', '--user', f'{uid}:{gid}',
                        '-e', 'ROS_DOMAIN_ID=' + os.environ.get('ROS_DOMAIN_ID', '92'),
                        '-e', 'ROS_LOG_DIR=/tmp/nuvo-ros-log',
                        '-v', str(root) + ':/credentials:ro', image,
                        'ros2', 'launch', 'nuvo_bringup', 'camera.launch.py',
                        'namespace:=https_contract', 'config:=/credentials/remote.yaml'],
                        stdout=log, stderr=subprocess.STDOUT)
                    observer.spin(lambda: observer.ready(3), seconds=20)
                    observer.verify('receipt')
                    assert process.poll() is None
                    report.update(runtime_uid=uid, tls_hostname='camera.test',
                                  numeric_connect_address='127.0.0.1',
                                  associated_frames=len(observer.associated()))
                except Exception:
                    log.flush()
                    log.seek(0)
                    print(log.read())
                    raise
                finally:
                    if process:
                        subprocess.run(['docker', 'stop', '--signal', 'SIGINT', '--time', '4', name],
                                       check=False, stdout=subprocess.DEVNULL, timeout=10)
                        process.wait(timeout=8)
                        assert process.returncode == 0, process.returncode
            assert server.revoked, 'HTTPS session was not revoked on clean shutdown'
            report.update(passed=True, session_revoked=True, requests=server.requests)
    finally:
        if server:
            server.shutdown()
            server.server_close()
        if thread:
            thread.join(3)
        observer.node.destroy_node()
        rclpy.shutdown()
        (output / 'https-contract.json').write_text(json.dumps(report, indent=2) + '\n')
    print('Remote TLS/WebSocket, numeric address with hostname verification, receipt stamps and ROS DDS passed')


if __name__ == '__main__':
    main()
