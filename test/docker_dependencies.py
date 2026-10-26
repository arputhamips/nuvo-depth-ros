"""Audit explicit HTTPS dependency requests and the component's direct ELF links.

ROS base/transitive packages can already include OpenSSL. HTTPS=OFF means Nuvo
adds no optional HTTPS apt requests or direct links, not an SSL-free ROS image.
"""
import json
import os
from pathlib import Path
import subprocess
import tempfile


def run(*args):
    return subprocess.check_output(args, text=True, timeout=30).strip()


def main():
    image = os.environ.get('NUVO_TEST_IMAGE', 'nuvo-depth-ros:ci')
    output = Path(os.environ.get('NUVO_TEST_OUTPUT_DIR', 'output/deployment'))
    output.mkdir(parents=True, exist_ok=True)
    https = run('docker', 'image', 'inspect', image, '--format', '{{index .Config.Labels "org.nuvo.client.https"}}')
    assert https in ('ON', 'OFF'), https
    report = dict(image=image, https=https, passed=False)
    for stage in ('build', 'runtime'):
        requested = run('docker', 'run', '--rm', image, 'cat',
                        '/usr/share/nuvo-build/' + stage + '-requested-https-packages.txt').splitlines()
        report[stage + '_requested_https_packages'] = requested
        assert bool(requested) == (https == 'ON'), (stage, https, requested)
    base = run('docker', 'run', '--rm', image, 'cat', '/usr/share/nuvo-build/runtime-base-packages.tsv')
    installed = run('docker', 'run', '--rm', image, 'dpkg-query', '-W', '-f=${Package}\t${Version}\n')
    (output / 'runtime-base-packages.tsv').write_text(base + '\n')
    (output / 'runtime-packages.tsv').write_text(installed + '\n')
    baseline = {line.split('\t')[0] for line in base.splitlines()}
    final = {line.split('\t')[0] for line in installed.splitlines()}
    report['optional_runtime_package_origin'] = {
        package: 'base' if package in baseline else 'added_or_transitive' if package in final else 'absent'
        # Ubuntu accepts libc-ares2 as an apt alias; dpkg records libcares2.
        for package in ('libssl3t64', 'libboost-system1.83.0', 'libcares2')}
    build_base = run('docker', 'run', '--rm', image, 'cat', '/usr/share/nuvo-build/build-base-packages.tsv')
    (output / 'build-base-packages.tsv').write_text(build_base + '\n')
    with tempfile.TemporaryDirectory(prefix='nuvo-elf-') as directory:
        container = run('docker', 'create', image)
        try:
            library = Path(directory) / 'libnuvo_camera_component.so'
            subprocess.run(['docker', 'cp', container + ':/ws/install/lib/libnuvo_camera_component.so', str(library)], check=True, timeout=30)
            dynamic = run('readelf', '-d', str(library))
            (output / 'component-dynamic.txt').write_text(dynamic + '\n')
            if https == 'OFF':
                for excluded in ('libssl.', 'libcrypto.', 'libboost_system.', 'libcares.'):
                    assert excluded not in dynamic, (excluded, dynamic)
            report['direct_elf_links_checked'] = True
        finally:
            subprocess.run(['docker', 'rm', container], check=True, capture_output=True, timeout=15)
    report['passed'] = True
    (output / 'dependency-contract.json').write_text(json.dumps(report, indent=2) + '\n')
    print('HTTPS=' + https + ': explicit package requests, base provenance and direct ELF links passed')


if __name__ == '__main__':
    main()
