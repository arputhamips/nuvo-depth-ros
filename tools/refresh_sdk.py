"""Refresh only the declared canonical SDK export; never edit upstream by hand."""
from pathlib import Path
import argparse, hashlib, json, shutil, subprocess, sys, tempfile
ROOT=Path(__file__).resolve().parents[1]
def main():
    parser=argparse.ArgumentParser();parser.add_argument('--source-repo',type=Path,default=ROOT.parent/'Nuvo Depth');parser.add_argument('--check',action='store_true');args=parser.parse_args()
    destination=ROOT/'nuvo_depth_client_vendor/upstream'
    if args.check:
        manifest=json.loads((destination/'SOURCE-MANIFEST.json').read_text())
        for name,record in manifest['files'].items():
            path=destination/name
            assert path.resolve().is_relative_to(destination.resolve()),'invalid manifest path'
            assert hashlib.sha256(path.read_bytes()).hexdigest()==record['sha256'],name
        assert {p.relative_to(destination).as_posix() for p in destination.rglob('*') if p.is_file() and '__pycache__' not in p.relative_to(destination).parts}==set(manifest['files'])|{'SOURCE-MANIFEST.json'}
        return
    with tempfile.TemporaryDirectory() as directory:
        staged=Path(directory)/'sdk'
        subprocess.run([sys.executable,str(args.source_repo/'tools/export_cpp_client.py'),str(staged)],check=True)
        old=set(json.loads((destination/'SOURCE-MANIFEST.json').read_text())['files']) if destination.exists() else set()
        new=json.loads((staged/'SOURCE-MANIFEST.json').read_text())['files']
        if old-set(new):raise SystemExit('Export removes files; review and remove those known stale snapshot files before refreshing')
        for source in staged.rglob('*'):
            if not source.is_file():continue
            target=destination/source.relative_to(staged)
            target.parent.mkdir(parents=True,exist_ok=True)
            if not target.exists() or target.read_bytes()!=source.read_bytes():shutil.copyfile(source,target)
if __name__=='__main__':main()
