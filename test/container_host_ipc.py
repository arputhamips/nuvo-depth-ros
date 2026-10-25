"""Native host socket/clock -> unprivileged Ubuntu ROS container -> host DDS."""
import os, pathlib, signal, subprocess, tempfile
os.environ.setdefault('RMW_IMPLEMENTATION','rmw_cyclonedds_cpp')
os.environ.setdefault('ROS_DOMAIN_ID','92')
import rclpy
from integration import Observer,Peer
def main():
    image=os.environ.get('NUVO_TEST_IMAGE','nuvo-depth-ros:development')
    name='nuvo-contract-'+str(os.getpid());rclpy.init()
    with tempfile.TemporaryDirectory(prefix='nuvo-container-') as directory:
        root=pathlib.Path(directory);os.chown(root,os.getuid(),20055);root.chmod(0o2750)
        (root/'camera.yaml').write_text('/**:\n  ros__parameters:\n    publish_points: true\n    point_stride: 1\n')
        peer=Peer(root/'data.sock','2');(root/'data.sock').chmod(0o660)
        observer=Observer('/container_contract')
        with tempfile.TemporaryFile(mode='w+') as log:
            process=subprocess.Popen(['docker','run','--rm','--init','--name',name,'--network','host','--ipc','host',
                '--group-add','20055','-e','RMW_IMPLEMENTATION=rmw_cyclonedds_cpp','-e','ROS_DOMAIN_ID='+os.environ['ROS_DOMAIN_ID'],
                '-v',str(root)+':/run/nuvo-depth:ro',image,'ros2','launch','nuvo_bringup','camera.launch.py',
                'namespace:=container_contract','config:=/run/nuvo-depth/camera.yaml'],stdout=log,stderr=subprocess.STDOUT)
            try:
                observer.spin(lambda:len(observer.messages['metadata'])>=3,seconds=20);observer.verify('local_boottime','2')
                peer.close();peer=Peer(root/'data.sock','4');(root/'data.sock').chmod(0o660)
                observer.spin(lambda:any(m.run_uuid=='4'*32 for m in observer.messages['metadata'].values()),seconds=15)
                observer.verify('local_boottime','4')
            except Exception:
                log.flush();log.seek(0);print(log.read());raise
            finally:
                subprocess.run(['docker','stop','--signal','SIGINT','--time','4',name],stdout=subprocess.DEVNULL,check=False)
                process.wait(timeout=8);peer.close();observer.node.destroy_node()
    rclpy.shutdown();print('Native host IPC, clock correlation, numeric GID, read-only directory mount and container DDS reconnect passed')
if __name__=='__main__':main()
