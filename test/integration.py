"""Exercise the real ROS executable over DDS, replay and a restarting IPC peer."""
import array, importlib.util, json, math, os, pathlib, signal, socket, struct, subprocess, sys, tempfile, threading, time
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from nuvo_camera_msgs.msg import FrameMetadata
from diagnostic_msgs.msg import DiagnosticArray
ROOT=pathlib.Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('fixture',ROOT/'nuvo_depth_client_vendor/upstream/tests/transports.py')
fixture=importlib.util.module_from_spec(spec);spec.loader.exec_module(fixture)
def packet(sequence=1,run='2',authority=1,geometry=True,live=False):
    wire=fixture.frame(sequence);size=fixture.HEADER.unpack_from(wire)[5]
    document=json.loads(wire[32:32+size]);document['run_uuid']=run*32;document['calibration_authority']=authority
    if live:document['capture_time_ns']=time.clock_gettime_ns(time.CLOCK_BOOTTIME)-10000000
    if not geometry:document.pop('calibration_geometry')
    return fixture.encode(document,wire[32+size:],sequence)
class Peer:
    def __init__(self,path,run):
        self.path=path;self.run=run;self.stop=threading.Event();self.server=socket.socket(socket.AF_UNIX,socket.SOCK_SEQPACKET)
        self.server.bind(str(path));self.server.listen();self.server.settimeout(.2);self.client=None
        self.thread=threading.Thread(target=self.serve,daemon=True);self.thread.start()
    def serve(self):
        import fcntl
        sequence=0
        while not self.stop.is_set():
            try:c,_=self.server.accept()
            except socket.timeout:continue
            except OSError:return
            self.client=c
            try:
                assert c.recv(48)[:4]==b'NVS2';c.send(b'NVA2')
                while not self.stop.is_set():
                    request=c.recv(48)
                    if not request:break
                    assert request[:4]==b'NVW2';self.stop.wait(.05)
                    sequence+=1;wire=packet(sequence,self.run,live=True)
                    fd=os.memfd_create('ros-fixture',os.MFD_ALLOW_SEALING);os.write(fd,wire)
                    fcntl.fcntl(fd,fcntl.F_ADD_SEALS,fcntl.F_SEAL_GROW|fcntl.F_SEAL_SHRINK|getattr(fcntl,'F_SEAL_FUTURE_WRITE',0x10))
                    ro=os.open('/proc/self/fd/'+str(fd),os.O_RDONLY);os.close(fd)
                    try:c.sendmsg([b'NVR2'+bytes(4)+struct.pack('>Q',len(wire))+bytes.fromhex(self.run*32)+struct.pack('>QQ',1,sequence)],[(socket.SOL_SOCKET,socket.SCM_RIGHTS,array.array('i',[ro]))])
                    finally:os.close(ro)
            except (OSError,AssertionError):pass
            finally:c.close();self.client=None
    def close(self):
        self.stop.set();self.server.close()
        if self.client:
            try:self.client.shutdown(socket.SHUT_RDWR)
            except OSError:pass
        self.thread.join(2);assert not self.thread.is_alive();self.path.unlink(missing_ok=True)
class Observer:
    def __init__(self,namespace):
        self.node=rclpy.create_node('nuvo_contract_observer');self.messages={name:{} for name in ('depth','validity','info','points','metadata')};self.diagnostics=[]
        self.node.create_subscription(DiagnosticArray,'/diagnostics',lambda message:self.diagnostics.extend(message.status),10)
        qos=QoSProfile(depth=20,reliability=ReliabilityPolicy.BEST_EFFORT)
        for name,topic,kind in [('depth','depth/image_rect',Image),('validity','depth/validity',Image),('info','depth/camera_info',CameraInfo),('points','points',PointCloud2),('metadata','frame_metadata',FrameMetadata)]:
            self.node.create_subscription(kind,namespace+'/'+topic,lambda message,key=name:self.record(key,message),qos)
    def record(self,key,message):
        stamp=message.header.stamp.sec*1000000000+message.header.stamp.nanosec;self.messages[key][stamp]=message
    def spin(self,predicate,seconds=15):
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:
            rclpy.spin_once(self.node,timeout_sec=.1)
            if predicate():return
        raise AssertionError('ROS DDS condition timed out: '+str({k:len(v) for k,v in self.messages.items()})+' diagnostics='+str([(s.name,s.message,[(v.key,v.value) for v in s.values]) for s in self.diagnostics[-2:]])+' nodes='+str(self.node.get_node_names_and_namespaces()))
    def verify(self,mode,run=None):
        common=set.intersection(*(set(values) for values in self.messages.values()))
        assert common,'no coherent associated set received'
        stamp=max(common);d,v,i,p,m=(self.messages[key][stamp] for key in self.messages)
        assert (d.width,d.height,d.encoding,d.step)==(2,1,'32FC1',8)
        z=struct.unpack('<ff',bytes(d.data));assert z[0]==2 and math.isnan(z[1]);assert list(v.data)==[1,6]
        assert (i.width,i.height,i.k[0],i.k[2],i.p[0])==(2,1,100.,1.,100.)
        assert len({item.header.frame_id for item in (d,v,i,p,m)})==1
        assert m.timestamp_mode==mode and m.source_capture_clock=='boottime' and m.calibration_sha256=='3'*64
        if run:assert m.run_uuid==run*32
        x,y,z=struct.unpack_from('<fff',bytes(p.data));assert abs(x+.02)<1e-6 and y==0 and z==2
        assert math.isnan(struct.unpack_from('<f',bytes(p.data),12)[0]);assert p.width==2 and not p.is_dense
def start(binary,namespace,log,**parameters):
    args=[binary,'--ros-args','-r','__ns:='+namespace]
    for key,value in dict(parameters,publish_points=True,point_stride=1).items():args+=['-p',key+':='+str(value).lower() if isinstance(value,bool) else key+':='+str(value)]
    return subprocess.Popen(args,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
def stop(process):
    os.killpg(process.pid,signal.SIGINT)
    try:process.wait(4)
    except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait();raise AssertionError('node failed interruptible shutdown')
    assert process.returncode==0,process.returncode
def main():
    prefix=subprocess.check_output(['ros2','pkg','prefix','nuvo_camera'],text=True).strip();binary=prefix+'/lib/nuvo_camera/nuvo_camera_node'
    rclpy.init()
    with tempfile.TemporaryDirectory() as directory:
        root=pathlib.Path(directory);recording=root/'capture.nvr'
        recording.write_bytes(b'NVR1\0\0\0\1'+b''.join(struct.pack('>Q',len(packet(j)))+packet(j) for j in range(1,21)))
        for clock in (False,True):
            observer=Observer('/contract_replay');log=open(root/'replay.log','w+')
            process=start(binary,'/contract_replay',log,source='replay',recording=recording,replay_loop=True,publish_clock=clock)
            try:
                observer.spin(lambda:len(observer.messages['metadata'])>=3)
                observer.verify('replay_clock' if clock else 'replay_rebased')
            except Exception:
                print('process=',process.pid,process.poll(),file=sys.stderr)
                subprocess.run(['ps','-T','-p',str(process.pid),'-o','pid,tid,comm,wchan'],check=False)
                log.flush();log.seek(0);print(log.read(),file=sys.stderr);raise
            finally:stop(process);observer.node.destroy_node();log.close()
        # A directory mount sees the new socket inode after daemon restart.
        path=root/'data.sock';peer=Peer(path,'2');observer=Observer('/contract_local');log=open(root/'local.log','w+')
        process=start(binary,'/contract_local',log,source='local',endpoint=path)
        try:
            observer.spin(lambda:len(observer.messages['metadata'])>=3);observer.verify('local_boottime','2')
            peer.close();peer=Peer(path,'4')
            observer.spin(lambda:any(m.run_uuid=='4'*32 for m in observer.messages['metadata'].values()));observer.verify('local_boottime','4')
        except Exception:
            log.flush();log.seek(0);print(log.read(),file=sys.stderr);raise
        finally:stop(process);peer.close();observer.node.destroy_node();log.close()
    rclpy.shutdown();print('ROS DDS replay, simulation timeline, point cloud and local reconnect contracts passed')
if __name__=='__main__':main()
