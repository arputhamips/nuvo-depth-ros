"""Exercise the real ROS executable over DDS, replay and a restarting IPC peer."""
import array, importlib.util, json, math, os, pathlib, signal, socket, struct, subprocess, sys, tempfile, threading, time
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from nuvo_camera_msgs.msg import FrameMetadata
from diagnostic_msgs.msg import DiagnosticArray
from rosgraph_msgs.msg import Clock
ROOT=pathlib.Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('fixture',ROOT/'nuvo_depth_client_vendor/upstream/tests/transports.py')
fixture=importlib.util.module_from_spec(spec);spec.loader.exec_module(fixture)
def packet(sequence=1,run='2',authority=1,geometry=True,live=False,capture_time_ns=None,capture_sequence=None):
    wire=fixture.frame(sequence);size=fixture.HEADER.unpack_from(wire)[5]
    document=json.loads(wire[32:32+size]);document['run_uuid']=run*32;document['calibration_authority']=authority
    if live:document['capture_time_ns']=time.clock_gettime_ns(time.CLOCK_BOOTTIME)-10000000
    if capture_time_ns is not None:document['capture_time_ns']=capture_time_ns
    if capture_sequence is not None:document['capture_sequence']=capture_sequence
    if not geometry:document.pop('calibration_geometry')
    return fixture.encode(document,wire[32+size:],sequence)
class Peer:
    def __init__(self,path,run,producer=None):
        self.path=path;self.run=run;self.stop=threading.Event();self.server=socket.socket(socket.AF_UNIX,socket.SOCK_SEQPACKET)
        self.server.bind(str(path));self.server.listen();self.server.settimeout(.2);self.client=None
        self.producer=producer;self.connections=0;self.requests=0;self.subscriptions=[]
        self.thread=threading.Thread(target=self.serve,daemon=True);self.thread.start()
    def serve(self):
        import fcntl
        sequence=0
        while not self.stop.is_set():
            try:c,_=self.server.accept()
            except socket.timeout:continue
            except OSError:return
            self.client=c
            self.connections+=1
            try:
                subscription=c.recv(48)
                assert len(subscription)==20 and subscription[:4]==b'NVS2'
                self.subscriptions.append(dict(zip(
                    ('products','cadence_millihz','preview_cadence_millihz','reserved'),
                    struct.unpack_from('>IIII',subscription,4))))
                c.send(b'NVA2')
                while not self.stop.is_set():
                    request=c.recv(48)
                    if not request:break
                    assert request[:4]==b'NVW2';self.stop.wait(.05)
                    self.requests+=1;sequence+=1
                    wire=self.producer(sequence,self.run) if self.producer else packet(sequence,self.run,live=True)
                    size=fixture.HEADER.unpack_from(wire)[5];document=json.loads(wire[32:32+size])
                    fd=os.memfd_create('ros-fixture',os.MFD_ALLOW_SEALING);os.write(fd,wire)
                    fcntl.fcntl(fd,fcntl.F_ADD_SEALS,fcntl.F_SEAL_GROW|fcntl.F_SEAL_SHRINK|getattr(fcntl,'F_SEAL_FUTURE_WRITE',0x10))
                    ro=os.open('/proc/self/fd/'+str(fd),os.O_RDONLY);os.close(fd)
                    try:c.sendmsg([b'NVR2'+bytes(4)+struct.pack('>Q',len(wire))+bytes.fromhex(document['run_uuid'])+struct.pack('>QQ',document['capture_epoch'],document['capture_sequence'])],[(socket.SOL_SOCKET,socket.SCM_RIGHTS,array.array('i',[ro]))])
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
    def __init__(self,namespace,observe_clock=False):
        self.node=rclpy.create_node('nuvo_contract_observer');self.messages={name:{} for name in ('depth','validity','info','points','metadata')};self.diagnostics=[];self.received={}
        self.node.create_subscription(DiagnosticArray,'/diagnostics',lambda message:self.diagnostics.extend(message.status),10)
        qos=QoSProfile(depth=20,reliability=ReliabilityPolicy.BEST_EFFORT)
        self.clocks=[]
        if observe_clock:
            self.node.create_subscription(Clock,'/clock',lambda message:self.clocks.append(
                message.clock.sec*1000000000+message.clock.nanosec),qos)
        for name,topic,kind in [('depth','depth/image_rect',Image),('validity','depth/validity',Image),('info','depth/camera_info',CameraInfo),('points','points',PointCloud2),('metadata','frame_metadata',FrameMetadata)]:
            self.node.create_subscription(kind,namespace+'/'+topic,lambda message,key=name:self.record(key,message),qos)
    def record(self,key,message):
        stamp=message.header.stamp.sec*1000000000+message.header.stamp.nanosec;self.messages[key][stamp]=message
        if key=='metadata':self.received[stamp]=time.time_ns()
    def associated(self,run=None):
        common=set.intersection(*(set(values) for values in self.messages.values()))
        return {stamp for stamp in common if run is None or self.messages['metadata'][stamp].run_uuid==run*32}
    def ready(self,count=3,run=None):
        return len(self.associated(run))>=count
    def spin(self,predicate,seconds=15):
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:
            rclpy.spin_once(self.node,timeout_sec=.1)
            if predicate():return
        raise AssertionError('ROS DDS condition timed out: '+str({k:len(v) for k,v in self.messages.items()})+' diagnostics='+str([(s.name,s.message,[(v.key,v.value) for v in s.values]) for s in self.diagnostics[-2:]])+' nodes='+str(self.node.get_node_names_and_namespaces()))
    def verify(self,mode,run=None):
        common=self.associated(run)
        assert common,'no coherent associated set received'
        stamp=max(common);d,v,i,p,m=(self.messages[key][stamp] for key in self.messages)
        assert (d.width,d.height,d.encoding,d.step)==(2,1,'32FC1',8)
        z=struct.unpack('<ff',bytes(d.data));assert z[0]==2 and math.isnan(z[1]);assert list(v.data)==[1,6]
        assert (i.width,i.height,i.k[0],i.k[2],i.p[0])==(2,1,100.,1.,100.)
        assert len({item.header.frame_id for item in (d,v,i,p,m)})==1
        assert m.timestamp_mode==mode and m.source_capture_clock=='boottime' and m.calibration_sha256=='3'*64
        if mode=='local_boottime':
            assert m.timestamp_uncertainty_ns>=0
            assert -10000000<=self.received[stamp]-stamp<2000000000,'local boottime was not mapped to ROS system time'
        if run:assert m.run_uuid==run*32
        x,y,z=struct.unpack_from('<fff',bytes(p.data));assert abs(x+.02)<1e-6 and y==0 and z==2
        assert math.isnan(struct.unpack_from('<f',bytes(p.data),12)[0]);assert p.width==2 and not p.is_dense
def start(binary,namespace,log,**parameters):
    args=[binary,'--ros-args','-r','__ns:='+namespace]
    for key,value in dict(parameters,publish_points=True,point_stride=1).items():args+=['-p',key+':='+str(value).lower() if isinstance(value,bool) else key+':='+str(value)]
    return subprocess.Popen(args,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)

def write_recording(path,packets):
    path.write_bytes(b'NVR1\0\0\0\1'+b''.join(struct.pack('>Q',len(wire))+wire for wire in packets))

def lazy_replay(binary,root):
    recording=root/'lazy.nvr';write_recording(recording,(packet(j) for j in range(1,101)))
    observer=Observer('/contract_lazy');log=open(root/'lazy.log','w+')
    process=start(binary,'/contract_lazy',log,source='replay',recording=recording)
    try:
        observer.spin(lambda:observer.ready())
        before=max(m.capture_sequence for m in observer.messages['metadata'].values())
        observer.node.destroy_node()
        # Allow DDS removal detection, then keep playback paused longer than
        # the entire source interval between several consecutive captures.
        time.sleep(1.5)
        observer=Observer('/contract_lazy')
        observer.spin(lambda:observer.ready())
        sequences=[m.capture_sequence for m in observer.messages['metadata'].values()]
        assert min(sequences)>before,('lazy replay rewound',before,sequences)
        assert max(observer.received.values())-min(observer.received.values())>=100000000,'lazy replay resumed in a catch-up burst'
        observer.verify('replay_rebased')
    except Exception:
        log.flush();log.seek(0);print(log.read(),file=sys.stderr);raise
    finally:stop(process);observer.node.destroy_node();log.close()

def local_edge_cases(binary,root):
    path=root/'edge.sock';future=time.clock_gettime_ns(time.CLOCK_BOOTTIME)+1500000000
    peer=Peer(path,'2',lambda sequence,run:packet(sequence,run,capture_time_ns=future,capture_sequence=1))
    observer=Observer('/contract_future');log=open(root/'future.log','w+')
    process=start(binary,'/contract_future',log,source='local',endpoint=path)
    try:
        def counted():
            return any(dict((v.key,v.value) for v in s.values).get('duplicate_frames','0')!='0' and
                       dict((v.key,v.value) for v in s.values).get('stale_frames','0')!='0' for s in observer.diagnostics)
        observer.spin(lambda:observer.ready(1) and counted())
        assert len(observer.messages['metadata'])==1,'a capture identity was published more than once'
        observer.verify('local_boottime','2')
    except Exception:
        log.flush();log.seek(0);print(log.read(),file=sys.stderr);raise
    finally:stop(process);peer.close();observer.node.destroy_node();log.close()

def provisional_replay(binary,root):
    recording=root/'provisional.nvr'
    write_recording(recording,(packet(j,authority=0) for j in range(1,31)))
    # A bad later record makes an unpaced scan observable without CPU timing
    # assertions: normal playback cannot reach it before the first diagnostic.
    with recording.open('ab') as output:output.write(struct.pack('>Q',1))
    observer=Observer('/contract_provisional');log=open(root/'provisional.log','w+')
    process=start(binary,'/contract_provisional',log,source='replay',recording=recording)
    try:
        observer.spin(lambda:any(s.name=='/contract_provisional/nuvo_camera' for s in observer.diagnostics))
        statuses=[s for s in observer.diagnostics if s.name=='/contract_provisional/nuvo_camera']
        assert 'provisional calibration' in statuses[-1].message,statuses[-1].message
        assert dict((v.key,v.value) for v in statuses[-1].values)['acquisition_errors']=='0'
        assert not observer.messages['metadata'],'provisional replay published without authorization'
    finally:stop(process);observer.node.destroy_node();log.close()

def receipt_timestamps(binary,root):
    path=root/'receipt.sock'
    peer=Peer(path,'2',lambda sequence,run:packet(sequence,run,capture_time_ns=1))
    log=open(root/'receipt.log','w+')
    process=start(binary,'/contract_receipt',log,source='local',endpoint=path,timestamp_mode='receipt')
    try:
        time.sleep(.6)
        assert peer.connections==0,'lazy live acquisition connected without measurement consumers'
        observer=Observer('/contract_receipt');observer.spin(lambda:observer.ready())
        observer.verify('receipt','2')
        for stamp,metadata in observer.messages['metadata'].items():
            assert metadata.source_capture_time_ns==1 and metadata.timestamp_uncertainty_ns==-1
            assert 0<=observer.received[stamp]-stamp<1000000000,'receipt timestamp does not label arrival'
    finally:stop(process);peer.close();observer.node.destroy_node();log.close()

def provisional_live(binary,root):
    authority=[0];path=root/'provisional.sock'
    peer=Peer(path,'2',lambda sequence,run:packet(sequence,run,authority=authority[0],live=True))
    observer=Observer('/contract_provisional_live');log=open(root/'provisional-live.log','w+')
    process=start(binary,'/contract_provisional_live',log,source='local',endpoint=path)
    try:
        observer.spin(lambda:any(s.name=='/contract_provisional_live/nuvo_camera' and
                                 'provisional calibration' in s.message for s in observer.diagnostics))
        requests=peer.requests
        time.sleep(.6)
        assert requests==1 and peer.requests==requests,'blocked provisional stream retained matcher demand'
        assert peer.client is None,'blocked provisional stream kept its native subscription open'
        assert not observer.messages['metadata'],'provisional live capture was published'
        authority[0]=1
        observer.spin(lambda:observer.ready(run='2'),seconds=8)
        assert peer.connections>=2,'validated calibration did not recreate the native subscription'
        observer.verify('local_boottime','2')
    finally:stop(process);peer.close();observer.node.destroy_node();log.close()

def invalid_replay_timeline(binary,root):
    recording=root/'out-of-range.nvr'
    write_recording(recording,[packet(1),packet(2,capture_time_ns=1000000000000000000)])
    observer=Observer('/contract_range');log=open(root/'range.log','w+')
    process=start(binary,'/contract_range',log,source='replay',recording=recording)
    try:
        observer.spin(lambda:any(s.name=='/contract_range/nuvo_camera' and
                                 'replay timestamp exceeds' in s.message for s in observer.diagnostics),seconds=4)
        assert len(observer.messages['metadata'])<=1,'out-of-range ROS timestamp was published'
    finally:stop(process);observer.node.destroy_node();log.close()
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
        write_recording(recording,(packet(j) for j in range(1,3)))
        for clock in (False,True):
            observer=Observer('/contract_replay',observe_clock=True);log=open(root/'replay.log','w+')
            process=start(binary,'/contract_replay',log,source='replay',recording=recording,replay_loop=True,publish_clock=clock)
            try:
                observer.spin(lambda:observer.ready(5) and (not clock or
                    len(observer.associated() & set(observer.clocks))>=5))
                observer.verify('replay_clock' if clock else 'replay_rebased')
                sequences=[m.capture_sequence for m in observer.messages['metadata'].values()]
                assert len(sequences)>len(set(sequences)),'replay loop was not exercised'
                if clock:
                    assert all(a<b for a,b in zip(observer.clocks,observer.clocks[1:])),observer.clocks
                    for stamp in observer.associated() & set(observer.clocks):
                        metadata=observer.messages['metadata'][stamp]
                        # The two source captures are 100 ms apart. Each loop
                        # begins one ns after the preceding output stamp.
                        offset=stamp-metadata.source_capture_time_ns
                        assert offset>=0 and offset%100000001==0,(
                            'clock is not on the monotonic source-relative loop timeline',stamp,metadata)
                else:
                    assert not observer.clocks,'publish_clock=false emitted /clock'
            except Exception:
                print('process=',process.pid,process.poll(),file=sys.stderr)
                subprocess.run(['ps','-T','-p',str(process.pid),'-o','pid,tid,comm,wchan'],check=False)
                log.flush();log.seek(0);print(log.read(),file=sys.stderr);raise
            finally:stop(process);observer.node.destroy_node();log.close()
        # A directory mount sees the new socket inode after daemon restart.
        path=root/'data.sock';peer=Peer(path,'2');observer=Observer('/contract_local');log=open(root/'local.log','w+')
        process=start(binary,'/contract_local',log,source='local',endpoint=path)
        try:
            observer.spin(lambda:observer.ready(run='2'));observer.verify('local_boottime','2')
            peer.close();peer=Peer(path,'4')
            observer.spin(lambda:observer.ready(run='4'));observer.verify('local_boottime','4')
        except Exception:
            log.flush();log.seek(0);print(log.read(),file=sys.stderr);raise
        finally:stop(process);peer.close();observer.node.destroy_node();log.close()
        lazy_replay(binary,root)
        local_edge_cases(binary,root)
        receipt_timestamps(binary,root)
        provisional_replay(binary,root)
        provisional_live(binary,root)
        invalid_replay_timeline(binary,root)
    rclpy.shutdown();print('ROS DDS replay, lazy pause, simulation timeline, receipt time, duplicate rejection, point cloud and local reconnect contracts passed')
if __name__=='__main__':main()
