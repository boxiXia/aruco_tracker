import socket
import msgpack
import numpy as np
import open3d as o3d
import multiprocessing as mp

def createSharedArray(dtype, shape, lock=True):
    """Create a new shared array. Return the shared array pointer, and a NumPy array view to it.
    Note that the buffer values are not initialized.
    ref: https://gist.github.com/rossant/7a46c18601a2577ac527f958dd4e452f
    """
    dtype = np.dtype(dtype)
    # Get a ctype type from the NumPy dtype.
    cdtype = np.ctypeslib.as_ctypes_type(dtype)
    # Create the RawArray instance.
    shared_arr = mp.Array(cdtype, int(np.prod(shape)),lock=lock)
    # Get a NumPy array view.
    arr = np.frombuffer(shared_arr.get_obj(), dtype=dtype).reshape(shape)
    return shared_arr, arr
    
class MarkerMapTracker:
    def __init__(self,
        ip_local = "127.0.0.1", port_local = 32000):
        
        self.local_address = (ip_local, port_local)
        self.BUFFER_LEN = 32768  # in bytes
        self.TIMEOUT = 0.2 #timeout duration
        self.ended = False
        self.t_robot = np.eye(4) # robot transform

        self._mp_array, self.pose = createSharedArray(dtype = np.float32, shape = (4,4), lock=True)
        self.recv_process =  mp.Process(target = self._loop,args = (self._mp_array,))

    def receive(self,max_attempts:int = 10,verbose=True):
        try:
            sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM) # UDP
            sock.settimeout(self.TIMEOUT)
            sock.bind(self.local_address) #Bind the socket to the port
            for k in range(max_attempts):# try max_attempts times
                try:
                    data = sock.recv(self.BUFFER_LEN)
                    sock.shutdown(socket.SHUT_RDWR) # closing connection
                    sock.close()
                    data_unpacked = msgpack.unpackb(data,use_list=False)
                    return data_unpacked
                except Exception as e:
                    if verbose:
                        print(f"receive(): try #{k}:{e}")
            sock.shutdown(socket.SHUT_RDWR)
            sock.close()
            raise TimeoutError("receive():tried too many times")
        except KeyboardInterrupt:
            sock.shutdown(socket.SHUT_RDWR)
            sock.close()
            raise KeyboardInterrupt
    
    def __del__(self):
        try:
            self.recv_process.join()
        except AssertionError: # process not yet started
            pass


    def start(self,gui=True):
        """start the recv process"""
        self._gui = gui
        self.recv_process.start()
        print("MarkerMapTracker: started recving frames")

    def _loop(self,mp_array):

        # pose of the robot (4x4)
        pose = np.frombuffer(mp_array.get_obj(), dtype=np.float32).reshape((4,4))

        t_track_to_cam = np.load("t_track_to_cam.npy")
        t_cam_to_botmarker = np.load("t_cam_to_botmarker.npy")

        t_track_to_bot = np.array([
            [0,0,-1,0],
            [0,1,0 ,0],
            [1,0,0 ,0.],
            [0,0,0,1]
        ])

        t_track_to_botmarker = t_track_to_cam@t_cam_to_botmarker
        t_botmarker_to_bot = np.linalg.inv(t_track_to_botmarker)@t_track_to_bot
        t_botmarker_to_bot[:3,3]=0 # only modify the orientation

        # add offset
        t_offset = np.eye(4)
        t_offset[0,3] = 0.03 # offset along x axis
        t_botmarker_to_bot = t_botmarker_to_bot@t_offset
        
        if self._gui:
            coord_0 = o3d.geometry.TriangleMesh.create_coordinate_frame(size=.5, origin=np.array([0., 0., 0.]))
            coord_1 = o3d.geometry.TriangleMesh.create_coordinate_frame(size=.5, origin=np.array([0., 0., 0.]))
            vis = o3d.visualization.VisualizerWithKeyCallback()
            vis.create_window(width=1024, height=1024)
            vis.add_geometry(coord_0)
            vis.add_geometry(coord_1)

            def signalEnd(vis):
                self.ended =True
            vis.register_key_callback(256, signalEnd)# key escape

        t_robot = np.eye(4)
        
        while(not self.ended):
            try:
                data = self.receive(verbose=False)
                # print(data)

                t_cam_to_marker = np.asarray(data).reshape((4,4))#@t_botmarker_to_bot
                t_track_to_bot = t_track_to_cam@t_cam_to_marker@t_botmarker_to_bot

                if self._gui:
                    coord_1.transform(t_track_to_bot@np.linalg.inv(t_robot))
                    vis.update_geometry(coord_1)
                    vis.poll_events()
                    vis.update_renderer()

                t_robot = t_track_to_bot[:]

                # mp_array.acquire()
                pose[:] = t_robot[:]
                # mp_array.release()

            except KeyboardInterrupt:
                break
            except Exception as e:
                pass
        if self._gui:
            vis.destroy_window()


    def calibrateWorldCoordiate(self):
        """calibarte the world coordinate using the calibartion board
        commandline:
        app_test_markermap -h -m aruco_calibration_grid_board_a4.yml -s 0.036
        """
        print(f"calibrate the world coordinate,place the calibration board at the world origin")
        data = self.receive(verbose=False)
        t_cam_to_track = np.asarray(data).reshape((4,4))# transform from camera to world (track)
        self.t_track_to_cam = np.linalg.inv(t_cam_to_track)
        path = "t_track_to_cam.npy"
        print(f"save self.t_track_to_cam to {path}")
        np.save(path,self.t_track_to_cam)
        
    def calibrateRobotCoodinate(self):
        """calibrate the robot coordinate"""
        print("before calibration, place the robot upright at the world origin")
        data = tracker.receive(verbose=False)
        self.t_cam_to_botmarker = np.asarray(data).reshape((4,4))
        path = "t_cam_to_botmarker.npy"
        print(f"save self.t_cam_to_botmarker to {path}")
        np.save(path,self.t_cam_to_botmarker)



if __name__ == "__main__":
    import argparse
    import time

    parser = argparse.ArgumentParser(description='Process some integers.')
    parser.add_argument('--calworld',help='calibarte the world coordinate using the calibartion board',default=False,action='store_true')
    parser.add_argument('--calrobot',help='calibarte the robot coordinate, place the robot upright at the world origin',default=False,action='store_true')

    args = parser.parse_args()

    tracker = MarkerMapTracker()


    if args.calworld: # calibarte the world coordinate
        tracker.calibrateWorldCoordiate()
    elif args.calrobot: # calibarte the robot coordinate
        tracker.calibrateRobotCoodinate()
    else: # run the marker map traking
        tracker.start(gui=True)
        while(tracker.recv_process.is_alive()):
            # tracker.mp_array.acquire()
            with np.printoptions(precision=2, suppress=True, floatmode='fixed',sign='+'):
                print(tracker.pose)
            # tracker.mp_array.release()
            time.sleep(0.5)