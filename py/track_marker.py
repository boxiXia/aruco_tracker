import socket
import msgpack
import numpy as np
import open3d as o3d

class MarkerMapTracker:
    def __init__(self,
        ip_local = "127.0.0.1", port_local = 32000):
        
        self.local_address = (ip_local, port_local)
        self.BUFFER_LEN = 32768  # in bytes
        self.TIMEOUT = 0.2 #timeout duration

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
        

tracker = MarkerMapTracker()

# while(1):
#     try:
#         data = tracker.receive(verbose=False)
#         data = np.asarray(data).reshape((4,4))
#         print(data)
#     except Exception as e:
#         pass
    
################################################################
from scipy.spatial.transform import Rotation as R
import open3d as o3d
import numpy as np
coord_0 = o3d.geometry.TriangleMesh.create_coordinate_frame(size=.5, origin=np.array([0., 0., 0.]))
coord_1 = o3d.geometry.TriangleMesh.create_coordinate_frame(size=.5, origin=np.array([0., 0., 0.]))
vis = o3d.visualization.VisualizerWithKeyCallback()
vis.create_window()
vis.add_geometry(coord_0)
vis.add_geometry(coord_1)

t1 = np.eye(4)

# t_world = np.array([
#     [0,0,1,0],
#     [-1,0,0,0],
#     [0,-1,0,0],
#     [0,0,0,1]
# ],dtype=float)

t_world = np.eye(4)


t_world = np.array([[0.073567934, 0.099872582, 0.99227685, -0.38515991],
       [-0.09691719, 0.99097961, -0.092556529, 0.050713513],
       [-0.99256986, -0.089359485, 0.082583688, 0.7048679],
       [0, 0, 0, 1]], dtype='float32')
t_world = np.linalg.inv(t_world)

ended=False
def signalEnd(vis):
    global ended
    ended =True
vis.register_key_callback(256, signalEnd)# key escape

while(not ended):
    try:
        data = tracker.receive(verbose=False)
        t1_ = t_world@np.asarray(data).reshape((4,4))
#         print(t1_)
#         t1_ = np.linalg.inv(t1_)
#         coord_1.transform(t_world@t1_@np.linalg.inv(t1)@np.linalg.inv(t_world))
        coord_1.transform(t1_@np.linalg.inv(t1))
        t1 = t1_[:]
#         coord_1.rotate(r, center=(0, 0, 0))
        vis.update_geometry(coord_1)
        vis.poll_events()
        vis.update_renderer()
    except KeyboardInterrupt:
        break
    except Exception as e:
        pass
vis.destroy_window()