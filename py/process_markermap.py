"""
helper to move the center of mass position to origin
"""
import open3d as o3d
# import open3d.visualization.gui as gui

import numpy as np

import yaml
import re
import argparse

    
class marker:
    def __init__(self, config):
        self.id = config['id']
        self.corners = np.asarray(config['corners'],dtype=float)         
    def text3d(self,vis):
        com_pos = self.corners.mean(axis=0)
        vis.add_3d_label(com_pos, f"{self.id}")
    def lineset(self):
        return o3d.geometry.LineSet(o3d.utility.Vector3dVector(self.corners),
                             o3d.utility.Vector2iVector(np.array([[0,1],[1,2],[2,3],[3,0]])))
    
    
def arucoTest(path,gui=True):
    # path = "..\\board_config.yml"
    # path = "..\\markerset.yml"
    # path = "..\\markerset.yml"

    with open(path,mode='r') as file:
        file_str = file.read()
        file_str_fixed = re.sub(":+ |:",": ",file_str)
        file_str_fixed = re.sub("YAML: 1.0","YAML 1.0",file_str_fixed)
    # check if file need fixing
    # opencv does not exactly follow the yaml convention,
    # so may need to replace ":" with ": "
    if file_str != file_str_fixed:
        # fix the string
        with open(path,mode='w') as file:
            file.write(file_str_fixed)
    config = yaml.load(file_str_fixed,Loader=yaml.FullLoader)
                
        
    markers = [marker(m) for m in config["aruco_bc_markers"]]
    
    # move marker com to origin
    com_pos = np.vstack([m.corners for m in markers]).mean(axis=0)
    for m in markers:
        m.corners -= com_pos

    if gui==True:
        app = o3d.visualization.gui.Application.instance
        app.initialize()
        vis = o3d.visualization.O3DVisualizer("Open3D - 3D Text", 1024, 768)
        
        for m in markers:
            m.text3d(vis)
            vis.add_geometry(f"{m.id}",m.lineset())
            
        vis.show_settings = True
        vis.show_axes = True
        vis.line_width = 5
        vis.reset_camera_to_default()

        app.add_window(vis)
        app.run() 

    # save to the config
    for k in range(len(markers)):
        config["aruco_bc_markers"][k]['corners'] = markers[k].corners.tolist()
    # opencv does not support higher yaml standard, so need to write custom composer
    header = "%YAML 1.0\n---\n"
    body = "\n".join([f"{key}: {item}" if key!= 'aruco_bc_markers' else f"{key}:" for key, item in config.items()])+"\n"
    marker_string = "\n".join([f"   - {m}" for m in config['aruco_bc_markers']])
    yml_str = (header+body+marker_string).replace("'","")# remove '
    with open(path,'w') as file:
        # doc = yaml.dump(config,file)
        file.write(yml_str)

if __name__ == "__main__":
    
    parser = argparse.ArgumentParser(description='Process some integers.')
    parser.add_argument('--path',help='marker set yaml to be centered',default="markerset.yml")
    parser.add_argument('--gui',help='visualize the marker map',type=bool,default=True)

    args = parser.parse_args()
    arucoTest(args.path, args.gui)
