import glob
from mpl_toolkits.axes_grid1 import ImageGrid
import matplotlib.pyplot as plt
import matplotlib as mpl
import numpy as np
from PIL import Image

dict_paths = glob.glob("ARUCO_MIP_25h7/*.png")
im = plt.imread(dict_paths[0])
ims = [plt.imread(p) for p in dict_paths]

nrows = 6
ncols = 5
pad_size = 300
n_dict = len(dict_paths)
n_img = n_dict/(nrows*ncols)
n_img = int(n_img)+1 if n_img>int(n_img) else int(n_img)

grid_size = im.shape[0]
for i in range(n_img):
    offset = nrows*ncols*i
    all_img = np.ones((nrows*grid_size+pad_size*2,ncols*grid_size+pad_size*2),dtype=np.float32)
    for row in range(nrows):
        for col in range(ncols):
            k = row*ncols+col + offset
            try:
                im = ims[k]
            except IndexError:
                break
            all_img[pad_size:,pad_size:][row*grid_size:(row+1)*grid_size,col*grid_size:(col+1)*grid_size] = im
        else:
            continue
        break
    for row in range(nrows):
        all_img[pad_size:,pad_size:][row*grid_size-1:row*grid_size+2,:] = 0.9
    for col in range(ncols):
        all_img[pad_size:,pad_size:][:,col*grid_size-1:col*grid_size+2,] = 0.9
#     all_img = np.pad(all_img,300,mode='constant',constant_values=1)
    gr_im= Image.fromarray(all_img*255.0).convert("L").save(f'ARUCO_MIP_25h7_{i}.png')
    
# fig = plt.figure(figsize=(ncols,nrows),dpi=300)
# plt.imshow(all_img, cmap='gray', interpolation = "nearest")
# plt.axis('off')
# plt.show()