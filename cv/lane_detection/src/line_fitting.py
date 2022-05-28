import cv2
import numpy as np
from sklearn.cluster import DBSCAN
from scipy import interpolate
import matplotlib.pyplot as plt
import time

img_pth = # Path to test image
SPLINE_DIM = 3
EXTRAPOLATE_VALUE = 0.05

def sort_by_cluster(labels,data):
    clusters = {}
    for n,pt in enumerate(data):
        if labels[n] not in clusters:
            clusters[labels[n]] = [pt]
        else:
            clusters[labels[n]].append(pt)

    return clusters

def lane_fitting(points):
    ''' Fitting lanes to a function with a variation on the sliding windows '''

    fit_points = []
    sorted_points_x = sorted(points,key=lambda x:x[1])
    sorted_points_y = sorted(points,key=lambda x:x[0])

    x_width = abs(sorted_points_x[-1][1] - sorted_points_x[0][1])
    y_width = abs(sorted_points_y[-1][0] - sorted_points_y[0][0])

    if ((x_width > 0.95*y_width) and (x_width < 1.05*y_width) and (x_width < 20 or y_width < 20)) or len(points) < 200:    # Hard-coded parameter, update maybe
        return None

    total_pts = len(points)
    num_windows = 30

    slice = int(total_pts//num_windows)

    #TODO: Instead of just using arbitrary slices, use local cluster like centers
    # to choose the points to be included in the average
    for n in range(num_windows):
        start_idx = n*slice
        end_idx = min((n+1)*slice,total_pts)

        group = np.array(points[start_idx:end_idx])
        x_avg = np.mean(group,axis=0)[1]
        y_avg = np.mean(group,axis=0)[0]

        sigma_x = np.sqrt(np.sum(np.power(group[:,1]-x_avg,2))/group.shape[0])
        sigma_y = np.sqrt(np.sum(np.power(group[:,0]-y_avg,2))/group.shape[0])

        if (sigma_x < 5) and (sigma_y < 5):
            fit_points.append([y_avg,x_avg])

    if len(fit_points) <= SPLINE_DIM:
        return None

    fit_points = np.array(fit_points)

    x = fit_points[:,1]
    y = fit_points[:,0]
    tck,u = interpolate.splprep([x,y],k=SPLINE_DIM,s=32)
    u = np.linspace(u[0]-EXTRAPOLATE_VALUE,u[-1]+EXTRAPOLATE_VALUE,500)
    out = interpolate.splev(u,tck)  # out is an array in the form of [[x_points], [y_points]]

    if type(out) == list:
        return [out[0].tolist(), out[1].tolist()]
    else:
        return out.tolist() # We shouldn't be hitting this, right? 


def fit_lanes(mask):
    out = []

    rows = np.where(input_norm==1)[0].reshape(-1,1)
    cols = np.where(input_norm==1)[1].reshape(-1,1)
    coords = np.concatenate((rows,cols),axis=1)     # (y,x) points
    
    if len(coords) > 0:
        clustering = DBSCAN(eps=9, min_samples=35).fit(coords)
        labels = clustering.labels_

        clusters = sort_by_cluster(labels,coords)

        for label,pts in clusters.items():
            if label == -1:
                continue
            else:
                _out = lane_fitting(pts)
                if _out is not None:
                    out += [_out]
                 
                _out = lane_fitting(pts)
                if _out is not None:
                    out += [_out]
                    
                # print(_out[0])
                # print(_out[1])
                # ax.plot(_out[0],_out[1],c='k')
                # ax.gca().invert_yaxis()
                
        # # Now we can save it to a numpy array.
        # data = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8)
        # data = data.reshape(fig.canvas.get_width_height()[::-1] + (3,))

        # If we haven't already shown or saved the plot, then we need to
        # draw the figure first...
        # fig.canvas.draw()
        
        return out
    return None

if __name__ == "__main__":
    input = cv2.imread(img_pth, cv2.IMREAD_GRAYSCALE)
    input_norm = input/255

    fit_lanes(input_norm)
    
