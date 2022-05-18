import cv2
import numpy as np
from sklearn.cluster import DBSCAN
from scipy import interpolate
import matplotlib.pyplot as plt
import time

img_pth = '/Users/jasonyuan/Desktop/Test9.png'
SPLINE_DIM = 3

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

    if (x_width < 15) or (y_width < 15):    # Hard-coded parameter, update maybe
        return None

    # print(sorted_points)
    pts_added = 0
    total_pts = len(points)
    num_windows = 20

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

        # print(sigma_x, sigma_y)

        if (sigma_x < 5) and (sigma_y < 5):
            fit_points.append([y_avg,x_avg])

    if len(fit_points) <= SPLINE_DIM:
        return None

    fit_points = np.array(fit_points)
    # print(fit_points)

    x = fit_points[:,1]
    y = fit_points[:,0]
    tck,u = interpolate.splprep([x,y],k=SPLINE_DIM,s=32)
    # print(tck)
    out = interpolate.splev(u,tck)

    if type(out) == list:
        return [out[0].tolist(), out[1].tolist()]
    else:
        return out.tolist()


def fit_lanes(mask):
    # fig = plt.figure()
    # ax = fig.add_subplot(111)

    rows = np.where(mask==1)[0].reshape(-1,1)
    cols = np.where(mask==1)[1].reshape(-1,1)
    coords = np.concatenate((rows,cols),axis=1)     # (y,x) points

    if len(coords) > 0:

        clustering = DBSCAN(eps=15, min_samples=30).fit(coords)
        labels = clustering.labels_

        for i,pt in enumerate(coords):
            if labels[i] == 0:
                color = 'g'
            elif labels[i] == 1:
                color = 'r'
            elif labels[i] == 2:
                color = 'y'
            elif labels[i] == 3:
                color = 'b'
            elif labels[i] == 4:
                color = 'm'
            elif labels[i] == 5:
                color = 'c'
            elif labels[i] == -1:
                color = 'k'

            # ax.scatter(pt[1],pt[0],c=color)

        clusters = sort_by_cluster(labels,coords)
        out = []

        for label,pts in clusters.items():
            if label == -1:
                continue
            else:
                # coefficients = lane_fitting(pts,15)
                # poly = np.poly1d(coefficients)
                # min_x = pts[0][1]
                # max_x = pts[len(pts)-1][1]
                #
                # xrange = np.linspace(min_x,max_x,endpoint=True)
                # plt.plot(xrange,poly(xrange),'-',c='k')
                _out = lane_fitting(pts)
                if _out is not None:
                    out += [_out]
                # print(out[0])
                # print(out[1])
                # ax.plot(out[0],out[1],c='k')
                # ax.gca().invert_yaxis()

        # If we haven't already shown or saved the plot, then we need to
        # draw the figure first...
        # fig.canvas.draw()

        # # Now we can save it to a numpy array.
        # data = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8)
        # data = data.reshape(fig.canvas.get_width_height()[::-1] + (3,))

        return out
    return None
    # end = time.perf_counter()
    # print(end-start)

if __name__ == "__main__":
    # start = time.perf_counter()
    input = cv2.imread(img_pth, cv2.IMREAD_GRAYSCALE)
    input_norm = input/255

    fit_lanes(input_norm)
    