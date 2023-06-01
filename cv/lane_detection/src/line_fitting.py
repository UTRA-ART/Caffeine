import cv2
import numpy as np
from sklearn.cluster import DBSCAN
from scipy import interpolate
import matplotlib.pyplot as plt
import time

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
    NUM_WINDOWS = 30

    slice = int(total_pts//NUM_WINDOWS)

    #TODO: Instead of just using arbitrary slices, use local cluster like centers
    # to choose the points to be included in the average
    for n in range(NUM_WINDOWS):
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

    # u = np.linspace(u[0]-EXTRAPOLATE_VALUE,u[-1]+EXTRAPOLATE_VALUE,500)
    out = interpolate.splev(u,tck)  # out is an array in the form of [[x_points], [y_points]]

    if type(out) == list:
        # print(np.array(out).T.tolist())
        return np.array(out).T.tolist()
    else:
        return None


def fit_lanes(mask):
    out = []

    smoothed_pred = cv2.morphologyEx(mask, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_CROSS,(3,3)))
    
    rows = np.where(smoothed_pred==1)[0].reshape(-1,1)
    cols = np.where(smoothed_pred==1)[1].reshape(-1,1)
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

        return out
    return None
