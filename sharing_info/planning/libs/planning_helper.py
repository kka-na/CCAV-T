import numpy as np
import copy
from math import *
from scipy.ndimage import gaussian_filter1d

from scipy.interpolate import splprep, splev, interp1d

from planning.libs.quadratic_spline_interpolate import QuadraticSplineInterpolate

lanelets = None
tiles = None
tile_size = None
lane_width = None

def euc_distance(pt1, pt2):
    return np.sqrt((pt2[0]-pt1[0])**2+(pt2[1]-pt1[1])**2)

def find_nearest_idx(pts, pt):
    min_dist = float('inf')
    min_idx = 0

    for idx, pt1 in enumerate(pts):
        dist = euc_distance(pt1, pt)
        if dist < min_dist:
            min_dist = dist
            min_idx = idx

    return min_idx

def lanelet_matching(t_pt):
    row = int(t_pt[0] // tile_size)
    col = int(t_pt[1] // tile_size)

    min_dist = float('inf')
    l_id, l_idx = None, None
    for i in range(-1, 2):
        for j in range(-1, 2):
            selected_tile = tiles.get((row+i, col+j))
            if selected_tile is not None:
                for id_, data in selected_tile.items():
                    for idx, pt in enumerate(data['waypoints']):
                        dist = euc_distance(t_pt, pt)
                        if dist < min_dist:
                            min_dist = dist
                            l_id = id_
                            l_idx = data['idx'][idx]
    if l_id is not None:
        return (l_id, l_idx)
    else:
        return None

def lanelet_matching2(t_pt):
    row = int(t_pt[0] // tile_size)
    col = int(t_pt[1] // tile_size)

    min_dist = float('inf')
    l_id, l_idx = None, None
    for i in range(-1, 2):
        for j in range(-1, 2):
            selected_tile = tiles.get((row+i, col+j))
            if selected_tile is not None:
                for id_, data in selected_tile.items():
                    for idx, pt in enumerate(data['waypoints']):
                        dist = euc_distance(t_pt, pt)
                        if dist > 0.5:
                            continue
                        if dist < min_dist:
                            min_dist = dist
                            l_id = id_
                            l_idx = data['idx'][idx]
    if l_id is not None:
        return (l_id, l_idx)
    else:
        return None
    
def get_straight_path(idnidx, path_len):
    s_n = idnidx[0]
    s_i = idnidx[1]
    wps = copy.deepcopy(lanelets[s_n]['waypoints'])
    lls_len = len(wps)
    ids = [s_n]*lls_len
    u_n = s_n
    u_i = s_i+int(path_len)#*M_TO_IDX)
    e_i = lls_len-int(s_i)
    wps = wps[s_i:u_i]
    while u_i >= lls_len:
        
        _u_n = get_possible_successor(u_n)
        if _u_n == None:
            e_i = len(wps)
            break
        u_n = _u_n
        u_i -= lls_len
        e_i += u_i
        u_wp = lanelets[u_n]['waypoints']
        lls_len = len(u_wp)
        ids.extend([u_n]*lls_len)
        wps += u_wp

    r = wps[:e_i]
    return r, [u_n, u_i]

def get_cut_idx_ids(id):
    idx_list = lanelets[id]['cut_idx']
    llen = idx_list[-1][-1]
    ids = [None]*llen
    for i in range(len(ids)):
        for j, (minv, maxv) in enumerate(idx_list):
            if minv <= i < maxv:
                ids[i]=id+'_'+str(j)
                break
    return ids

def get_merged_point(idnidx, path_len, to=1):
        wps, [u_n, u_i],_ = get_straight_path(idnidx, path_len, '')
        c_pt = wps[-1]
        l_id, r_id = get_neighbor( u_n)
        n_id = l_id if to == 1 else r_id
        if n_id != None:
            r = lanelets[n_id]['waypoints']
            u_n = n_id
            u_i = find_nearest_idx(r, c_pt)

        return [u_n, u_i]
    
def get_possible_successor(node, prior='Right'):
    successor = None
    left_lanes, right_lanes, me = get_whole_neighbor(node)
    if len(lanelets[node]['successor']) <= 0:
        if prior == 'Left':
            check_a = left_lanes
            check_b = right_lanes
        else:   
            check_a = right_lanes
            check_b = left_lanes
        
        most_successor = find_most_successor(check_a)
        if most_successor == None:
            most_successor = find_most_successor(check_b)

        successor = most_successor
    else:
        if lanelets[node]['laneNo'] is not None:
            lane_num = int(lanelets[node]['laneNo'])
            for s in lanelets[node]['successor']:
                if int(lanelets[s]['laneNo']) == lane_num:
                    successor = s
                    break
        if successor is None:
            if prior == 'Left':
                i = 0
            else:
                i = -1
            successor = lanelets[node]['successor'][i]
    return successor

def get_whole_neighbor(node):
    num = 1
    find_node = node
    left_most = True
    right_most = True
    left_lanes = []
    right_lanes = []

    while left_most:
        if lanelets[find_node]['adjacentLeft'] != None:
            find_node = lanelets[find_node]['adjacentLeft']
            left_lanes.append(find_node)
            num += 1
        else:
            left_most = False
            find_node = node
    
    while right_most:
        if lanelets[find_node]['adjacentRight'] != None:
            find_node = lanelets[find_node]['adjacentRight']
            right_lanes.append(find_node)
            num += 1
            
        else:
            right_most = False
            find_node = node

    me_idx = len(left_lanes)

    return left_lanes, right_lanes, me_idx

def find_most_successor(check_l):
    most_successor = None
    for c in check_l:
        if len(lanelets[c]['successor']) <= 0:
            continue
        else:
            most_successor = lanelets[c]['successor'][0]
            break
    return most_successor


def get_neighbor(node):
    l_id = lanelets[node]['adjacentLeft']
    r_id = lanelets[node]['adjacentRight']
    return l_id, r_id

def gaussian_smoothing_2d(points, sigma=1):
    wx, wy = zip(*points)
    smoothed_wx = gaussian_filter1d(wx, sigma=sigma, mode='nearest')
    smoothed_wy = gaussian_filter1d(wy, sigma=sigma, mode='nearest')
    return list(zip(smoothed_wx, smoothed_wy))

def smooth_interpolate(points):
    precision = 1
    points = filter_same_points(points)
    wx, wy = zip(*points)
    itp = QuadraticSplineInterpolate(list(wx), list(wy))
    itp_points = []
    num_points = int((itp.s[-1] - itp.s[0]) / precision) + 1
    for ds in np.linspace(itp.s[0], itp.s[-1], num_points):
        x, y = itp.calc_position(ds)
        itp_points.append((float(x), float(y)))
    return itp_points


def filter_same_points(points):
    filtered_points = []
    pre_pt = None

    for pt in points:
        if pre_pt is None or pt != pre_pt:
            filtered_points.append(pt)

        pre_pt = pt

    return filtered_points

def calc_norm_vec(points):
    theta = atan2(points[1][1]-points[0][1], points[1][0]-points[0][0])
    A = sin(theta)
    B = -cos(theta)
    return A,B,theta

def calc_kappa(epoints, npoints):
    if abs(npoints[1][1]-epoints[1]) == 0 or abs(npoints[1][0]-epoints[0]) == 0:
        Rk = 0
    elif abs(npoints[0][1]-epoints[1]) == 0 or abs(npoints[0][0]-epoints[0]) == 0:
        Rk = 0
    else:
        if abs(npoints[1][1]-epoints[1]) < abs(npoints[1][0]-epoints[0]):
            dydx2 = (npoints[1][1]-epoints[1])/(npoints[1][0]-epoints[0])
            dydx1 = (npoints[0][1]-epoints[1])/(npoints[0][0]-epoints[0])
            dydx = (dydx2+dydx1)/2
            d2ydx2 = 2*(dydx2-dydx1)/(npoints[1][0]-npoints[0][0])
            Rk = d2ydx2/((1+(dydx)**2)**(3/2))
        else:
            dxdy2 = (npoints[1][0]-epoints[0])/(npoints[1][1]-epoints[1])
            dxdy1 =(npoints[0][0]-epoints[0])/(npoints[0][1]-epoints[1])
            dxdy = (dxdy2+dxdy1)/2
            d2xdy2 = 2*(dxdy2-dxdy1)/(npoints[1][1]-npoints[0][1])
            Rk = d2xdy2/((1+(dxdy)**2)**(3/2))
    return Rk

def get_profiles(path_len, max_vel, sec_to_reach):
    total_time = int((path_len / max_vel))
    peak_time = sec_to_reach/2
    accel_time = sec_to_reach
    t = np.linspace(0, total_time, total_time*10)

    acceleration = np.zeros_like(t)  # 가속도 초기화

    acceleration[t <= peak_time] = (t[t <= peak_time] / peak_time)
    acceleration[(t > peak_time) & (t < accel_time)] = (1- (t[(t > peak_time) & (t < accel_time)] - peak_time) / peak_time) 

    decel_start = total_time - accel_time
    decel_peak = total_time - accel_time + peak_time
    acceleration[(t >= decel_start) & (t <= decel_peak)] = -((t[(t >= decel_start) & (t <= decel_peak)] - decel_start) / peak_time)
    acceleration[(t > decel_peak) & (t < total_time)] = -1+((t[(t > decel_peak) & (t < total_time)] - decel_peak) / peak_time) 

    acceleration = acceleration * (max_vel/peak_time)

    velocity = np.cumsum(acceleration) * (t[1] - t[0])
    distance = np.cumsum(velocity) * (t[1] - t[0])

    return acceleration, velocity, distance

def limit_path_length(path, max_length):
    if len(path) <= max_length:
        return path
    
    new_path = [path[0]]
    interval = (len(path)-1)/(max_length-2)

    for i in range(1, max_length -1):
        index = int(i*interval)
        new_path.append(path[index])
    
    new_path.append(path[-1])
    return new_path

def calculate_R_list(points, base_offset=2, step_size=40):
    Rs = []
    numpoints = len(points)
    last_R = 99999
    last_offset = step_size * 2

    for i in range(numpoints):
        if i + base_offset < numpoints - last_offset:
            epoints = points[i + base_offset]
            npoints = [points[i + base_offset + step_size], points[i + base_offset + 2 * step_size]]
            kappa = calc_kappa(epoints, npoints)
            R = abs(1 / kappa) if kappa != 0 else 99999
            last_R = R
            Rs.append(R)
        else:
            Rs.append(last_R)
    return Rs

def interpolate_path(final_global_path, sample_rate=3, smoothing_factor=2.5, interp_points=3):
    local_path = np.array([(point[0], point[1]) for point in final_global_path])

    sampled_indices = np.arange(0, len(local_path), sample_rate)
    sampled_local_path = local_path[sampled_indices]

    tck, u = splprep([sampled_local_path[:, 0], sampled_local_path[:, 1]], s=smoothing_factor)
    t_new = np.linspace(0, 1, len(sampled_local_path) * interp_points)
    path_interp = np.array(splev(t_new, tck)).T
    path_interp_list = path_interp.tolist()

    R_list = calculate_R_list(path_interp_list)

    return path_interp_list, R_list

def calc_caution_by_ttc(obstacle, local_pose, ego_vel, th=10):
    if len(obstacle) < 1:
        return False
    obs_vel = obstacle[3]
    rel_vel = ego_vel - obs_vel
    if rel_vel > 0:
        dist = euc_distance(local_pose, [obstacle[0], obstacle[1]])
        ttc = dist/rel_vel
    else:
        ttc = float('inf')
    
    if ttc <= th:
        return True
    else:
        return False

def object_to_frenet(path, obj_pose):
    if path is None:
        return None
    if len(path) > 0:  
        centerline = np.array([(point[0], point[1]) for point in path])
        if centerline.shape[0] < 2:
            return None  # Or handle this case accordingly

        point = np.array(obj_pose)

        tangents = np.gradient(centerline, axis=0)
        tangents = tangents / np.linalg.norm(tangents, axis=1)[:, np.newaxis]
        
        normals = np.column_stack([-tangents[:, 1], tangents[:, 0]])
        
        distances = np.linalg.norm(centerline - point, axis=1)
        
        closest_index = np.argmin(distances)
        closest_point = centerline[closest_index]
        tangent = tangents[closest_index]
        normal = normals[closest_index]
        
        vector_to_point = point - closest_point
        d = np.dot(vector_to_point, normal)

        s = np.sum(np.linalg.norm(np.diff(centerline[:closest_index + 1], axis=0), axis=1))

        vector_from_start = point - centerline[0]  
        if np.dot(tangents[0], vector_from_start) < 0:  
            s = -np.linalg.norm(vector_from_start)  

        return s, d
    else:
        return None