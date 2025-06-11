import math

import numpy as np
import copy
import networkx as nx
from itertools import combinations


def is_UAV_associated(subroute_with_flag):
    return subroute_with_flag[1] != -1


def updatetime(distance_truck, speed_truck, subpath, distance_uav, speed_uav, t):
    for key in t:
        t[key] = 0

    for sub in subpath:
        path = sub[0]
        for i in range(1, len(path) - 1):
            t[path[i]] = t[path[i - 1]] + distance_truck[path[i - 1]][path[i]] / speed_truck

        if sub[1] != -1:
            a = path[0]
            b = path[-1]
            t_u = t[a] + (distance_uav[a][sub[1]] + distance_uav[sub[1]][b]) / speed_uav
            t[path[-1]] = max(t_u, t[path[-2]] + distance_truck[path[-2]][path[-1]] / speed_truck)
        else:
            t[path[-1]] = t[path[-2]] + distance_truck[path[-2]][path[-1]] / speed_truck

    return t


def updatepath(servedbyUAV, best_insert, distance_truck, speed_truck, truckpath, subpath, distance_uav, speed_uav, t,
               C_prime):
    i, j, k = best_insert
    if servedbyUAV:
        truckpath.remove(j)
        for sub in subpath:
            if j in sub[0]:
                sub[0].remove(j)
                break

        for sub in subpath:
            if i in sub[0] and k in sub[0]:
                i_index = sub[0].index(i)
                k_index = sub[0].index(k)
                index = subpath.index(sub)

                subpath.remove(sub)

                if i_index > 0:
                    subpath.insert(index, (sub[0][:i_index + 1], -1))
                if k_index - i_index > 0:
                    subpath.insert(index + 1, (sub[0][i_index:k_index + 1], j))
                if len(sub[0]) - 1 - k_index > 0:
                    subpath.insert(index + 2, (sub[0][k_index:], -1))

                break

        for item in [i, j, k]:
            if item in C_prime:
                C_prime.remove(item)
    else:
        truckpath.remove(j)
        for sub in subpath:
            if j in sub[0]:
                sub[0].remove(j)
                break

        i_index = truckpath.index(i)
        truckpath.insert(i_index + 1, j)
        for sub in subpath:
            if i in sub[0] and k in sub[0]:
                i_sub_index = sub[0].index(i)
                sub[0].insert(i_sub_index + 1, j)
                break

    updatetime(distance_truck, speed_truck, subpath, distance_uav, speed_uav, t)
    return truckpath, subpath, t, C_prime


def caluavcost(j, t, subpath, distance_uav, speed_uav, savings, maxsaving, servedbyUAV, best_insert, e, s_l, s_r):
    path = subpath[0]
    for i_index in range(len(path) - 1):
        i = path[i_index]
        for k_index in range(i_index + 1, len(path) - 1):
            k = path[k_index]
            if i == j or j == k or i == k:
                continue
            tau_i_j_uav = distance_uav[i][j] / speed_uav
            tau_j_k_uav = distance_uav[j][k] / speed_uav

            if tau_i_j_uav + tau_j_k_uav <= e:
                if j in path and i_index < path.index(j) < k_index:
                    t_u_k = t[k] - savings
                else:
                    t_u_k = t[k]
                cost = max(0, max((t_u_k - t[i]) + s_l + s_r, tau_i_j_uav + tau_j_k_uav + s_l + s_r) - (t_u_k - t[i]))

                if savings - cost > maxsaving:
                    maxsaving = savings - cost
                    servedbyUAV = True
                    best_insert = (i, j, k)

    return maxsaving, servedbyUAV, best_insert


def caltruckcost(j, t, distance_truck, speed_truck, sub_path, distance_uav, speed_uav, savings, maxsaving, servedbyUAV,
                 best_insert, e, s_t, s_r):
    path = sub_path[0]
    a = path[0]
    b = path[-1]
    for index in range(len(path) - 1):
        i = path[index]
        k = path[index + 1]

        if i == j or j == k or i == k:
            continue
        cost = (distance_truck[i][j] + distance_truck[j][k] - distance_truck[i][k]) / speed_truck
        t_u_b = s_t + (distance_uav[a][j] + distance_uav[j][b]) / speed_uav + s_r
        t_p_b = t[b] + cost
        if t_u_b - t_p_b > 0:
            cost = max(0, cost - (t_u_b - t_p_b))
        if t[b] - t[a] + cost < e:
            if savings - cost > maxsaving:
                maxsaving = savings - cost
                servedbyUAV = False
                best_insert = (i, j, k)

    return maxsaving, servedbyUAV, best_insert


def saving(j, t, distance_truck, speed_truck, truckpath, trucksubpath, distance_uav, speed_uav, s_r):
    j_index = truckpath.index(j)
    i = truckpath[j_index - 1]
    k = truckpath[j_index + 1]
    savings = (distance_truck[i][j] + distance_truck[j][k] - distance_truck[i][k]) / speed_truck
    for subpath in trucksubpath:
        if j in subpath[0]:
            if is_UAV_associated(subpath):
                a = subpath[0][0]
                b = subpath[0][-1]
                t_u_b = t[a] + (distance_uav[a][subpath[1]] + distance_uav[subpath[1]][b]) / speed_uav + s_r
                t_p_b = t[b] - savings
                if t_p_b < t_u_b:
                    savings = max(0, savings + (t_p_b - t_u_b))
            break
    return savings


def solvetsp(C, distances_truck, truck_speed):
    # Nearest neighbor initial solution
    n = len(C)
    truckRoute = [0]
    visited = {0}
    t = {0: 0}
    current_node = 0

    while len(visited) <= n:
        nearest_node = None
        nearest_distance = float('inf')

        for node in C:
            if node not in visited and distances_truck[current_node][node] < nearest_distance:
                nearest_node = node
                nearest_distance = distances_truck[current_node][node]

        if nearest_node is not None:
            truckRoute.append(nearest_node)
            visited.add(nearest_node)
            t[nearest_node] = t[current_node] + (nearest_distance / truck_speed)
            current_node = nearest_node

    truckRoute.append(0)
    t[0] = t[current_node] + (distances_truck[current_node][0] / truck_speed)

    # 2-opt optimization
    improved = True
    while improved:
        improved = False
        for i in range(1, len(truckRoute) - 2):
            for j in range(i + 1, len(truckRoute) - 1):
                a, b = truckRoute[i - 1], truckRoute[i]
                c, d = truckRoute[j], truckRoute[j + 1]

                # Current distance
                current = distances_truck[a][b] + distances_truck[c][d]
                # Proposed new distance
                proposed = distances_truck[a][c] + distances_truck[b][d]

                if proposed < current:
                    # Reverse the path between i and j
                    truckRoute[i:j + 1] = truckRoute[j:i - 1:-1]
                    improved = True

        # Recalculate times after route change
        t = {0: 0}
        for i in range(1, len(truckRoute)):
            prev_node = truckRoute[i - 1]
            curr_node = truckRoute[i]
            t[curr_node] = t[prev_node] + (distances_truck[prev_node][curr_node] / truck_speed)

    return truckRoute, t


def fstsp(C, C_prime, distances_truck, truck_speed, e, distances_uav, uav_speed, s_l, s_r):
    truckpath, t = solvetsp(C, distances_truck, truck_speed)
    len_in = len(truckpath)
    trucksubpath = [(copy.deepcopy(truckpath), -1)]
    maxsavings = 0
    servedbyUAV = False
    best_insert = None
    while True:
        for j in C_prime:
            savings = saving(j, t, distances_truck, truck_speed, truckpath, trucksubpath, distances_uav, uav_speed, s_r)
            for subpath in trucksubpath:
                if is_UAV_associated(subpath):
                    maxsavings, servedbyUAV, best_insert = caltruckcost(j, t, distances_truck, truck_speed, subpath,
                                                                        distances_uav, uav_speed, savings, maxsavings,
                                                                        servedbyUAV, best_insert, e, s_l, s_r)
                else:
                    maxsavings, servedbyUAV, best_insert = caluavcost(j, t, subpath, distances_uav, uav_speed, savings,
                                                                      maxsavings, servedbyUAV, best_insert, e, s_l, s_r)

        if maxsavings > 0:
            truckpath, trucksubpath, t, C_prime = updatepath(servedbyUAV, best_insert, distances_truck, truck_speed,
                                                             truckpath, trucksubpath, distances_uav, uav_speed, t,
                                                             C_prime)
            maxsavings = 0
            servedbyUAV = False
            best_insert = None
        else:
            break
    len_fin = len(truckpath)
    print(f"Initial length: {len_in}, Final length: {len_fin}")
    return trucksubpath, t


def manhattan_distance(point1, point2):
    return abs(point1[0] - point2[0]) + abs(point1[1] - point2[1])


def euclidean_distance(point1, point2):
    return np.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)


def run_delivery_simulation(n_points=5, grid_size=10, truck_speed=1.0, uav_speed=2.0, e=10.0, s_l=0.5, s_r=0.5):
    # Generate random points
    points = [
        (0, 0),  # Depot
        (14, 23), (37, 8), (5, 49), (28, 12), (19, 45),
        (42, 31), (7, 18), (33, 50), (50, 3), (21, 27),
        (9, 42), (45, 15), (12, 38), (31, 22), (26, 7),
        (48, 34), (3, 29), (17, 11), (39, 46), (24, 19),
        (30, 27), (17, 35), (24, 55), (76, 33), (86, 27)
    ]
    C = list(range(1, n_points + 1))  # Nodes excluding depot (0)
    C_prime = copy.deepcopy(C)  # Copy for unserved nodes

    # Compute distance matrices
    distances_truck = np.zeros((n_points + 1, n_points + 1))
    distances_uav = np.zeros((n_points + 1, n_points + 1))
    for i in range(n_points + 1):
        for j in range(n_points + 1):
            distances_truck[i][j] = manhattan_distance(points[i], points[j])
            distances_uav[i][j] = euclidean_distance(points[i], points[j])

    # Run the fstsp algorithm
    trucksubpath, t = fstsp(C, C_prime, distances_truck, truck_speed, e, distances_uav, uav_speed, s_l, s_r)
    return points, trucksubpath, t

def calculate_distance(point1, point2):
    #计算两点之间的欧几里得距离
    return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)


if __name__ == "__main__":
    # Parameters
    n_points = 25  # Number of delivery points (excluding depot)
    grid_size = 50  # Grid size (50x50)
    truck_speed = 1.0  # Truck speed (units per time)
    uav_speed = 2.0  # UAV speed (units per time)
    e = 15.0  # UAV endurance
    s_l = 0.5  # UAV launch time
    s_r = 0.5  # UAV recovery time

    # Run simulation
    points, trucksubpath, t = run_delivery_simulation(
        n_points=n_points,
        grid_size=grid_size,
        truck_speed=truck_speed,
        uav_speed=uav_speed,
        e=e,
        s_l=s_l,
        s_r=s_r
    )


    print("Delivery Points:", points)
    print("Truck Subpaths and UAV Usage:", trucksubpath)
    print("Arrival Times:", t)

    """
    delivery_points = [
        (0, 0),  # Depot
        (14, 23), (37, 8), (5, 49), (28, 12), (19, 45),
        (42, 31), (7, 18), (33, 50), (50, 3), (21, 27),
        (9, 42), (45, 15), (12, 38), (31, 22), (26, 7),
        (48, 34), (3, 29), (17, 11), (39, 46), (24, 19),
        (30, 27), (17, 35), (24,55), (76, 33), (86, 27)
    ]

    truck_time = 0.0
    uav_time = 0.0
    visited_nodes = set()  # 用于记录所有被访问过的节点

    for subpath in trucksubpath:
        nodes, uav_target = subpath
        truck_nodes = nodes  # 卡车访问的节点序列

        # 记录卡车访问的节点
        for node in truck_nodes:
            visited_nodes.add(node)

        # 卡车行驶时间（分段计算到达每个节点的时间）
        truck_arrival_times = [truck_time]  # 记录卡车到达每个节点的时间
        for i in range(len(truck_nodes) - 1):
            from_node = truck_nodes[i]
            to_node = truck_nodes[i + 1]
            distance = calculate_distance(delivery_points[from_node], delivery_points[to_node])
            truck_arrival_times.append(truck_arrival_times[-1] + distance / truck_speed)

        truck_time = truck_arrival_times[-1]  # 卡车完成当前子路径的时间

        # 处理无人机任务
        if uav_target != -1:
            visited_nodes.add(uav_target)  # 记录无人机访问的节点
            launch_node = truck_nodes[0]  # 无人机从子路径的第一个节点起飞
            recovery_node = truck_nodes[-1]  # 无人机返回到子路径的最后一个节点

            # 无人机发射时间（卡车必须已到达发射节点）
            uav_launch_time = truck_arrival_times[0]  # 卡车到达发射节点的时间
            uav_time = max(uav_time, uav_launch_time) + s_l  # 发射时间

            # 无人机飞到目标节点
            distance_to_target = calculate_distance(delivery_points[launch_node], delivery_points[uav_target])
            uav_flight_time = distance_to_target / uav_speed
            uav_time += uav_flight_time

            # 无人机从目标节点返回回收节点
            distance_to_recovery = calculate_distance(delivery_points[uav_target], delivery_points[recovery_node])
            uav_time += distance_to_recovery / uav_speed

            # 无人机回收时间
            uav_time += s_r

            # 卡车必须等待无人机返回（如果卡车先到）
            truck_arrival_at_recovery = truck_arrival_times[-1]  # 卡车到达回收节点的时间
            if uav_time > truck_arrival_at_recovery:
                truck_time = uav_time  # 卡车等待无人机
            else:
                uav_time = truck_arrival_at_recovery  # 无人机等待卡车

    # 检查是否所有节点都被访问过
    all_nodes = set(range(len(delivery_points)))
    unvisited_nodes = all_nodes - visited_nodes
    if unvisited_nodes:
        raise ValueError(f"以下节点未被访问: {sorted(unvisited_nodes)}")

    print(max(truck_time, uav_time))
    """