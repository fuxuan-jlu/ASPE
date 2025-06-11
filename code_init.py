# code_init.py
import numpy as np
import copy

class CodeInit:
    def __init__(self):
        pass

    def get_initial_code(self):
        """Return the initial code that will be evolved"""
        return """
import numpy as np
import copy

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

def updatepath(servedbyUAV, best_insert, distance_truck, speed_truck, truckpath, subpath, distance_uav, speed_uav, t, C_prime):
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

def caltruckcost(j, t, distance_truck, speed_truck, sub_path, distance_uav, speed_uav, savings, maxsaving, servedbyUAV, best_insert, e, s_t, s_r):
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
                #savings = min(savings, t_p_b - t_u_b)
            break
    return savings

def solvetsp(C, distances_truck, truck_speed):
    # C include i nodi senza il depot, che è 0
    n = len(C)

    # Inizializza il percorso partendo e tornando al depot (nodo 0)
    truckRoute = [0]

    # Tiene traccia dei nodi visitati
    visited = {0}

    # Dizionario che contiene il tempo di arrivo a ogni nodo
    t = {0: 0}  # Il truck parte dal depot al tempo 0

    current_node = 0

    while len(visited) <= n:  # Finché non si visitano tutti i nodi
        nearest_node = None
        nearest_distance = float('inf')

        # Trova il nodo più vicino non ancora visitato
        for node in C:
            if node not in visited and distances_truck[current_node][node] < nearest_distance:
                nearest_node = node
                nearest_distance = distances_truck[current_node][node]

        # Aggiorna il percorso e il tempo
        if nearest_node is not None:
            truckRoute.append(nearest_node)
            visited.add(nearest_node)
            t[nearest_node] = t[current_node] + (nearest_distance / truck_speed)
            current_node = nearest_node

    # Torna al depot (nodo 0)
    truckRoute.append(0)
    t[0] = t[current_node] + (distances_truck[current_node][0] / truck_speed)

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
                    maxsavings, servedbyUAV, best_insert = caltruckcost(j, t, distances_truck, truck_speed, subpath, distances_uav, uav_speed, savings, maxsavings, servedbyUAV, best_insert, e, s_l, s_r)
                else:
                    maxsavings, servedbyUAV, best_insert = caluavcost(j, t, subpath, distances_uav, uav_speed, savings, maxsavings, servedbyUAV, best_insert, e, s_l, s_r)

        if maxsavings > 0:
            truckpath, trucksubpath, t, C_prime = updatepath(servedbyUAV, best_insert, distances_truck, truck_speed, truckpath, trucksubpath, distances_uav, uav_speed, t, C_prime)
            maxsavings = 0
            servedbyUAV = False
            best_insert = None
        else:
            break
    len_fin = len(truckpath)
    print(f"Lunghezza iniziale : {len_in}, lunghezza finale : {len_fin}")
    return trucksubpath, t

def manhattan_distance(point1, point2):
    return abs(point1[0] - point2[0]) + abs(point1[1] - point2[1])


# Function to generate Euclidean distance for UAV
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
        (30, 27), (17, 35), (24,55), (76, 33), (86, 27)
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

if __name__ == "__main__":
    # Parameters
    n_points = 25  # Number of delivery points (excluding depot)
    grid_size = 10  # Grid size (10x10)
    truck_speed = 1.0  # Truck speed (units per time)
    uav_speed = 2.0  # UAV speed (units per time)
    e = 15.0  # UAV endurance
    s_l = 0.5  # UAV launch time
    s_r = 0.5  # UAV recovery time

    # Run simulation
    points, trucksubpath, t = run_delivery_simulation(n_points, grid_size, truck_speed, uav_speed, e, s_l, s_r)

    # Print results
    print("Delivery Points:", points)
    print("Truck Subpaths and UAV Usage:", trucksubpath)
    print("Arrival Times:", t)
"""