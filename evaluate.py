import re
import sys
import time
import io
import math
from multiprocessing import Process, Queue

points = [
    (0, 0),  # Depot
    (14, 23), (37, 8), (5, 49), (28, 12), (19, 45),
    (42, 31), (7, 18), (33, 50), (50, 3), (21, 27),
    (9, 42), (45, 15), (12, 38), (31, 22), (26, 7),
    (48, 34), (3, 29), (17, 11), (39, 46), (24, 19),
    (30, 27), (17, 35), (24, 55), (76, 33), (86, 27)
]

truck_speed = 1.0  # Truck speed (units per time)
uav_speed = 2.0  # UAV speed (units per time)
e = 15.0  # UAV endurance
s_l = 0.5  # UAV launch time
s_r = 0.5  # UAV recovery time


def worker(code_str, queue):
    namespace = {'__name__': '__main__'}
    start_time = time.time()

    # 子进程内部重定向 stdout
    old_stdout = sys.stdout
    sys.stdout = io.StringIO()  # 重定向到内存缓冲区

    try:
        exec(code_str, namespace, namespace)
        end_time = time.time()
        queue.put((
            namespace.get('trucksubpath'),
            None,
            end_time - start_time
        ))
    except Exception as e:
        queue.put((None, e, 0))
    finally:

        sys.stdout = old_stdout  # 恢复 stdout（可选，因为子进程会退出）

def Evaluate(code_str):
    """
    提取并执行 `code_str` 中花括号里的 Python 代码，
    返回  运行时间(秒) + 配送完成时间  的浮动值。

    如果运行时间超过5分钟，则返回极大值并结束程序。
    """
    print("evaluate begin!")

    # 2. 清理和准备代码
    code_str = re.sub(r'^[\'"]|[\'"]$', '', code_str)
    code_str = code_str.encode().decode('unicode_escape')

    # 3. 做好 stdout 重定向，准备计时
    old_stdout = sys.stdout
    sys.stdout = io.StringIO()

    queue = Queue()
    p = Process(target=worker, args=(code_str, queue))
    p.start()

    # 非阻塞检查，避免固定等待 300 秒
    start_time = time.time()
    timeout = 300
    while True:
        if not queue.empty():
            break
        if time.time() - start_time > timeout:
            p.terminate()
            #p.join()
            sys.stdout = old_stdout
            return 1000
        time.sleep(0.1)  # 避免忙等待

    trucksubpath, error, run_time = queue.get()
    #p.join()  # 确保子进程退出
    p.terminate()
    sys.stdout = old_stdout

    if error is not None or trucksubpath is None:
        return 1000

    try:
        completion_time = get_com_time(trucksubpath, points, truck_speed, uav_speed, s_l, s_r)
    except Exception as e:
        raise ValueError(str(e))
        #return 1000
    print("evaluate success!")
    return run_time + completion_time

def calculate_distance(point1, point2):
    #计算两点之间的欧几里得距离
    return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)


def get_com_time(trucksubpath, delivery_points, truck_speed, uav_speed, s_l, s_r):
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

    return max(truck_time, uav_time)


