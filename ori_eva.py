import re
import sys
import time
import io
import math

points = [(0, 0), (0, 9), (3, 1), (1, 8), (10, 2), (5, 6), (7, 10), (7, 0), (7, 8), (2, 5), (8, 9), (8, 1), (6, 7),
          (9, 4), (5, 8), (8, 8), (6, 6), (6, 4), (9, 6), (9, 7), (2, 2)]
truck_speed = 1.0  # Truck speed (units per time)
uav_speed = 2.0  # UAV speed (units per time)
e = 15.0  # UAV endurance
s_l = 0.5  # UAV launch time
s_r = 0.5  # UAV recovery time


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
    start_time = time.time()

    # 4. 设置超时检查（5分钟即300秒）
    max_execution_time = 300  # 5 minutes

    # 5. 执行代码；把 __name__ 设为 "__main__" 以触发主程序
    namespace = {'__name__': '__main__'}
    try:
        # 检查执行时间，如果超过最大限制，立即返回极大值
        if time.time() - start_time > max_execution_time:
            print("Execution time exceeded the 5-minute limit. Returning large value.")
            return 1000

        exec(code_str, namespace, namespace)

    finally:
        # 6. 计算运行时长并恢复 stdout
        run_time = time.time() - start_time
        sys.stdout = old_stdout

    if 'trucksubpath' not in namespace:
        raise RuntimeError("执行后未发现trucksubpath，无法得到配送完成时间")
    try:
        tru_sub_path = namespace['trucksubpath']
        completion_time = get_com_time(tru_sub_path, points, truck_speed, uav_speed, s_l, s_r)
    except Exception as e:
        return 1000

    """
    # 7. 取得配送完成时间
    if 't' not in namespace:
        raise RuntimeError("执行后未发现变量 t，无法得到配送完成时间")
    t_dict = namespace['t']
    if not isinstance(t_dict, dict) or not t_dict:
        raise RuntimeError("变量 t 不是非空 dict，无法解析完成时间")
    completion_time = t_dict[0]
    """

    print("evaluate success!")

    # 8. 返回运行时间和配送完成时间之和
    return run_time + completion_time


def calculate_distance(point1, point2):
    # 计算两点之间的欧几里得距离
    return math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)


def get_com_time(trucksubpath, delivery_points, truck_speed, uav_speed, s_l, s_r):
    truck_time = 0.0
    uav_time = 0.0
    # current_truck_pos = delivery_points[0]  # 初始位置是仓库（节点0）

    for subpath in trucksubpath:
        nodes, uav_target = subpath
        truck_nodes = nodes  # 卡车访问的节点序列

        # 卡车行驶时间（分段计算到达每个节点的时间）
        truck_arrival_times = [truck_time]  # 记录卡车到达每个节点的时间
        for i in range(len(truck_nodes) - 1):
            from_node = truck_nodes[i]
            to_node = truck_nodes[i + 1]
            distance = calculate_distance(delivery_points[from_node], delivery_points[to_node])
            truck_arrival_times.append(truck_arrival_times[-1] + distance / truck_speed)

        truck_time = truck_arrival_times[-1]  # 卡车完成当前子路径的时间
        # current_truck_pos = delivery_points[truck_nodes[-1]]

        # 处理无人机任务
        if uav_target != -1:
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

    return max(truck_time, uav_time)


