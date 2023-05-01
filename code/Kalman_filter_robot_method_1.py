import numpy as np
import matplotlib.pyplot as plt
from rosbags.rosbag1 import Reader
from rosbags.serde import deserialize_cdr, ros1_to_cdr
from rosbags.typesys import get_types_from_msg, register_types
from filterpy.stats import plot_covariance_ellipse

def get_rosbag_data():
    with Reader('data/propre_1.bag') as reader:
        typs = {}
        for name, topic in reader.topics.items():
            typs.update(get_types_from_msg(topic.msgdef, topic.msgtype))
        register_types(typs)
        
        # topic and msgtype information is available on .connections list
        for connection in reader.connections:
            print(connection.topic, connection.msgtype)

        lidar_1 = []
        lidar_2 = []
        time_vec = []
        odom_1 = []
        i = 0

        # iterate over messages
        for connection, timestamp, rawdata in reader.messages():
            i += 1
            if i > 50:
                if connection.topic == '/robot_1/merged_datas_bis':
                    lidar = deserialize_cdr(ros1_to_cdr(rawdata, connection.msgtype), connection.msgtype)
                    lidar_1.append((lidar.robot_1[1].position.x, lidar.robot_1[1].position.y, lidar.robot_1[1].position.z))
                    lidar_2.append((lidar.robot_1[2].position.x, lidar.robot_1[2].position.y, lidar.robot_1[2].position.z))
                    odom_1.append((lidar.robot_1[0].vitesse.x, lidar.robot_1[0].vitesse.y, lidar.robot_1[0].vitesse.z, lidar.robot_1[0].position.x))
                    time_vec.append(time_vec[-1] + 0.1 if time_vec else 0)
        
        lidar_1 = np.array(lidar_1)
        lidar_2 = np.array(lidar_2)
        odom_1 = np.array(odom_1)

        return lidar_1, lidar_2, time_vec, odom_1
    
if __name__ == '__main__':
    reg_data_x = []
    reg_pred1_x = []
    reg_pred2_x = []
    reg_pred1_y = []
    reg_pred2_y = []
    reg_pred1_t = []
    reg_pred2_t = []
    acc = [0]
    reg_pred1_x_v = [0]
    reg_pred2_x_v = [0]
    reg_pred1_y_v = [0]
    reg_pred2_y_v = [0]
    reg_pred1_t_v = [0]
    reg_pred2_t_v = [0]
    reg_temp = []
    reg_lid_1_x = []
    reg_lid_2_x = []
    reg_data_y = []
    reg_lid_1_y = []
    reg_lid_2_y = []
    reg_data_t = []
    reg_lid_1_t = []
    reg_lid_2_t = []
    reg_upgrade_x = []
    reg_upgrade_y = []
    reg_upgrade_t = []
    reg_upgrade_x_v = [0]
    reg_upgrade_y_v = [0]
    reg_upgrade_t_v = [0]
    reg_data_x_v = [0]
    reg_data_y_v = [0]
    reg_data_t_v = [0]
    lidar_1, lidar_2, time, odo = get_rosbag_data()
    dt = 0.1
    t = 0
    Q_std_pos = 0.008
    Q_std = 0.008
    Q_p = 0.4
    Q_v = 0.4
    R_std_11 = 0.01
    R_std_21 = 0.01
    A = np.array([[1, dt, 0, 0, 0, 0],
                 [0, 1, 0, 0, 0, 0],
                 [0, 0, 1, dt, 0, 0], 
                 [0, 0, 0, 1, 0, 0], 
                 [0, 0, 0, 0, 1, dt],
                 [0, 0, 0, 0, 0, 1]])
    H = np.array([[1, 0, 0, 0, 0, 0],
                  [1, 0, 0, 0, 0, 0],
                    [0, 0, 1, 0, 0, 0],
                    [0, 0, 1, 0, 0, 0],
                    [0, 0, 0, 0, 1, 0],
                    [0, 0, 0, 0, 1, 0]])
    H_1 = np.identity(6)
    P = np.identity(6)
    P_1 = np.identity(6)
    Q = np.array([[dt**4*Q_std_pos**2/4, dt**3*Q_std_pos**2/2, 0, 0, 0, 0],
                  [dt**3*Q_std_pos**2/2, dt**2*Q_std_pos**2, 0, 0, 0, 0], 
                    [0, 0, dt**4*Q_std_pos**2/4, dt**3*Q_std_pos**2 / 2, 0, 0],
                    [0, 0, dt**3*Q_std_pos**2/2, dt**2 * Q_std_pos**2, 0, 0],
                    [0, 0, 0, 0, dt**4*Q_std_pos**2/4, dt**3*Q_std_pos**2/2],
                    [0, 0, 0, 0, dt**3*Q_std_pos**2/2, dt**2*Q_std_pos**2]])
    Q_1 = np.array([[dt**4*Q_p**2/4, dt**3*Q_p*Q_v/2, 0, 0, 0, 0],
                    [dt**3*Q_p*Q_v/2, dt**2*Q_v ** 2, 0, 0, 0, 0],
                    [0, 0, dt**4*Q_p**2/4, dt**3*Q_p*Q_v/2, 0, 0],
                    [0, 0, dt**3*Q_p*Q_v/2, dt**2*Q_v ** 2, 0, 0],
                    [0, 0, 0, 0, dt**4*Q_p**2/4, dt**3*Q_p*Q_v/2],
                    [0, 0, 0, 0, dt**3*Q_p*Q_v/2, dt**2*Q_v ** 2]])
    R = np.array([[R_std_11 ** 2, 0, 0, 0, 0, 0],
                  [0, R_std_21 ** 2, 0, 0, 0, 0], 
                    [0, 0, R_std_11 ** 2, 0, 0, 0],
                    [0, 0, 0, R_std_21 ** 2, 0, 0], 
                    [0, 0, 0, 0, R_std_11 ** 2, 0],
                    [0, 0, 0, 0, 0, R_std_21 ** 2]])
    R_1 = np.array([[Q_std_pos ** 2, 0, 0, 0, 0, 0],
                  [0, Q_std ** 2, 0, 0, 0, 0],
                    [0, 0, Q_std_pos ** 2, 0, 0, 0],
                    [0, 0, 0, Q_std ** 2, 0, 0],
                    [0, 0, 0, 0, Q_std_pos ** 2, 0],
                    [0, 0, 0, 0, 0, Q_std ** 2]])
    B = np.array([[dt ** 2 / 2, 0, 0],
                  [dt, 0, 0], 
                    [0, dt ** 2 / 2, 0],
                    [0, dt, 0], 
                    [0, 0, dt ** 2 / 2],
                    [0, 0, dt]])
    u = np.array([[0], 
                  [0], 
                    [0]])
    u_1 = np.array([[0], 
                  [0], 
                    [0]])
    x_f1 = np.array([[lidar_1[0][0]],
                  [0], 
                    [lidar_1[0][1]],
                    [0], 
                    [lidar_1[0][2]],
                    [0]])
    reg_data_x.append(x_f1[0][0])
    reg_temp.append(t)
    reg_lid_1_x.append(lidar_1[0][0])
    reg_lid_2_x.append(lidar_2[0][0])
    reg_data_y.append(x_f1[2][0])
    reg_lid_1_y.append(lidar_1[0][1])
    reg_lid_2_y.append(lidar_2[0][1])
    reg_data_t.append(x_f1[4][0])
    reg_lid_1_t.append(lidar_1[0][2])
    reg_lid_2_t.append(lidar_2[0][2])
    reg_upgrade_x.append(x_f1[0][0])
    reg_upgrade_y.append(x_f1[2][0])
    reg_upgrade_t.append(x_f1[4][0])
    reg_pred1_x.append(x_f1[0][0])
    reg_pred1_y.append(x_f1[2][0])
    reg_pred1_t.append(x_f1[4][0])
    reg_pred2_x.append(x_f1[0][0])
    reg_pred2_y.append(x_f1[2][0])
    reg_pred2_t.append(x_f1[4][0])
    for i in range(len(time)):
        x_prev = x_f1.copy()
        x_pred1 = x_f1.copy()
        x_pred2 = x_f1.copy()
        t += 1
        if odo[i][3] != 0:
            x_pred1[1][0] = odo[i][0]
            x_pred1[3][0] = odo[i][1]
            x_pred1[5][0] = odo[i][2]
            if abs(np.sqrt(odo[i][0] ** 2 + odo[i][1] ** 2) - np.sqrt(x_f1[1][0] ** 2 + x_f1[3][0] ** 2)) > 0.03:
                u_1 = np.array([[-(x_f1[1][0] - odo[i][0]) / dt *0.3],
                                [-(x_f1[3][0] - odo[i][1]) / dt * 0.3],
                                [0]])
                Q_p = 0.4
                Q_v = 0.4
                acc.append(1)
            else:
                u_1 = np.array([[0],
                                [0],
                                [0]])
                acc.append(0)
                Q_p = 0.00
                Q_v = 0.00
        else:
            u_1 = np.array([[0],
                            [0],
                            [0]])
            acc.append(0)
            Q_p = 0.001
            Q_v = 0.001
        x_pred1 = A @ x_pred1 + B @ u
        P = A @ P @ A.T + Q
        S = H @ P @ H.T + R
        K = P @ H.T @ np.linalg.inv(S)
        x_meas = np.array([[lidar_1[i][0]],
                            [lidar_2[i][0]], 
                            [lidar_1[i][1]],
                            [lidar_2[i][1]], 
                            [lidar_1[i][2]],
                            [lidar_2[i][2]]])
        if lidar_1[i][0] != 0 and lidar_1[i][1] != 0:
            H[0][0] = 1
            H[2][2] = 1
            H[4][4] = 1
        else:
            H[0][0] = 0
            H[2][2] = 0
            H[4][4] = 0
        if lidar_2[i][0] != 0 and lidar_2[i][1] != 0:
            H[1][0] = 1
            H[3][2] = 1
            H[5][4] = 1
        else:
            H[1][0] = 0
            H[3][2] = 0
            H[5][4] = 0
        
        x_f1 = x_pred1 + K @ (x_meas - H @ x_pred1)
        P = (np.eye(P.shape[0]) - K @ H) @ P
        x_pred2 = A @ x_pred2 + B @ u_1
        P_1 = A @ P_1 @ A.T + Q_1
        S_1 = H_1 @ P_1 @ H_1.T + R_1
        K_1 = P_1 @ H_1.T @ np.linalg.inv(S_1)
        x_f2 = x_pred2 + K_1 @ (x_f1 - H_1 @ x_pred2)
        P_1 = (np.eye(P_1.shape[0]) - K_1 @ H_1) @ P_1



        reg_data_x.append(x_f1[0][0])
        reg_temp.append(t)
        reg_lid_1_x.append(lidar_1[i][0] if lidar_1[i][0] != 0 else None)
        reg_lid_2_x.append(lidar_2[i][0] if lidar_2[i][0] != 0 else None)
        reg_data_y.append(x_f1[2][0])
        reg_lid_1_y.append(lidar_1[i][1] if lidar_1[i][1] != 0 else None)
        reg_lid_2_y.append(lidar_2[i][1] if lidar_2[i][1] != 0 else None)
        reg_data_t.append(x_f1[4][0])
        reg_lid_1_t.append(lidar_1[i][2] if lidar_1[i][2] != 0 else None)
        reg_lid_2_t.append(lidar_2[i][2] if lidar_2[i][2] != 0 else None)
        reg_upgrade_x.append(x_f1[0][0])
        reg_upgrade_y.append(x_f1[2][0])
        reg_upgrade_t.append(x_f1[4][0])
        reg_data_x_v.append(x_f1[1][0])
        reg_data_y_v.append(x_f1[3][0])
        reg_data_t_v.append(x_f1[5][0])
        reg_upgrade_x_v.append(x_f2[1][0])
        reg_upgrade_y_v.append(x_f2[3][0])
        reg_upgrade_t_v.append(x_f2[5][0])
        reg_pred1_x.append(x_pred1[0][0])
        reg_pred1_y.append(x_pred1[2][0])
        reg_pred1_t.append(x_pred1[4][0])
        reg_pred2_x.append(x_pred2[0][0])
        reg_pred2_y.append(x_pred2[2][0])
        reg_pred2_t.append(x_pred2[4][0])
        reg_pred1_x_v.append(x_pred1[1][0])
        reg_pred1_y_v.append(x_pred1[3][0])
        reg_pred1_t_v.append(x_pred1[5][0])
        reg_pred2_x_v.append(x_pred2[1][0])
        reg_pred2_y_v.append(x_pred2[3][0])
        reg_pred2_t_v.append(x_pred2[5][0])
    plt.figure("Test en x")
    plt.plot(reg_temp, reg_data_x, 'b-', label='Kalman filter')
    plt.plot(reg_temp, reg_lid_1_x, 'r-', label='Données Lidar1')
    plt.plot(reg_temp, reg_lid_2_x, 'g-', label='Données Lidar2')
    # plt.plot(reg_temp, reg_upgrade_x, 'y-', label='Kalman filter amélioré')
    # plt.plot(reg_temp, reg_pred1_x, 'k-', label='Pred 1')
    # plt.plot(reg_temp, reg_pred2_x, 'c-', label='Pred 2')
    plt.xlabel('Time [s]')
    plt.ylabel('Position x')
    plt.legend()
    plt.figure("Test en y")
    plt.plot(reg_temp, reg_data_y, 'b-', label='Kalman filter')
    plt.plot(reg_temp, reg_lid_1_y, 'r-', label='Données Lidar1')
    plt.plot(reg_temp, reg_lid_2_y, 'g-', label='Données Lidar2')
    # plt.plot(reg_temp, reg_upgrade_y, 'y-', label='Kalman filter amélioré')
    # plt.plot(reg_temp, reg_pred1_y, 'k-', label='Pred 1')
    # plt.plot(reg_temp, reg_pred2_y, 'c-', label='Pred 2')
    plt.xlabel('Time [s]')
    plt.ylabel('Position y')
    plt.legend()
    # plt.figure("Test en t")
    # plt.plot(reg_temp, reg_data_t)
    # plt.plot(reg_temp, reg_lid_1_t)
    # plt.plot(reg_temp, reg_lid_2_t)
    # plt.plot(reg_temp, reg_upgrade_t)
    plt.figure("Test vitesse en x")
    plt.plot(reg_temp, reg_data_x_v, 'b-', label='Kalman filter')
    # plt.plot(reg_temp, reg_upgrade_x_v, 'r-', label='Kalman filter amélioré')
    # plt.plot(reg_temp, reg_pred1_x_v, 'y-', label='Pred 1')
    # plt.plot(reg_temp, reg_pred2_x_v, 'k-', label='Pred 2')
    # plt.plot(reg_temp, acc, 'c-', label = 'Acceleration')
    plt.xlabel('Time [s]')
    plt.ylabel('Vitesse x')
    plt.legend()
    plt.figure("Test vitesse en y")
    plt.plot(reg_temp, reg_data_y_v, 'b-', label='Kalman filter')
    # plt.plot(reg_temp, reg_upgrade_y_v, 'r-', label='Kalman filter amélioré')
    # plt.plot(reg_temp, reg_pred1_y_v, 'y-', label='Pred 1')
    # plt.plot(reg_temp, reg_pred2_y_v, 'k-', label='Pred 2')
    plt.xlabel('Time [s]')
    plt.ylabel('Vitesse y')
    plt.legend()

    plt.figure("Test sur plateau")
    # plt.plot(reg_temp, reg_data_y, 'b-', label='Kalman filter')
    plt.plot(reg_lid_1_x, reg_lid_1_y, 'r+', label='Données Lidar1')
    plt.plot(reg_lid_2_x, reg_lid_2_y, 'g+', label='Données Lidar2')
    plt.plot(reg_data_x, reg_data_y, 'b-', label='Kalman filter')
    # plt.plot(reg_temp, reg_pred1_y, 'k-', label='Pred 1')
    # plt.plot(reg_temp, reg_pred2_y, 'c-', label='Pred 2')
    plt.xlabel('Position x')
    plt.ylabel('Position y')
    plt.legend()
    plt.show()