import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from rosbags.rosbag1 import Reader
from rosbags.serde import deserialize_cdr, ros1_to_cdr
from rosbags.typesys import get_types_from_msg, register_types

class KalmanFilter:
    def __init__(self, A, B, H, Q, R, P, x0):
        self.A = A
        self.B = B
        self.H = H
        self.Q = Q
        self.R = R
        self.P = P
        self.x = x0

    def predict(self, u=np.zeros((4,1))):
        self.x = self.A @ self.x + self.B @ u
        self.P = self.A @ self.P @ self.A.T + self.Q

    def update(self, z):
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ (z - self.H @ self.x)
        self.P = (np.eye(self.P.shape[0]) - K @ self.H) @ self.P


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
        i = 0

        # iterate over messages
        for connection, timestamp, rawdata in reader.messages():
            if connection.topic == '/robot_1/merged_datas_bis':
                lidar = deserialize_cdr(ros1_to_cdr(rawdata, connection.msgtype), connection.msgtype)
                lidar_1.append((lidar.robot_1[1].position.x, lidar.robot_1[1].position.y, lidar.robot_1[1].position.z))
                lidar_2.append((lidar.robot_1[2].position.x, lidar.robot_1[2].position.y, lidar.robot_1[2].position.z))
                time_vec.append(time_vec[-1] + 0.1 if time_vec else 0)
        
        lidar_1 = np.array(lidar_1)
        lidar_2 = np.array(lidar_2)

        return time_vec, lidar_1, lidar_2

def main(init_state):
    # Define system matrices
    # We whant to estimate the position and speed of the robot and his heading angle and velocity
    # So the state vector is [x, xdot, y, ydot]
    # In addition to that we want the system to be updated every 0.01 seconds
    dt = 0.1

    # Let's define the state transition matrix
    A = np.array([[1, dt,  0,  0],
                  [0,  1,  0,  0],
                  [0,  0,  1, dt],
                  [0,  0,  0,  1]])
    
    # We don't have any control input so the control matrix is just a zero matrix and the control vector is just a zero vector
    B = np.zeros((4, 4))
    u = np.zeros((4, 1))

    # Let's define the measurement matrix, we just have the mesured x,y position
    H = np.array([[1, 0, 0, 0],
                  [1, 0, 0, 0],
                  [0, 0, 1, 0],
                  [0, 0, 1, 0]])
    
    # Let's define the process noise covariance matrix
    Q_std_pos = 0.008
    Q = np.array([[(dt**4*Q_std_pos**2)/4, dt**3*Q_std_pos**2/2, 0, 0,],
                  [dt**3*Q_std_pos**2/2, dt**2*Q_std_pos**2, 0, 0,],
                  [0, 0, dt**4*Q_std_pos**2/4, dt**3*Q_std_pos**2/2,],
                  [0, 0, dt**3*Q_std_pos**2/2, dt**2*Q_std_pos**2]])

    # Let's define the measurement noise covariance matrix
    lidar_1_pos_std = 0.00157
    lidar_2_pos_std = 0.0039
    R = np.array([[lidar_1_pos_std**2, 0, 0, 0],
                  [0, lidar_2_pos_std**2, 0, 0],
                  [0, 0, lidar_1_pos_std**2, 0],
                  [0, 0, 0, lidar_2_pos_std**2]])
    
    # For the initial covariance matrix we just take a value that assure convergence
    P = np.eye(4)/100

    # Finally let's define the initial state vector
    x0 = np.array([[init_state[0], 0, init_state[1], 0]]).T

    # Create the Kalman filter
    kf = KalmanFilter(A, B, H, Q, R, P, x0)

    # Get the data from the rosbag
    time_vect, lidar_1_vect, lidar_2_vect = get_rosbag_data()

    saved_x = []
    saved_time = []
    saved_var = []

    for lidar_1, lidar_2, time in zip(lidar_1_vect, lidar_2_vect, time_vect):
        # Predict the next state
        kf.predict()

        # We first initialize the measurement matrix
        kf.H = np.zeros((4, 4))

        z = np.array([[lidar_1[0]], [lidar_2[0]], [lidar_1[1]], [lidar_2[1]]])
        
        
        # If all the components of lidar_1 are 0 it means that don't have any measurement from this lidar
        if not np.all(lidar_1 == 0):
            # Now we check if the measurement is correct
            pos_dev = float(np.sqrt((lidar_1[0] - kf.x[0])**2 + (lidar_1[1] - kf.x[2])**2))
            if pos_dev <= 0.15:
                kf.H[0, 0] = 1
                kf.H[2, 2] = 1
            else :
                print("Lidar 1 position measurement is not correct")

        # If all the components of lidar_2 are 0 it means that don't have any measurement from this lidar
        if not np.all(lidar_2 == 0):
            # Now we check if the measurement is correct
            pos_dev = float(np.sqrt((lidar_2[0] - kf.x[0])**2 + (lidar_2[1] - kf.x[2])**2))
            if pos_dev <= 0.15:
                kf.H[1, 0] = 1
                kf.H[3, 2] = 1

        kf.update(z)

        saved_x.append(kf.x)
        saved_time.append(time)
        saved_var.append(kf.P)

    return saved_x, saved_var, saved_time

def clean_zeros(lidar_vect):
    # Replace all zeros vectors with None for display purpose
    for i in range(len(lidar_vect)):
        if np.all(lidar_vect[i,:] == 0):
            lidar_vect[i,:] = None
    
    return lidar_vect

# def animate(k):
#     i = min(k, saved_x[:,1].size)
#     line.set_data(saved_x[:i, 0],saved_x[:i, 2])
#     point.set_data(saved_x[i, 0], saved_x[i, 2])
#     return line, point

if __name__ == '__main__':
    saved_x, saved_var, saved_time = main([0.5, 1.5])
    saved_x = np.array(saved_x)
    saved_var = np.array(saved_var)

    # Get the data from the rosbag
    time_vect, lidar_1_vect, lidar_2_vect = get_rosbag_data()

    lidar_1_vect = clean_zeros(lidar_1_vect)
    lidar_2_vect = clean_zeros(lidar_2_vect)
    # print("Std of lidar 1 : ", np.nanstd(lidar_1_vect[100:200, 0], axis=0))
    # print("Std of lidar 2 : ", np.nanstd(lidar_2_vect[100:200, 0], axis=0))
    
    
    plt.figure("x Position")
    plt.plot(saved_time, saved_x[:, 0], 'b-',label='Kalman filter')
    plt.plot(time_vect, lidar_1_vect[:, 0], 'g+',label='Raw x lidar 1')
    plt.plot(time_vect, lidar_2_vect[:, 0], 'r+',label='Raw x lidar 2')
    plt.xlabel('Time [s]')
    plt.ylabel('X Position [m]')
    plt.legend()

    plt.figure("y Position")
    plt.plot(saved_time, saved_x[:, 2], 'b-',label='Kalman filter')
    plt.plot(time_vect, lidar_1_vect[:, 1], 'g+',label='Raw y lidar 1')
    plt.plot(time_vect, lidar_2_vect[:, 1], 'r+',label='Raw y lidar 2')
    plt.xlabel('Time [s]')
    plt.ylabel('Y Position [m]')
    plt.legend()

    # Plot the results
    img = plt.imread("playground.png")
    fig, ax = plt.subplots()
    ax.set_title("Position sur le Plateau")
    ax.imshow(img, extent=[0, 3, 0, 2])
    plt.plot(saved_x[:, 0], saved_x[:, 2], 'b-', label='Kalman filter')
    plt.plot(lidar_1_vect[:, 0], lidar_1_vect[:, 1], 'g+', label='Raw Lidar 1')
    plt.plot(lidar_2_vect[:, 0], lidar_2_vect[:, 1], 'r+', label='Raw Lidar 2')
    plt.xlabel('X Position [m]')
    plt.ylabel('Y Position [m]')
    plt.legend()

    plt.show()

    # fig, ax = plt.subplots()

    # # Création de la ligne qui sera mise à jour au fur et à mesure
    # line, = ax.plot([],[], color='blue')
    # point, = ax.plot([], [], ls="none", marker="o")

    # #Gestion des limites de la fenêtre
    # ax.set_xlim([0, 3])
    # ax.set_ylim([0, 2])

    # ani = animation.FuncAnimation(fig=fig, func=animate, frames=range(saved_x[:,1].size), interval=50, blit=True)
    # ani.save(filename="courbe.mp4", dpi =80, fps=20)



