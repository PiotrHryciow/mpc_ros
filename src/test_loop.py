from MPC import ModelPredictiveControl
import math
import numpy as np
import matplotlib

matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.animation import PillowWriter


def circle(t):
    x = math.cos(t)
    y = math.sin(t)
    return np.array([[x], [y]])


def main():

    N = 7
    dt = 0.2
    Lf = 0.2
    states_list = np.zeros((4, N + 1))
    # controls_list = np.zeros((2, N))

    # states_sol = []
    # control_sol = []
    states_sol = np.empty([4, 1])
    control_sol = np.empty([2, 1])

    output_list = []

    mpc = ModelPredictiveControl(N, dt, Lf, [2, -1, 0.3, 10, -5])
    # control, state = mpc.solve(ref, X0)
    path = np.array([[0], [0]])
    k = 50
    rotations = 2

    for i in range(k + 1):
        path = np.hstack((path, circle((rotations * i * 2 * math.pi) / k)))
        print(path)

    for i in range(k + 2):
        if k - i > N:
            ref = path[:, i : i + N + 1]
        else:
            ref = np.hstack((ref[:, 1:], path[:, -1:]))

        X0 = states_list[:, 1]
        # print(X0)
        controls_list, states_list = mpc.solve(ref, X0)
        # print(states_list[:, :1])
        # print(states_list[:, :1].shape)
        output_list.append(states_list[:2, :])
        states_sol = np.hstack((states_sol, states_list[:, :1]))

    # fig = plt.figure()
    # plt.plot(state[0], 'r',label="x")
    # plt.plot(ref[0],'r--',label="x_ref")

    # plt.plot(state[1],'b',label="y")
    # plt.plot(ref[1],'b--',label="y_ref")
    # plt.legend(loc="upper left")

    fig_path = plt.figure()
    plt.plot(states_sol[0], states_sol[1], label="path")
    plt.plot(path[0], path[1], "--", label="reference path")
    plt.legend(loc="upper left")

    # fig_controls = plt.figure()
    # plt.plot(state[2],label="heading")
    # plt.plot(state[3],label="vel")
    # plt.plot(control[0], '--',label="acc")
    # plt.plot(control[1],'--',label="steering angle")
    # plt.legend(loc="upper left")

    # fig_gif = plt.figure()
    # (data,) = plt.plot([], [], "ko", label="position")
    # (data_ref,) = plt.plot([], [], "r.", label="mpc prediction")
    # plt.plot(path[0], path[1], "b--", label="reference path")
    # plt.xlabel("y position")
    # plt.ylabel("x position")
    # plt.legend(loc="upper left")
    # writer = PillowWriter(fps=5)
    # with writer.saving(fig_gif, "test.gif", 100):
    #     for i in range(k + 2):
    #         # print(output_list[i][0])
    #         # plt.plot(states_sol[0, i], states_sol[1, i],"ro", label="path")
    #         data.set_data(states_sol[0, i], states_sol[1, i])
    #         # plt.plot(output_list[i][0], output_list[i][1], "r.")
    #         data_ref.set_data(output_list[i][0], output_list[i][1])

    #         writer.grab_frame()

    plt.show()


if __name__ == "__main__":
    main()
