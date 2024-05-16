from MPC import ModelPredictiveControl
import numpy as np
import matplotlib

matplotlib.use("TkAgg")
import matplotlib.pyplot as plt


def main():
    ref = np.array(
        [[0, 1, 2, 3, 4, 5, 5, 5, 5, 5, 5], [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10]]
    )
    X0 = [1, 0, 2, 4]

    mpc = ModelPredictiveControl(10, 0.2, [0.1], [2, -1, 0.3, 20, -5])
    control, state = mpc.solve(ref, X0)

    print()

    fig = plt.figure()

    plt.plot(state[0], "r", label="x")
    plt.plot(ref[0], "r--", label="x_ref")

    plt.plot(state[1], "b", label="y")
    plt.plot(ref[1], "b--", label="y_ref")

    plt.legend(loc="upper left")

    fig_path = plt.figure()
    plt.plot(state[0], state[1], label="path")
    plt.plot(ref[0], ref[1], "--", label="reference path")
    plt.legend(loc="upper left")

    fig_controls = plt.figure()
    plt.plot(state[2], label="heading")
    plt.plot(state[3], label="vel")
    plt.plot(control[0], "--", label="acc")
    plt.plot(control[1], "--", label="steering angle")
    plt.legend(loc="upper left")

    plt.show()


if __name__ == "__main__":
    main()
