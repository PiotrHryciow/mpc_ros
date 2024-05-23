import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


def string_to_array(df: pd.DataFrame, column: str):
    arr = []
    length = df.count()[column]
    for i in range(length):
        data = df.loc[i, column]
        if type(data) == str:
            data = data.strip("[]")
            data = np.fromstring(data, dtype=float, sep=" ")
        # print(type(data))
        arr.append(data)
    arr_2d = np.array(arr)

    return arr_2d


def string_to_params(df: pd.DataFrame, column: str):
    arr = []
    length = df.count()[column]
    for i in range(length):
        data = df.loc[i, column]
        if type(data) == str:
            data = data.strip("[]")
            data = np.fromstring(data, dtype=float, sep=" ")
        # print(type(data))
        arr.append(data)

    return arr


def main():
    name = "luk/luk3.csv"
    df = pd.read_csv("~/mgr/wyniki/" + name, header=0, index_col=0)
    # df["state_x"] = df["state_x"].apply(str_to_list)
    # print(df["state_x"].iloc[0])  # to see the first vector as a numpy array
    # print(type(df["state_x"].iloc[0]))  # to confirm the type is numpy array

    params = string_to_params(df, "params")

    # --------- figure of position ---------
    state_x = string_to_array(df, "state_x")
    state_y = string_to_array(df, "state_y")

    path_x = string_to_array(df, "path_x")
    path_y = string_to_array(df, "path_y")

    fig_path = plt.figure()
    plt.plot(state_x[:, 0], state_y[:, 0], "-o", label="path")
    plt.plot(path_x, path_y, ".", label="reference path")
    plt.legend(loc="upper left")
    plt.xlabel("x position")
    plt.ylabel("y position")
    plt.title(f"R = {params[1]}, control horizon = {params[3]}")

    # --------- figure of velocity and acceleration ---------
    state_v = string_to_array(df, "state_v")
    control_a = string_to_array(df, "control_a")

    fig_vel = plt.figure()
    plt.plot(state_v[:, 0], "b-o", label="velocity")
    plt.axhline(y=params[4][3], color="lightskyblue", label="velocity max")
    plt.axhline(y=params[4][4], color="lightskyblue", label="velocity min")

    plt.plot(control_a[:, 0], "g-o", label="acceleration control input")
    plt.axhline(y=params[4][0], color="palegreen", label="acceleration max")
    plt.axhline(y=params[4][1], color="palegreen", label="acceleration min")
    plt.legend(loc="upper left")
    plt.xlabel("time")
    # plt.ylabel("y position")
    plt.title("Velocity and acceleration graph")

    # --------- figure of steering angles ---------
    state_psi = string_to_array(df, "state_psi")
    control_delta = string_to_array(df, "control_delta")

    fig_steering = plt.figure()
    plt.plot(state_psi[:, 0], "k-o", label="psi - heading direction")
    plt.plot(control_delta[:, 0], "b-o", label="delta - wheel steering angle")
    # plt.plot(state_x[:, 0], "-o", label="x position")
    # plt.plot(state_y[:, 0], "-o", label="y position")
    plt.axhline(y=params[4][2], color="lightskyblue", label="delta max")
    plt.axhline(y=-params[4][2], color="lightskyblue", label="delta min")
    plt.legend(loc="upper left")
    plt.xlabel("time")

    plt.show()
    # print(state_x)
    # data = df.loc[1, "data1"]
    # # print(data)
    # print(type(data))
    # # print(data[0])
    # data_new = df.loc[1, "data1"].strip("[]")
    # data_array = np.fromstring(data_new, dtype=int, sep=" ")
    # print(data_array)
    # print(type(data_array))


if __name__ == "__main__":
    main()
