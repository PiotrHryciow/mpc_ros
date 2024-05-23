import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import PillowWriter


def string_to_array(df: pd.DataFrame, column: str):
    arr = []
    length = df.count()[column]
    for i in range(length):
        data = df.loc[i, column]
        # print(data, type(data))
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
        # print(data, type(data))
        if type(data) == str:
            data = data.strip("[]")
            data = np.fromstring(data, dtype=float, sep=" ")
        # print(type(data))
        arr.append(data)

    return arr


def main():
    name = "prosta/prosta1.csv"
    df = pd.read_csv("~/mgr/wyniki/" + name, header=0, index_col=0)
    # df["state_x"] = df["state_x"].apply(str_to_list)
    # print(df["state_x"].iloc[0])  # to see the first vector as a numpy array
    # print(type(df["state_x"].iloc[0]))  # to confirm the type is numpy array

    params = string_to_params(df, "params")

    state_x = string_to_array(df, "state_x")
    state_y = string_to_array(df, "state_y")

    path_x = string_to_array(df, "path_x")
    path_y = string_to_array(df, "path_y")

    fig_gif = plt.figure()
    plt.plot(state_x[:, 0], state_y[:, 0], "g", label="path")
    (data,) = plt.plot([], [], "ko", label="position")
    (data_pred,) = plt.plot([], [], "r.", label="mpc prediction")
    plt.plot(path_x, path_y, "b--", label="reference path")

    plt.title(f"R = {params[1]}, control horizon = {params[3]}")

    plt.xlabel("y position")
    plt.ylabel("x position")
    plt.legend(loc="upper left")
    writer = PillowWriter(fps=2.5)
    with writer.saving(fig_gif, "test.gif", 300):
        for i in range(len(state_x)):
            data.set_data(state_x[i, 0], state_y[i, 0])
            data_pred.set_data(state_x[i], state_y[i])

            writer.grab_frame()

    # plt.show()
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
