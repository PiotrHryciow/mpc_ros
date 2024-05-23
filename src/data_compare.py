import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

import os


def string_to_array(df: pd.DataFrame, column: str):
    arr = []
    length = df.count()[column]
    for i in range(length):
        data = df.loc[i, column]
        print(data, type(data))
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
        print(data, type(data))
        if type(data) == str:
            data = data.strip("[]")
            data = np.fromstring(data, dtype=float, sep=" ")
        # print(type(data))
        arr.append(data)

    return arr


def main():
    file_dir = "/home/piotr/mgr/wyniki/tylem/"
    file_list = os.listdir(file_dir)

    fig_path = plt.figure()

    for file in file_list:
        df = pd.read_csv(file_dir + file, header=0, index_col=0)

        params = string_to_params(df, "params")
        state_x = string_to_array(df, "state_x")
        state_y = string_to_array(df, "state_y")

        plt.plot(
            state_x[:, 0],
            state_y[:, 0],
            "o",
            label=f"R = {params[1]}, control horizon = {params[3]}",
        )

        path_x = string_to_array(df, "path_x")
        path_y = string_to_array(df, "path_y")
        # print(state_x[:, 0])

        plt.plot(path_x, path_y, label="reference path")
    plt.legend(loc="upper left")
    plt.xlabel("x position")
    plt.ylabel("y position")
    plt.title(f"R = {params[1]}, control horizon = {params[3]}")

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
