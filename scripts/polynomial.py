def polynomials_fit(x_list, y_list, polys_count):
    poly = []
    for i in range(0, polys_count):
        x_n = [x_list[i * 2 + 1] - 0.02, x_list[i * 2 + 1] - 0.01, x_list[i * 2 + 1], x_list[i * 2 + 2],
               x_list[i * 2 + 2] + 0.01, x_list[i * 2 + 2] + 0.02]
        x_n_line = [x_list[i * 2], x_list[i * 2 + 1]]
        # print("x_n_line: ", x_n_line)
        # print("x_n: ", x_n)
        y_n = [y_list[i * 2 + 1], y_list[i * 2 + 1], y_list[i * 2 + 1], y_list[i * 2 + 2], y_list[i * 2 + 2],
               y_list[i * 2 + 2]]
        y_n_line = [y_list[i * 2], y_list[i * 2 + 1]]
        # print("y_n_line: ", y_n_line)
        # print("y_n: ", y_n)
        poly.append(np.polyfit(x_n_line, y_n_line, 1))
        poly.append(np.polyfit(x_n, y_n, 3))
    x_n_line_last = [x_list[-2], x_list[-1]]
    y_n_line_last = [y_list[-2], y_list[-1]]
    poly.append(np.polyfit(x_n_line_last, y_n_line_last, 1))
    # print("polys: ", polys)
    return poly