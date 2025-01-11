import matplotlib.pyplot as plt


def plot_path_before_and_after_spline(path_before, path_after, separate_plots=False):        
    idx_before = range(0, len(path_before[0]))
    idx_after = range(0, len(path_after[0]))
    print(idx_before)
    x_before = [path_before[0][i] for i in idx_before]
    y_before = [path_before[1][i] for i in idx_before]
    x_after = [path_after[0][i] for i in idx_after]
    y_after = [path_after[1][i] for i in idx_after]
    if separate_plots:
        # Plot the path into two subplots
        fig, ax = plt.subplots(2, 1, figsize=(10, 10))
        ax[0].plot(x_before, y_before, 'r--', label="Before Spline")
        ax[0].set_title("Before Spline")
        ax[0].set_xlabel("X")
        ax[0].set_ylabel("Y")
        ax[0].legend()
        ax[1].plot(x_after, y_after, 'b', label="After Spline")
        ax[1].set_title("After Spline")
        ax[1].set_xlabel("X")
        ax[1].set_ylabel("Y")
        ax[1].legend()
        plt.show()
    else:
        fig, ax = plt.subplots(1, 1, figsize=(10, 10))
        ax.plot(x_before, y_before, 'r', label="Before Spline")
        ax.plot(x_after, y_after, 'b', label="After Spline")
        ax.set_title("Before and After Spline")
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.legend()
        plt.show()
    pass
    
    
if __name__ == '__main__':
    path_before_x = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
    path_before_y = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
    path_before = [path_before_x, path_before_y]
    path_after_x = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
    path_after_y = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
    path_after = [path_after_x, path_after_y]
    plot_path_before_and_after_spline(path_before, path_after)
