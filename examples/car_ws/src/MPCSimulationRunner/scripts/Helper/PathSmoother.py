import math


class PathSmoother:
    def __init__(self):
        pass
    
    def __str__(self):
        return "Path Smoothing using Exponential Moving Average"
    
    @staticmethod
    def path_smoother_forward(original_path):
    # x, y, heading

        forward_path = [[0 for i in range(len(original_path[j]))] for j in range(len(original_path))]

        alpha = 0.98

        forward_path[0] = [original_path[0][0], original_path[0][1], original_path[0][2]]

        for i in range(1, len(original_path)):

            dist = ((original_path[i - 1][0] - original_path[i][0]) ** 2 + (original_path[i - 1][1] - original_path[i][1]) ** 2) ** 0.5
            tempAngle = math.atan2(original_path[i][1] - original_path[i - 1][1], original_path[i][0] - original_path[i - 1][0])

            if i == 1:
                nextAngle = tempAngle
            else:
                nextAngle = alpha * forward_path[i - 1][2] + (1 - alpha) * tempAngle
            
            point = [math.cos(nextAngle) * dist, math.sin(nextAngle) * dist]

            forward_path[i][0] = forward_path[i - 1][0] + point[0]
            forward_path[i][1] = forward_path[i - 1][1] + point[1]
            forward_path[i][2] = nextAngle

        return forward_path

    @staticmethod
    def path_smoother_backward(_original_path):
        # x, y, heading
        original_path = [[_original_path[len(_original_path) - i - 1][j] for j in range(len(_original_path[i]))] for i in range(len(_original_path))]

        backward_path = [[0 for i in range(len(original_path[j]))] for j in range(len(original_path))]

        alpha = 0.98

        backward_path[0] = [original_path[0][0], original_path[0][1], 3.141592]

        for i in range(1, len(original_path)):

            dist = ((original_path[i - 1][0] - original_path[i][0]) ** 2 + (original_path[i - 1][1] - original_path[i][1]) ** 2) ** 0.5
            tempAngle = math.atan2(original_path[i][1] - original_path[i - 1][1], original_path[i][0] - original_path[i - 1][0])
            if abs(backward_path[i - 1][2] - tempAngle) >= 3.141592:
                tempAngle += 2 * 3.141592

            if i == 1:
                nextAngle = 3.141592
            else:
                nextAngle = alpha * backward_path[i - 1][2] + (1 - alpha) * tempAngle
            
            point = [math.cos(nextAngle) * dist, math.sin(nextAngle) * dist]

            backward_path[i][0] = backward_path[i - 1][0] + point[0]
            backward_path[i][1] = backward_path[i - 1][1] + point[1]
            backward_path[i][2] = nextAngle

        return backward_path

    @staticmethod
    def path_smoother(forward_path, _backward_path):
        if len(forward_path) != len(_backward_path):
            print("Not matched path length\n")
            return False

        backward_path = [[_backward_path[len(_backward_path) - i - 1][j] for j in range(len(_backward_path[i]))] for i in range(len(_backward_path))]

        smoothed_path = [[0 for i in range(len(forward_path[j]))] for j in range(len(forward_path))]

        for i in range(len(smoothed_path)):
            x = (forward_path[i][0] + backward_path[i][0]) / 2
            y = (forward_path[i][1] + backward_path[i][1]) / 2

            smoothed_path[i][0:2] = [x, y]

            if i > 0:
                
                smoothed_path[i - 1][2] = math.atan2(smoothed_path[i][1] - smoothed_path[i - 1][1], smoothed_path[i][0] - smoothed_path[i - 1][0])
            
            if i == len(smoothed_path) - 1:
                smoothed_path[i][2] = smoothed_path[i - 1][2]

        return smoothed_path
