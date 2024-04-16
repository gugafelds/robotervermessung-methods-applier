import numpy as np


class EuclidianDistance:
    def __init__(self, data):
        self.data = data
        self.n_points_ist = len(self.data['x_ist'])
        self.n_points_soll = len(self.data['x_soll'])

    def get_point_soll(self, i):
        return np.array([self.data['x_soll'][i], self.data['y_soll'][i], self.data['z_soll'][i]])

    def get_point_ist(self, j):
        return np.array([self.data['x_ist'][j], self.data['y_ist'][j], self.data['z_ist'][j]])

    def interpolate(self, p_soll_i, index_before, index_after, n=100):
        if index_before < 0:
            index_before = 0
        p_ist_interpolation_final, p_ist_interpolation_initial = self.get_point_ist(index_after), self.get_point_ist(
            index_before)
        curve_direction = p_ist_interpolation_final - p_ist_interpolation_initial
        t = np.linspace(0, 1, n)
        distances_interpolated = []
        parameter_t = []
        for t_i in t:
            p_ist_interpolation_i = p_ist_interpolation_initial + t_i * curve_direction
            distance = np.linalg.norm(p_ist_interpolation_i - p_soll_i)
            distances_interpolated.append(distance)
            parameter_t.append(t_i)
        distances_interpolated = np.array(distances_interpolated)
        t_min = parameter_t[distances_interpolated.argmin()]
        d_min = distances_interpolated.min()
        p_min = p_ist_interpolation_initial + t_min * curve_direction

        return d_min, p_min

    def compute_distance_method(self):
        euclidian_distances = []
        points_interpolation = []
        for i in range(self.n_points_soll - 1):
            p_soll_i = self.get_point_soll(i)
            distances = []
            for j in range(self.n_points_ist - 1):
                p_ist = self.get_point_ist(j)
                distance = np.linalg.norm(p_ist - p_soll_i)
                distances.append(distance)
            distances = np.array(distances)
            index = distances.argmin()
            d_min_1, p_min_1 = self.interpolate(p_soll_i, index - 1, index)
            d_min_2, p_min_2 = self.interpolate(p_soll_i, index, index + 1)
            d_min, p_min = d_min_2, p_min_2
            if d_min_1 < d_min_2:
                d_min, p_min = d_min_1, p_min_1
            points_interpolation.append(p_min)
            euclidian_distances.append(d_min)

        euclidian_distances = np.array(euclidian_distances)
        return {
            "max_distances": euclidian_distances.max(),
            "average_distances": sum(euclidian_distances) / len(euclidian_distances)
        }