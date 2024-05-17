import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
from db_connect import DBConnect

class DTW_Johnen:
    def __init__(self, data):
        self.data = data
        self.data_id = self.data['trajectory_header_id']
        self.db = DBConnect()
        x_ist = np.array(self.data['x_ist']).reshape(-1, 1)
        y_ist = np.array(self.data['y_ist']).reshape(-1, 1)
        z_ist = np.array(self.data['z_ist']).reshape(-1, 1)
        x_soll = np.array(self.data['x_soll']).reshape(-1, 1)
        y_soll = np.array(self.data['y_soll']).reshape(-1, 1)
        z_soll = np.array(self.data['z_soll']).reshape(-1, 1)
        q1_ist = np.array(self.data['q1_ist']).reshape(-1, 1)
        q2_ist = np.array(self.data['q2_ist']).reshape(-1, 1)
        q3_ist = np.array(self.data['q3_ist']).reshape(-1, 1)
        q4_ist = np.array(self.data['q4_ist']).reshape(-1, 1)
        q1_soll = np.array(self.data['q1_soll']).reshape(-1, 1)
        q2_soll = np.array(self.data['q2_soll']).reshape(-1, 1)
        q3_soll = np.array(self.data['q3_soll']).reshape(-1, 1)
        q4_soll = np.array(self.data['q4_soll']).reshape(-1, 1)
        
        self.robotdata_ist = np.hstack((x_ist, y_ist, z_ist, q1_ist, q2_ist, q3_ist, q4_ist))
        self.robotdata_soll = np.hstack((x_soll, y_soll, z_soll, q1_soll, q2_soll, q3_soll, q4_soll))
        pass
    
    def euclDist(self, i, j, pathX, pathY):
        return np.linalg.norm(pathX[i, :] - pathY[j, :])

    def IsInterpolation(self, param):
        return param > 0 and param < 1

    def Interpolate(self, start, end, parameter):
        return start + (end - start) * parameter

    def minDistParam(self, x1, x2, y):
        dx = x2 - x1  # Bahnsegment 
        dy = y - x1   # Abstand Punkt-Bahnsegment
        dxy = (np.dot(dy, dx) / np.linalg.norm(dx)**2) * dx  # Projektion dy auf dx
        
        if np.dot(dx, dy) > 0:  # gleiche Richtung: Winkel < 90°
            param = np.linalg.norm(dxy) / np.linalg.norm(dx)
            if param > 1:
                mindist = np.inf
            else:
                mindist = np.linalg.norm(y - (x1 + dxy))
        elif np.dot(dx, dy) == 0:  # senkrecht: Winkel = 90°
            param = 0
            mindist = np.linalg.norm(dy)
        else:  # entgegengesetze Richtung: Winkel > 90°
            param = -np.linalg.norm(dxy) / np.linalg.norm(dx)
            mindist = np.inf
        
        return mindist, param

    # Load data

    def dtw_selective_interpolation(self):

        X = self.robotdata_soll
        Y = self.robotdata_ist

        M = X.shape[0]
        N = Y.shape[0]

        AccumulatedDistance = np.zeros((M, N))
        AccumulatedDistanceX = np.zeros((M, N))
        AccumulatedDistanceY = np.zeros((M, N))
        ParamX = np.zeros((M, N))
        ParamY = np.zeros((M, N))
        minParams = np.empty((M, N), dtype=object)
        minParamsX = np.empty((M, N), dtype=object)
        minParamsY = np.empty((M, N), dtype=object)

        # Start values for X
        for i in range(1, M):
            mindist, param = self.minDistParam(X[i-1, :], X[i, :], Y[0, :])
            AccumulatedDistanceX[i, 0] = AccumulatedDistanceX[i-1, 0] + mindist
            AccumulatedDistanceY[i, 0] = np.inf
            ParamX[i, 0] = param
            ParamY[i, 0] = np.nan
            minParams[i, 0] = [0, 1]
            minParamsX[i, 0] = [0, 1]
            minParamsY[i, 0] = [np.nan, np.nan]
            AccumulatedDistance[i, 0] = AccumulatedDistance[i-1, 0] + self.euclDist(i, 0, X, Y)
            if self.IsInterpolation(ParamX[i, 0]) and AccumulatedDistanceX[i, 0] < AccumulatedDistance[i-1, 0]:
                AccumulatedDistance[i, 0] = AccumulatedDistanceX[i, 0] + self.euclDist(i, 0, X, Y)
                minParams[i, 0] = [ParamX[i, 0], 1]

        # Start values for Y
        for j in range(1, N):
            mindist, param = self.minDistParam(Y[j-1, :], Y[j, :], X[0, :])
            AccumulatedDistanceX[0, j] = np.inf
            AccumulatedDistanceY[0, j] = AccumulatedDistance[0, j-1] + mindist
            ParamX[0, j] = np.nan
            ParamY[0, j] = param
            minParams[0, j] = [1, 0]
            minParamsX[0, j] = [np.nan, np.nan]
            minParamsY[0, j] = [1, 0]
            AccumulatedDistance[0, j] = AccumulatedDistance[0, j-1] + self.euclDist(0, j, X, Y)
            if self.IsInterpolation(ParamX[0, j]) and AccumulatedDistanceY[0, j] < AccumulatedDistance[0, j-1]:
                AccumulatedDistance[0, j] = AccumulatedDistanceY[0, j] + self.euclDist(0, j, X, Y)
                minParams[0, j] = [1, ParamY[0, j]]

        # Calculating accumulated distance
        for i in range(1, M):
            for j in range(1, N):
                mindist, param = self.minDistParam(X[i-1, :], X[i, :], Y[j, :])
                AccumulatedDistanceX[i, j] = mindist
                ParamX[i, j] = param
                mindist, param = self.minDistParam(Y[j-1, :], Y[j, :], X[i, :])
                AccumulatedDistanceY[i, j] = mindist
                ParamY[i, j] = param

                minCost = np.inf
                if self.IsInterpolation(ParamX[i, j]):
                    if AccumulatedDistanceX[i, j-1] < minCost and self.IsInterpolation(ParamX[i, j-1]) and ParamX[i, j-1] <= ParamX[i, j]:
                        minCost = AccumulatedDistanceX[i, j-1]
                        minParamsX[i, j] = [ParamX[i, j-1], 0]
                    elif AccumulatedDistanceY[i-1, j] < minCost and self.IsInterpolation(ParamY[i-1, j]):
                        minCost = AccumulatedDistanceY[i-1, j]
                        minParamsX[i, j] = [0, ParamY[i-1, j]]
                    elif AccumulatedDistance[i-1, j] < minCost:
                        minCost = AccumulatedDistance[i-1, j]
                        minParamsX[i, j] = [0, 1]
                    elif AccumulatedDistance[i-1, j-1] < minCost and not self.IsInterpolation(ParamX[i, j-1]) and not self.IsInterpolation(ParamY[i-1, j]) and self.euclDist(i-1, j-1, X, Y) <= self.euclDist(i-1, j, X, Y):
                        minCost = AccumulatedDistance[i-1, j-1]
                        minParamsX[i, j] = [0, 0]
                
                AccumulatedDistanceX[i, j] = AccumulatedDistanceX[i, j] + minCost

                minCost = np.inf
                if self.IsInterpolation(ParamY[i, j]):
                    if AccumulatedDistanceX[i, j-1] < minCost and self.IsInterpolation(ParamX[i, j-1]):
                        minCost = AccumulatedDistanceX[i, j-1]
                        minParamsY[i, j] = [ParamX[i, j-1], 0]
                    elif AccumulatedDistanceY[i-1, j] < minCost and self.IsInterpolation(ParamY[i-1, j]) and ParamY[i-1, j] <= ParamY[i, j]:
                        minCost = AccumulatedDistanceY[i-1, j]
                        minParamsY[i, j] = [0, ParamY[i-1, j]]
                    elif AccumulatedDistance[i, j-1] < minCost:
                        minCost = AccumulatedDistance[i, j-1]
                        minParamsY[i, j] = [1, 0]
                    elif AccumulatedDistance[i-1, j-1] < minCost and not self.IsInterpolation(ParamX[i, j-1]) and not self.IsInterpolation(ParamY[i-1, j]) and self.euclDist(i-1, j-1, X, Y) <= self.euclDist(i, j-1, X, Y):
                        minCost = AccumulatedDistance[i-1, j-1]
                        minParamsY[i, j] = [0, 0]
                AccumulatedDistanceY[i, j] = AccumulatedDistanceY[i, j] + minCost

                minCost = np.inf
                if self.IsInterpolation(ParamX[i, j]) and AccumulatedDistanceX[i, j] < minCost:
                    minCost = AccumulatedDistanceX[i, j]
                    minParams[i, j] = [ParamX[i, j], 1]
                if self.IsInterpolation(ParamY[i, j]) and AccumulatedDistanceY[i, j] < minCost:
                    minCost = AccumulatedDistanceY[i, j]
                    minParams[i, j] = [1, ParamY[i, j]]
                if AccumulatedDistanceX[i, j-1] < minCost and self.IsInterpolation(ParamX[i, j-1]) and self.euclDist(i, j, X, Y) <= self.euclDist(i, j-1, X, Y) and not self.IsInterpolation(ParamY[i, j]) and (ParamX[i, j] < ParamX[i, j-1] or not self.IsInterpolation(ParamX[i, j])):
                    minCost = AccumulatedDistanceX[i, j-1]
                    minParams[i, j] = [ParamX[i, j-1], 0]
                if AccumulatedDistanceY[i-1, j] < minCost and self.IsInterpolation(ParamY[i-1, j]) and self.euclDist(i, j, X, Y) <= self.euclDist(i-1, j, X, Y) and not self.IsInterpolation(ParamX[i, j]) and (ParamY[i, j] < ParamY[i-1, j] or not self.IsInterpolation(ParamY[i, j])):
                    minCost = AccumulatedDistanceY[i-1, j]
                    minParams[i, j] = [0, ParamY[i-1, j]]
                if AccumulatedDistance[i, j-1] < minCost and not self.IsInterpolation(ParamY[i, j]):
                    minCost = AccumulatedDistance[i, j-1]
                    minParams[i, j] = [1, 0]
                if AccumulatedDistance[i-1, j] < minCost and not self.IsInterpolation(ParamX[i, j]):
                    minCost = AccumulatedDistance[i-1, j]
                    minParams[i, j] = [0, 1]
                if AccumulatedDistance[i-1, j-1] < minCost and self.euclDist(i, j, X, Y) <= self.euclDist(i-1, j, X, Y) and self.euclDist(i, j, X, Y) <= self.euclDist(i, j-1, X, Y) and not self.IsInterpolation(ParamX[i, j]) and not self.IsInterpolation(ParamY[i, j]) and not self.IsInterpolation(ParamX[i, j-1]) and not self.IsInterpolation(ParamY[i-1, j]) and self.euclDist(i-1, j-1, X, Y) <= self.euclDist(i-1, j, X, Y) and self.euclDist(i-1, j-1, X, Y) <= self.euclDist(i, j-1, X, Y):
                    minCost = AccumulatedDistance[i-1, j-1]
                    minParams[i, j] = [0, 0]
                assert minCost != np.inf
                AccumulatedDistance[i, j] = minCost + self.euclDist(i, j, X, Y)

        # Initializing indexes for backtracking
        i = M - 1
        j = N - 1
        MappingIndexes = [[i, j]]
        result = np.array([X[i, :], Y[j, :]]).flatten()

        lastParam = [0, 0]

        while i > 0 or j > 0:
            if lastParam == [0, 0] or lastParam == [0, 1] or lastParam == [1, 0]:
                lastParam = minParams[i, j]
            elif lastParam[0] > 0 and lastParam[1] == 0 or (lastParam[0] > 0 and lastParam[1] == 1):
                lastParam = minParamsX[i, j]
            elif (lastParam[0] == 0 and lastParam[1] > 0) or (lastParam[0] == 1 and lastParam[1] > 0):
                lastParam = minParamsY[i, j]
            else:
                raise ValueError('Invalid state.')
            
            if i == 0:
                result = np.vstack((np.hstack((X[0, :], self.Interpolate(Y[j - 1, :], Y[j, :], lastParam[1]))), result))
            elif j == 0:
                result = np.vstack((np.hstack((self.Interpolate(X[i - 1, :], X[i, :], lastParam[0]), Y[0, :])), result))
            else:
                result = np.vstack((np.hstack((self.Interpolate(X[i - 1, :], X[i, :], lastParam[0]), self.Interpolate(Y[j - 1, :], Y[j, :], lastParam[1]))), result))

            MappingIndexes.append([i - 1 + lastParam[0], j - 1 + lastParam[1]])
            assert i - 1 + lastParam[0] >= 0
            assert j - 1 + lastParam[1] >= 0

            if lastParam[0] == 0:
                i -= 1
            if lastParam[1] == 0:
                j -= 1

        # Indexes of interpolated path points
        ix, iy = zip(*MappingIndexes)
        # Interpolated paths
        dtwX = result[:, :7]
        dtwY = result[:, 7:]
        path = [ix, iy]

        # Distances between interpolated paths
        distances = np.zeros(len(dtwX))
        for i in range(len(dtwX)):
            dist = self.euclDist(i, i, dtwX, dtwY)
            distances[i] = dist

        # Maximum and average distance
        maxDistance = np.max(distances)
        averageDistance = np.mean(distances)

        dtw_metrics_data = {
            "trajectory_header_id": self.data_id,
            "dtw_max_distance": maxDistance,
            "dtw_average_distance": averageDistance,
            "dtw_distances": distances.tolist(),
            "dtw_X": dtwX.tolist(),
            "dtw_Y": dtwY.tolist(),
            "dtw_accdist": AccumulatedDistance.tolist(),
            "dtw_path": path,
            "metric_type": "dtw_johnen"
        }

        saved = self.try_save_results(dtw_metrics_data)

        return {
            **dtw_metrics_data,
            "saved_metrics": saved,
        }

    def try_save_results(self, results):
            self.db.connect()
            metric_exists = self.db.collection_metrics.count_documents(
                {
                    "trajectory_header_id": self.data_id,
                    "metric_type": "dtw_johnen"
                }
            )

            if metric_exists == 0:
                self.db.collection_metrics.insert_one(results)
                return True

            return False

