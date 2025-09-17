# New BNN model implementation
from turtle import distance
from tensorflow_probability.python.layers import DenseFlipout
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import InputLayer, Dropout, Dense
import tensorflow as tf
import numpy as np
import pandas as pd
import math


class DBNNMonitor:

    def __init__(self):
        self.model = None

    def calculate_euclidean_distance(self, pos1, pos2):
        """
        두 위치 간의 유클리드 거리 계산
        """
        return math.sqrt(
            (pos1[0] - pos2[0]) ** 2
            + (pos1[1] - pos2[1]) ** 2
            + (pos1[2] - pos2[2]) ** 2
        )

    def calculate_min_distance(self, position, enemy_positions):
        # Calculate Euclidean distance to each enemy position
        distances = [
            math.sqrt(
                (position[0] - enemy[0]) ** 2
                + (position[1] - enemy[1]) ** 2
                + (position[2] - enemy[2]) ** 2
            )
            for enemy in enemy_positions
        ]

        # Get the minimum distance, or 30 if all distances are greater than 30
        min_distance = min(distances, default=30)
        return min(min_distance, 30)

    def create_bnn_model(self, input_shape=(10,)):
        self.model = Sequential()
        self.model.add(InputLayer(input_shape=input_shape))

        # Hidden layers with flipout (Bayesian layers)
        self.model.add(DenseFlipout(64, activation="relu"))
        self.model.add(Dropout(0.1))
        self.model.add(DenseFlipout(64, activation="relu"))
        self.model.add(Dropout(0.1))

        self.model.add(Dense(7))  # 7 output for the different risk assessments

        self.model.compile(optimizer="adam", loss="mean_squared_error", metrics=["mae"])
        return self.model

    def load_bnn_model_weights(self, weights_path):
        self.model.load_weights(weights_path)

    def predict_with_bnn_model(self, data):
        for uav in data:
            if uav["AircraftID"] == 2:
                uav_data = uav
                position = [
                    uav_data["Coordinate"]["Latitude"],
                    uav_data["Coordinate"]["Longitude"],
                    uav_data["Coordinate"]["Altitude"],
                ]
                enemy_positions = [
                    [
                        target["Coordinate"]["Latitude"],
                        target["Coordinate"]["Longitude"],
                        target["Coordinate"]["Altitude"],
                    ]
                    for target in uav_data["SituationAwarenessInfo"]["TargetList"]
                ]

                distance_to_nearest_enemy = self.calculate_min_distance(
                    position, enemy_positions
                )

                # 다른 UAV들과의 거리 계산
                min_distance_to_uav = float("inf")
                for other_uav in data:
                    if other_uav["AircraftID"] != 2:  # UAV 2는 제외
                        other_position = [
                            other_uav["Coordinate"]["Latitude"],
                            other_uav["Coordinate"]["Longitude"],
                            other_uav["Coordinate"]["Altitude"],
                        ]
                        distance_to_other_uav = self.calculate_euclidean_distance(
                            position, other_position
                        )
                        min_distance_to_uav = min(
                            min_distance_to_uav, distance_to_other_uav
                        )

                np.random.seed(42)  # For reproducibility

                feature1 = 0  # Time deviation (0-1)
                feature2 = (
                    uav_data["Fuel"]["Amount"] / 1000
                )  # Remaining battery/resource percentage (0-1)
                feature3 = 0  # Environmental condition severity (0-1)
                feature4 = (
                    min_distance_to_uav  # Distance to nearest obstacle (1-20 meters)
                )

                feature5 = len(
                    uav_data["SituationAwarenessInfo"]["TargetList"]
                )  # Number of detected enemy threats (0-10)
                feature6 = (
                    distance_to_nearest_enemy  # Distance to nearest enemy (1-30 meters)
                )

                feature7 = np.random.uniform(
                    1, 30, 1
                )  # Distance to nearest friendly base (1-30 meters)
                feature8 = np.random.rand(1)  # Mission complexity (0-1)
                feature9 = uav_data["Velocity"]["Speed"]  # UAV velocity (10-50 m/s)
                feature10 = uav_data["Coordinate"][
                    "Altitude"
                ]  # UAV altitude (50-500 meters)

                def safe_float(value):
                    """
                    값을 안전하게 float로 변환하는 함수.
                    numpy.ndarray의 경우 첫 번째 값을 반환.
                    변환 불가능한 경우 기본값 0.0 반환.
                    """
                    if isinstance(value, np.ndarray):
                        return float(value[0])  # 배열의 첫 번째 값 사용
                    try:
                        return float(value)
                    except (ValueError, TypeError):
                        return 0.0

                # Feature 생성
                features = np.array(
                    [
                        safe_float(feature1),  # <class 'int'>
                        safe_float(feature2),  # <class 'float'>
                        safe_float(feature3),  # <class 'int'>
                        safe_float(feature4),  # <class 'float'>
                        safe_float(feature5),  # <class 'int'>
                        safe_float(feature6),  # <class 'int'>
                        safe_float(feature7),  # <class 'numpy.ndarray'>
                        safe_float(feature8),  # <class 'numpy.ndarray'>
                        safe_float(feature9),  # <class 'float'>
                        safe_float(feature10),  # <class 'int'>
                    ]
                ).reshape(1, -1)

                result = self.model.predict(features)
                result_df = pd.DataFrame(
                    result / 10,
                    columns=[
                        "Schedule_Adherence_Risk",
                        "Sustainability_Risk",
                        "Operational_Risk",
                        "Collision_Risk",
                        "Enemy_Risk",
                        "Probability_to_Kill",
                        "Mission_Success_Rate",
                    ],
                )
                # DataFrame을 dict로 변환 (각 행을 dict 형태로 저장)
                result_dict = result_df.to_dict(orient="records")
                return result_dict

            else:
                continue


"""
Feature1: Time Deviation

A random value between 0 and 1 representing how much the UAV deviates from the mission schedule (time-wise). A higher value indicates greater deviation.
Feature2: Remaining Battery/Resource Percentage

A random value between 0 and 1 representing the percentage of the UAV's remaining battery or resources. A higher value means more resources remain.
Feature3: Environmental Condition Severity

A random value between 0 and 1 representing the severity of environmental conditions (e.g., weather, terrain). A higher value means more severe conditions.
Feature4: Distance to Nearest Obstacle

A random value between 1 and 20 meters representing the UAV's distance to the nearest obstacle. A higher value means a safer distance.
Feature5: Number of Detected Enemy Threats

An integer between 0 and 10 representing the number of detected enemy threats around the UAV. A higher value means more enemies detected.
Feature6: Distance to Nearest Enemy

A random value between 1 and 30 meters representing the distance to the nearest enemy threat. A higher value means a safer distance.
Feature7: Distance to Nearest Friendly Base

A random value between 1 and 30 meters representing the distance to the nearest friendly base. A higher value means the UAV is further away from safety.
Feature8: Mission Complexity

A random value between 0 and 1 representing the complexity of the current mission. A higher value means a more complex mission.
Feature9: UAV Velocity

A random value between 10 and 50 m/s representing the velocity at which the UAV is moving. A higher value means a faster speed.
Feature10: UAV Altitude

A random value between 50 and 500 meters representing the UAV's altitude. A higher value means the UAV is flying at a higher altitude.

"""
