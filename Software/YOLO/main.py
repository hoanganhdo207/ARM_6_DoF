import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO
import sim

from filterpy.kalman import KalmanFilter
import numpy as np
from filterpy.kalman import KalmanFilter
import numpy as np
# move axis (camera coordinate to global coordinate)
Rotate =  np.array([
    [1 ,0,0],
    [0,0.9848,-0.1736],
    [0,-0.1736,-0.9848]
])
Transmit =  np.array([
    [0],[0.4],[0.64]
])
def move_to_new_axis(point):
    x, y, z = point
    _point = np.array([[x],[y],[z]])
    new_point = Transmit + Rotate@_point
    new_point[0][0] = -new_point[0][0]
    #new_point[1][0] = -new_point[1][0]
    return new_point[:,0]


class Kalman2D:
    def __init__(self):
        self.kf = KalmanFilter(dim_x=4, dim_z=2)  # [x, y, vx, vy] và đo [x, y]

        # State transition matrix (dự đoán trạng thái tiếp theo)
        self.kf.F = np.array([[1, 0, 1, 0], 
                              [0, 1, 0, 1], 
                              [0, 0, 1, 0], 
                              [0, 0, 0, 1]])

        # Measurement function (chỉ đo được x, y)
        self.kf.H = np.array([[1, 0, 0, 0],
                              [0, 1, 0, 0]])

        # Covariance matrices
        self.kf.P *= 1000.  # khởi tạo sai số lớn
        self.kf.R *= 10.    # noise đo lường
        self.kf.Q = np.eye(4) * 0.1  # noise hệ thống

        # Khởi tạo state
        self.kf.x = np.array([[0], [0], [0], [0]])  # x, y, vx, vy

    def update(self, x, y):
        """Cập nhật với giá trị đo được (YOLO phát hiện)"""
        self.kf.update(np.array([[x], [y]]))

    def predict(self):
        """Dự đoán vị trí tiếp theo"""
        self.kf.predict()
        return self.kf.x[:2].flatten()  # chỉ lấy x, y

class Kalman3D:
    def __init__(self):
        self.kf = KalmanFilter(dim_x=6, dim_z=3)

        # State transition matrix (dt = 1)
        self.kf.F = np.array([
            [1, 0, 0, 1, 0, 0],
            [0, 1, 0, 0, 1, 0],
            [0, 0, 1, 0, 0, 1],
            [0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1]
        ])

        # Measurement function
        self.kf.H = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0]
        ])

        # Initial state uncertainty
        self.kf.P *= 100.

        # Measurement noise
        self.kf.R = np.eye(3) * 0.05  # điều chỉnh theo độ nhiễu của RealSense

        # Process noise
        self.kf.Q = np.eye(6) * 0.01

        # Initial state
        self.kf.x = np.zeros((6, 1))

    def update(self, x, y, z):
        """Cập nhật với đo lường 3D mới"""
        self.kf.update(np.array([[x], [y], [z]]))

    def predict(self):
        """Dự đoán vị trí 3D tiếp theo"""
        self.kf.predict()
        return self.kf.x[:3].flatten()  # x, y, z


sim.simxFinish(-1)

clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)  # Connect to CoppeliaSim
if clientID != -1:print("✅ Đã kết nối với remote API server")
else:
    print("Kết nối không thành công!")
    sys.exit("Không thể kết nối")

transform3D = np.array([
    [1 ,0,0,0],
    [0,0.9848,0.1736,-0.29],
    [0,-0.1736,0.9848,-0.7057],
    [0,0,0,1]
])

obs_dim ={
    "Bottle":[0.06,0.2],
    "Box":[0.08, 0.21],
    "cup":[0.15, 0.3]
} #[[0.1,0.5], [0.05, 0.2], [0.15, 0.3]]
# pos_track={
#     "bottle":Kalman3D(),
#     "mouse":Kalman3D(),
#     "cup":Kalman3D()
# }
# box_track={
#     "bottle":Kalman2D(),
#     "mouse":Kalman2D(),
#     "cup":Kalman2D()
# }
new_center = np.array([0.005, 0.005, 0.7125])
obs_pos ={
    "Bottle":[],
    "Box":[],
    "cup":[]
}
# === Load YOLO model ===
model = YOLO("D:/ARMROBOT_6DOF/YOLOv8/best2.pt")  # Hoặc model segment nếu muốn
threshold = 0.5

# === Khởi tạo pipeline RealSense ===
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
profile = pipeline.start(config)

# Align depth với color
align = rs.align(rs.stream.color)
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
counter = 20
try:
    while counter>0:
        counter-=1
        # === Lấy frame và căn chỉnh ===
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()

        if not color_frame or not depth_frame:
            continue

        frame = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        # Lấy thông tin nội tại của camera để chiếu ngược pixel sang tọa độ không gian
        color_intrin = color_frame.profile.as_video_stream_profile().intrinsics

        # === Chạy YOLO detection ===
        results = model(frame)[0]
        data_to_send = []
        object_names = []
        if len(results.boxes.data.tolist())>0:
            # === Duyệt qua từng box ===
            for result in results.boxes.data.tolist():
                x1, y1, x2, y2, score, class_id = result
                if score > threshold:
                    object_name = results.names[int(class_id)]
                    x1, y1, x2, y2 = map(int, [x1, y1, x2, y2])
                    cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                    # try:
                    #     box_track[object_name].update(cx,cy)
                    #     cx,xy = box_track[object_name].predict()
                    #     cx, cy = int(cx), int(cy)
                    # except:
                    #     pass
                    cv2.circle(frame, (cx, cy), radius=3, color=(0, 0, 255), thickness=-1)
                    # Lấy giá trị độ sâu
                    if 0 <= cx < depth_image.shape[1] and 0 <= cy < depth_image.shape[0]: