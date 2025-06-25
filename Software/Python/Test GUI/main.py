import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d import Axes3D
import math
import cv2
import threading
import time
from PIL import Image, ImageTk
import os

# Cài đặt để sử dụng YOLO
try:
    # Sử dụng ultralytics YOLO
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False
    print("Cảnh báo: ultralytics không được cài đặt. Chạy: pip install ultralytics")

class RobotArmGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Robot Tay Máy 6 DOF với YOLO Vision")
        self.root.geometry("1400x900")
        
        # Khởi tạo các góc khớp (độ)
        self.joint_angles = [0, 0, 0, 0, 0, 0]
        
        # Thông số DH (Denavit-Hartenberg) cho robot 6 DOF
        self.dh_params = [
            [0, 90, 0.2, 0],      # Joint 1
            [0.3, 0, 0, 0],       # Joint 2
            [0.25, 0, 0, 0],      # Joint 3
            [0, 90, 0.15, 0],     # Joint 4
            [0, -90, 0, 0],       # Joint 5
            [0, 0, 0.1, 0]        # Joint 6
        ]
        
        # Khởi tạo camera và YOLO
        self.cap = None
        self.yolo_model = None
        self.camera_running = False
        self.auto_mode = False
        self.current_frame = None
        self.detection_results = []
        
        # Vị trí đã định nghĩa sẵn cho các loại vật thể
        self.object_positions = {
            'person': [45, -60, 80, 0, 45, 0],
            'bottle': [90, -45, 60, 0, 90, 0],
            'cup': [-45, -30, 70, 0, 60, 0],
            'apple': [0, -75, 85, 0, 30, 0],
            'banana': [135, -50, 75, 0, 75, 0],
            'cell phone': [-90, -40, 65, 0, 45, 0],
            'book': [180, -35, 55, 0, 80, 0],
            'default': [0, -45, 90, 0, 45, 0]
        }
        
        self.setup_gui()
        self.init_yolo()
        self.update_visualization()
    
    def setup_gui(self):
        # Notebook để tổ chức các tab
        notebook = ttk.Notebook(self.root)
        notebook.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Tab 1: Điều khiển thủ công
        manual_frame = ttk.Frame(notebook)
        notebook.add(manual_frame, text="Điều Khiển Thủ Công")
        self.setup_manual_control(manual_frame)
        
        # Tab 2: Chế độ tự động với YOLO
        auto_frame = ttk.Frame(notebook)
        notebook.add(auto_frame, text="Chế độ Tự Động (YOLO)")
        self.setup_auto_control(auto_frame)
    
    def setup_manual_control(self, parent):
        """Thiết lập giao diện điều khiển thủ công"""
        main_frame = ttk.Frame(parent)
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        # Frame điều khiển (bên trái)
        control_frame = ttk.LabelFrame(main_frame, text="Điều Khiển Khớp", padding=10)
        control_frame.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 10))
        
        # Tạo slider cho từng khớp
        self.sliders = []
        self.angle_labels = []
        
        for i in range(6):
            joint_frame = ttk.Frame(control_frame)
            joint_frame.pack(fill=tk.X, pady=5)
            
            ttk.Label(joint_frame, text=f"Khớp {i+1}:").pack(anchor=tk.W)
            
            slider = tk.Scale(joint_frame, from_=-180, to=180, orient=tk.HORIZONTAL,
                            command=lambda val, idx=i: self.update_joint(idx, val))
            slider.pack(fill=tk.X)
            slider.set(self.joint_angles[i])
            self.sliders.append(slider)
            
            angle_label = ttk.Label(joint_frame, text=f"{self.joint_angles[i]}°")
            angle_label.pack(anchor=tk.W)
            self.angle_labels.append(angle_label)
        
        # Nút điều khiển
        button_frame = ttk.Frame(control_frame)
        button_frame.pack(fill=tk.X, pady=10)
        
        ttk.Button(button_frame, text="Reset Về 0", 
                  command=self.reset_joints).pack(fill=tk.X, pady=2)
        ttk.Button(button_frame, text="Vị Trí Home", 
                  command=self.home_position).pack(fill=tk.X, pady=2)
        ttk.Button(button_frame, text="Lưu Vị Trí", 
                  command=self.save_position).pack(fill=tk.X, pady=2)
        
        # Thông tin vị trí
        info_frame = ttk.LabelFrame(control_frame, text="Thông Tin Vị Trí", padding=5)
        info_frame.pack(fill=tk.X, pady=10)
        
        self.position_label = ttk.Label(info_frame, text="X: 0.00\nY: 0.00\nZ: 0.00")
        self.position_label.pack()
        
        # Frame hiển thị 3D
        viz_frame = ttk.LabelFrame(main_frame, text="Mô Hình 3D Robot", padding=5)
        viz_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)
        
        self.fig = Figure(figsize=(8, 6), dpi=100)
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.canvas = FigureCanvasTkAgg(self.fig, viz_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
    
    def setup_auto_control(self, parent):
        """Thiết lập giao diện chế độ tự động"""
        main_frame = ttk.Frame(parent)
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        # Frame điều khiển camera và YOLO (bên trái)
        control_frame = ttk.LabelFrame(main_frame, text="Điều Khiển Camera & YOLO", padding=10)
        control_frame.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 10))
        
        # Điều khiển camera
        camera_frame = ttk.LabelFrame(control_frame, text="Camera", padding=5)
        camera_frame.pack(fill=tk.X, pady=5)
        
        self.camera_btn = ttk.Button(camera_frame, text="Bật Camera", 
                                   command=self.toggle_camera)
        self.camera_btn.pack(fill=tk.X, pady=2)
        
        ttk.Button(camera_frame, text="Chụp Ảnh", 
                  command=self.capture_image).pack(fill=tk.X, pady=2)
        
        ttk.Button(camera_frame, text="Tải Ảnh", 
                  command=self.load_image).pack(fill=tk.X, pady=2)
        
        # Điều khiển YOLO
        yolo_frame = ttk.LabelFrame(control_frame, text="YOLO Detection", padding=5)
        yolo_frame.pack(fill=tk.X, pady=5)
        
        self.yolo_status = ttk.Label(yolo_frame, 
                                   text="YOLO: Chưa sẵn sàng" if not YOLO_AVAILABLE else "YOLO: Sẵn sàng")
        self.yolo_status.pack(pady=2)
        
        ttk.Button(yolo_frame, text="Phát Hiện Vật Thể", 
                  command=self.detect_objects).pack(fill=tk.X, pady=2)
        
        # Chế độ tự động
        auto_frame = ttk.LabelFrame(control_frame, text="Chế độ Tự Động", padding=5)
        auto_frame.pack(fill=tk.X, pady=5)
        
        self.auto_btn = ttk.Button(auto_frame, text="Bật Chế Độ Tự Động", 
                                 command=self.toggle_auto_mode)
        self.auto_btn.pack(fill=tk.X, pady=2)
        
        self.auto_status = ttk.Label(auto_frame, text="Trạng thái: Thủ công")
        self.auto_status.pack(pady=2)
        
        # Danh sách vật thể phát hiện
        detection_frame = ttk.LabelFrame(control_frame, text="Vật Thể Phát Hiện", padding=5)
        detection_frame.pack(fill=tk.X, pady=5)
        
        self.detection_listbox = tk.Listbox(detection_frame, height=6)
        self.detection_listbox.pack(fill=tk.X)
        self.detection_listbox.bind('<<ListboxSelect>>', self.on_object_select)
        
        ttk.Button(detection_frame, text="Di Chuyển Đến Vật Thể", 
                  command=self.move_to_selected_object).pack(fill=tk.X, pady=2)
        
        # Cài đặt vị trí cho vật thể
        position_frame = ttk.LabelFrame(control_frame, text="Cài Đặt Vị Trí", padding=5)
        position_frame.pack(fill=tk.X, pady=5)
        
        ttk.Button(position_frame, text="Lưu Vị Trí Cho Vật Thể", 
                  command=self.save_object_position).pack(fill=tk.X, pady=2)
        
        # Frame hiển thị camera và kết quả
        display_frame = ttk.Frame(main_frame)
        display_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)
        
        # Hiển thị camera
        camera_display_frame = ttk.LabelFrame(display_frame, text="Camera/Ảnh", padding=5)
        camera_display_frame.pack(fill=tk.BOTH, expand=True, pady=(0, 5))
        
        self.camera_label = ttk.Label(camera_display_frame, text="Chưa có hình ảnh")
        self.camera_label.pack(expand=True)
        
        # Hiển thị kết quả detection
        result_frame = ttk.LabelFrame(display_frame, text="Kết Quả Phát Hiện", padding=5)
        result_frame.pack(fill=tk.X)
        
        self.result_text = tk.Text(result_frame, height=6, wrap=tk.WORD)
        self.result_text.pack(fill=tk.X)
        
        scrollbar = ttk.Scrollbar(result_frame, orient=tk.VERTICAL, command=self.result_text.yview)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.result_text.config(yscrollcommand=scrollbar.set)
    
    def init_yolo(self):
        """Khởi tạo YOLO model"""
        if YOLO_AVAILABLE:
            try:
                self.yolo_model = YOLO('yolov8n.pt')  # Tải model nhẹ nhất
                self.yolo_status.config(text="YOLO: Sẵn sàng")
                self.log_result("YOLO model đã được tải thành công!")
            except Exception as e:
                self.yolo_status.config(text=f"YOLO: Lỗi - {str(e)}")
                self.log_result(f"Lỗi khi tải YOLO: {str(e)}")
        else:
            self.log_result("YOLO không khả dụng. Cài đặt ultralytics để sử dụng.")
    
    def toggle_camera(self):
        """Bật/tắt camera"""
        if not self.camera_running:
            self.cap = cv2.VideoCapture(0)
            if self.cap.isOpened():
                self.camera_running = True
                self.camera_btn.config(text="Tắt Camera")
                self.update_camera_feed()
                self.log_result("Camera đã được bật")
            else:
                messagebox.showerror("Lỗi", "Không thể mở camera")
        else:
            self.camera_running = False
            if self.cap:
                self.cap.release()
            self.camera_btn.config(text="Bật Camera")
            self.log_result("Camera đã được tắt")
    
    def update_camera_feed(self):
        """Cập nhật feed camera"""
        if self.camera_running and self.cap:
            ret, frame = self.cap.read()
            if ret:
                self.current_frame = frame.copy()
                # Chuyển đổi BGR sang RGB
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                # Resize để phù hợp với giao diện
                frame_resized = cv2.resize(frame_rgb, (400, 300))
                
                # Chuyển đổi thành PhotoImage
                image = Image.fromarray(frame_resized)
                photo = ImageTk.PhotoImage(image)
                
                self.camera_label.config(image=photo, text="")
                self.camera_label.image = photo
                
                # Nếu đang ở chế độ tự động, tự động phát hiện
                if self.auto_mode:
                    self.detect_objects()
            
            # Lặp lại sau 30ms
            self.root.after(30, self.update_camera_feed)
    
    def capture_image(self):
        """Chụp ảnh từ camera"""
        if self.current_frame is not None:
            filename = f"captured_{int(time.time())}.jpg"
            cv2.imwrite(filename, self.current_frame)
            self.log_result(f"Đã chụp ảnh: {filename}")
        else:
            messagebox.showwarning("Cảnh báo", "Không có hình ảnh để chụp")
    
    def load_image(self):
        """Tải ảnh từ file"""
        filename = filedialog.askopenfilename(
            title="Chọn ảnh",
            filetypes=[("Image files", "*.jpg *.jpeg *.png *.bmp *.tiff")]
        )
        if filename:
            image = cv2.imread(filename)
            if image is not None:
                self.current_frame = image.copy()
                # Hiển thị ảnh
                image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                image_resized = cv2.resize(image_rgb, (400, 300))
                photo = ImageTk.PhotoImage(Image.fromarray(image_resized))
                self.camera_label.config(image=photo, text="")
                self.camera_label.image = photo
                self.log_result(f"Đã tải ảnh: {os.path.basename(filename)}")
    
    def detect_objects(self):
        """Phát hiện vật thể bằng YOLO"""
        if not YOLO_AVAILABLE or self.yolo_model is None:
            self.log_result("YOLO không khả dụng")
            return
        
        if self.current_frame is None:
            self.log_result("Không có hình ảnh để phân tích")
            return
        
        try:
            # Chạy YOLO detection
            results = self.yolo_model(self.current_frame)
            
            # Xử lý kết quả
            self.detection_results = []
            self.detection_listbox.delete(0, tk.END)
            
            for result in results:
                boxes = result.boxes
                if boxes is not None:
                    for box in boxes:
                        # Lấy thông tin detection
                        cls = int(box.cls[0])
                        conf = float(box.conf[0])
                        class_name = self.yolo_model.names[cls]
                        
                        if conf > 0.5:  # Chỉ lấy detection có confidence > 0.5
                            self.detection_results.append({
                                'class': class_name,
                                'confidence': conf,
                                'box': box.xyxy[0].tolist()
                            })
                            
                            # Thêm vào listbox
                            self.detection_listbox.insert(tk.END, 
                                f"{class_name} ({conf:.2f})")
            
            # Hiển thị ảnh với bounding boxes
            self.display_detection_results()
            self.log_result(f"Phát hiện {len(self.detection_results)} vật thể")
            
        except Exception as e:
            self.log_result(f"Lỗi khi phát hiện vật thể: {str(e)}")
    
    def display_detection_results(self):
        """Hiển thị kết quả detection với bounding boxes"""
        if self.current_frame is None or not self.detection_results:
            return
        
        # Vẽ bounding boxes
        display_frame = self.current_frame.copy()
        
        for detection in self.detection_results:
            x1, y1, x2, y2 = [int(coord) for coord in detection['box']]
            class_name = detection['class']
            confidence = detection['confidence']
            
            # Vẽ rectangle
            cv2.rectangle(display_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            # Vẽ label
            label = f"{class_name} {confidence:.2f}"
            cv2.putText(display_frame, label, (x1, y1-10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        # Hiển thị
        frame_rgb = cv2.cvtColor(display_frame, cv2.COLOR_BGR2RGB)
        frame_resized = cv2.resize(frame_rgb, (400, 300))
        photo = ImageTk.PhotoImage(Image.fromarray(frame_resized))
        self.camera_label.config(image=photo)
        self.camera_label.image = photo
    
    def on_object_select(self, event):
        """Xử lý khi chọn vật thể từ listbox"""
        selection = self.detection_listbox.curselection()
        if selection:
            index = selection[0]
            if index < len(self.detection_results):
                obj = self.detection_results[index]
                self.log_result(f"Đã chọn: {obj['class']} (confidence: {obj['confidence']:.2f})")
    
    def move_to_selected_object(self):
        """Di chuyển robot đến vật thể đã chọn"""
        selection = self.detection_listbox.curselection()
        if not selection:
            messagebox.showwarning("Cảnh báo", "Vui lòng chọn một vật thể")
            return
        
        index = selection[0]
        if index < len(self.detection_results):
            obj = self.detection_results[index]
            class_name = obj['class']
            
            # Lấy vị trí đã định nghĩa cho loại vật thể này
            if class_name in self.object_positions:
                target_angles = self.object_positions[class_name]
            else:
                target_angles = self.object_positions['default']
            
            # Di chuyển robot
            self.move_robot_to_position(target_angles)
            self.log_result(f"Robot đang di chuyển đến {class_name}")
    
    def move_robot_to_position(self, target_angles):
        """Di chuyển robot đến vị trí mục tiêu với animation"""
        def animate_movement():
            steps = 50
            current = self.joint_angles.copy()
            
            for step in range(steps + 1):
                progress = step / steps
                for i in range(6):
                    angle = current[i] + (target_angles[i] - current[i]) * progress
                    self.sliders[i].set(angle)
                    self.joint_angles[i] = angle
                    self.angle_labels[i].config(text=f"{angle:.1f}°")
                
                self.update_visualization()
                time.sleep(0.05)
        
        # Chạy animation trong thread riêng
        thread = threading.Thread(target=animate_movement)
        thread.daemon = True
        thread.start()
    
    def save_object_position(self):
        """Lưu vị trí hiện tại cho vật thể đã chọn"""
        selection = self.detection_listbox.curselection()
        if not selection:
            messagebox.showwarning("Cảnh báo", "Vui lòng chọn một vật thể")
            return
        
        index = selection[0]
        if index < len(self.detection_results):
            obj = self.detection_results[index]
            class_name = obj['class']
            
            # Lưu vị trí hiện tại
            self.object_positions[class_name] = self.joint_angles.copy()
            self.log_result(f"Đã lưu vị trí cho {class_name}: {self.joint_angles}")
            messagebox.showinfo("Thành công", f"Đã lưu vị trí cho {class_name}")
    
    def toggle_auto_mode(self):
        """Bật/tắt chế độ tự động"""
        self.auto_mode = not self.auto_mode
        if self.auto_mode:
            self.auto_btn.config(text="Tắt Chế Độ Tự Động")
            self.auto_status.config(text="Trạng thái: Tự động")
            self.log_result("Đã bật chế độ tự động")
        else:
            self.auto_btn.config(text="Bật Chế Độ Tự Động")
            self.auto_status.config(text="Trạng thái: Thủ công")
            self.log_result("Đã tắt chế độ tự động")
    
    def log_result(self, message):
        """Ghi log kết quả"""
        if hasattr(self, 'result_text'):
            timestamp = time.strftime("%H:%M:%S")
            self.result_text.insert(tk.END, f"[{timestamp}] {message}\n")
            self.result_text.see(tk.END)
    
    # Các phương thức từ code gốc
    def update_joint(self, joint_idx, value):
        """Cập nhật góc khớp khi slider thay đổi"""
        self.joint_angles[joint_idx] = float(value)
        self.angle_labels[joint_idx].config(text=f"{float(value):.1f}°")
        self.update_visualization()
    
    def dh_transform(self, a, alpha, d, theta):
        """Tính ma trận biến đổi DH"""
        theta_rad = math.radians(theta)
        alpha_rad = math.radians(alpha)
        
        ct = math.cos(theta_rad)
        st = math.sin(theta_rad)
        ca = math.cos(alpha_rad)
        sa = math.sin(alpha_rad)
        
        return np.array([
            [ct, -st*ca, st*sa, a*ct],
            [st, ct*ca, -ct*sa, a*st],
            [0, sa, ca, d],
            [0, 0, 0, 1]
        ])
    
    def forward_kinematics(self):
        """Tính toán động học thuận"""
        positions = [np.array([0, 0, 0])]
        T = np.eye(4)
        
        for i in range(6):
            self.dh_params[i][3] = self.joint_angles[i]
            T_i = self.dh_transform(*self.dh_params[i])
            T = T @ T_i
            pos = T[:3, 3]
            positions.append(pos)
        
        return positions, T
    
    def update_visualization(self):
        """Cập nhật hiển thị 3D"""
        self.ax.clear()
        
        positions, end_transform = self.forward_kinematics()
        
        x_coords = [pos[0] for pos in positions]
        y_coords = [pos[1] for pos in positions]
        z_coords = [pos[2] for pos in positions]
        
        self.ax.plot(x_coords, y_coords, z_coords, 'b-', linewidth=3, label='Tay robot')
        self.ax.scatter(x_coords[:-1], y_coords[:-1], z_coords[:-1], 
                       c='red', s=50, label='Khớp')
        self.ax.scatter(x_coords[-1], y_coords[-1], z_coords[-1], 
                       c='green', s=100, marker='s', label='End-effector')
        
        # Vẽ hệ trục tọa độ
        self.ax.quiver(0, 0, 0, 0.1, 0, 0, color='red', arrow_length_ratio=0.1)
        self.ax.quiver(0, 0, 0, 0, 0.1, 0, color='green', arrow_length_ratio=0.1)
        self.ax.quiver(0, 0, 0, 0, 0, 0.1, color='blue', arrow_length_ratio=0.1)
        
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_zlabel('Z (m)')
        self.ax.set_title('Robot Tay Máy 6 Bậc Tự Do')
        self.ax.legend()
        
        max_range = 0.8
        self.ax.set_xlim([-max_range, max_range])
        self.ax.set_ylim([-max_range, max_range])
        self.ax.set_zlim([0, max_range])
        
        end_pos = positions[-1]
        if hasattr(self, 'position_label'):
            self.position_label.config(
                text=f"X: {end_pos[0]:.3f} m\nY: {end_pos[1]:.3f} m\nZ: {end_pos[2]:.3f} m"
            )
        
        self.canvas.draw()
    
    def reset_joints(self):
        """Reset tất cả khớp về 0 độ"""
        for i, slider in enumerate(self.sliders):
            slider.set(0)
            self.joint_angles[i] = 0
            self.angle_labels[i].config(text="0.0°")
        self.update_visualization()
    
    def home_position(self):
        """Đưa robot về vị trí home"""
        home_angles = [0, -90, 90, 0, 90, 0]
        for i, (slider, angle) in enumerate(zip(self.sliders, home_angles)):
            slider.set(angle)
            self.joint_angles[i] = angle
            self.angle_labels[i].config(text=f"{angle}.0°")
        self.update_visualization()
    
    def save_position(self):
        """Lưu vị trí hiện tại"""
        position_str = ", ".join([f"{angle:.1f}°" for angle in self.joint_angles])
        messagebox.showinfo("Vị Trí Đã Lưu", 
                           f"Các góc khớp: [{position_str}]")
    
    def __del__(self):
        """Cleanup khi đóng ứng dụng"""
        if self.cap:
            self.cap.release()

def main():
    root = tk.Tk()
    app = RobotArmGUI(root)
    
    def on_closing():
        if app.camera_running:
            app.camera_running = False
            if app.cap:
                app.cap.release()
        root.destroy()
    
    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.mainloop()

if __name__ == "__main__":
    main()