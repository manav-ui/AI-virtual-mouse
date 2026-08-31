# AI-virtual-mousedd
import cv2
import mediapipe as mp
import pyautogui
#import numpy as np
import tkinter as tk
from tkinter import ttk, Scale
import threading
import time
from PIL import Image, ImageTk
import math

# For volume control
from ctypes import cast, POINTER
from comtypes import CLSCTX_ALL
from pycaw.pycaw import AudioUtilities, IAudioEndpointVolume

class VirtualMouseApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Virtual Mouse Controller")
        self.root.geometry("800x600")
        self.root.resizable(True, True)
        
        # Initialize MediaPipe hands
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.7
        )
        self.mp_drawing = mp.solutions.drawing_utils
        
        # Screen dimensions
        self.screen_width, self.screen_height = pyautogui.size()
        
        # Initialize webcam
        self.cap = None
        
        # Control variables
        self.is_tracking = False
        self.smoothing = 5  # Smoothing factor
        self.prev_x, self.prev_y = 0, 0
        self.gesture_cooldown = 0
        self.last_scroll_y = None
        self.last_volume_x = None
        
        # Volume control setup
        devices = AudioUtilities.GetSpeakers()
        interface = devices.Activate(IAudioEndpointVolume._iid_, CLSCTX_ALL, None)
        self.volume = cast(interface, POINTER(IAudioEndpointVolume))
        self.min_vol, self.max_vol, _ = self.volume.GetVolumeRange()
        
        # Create GUI
        self.create_widgets()
        
        # Thread for processing
        self.processing_thread = None

        # Store reference to PhotoImage to prevent garbage collection
        self.photo = None
        
    def create_widgets(self):
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        control_frame = ttk.LabelFrame(main_frame, text="Controls", padding="10")
        control_frame.pack(side=tk.LEFT, fill=tk.Y, padx=10, pady=10)
        
        self.toggle_btn = ttk.Button(control_frame, text="Start Tracking", command=self.toggle_tracking)
        self.toggle_btn.pack(pady=10, fill=tk.X)
        
        ttk.Label(control_frame, text="Cursor Sensitivity:").pack(pady=(10, 0))
        self.sensitivity = tk.DoubleVar(value=2.0)
        sensitivity_slider = Scale(control_frame, from_=0.5, to=5.0, resolution=0.1, 
                                  orient=tk.HORIZONTAL, variable=self.sensitivity)
        sensitivity_slider.pack(fill=tk.X, pady=5)
        
        ttk.Label(control_frame, text="Cursor Smoothing:").pack(pady=(10, 0))
        self.smoothing_var = tk.IntVar(value=self.smoothing)
        smoothing_slider = Scale(control_frame, from_=1, to=10, orient=tk.HORIZONTAL, 
                                variable=self.smoothing_var, command=self.update_smoothing)
        smoothing_slider.pack(fill=tk.X, pady=5)
        
        guide_frame = ttk.LabelFrame(main_frame, text="Gesture Guide", padding="10")
        guide_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        gestures = [
            ("Index finger extended", "Move cursor"),
            ("Index + Middle fingers extended", "Right click"),
            ("Thumb + Index finger pinched", "Left click"),
            ("All fingers extended", "Scroll (up/down) & Volume (left/right)"),
            ("Fist", "Screenshot")
        ]
        
        for gesture, action in gestures:
            gesture_frame = ttk.Frame(guide_frame)
            gesture_frame.pack(fill=tk.X, pady=5)
            ttk.Label(gesture_frame, text=gesture, width=30).pack(side=tk.LEFT)
            ttk.Label(gesture_frame, text="→").pack(side=tk.LEFT, padx=5)
            ttk.Label(gesture_frame, text=action).pack(side=tk.LEFT)
        
        self.video_frame = ttk.LabelFrame(main_frame, text="Camera Feed", padding="10")
        self.video_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        self.canvas = tk.Canvas(self.video_frame, bg="black")
        self.canvas.pack(fill=tk.BOTH, expand=True)
        
        self.status_var = tk.StringVar(value="Ready")
        status_bar = ttk.Label(self.root, textvariable=self.status_var, relief=tk.SUNKEN, anchor=tk.W)
        status_bar.pack(side=tk.BOTTOM, fill=tk.X)
    
    def update_smoothing(self, value):
        self.smoothing = self.smoothing_var.get()
    
    def toggle_tracking(self):
        if not self.is_tracking:
            self.start_tracking()
        else:
            self.stop_tracking()
    
    def start_tracking(self):
        if self.is_tracking:
            return
            
        self.is_tracking = True
        self.toggle_btn.config(text="Stop Tracking")
        self.status_var.set("Tracking active")
        
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        self.processing_thread = threading.Thread(target=self.process_frames, daemon=True)
        self.processing_thread.start()
    
    def stop_tracking(self):
        self.is_tracking = False
        self.toggle_btn.config(text="Start Tracking")
        self.status_var.set("Tracking stopped")
        
        if self.cap is not None:
            self.cap.release()
            self.cap = None
    
    def process_frames(self):
        while self.is_tracking and self.cap is not None:
            success, image = self.cap.read()
            if not success:
                continue
                
            image = cv2.flip(image, 1)
            image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            results = self.hands.process(image_rgb)
            
            if results.multi_hand_landmarks:
                for hand_landmarks in results.multi_hand_landmarks:
                    self.mp_drawing.draw_landmarks(
                        image, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
                    self.control_mouse(hand_landmarks)
            
            self.display_image(image)
            
            if self.gesture_cooldown > 0:
                self.gesture_cooldown -= 1

            time.sleep(0.01)
    
    def display_image(self, image):
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        canvas_width = self.canvas.winfo_width()
        canvas_height = self.canvas.winfo_height()
        if canvas_width > 1 and canvas_height > 1:
            image_rgb = cv2.resize(image_rgb, (canvas_width, canvas_height))
            pil_image = Image.fromarray(image_rgb)
            self.photo = ImageTk.PhotoImage(image=pil_image)
            self.canvas.create_image(0, 0, image=self.photo, anchor=tk.NW)
    
    def control_mouse(self, hand_landmarks):
        landmarks = [(lm.x, lm.y, lm.z) for lm in hand_landmarks.landmark]
        is_index_up = self.is_finger_up(landmarks, 8, 5)
        is_middle_up = self.is_finger_up(landmarks, 12, 9)
        is_ring_up = self.is_finger_up(landmarks, 16, 13)
        is_pinky_up = self.is_finger_up(landmarks, 20, 17)
        thumb_index_distance = self.calculate_distance(landmarks[4], landmarks[8])

        gesture_name = "No gesture detected"

        # Move cursor (index finger up, others down)
        if is_index_up and not is_middle_up and not is_ring_up and not is_pinky_up:
            gesture_name = "Move cursor"
            index_tip_x, index_tip_y = landmarks[8][0], landmarks[8][1]
            target_x = int(index_tip_x * self.screen_width * self.sensitivity.get())
            target_y = int(index_tip_y * self.screen_height * self.sensitivity.get())
            if self.prev_x == 0 and self.prev_y == 0:
                self.prev_x, self.prev_y = target_x, target_y
            else:
                target_x = self.prev_x + (target_x - self.prev_x) // self.smoothing
                target_y = self.prev_y + (target_y - self.prev_y) // self.smoothing
                self.prev_x, self.prev_y = target_x, target_y
            target_x = max(0, min(self.screen_width - 1, target_x))
            target_y = max(0, min(self.screen_height - 1, target_y))
            pyautogui.moveTo(target_x, target_y)
            self.last_scroll_y = None
            self.last_volume_x = None

        # Left click (thumb and index finger pinched, index up, others down)
        elif (thumb_index_distance < 0.04 and is_index_up and not is_middle_up and not is_ring_up and not is_pinky_up):
            gesture_name = "Left click"
            if self.gesture_cooldown == 0:
                pyautogui.click()
                self.gesture_cooldown = 20
            self.last_scroll_y = None
            self.last_volume_x = None

        # Right click (index and middle fingers up, others down)
        elif is_index_up and is_middle_up and not is_ring_up and not is_pinky_up:
            gesture_name = "Right click"
            if self.gesture_cooldown == 0:
                pyautogui.rightClick()
                self.gesture_cooldown = 20
            self.last_scroll_y = None
            self.last_volume_x = None

        # Scroll & Volume mode (all fingers up)
        elif is_index_up and is_middle_up and is_ring_up and is_pinky_up:
            hand_y = landmarks[0][1]  # wrist y
            hand_x = landmarks[0][0]  # wrist x

            # Scroll
            if self.last_scroll_y is not None:
                dy = hand_y - self.last_scroll_y
                scroll_amount = int(dy * 100)
                if abs(scroll_amount) > 1:
                    pyautogui.scroll(-scroll_amount)
                    gesture_name = "Scrolling"
            self.last_scroll_y = hand_y

            # Volume
            if self.last_volume_x is not None:
                dx = hand_x - self.last_volume_x
                if abs(dx) > 0.01:
                    current_vol = self.volume.GetMasterVolumeLevel()
                    # Map dx to volume range
                    new_vol = current_vol + dx * 20  # Adjust 20 for sensitivity
                    new_vol = max(self.min_vol, min(self.max_vol, new_vol))
                    self.volume.SetMasterVolumeLevel(new_vol, None)
                    gesture_name += " & Volume"
            self.last_volume_x = hand_x

        # Fist (all fingers down) - Screenshot
        elif not is_index_up and not is_middle_up and not is_ring_up and not is_pinky_up:
            gesture_name = "Fist (Screenshot)"
            if self.gesture_cooldown == 0:
                self.take_screenshot()
                self.gesture_cooldown = 20
            self.last_scroll_y = None
            self.last_volume_x = None

        else:
            self.last_scroll_y = None
            self.last_volume_x = None

        self.status_var.set(f"Detected: {gesture_name}")
    
    def take_screenshot(self):
        screenshot = pyautogui.screenshot()
        filename = f"screenshot_{int(time.time())}.png"
        screenshot.save(filename)
        self.status_var.set(f"Screenshot saved: {filename}")
    
    def is_finger_up(self, landmarks, tip_idx, pip_idx):
        return landmarks[tip_idx][1] < landmarks[pip_idx][1]
    
    def calculate_distance(self, point1, point2):
        return math.hypot(point1[0] - point2[0], point1[1] - point2[1])
    
    def on_closing(self):
        self.stop_tracking()
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = VirtualMouseApp(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()
