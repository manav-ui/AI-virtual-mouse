# 🖱️ AI Virtual Mouse Controller

An AI-powered Virtual Mouse Controller that allows you to control your computer using **hand gestures** through a webcam.

The project uses **Computer Vision and Hand Tracking** to detect finger gestures and perform various system actions such as moving the cursor, clicking, scrolling, controlling volume, and taking screenshots.

## 🚀 Features

* 🖱️ Move mouse cursor using hand gestures
* 👆 Left click using Thumb + Index finger pinch
* 👉 Right click using Index + Middle fingers
* 📜 Scroll using hand movement
* 🔊 Control system volume using horizontal hand movement
* 📸 Take screenshots using a fist gesture
* 🎥 Real-time webcam feed
* 🎛️ Adjustable cursor sensitivity
* 🌊 Adjustable cursor smoothing
* 🖥️ Simple GUI built with Tkinter

---

## 🛠️ Technologies Used

* Python
* OpenCV
* MediaPipe
* PyAutoGUI
* Tkinter
* Pillow (PIL)
* Pycaw
* Comtypes

---

## ✋ Gesture Controls

| Gesture                           | Action                  |
| --------------------------------- | ----------------------- |
| ☝️ Index Finger Extended          | Move Cursor             |
| 🤞 Index + Middle Finger Extended | Right Click             |
| 🤏 Thumb + Index Finger Pinch     | Left Click              |
| 🖐️ All Fingers Extended          | Scroll & Volume Control |
| ✊ Fist                            | Take Screenshot         |

---

## 📂 Project Structure

```text
AI-Virtual-Mouse/
│
├── virtual_mouse.py
├── README.md
└── requirements.txt
```

---

## ⚙️ Installation

### 1. Clone the Repository

```bash
git clone https://github.com/your-username/AI-Virtual-Mouse.git
```

### 2. Navigate to the Project Directory

```bash
cd AI-Virtual-Mouse
```

### 3. Install Dependencies

```bash
pip install opencv-python mediapipe pyautogui pillow pycaw comtypes
```

---

## ▶️ Run the Project

```bash
python virtual_mouse.py
```

Make sure your webcam is connected and accessible.

---

## 🧠 How It Works

1. The webcam captures real-time video.
2. OpenCV processes each frame.
3. MediaPipe detects hand landmarks.
4. The program identifies which fingers are raised.
5. Different hand gestures trigger different system actions.
6. PyAutoGUI performs mouse operations.
7. Pycaw controls the system volume.
8. The Tkinter GUI displays the camera feed and control options.

---

## 🎛️ Controls

The application provides:

* **Start/Stop Tracking button**
* **Cursor Sensitivity Slider**
* **Cursor Smoothing Slider**
* **Real-time Camera Feed**
* **Gesture Detection Status**

---

## 📸 Screenshots

When a fist gesture is detected, the application automatically captures a screenshot and saves it with a timestamp.

Example:

```text
screenshot_1720000000.png
```

---

## ⚠️ Requirements

* Python 3.8+
* Webcam
* Windows OS (for Pycaw volume control)

---

## 🔮 Future Improvements

* Add drag and drop functionality
* Add double-click gesture
* Add gesture customization
* Support multiple hands
* Add brightness control
* Improve gesture recognition accuracy
* Add cross-platform volume control
* Add dark mode UI

---

## 🤝 Contributing

Contributions are welcome!

1. Fork the repository
2. Create a new branch

```bash
git checkout -b feature-name
```

3. Make your changes
4. Commit your changes

```bash
git commit -m "Add new feature"
```

5. Push to your branch

```bash
git push origin feature-name
```

6. Create a Pull Request

---

## 📄 License

This project is open-source and available for educational purposes.

---

## ⭐ Show Your Support

If you like this project, consider giving it a ⭐ on GitHub!

---

**Made with ❤️ using Python, Computer Vision, and AI**
