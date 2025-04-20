# Cat Laser Pointer Tracker 

![Demo Systemu](demo.gif) *(placeholder for demo image)*

## 🚀 Project Overview
Computer vision + IoT system that creates interactive play sessions for cats using AI-controlled laser pointer.

## ✨ Key Features
- **Real-time YOLOv8 cat detection** (80%+ accuracy)
- **Dual-control modes**:
  - 🤖 **Auto**: AI strategies (Explore/Escape/Confuse)
  - 🎮 **Manual**: Keyboard control (WASD/arrows)
- **Safety System**:
  - 🛡️ 100px min distance enforcement
  - 📉 Movement smoothing algorithms
- **Precision servo control** (0.1° resolution)

## 🧩 System Architecture
```bash
CatLaserPointerToy/
├── main.py                # Main application thread
├── config.py              # Tunable parameters
├── controllers/           # Control subsystems
│   ├── laser_controller.py # Smoothing algorithms
│   ├── cat_tracker.py      # Movement analytics
│   ├── ai_controller.py    # Behavior strategies
│   └── servo_controller.py # Hardware interface
├── utils/                 # Helper modules
│   ├── geometry.py         # Spatial calculations
│   └── drawing.py          # Visual feedback
└── ServoDriver/              # ESP32 source
    └── ServoDriver.ino
```
```mermaid
graph LR
    PC[Laptop] -- USB --- ESP32
    ESP32 -- UART --- SC09_1[Servo 1]
    ESP32 -- UART --- SC09_2[Servo 2]
    Webcam --- PC
````

    
## 🎮 Keyboard Control Reference

| Key            | Function                     | Mode          | Visual Feedback       |
|----------------|------------------------------|---------------|-----------------------|
| `SPACEBAR`     | Confirm calibration          | Calibration    | Green flash           |
| `LEFT CLICK`   | Add calibration point        | Calibration    | Red circle + number   |
| `R`            | Reset calibration            | Any            | Red flash             |
| `S`            | Toggle simulation mode       | Any            | Yellow status LED     |
| `C`            | Center servos                | Any            | White flash           |
| `W` / `↑`      | Move laser up                | Manual         | -                     |
| `A` / `←`      | Move laser left              | Manual         | -                     |
| `S` / `↓`      | Move laser down              | Manual         | -                     |
| `D` / `→`      | Move laser right             | Manual         | -                     |
| `+`            | Increase movement speed      | Manual         | Speedometer increase  |
| `-`            | Decrease movement speed      | Manual         | Speedometer decrease  |
| `M`            | Toggle manual/auto mode      | Any            | Blue LED toggle       |
| `ESCAPE`       | Emergency stop               | Any            | All LEDs red          |
| `Q`            | Quit application             | Any            | System shutdown       |

## Speed Adjustment Scale
| Key Presses | Speed Multiplier | Effect                     |
|-------------|------------------|----------------------------|
| 0 (default) | 1.0x             | Base speed (15px/frame)    |
| 1-3 `+`     | 1.2x - 1.8x      | Gradual acceleration       |
| 4+ `+`      | 2.0x (max)       | Maximum tracking speed     |
| `-`         | 0.8x per press   | Smooth deceleration        |

> **Note**: Manual mode overrides AI tracking when active.  
> **LED Colors**:  
> 🔵 Manual mode | 🟢 Auto mode | 🟡 Simulation | 🔴 Error/Stop



