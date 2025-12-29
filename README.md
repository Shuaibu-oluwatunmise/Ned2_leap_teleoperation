# 🤖 AI-Enhanced Robotic Arm Teleoperation System

[![Python 3.8+](https://img.shields.io/badge/Python-3.8+-blue.svg)](https://www.python.org/downloads/)
[![ROS2 Jazzy](https://img.shields.io/badge/ROS2-Jazzy-blue.svg)](https://docs.ros.org/en/jazzy/)
[![Machine Learning](https://img.shields.io/badge/ML-99.51%25%20Accuracy-brightgreen.svg)](docs/Training%20Results.txt)
[![scikit-learn](https://img.shields.io/badge/scikit--learn-1.3+-orange.svg)](https://scikit-learn.org/)
[![License](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

> **Natural hand gesture control for robotic manipulation with AI-powered movement classification achieving 99.51% accuracy**

## 📋 Table of Contents
- [Overview](#overview)
- [Key Features](#key-features)
- [System Architecture](#system-architecture)
- [Technologies Used](#technologies-used)
- [Quick Start](#quick-start)
- [Project Structure](#project-structure)
- [Machine Learning Pipeline](#machine-learning-pipeline)
- [Results](#results)
- [Next Steps](#next-steps)
- [Documentation](#documentation)
- [Contributing](#contributing)
- [Acknowledgments](#acknowledgments)

---

## 🎯 Overview

This project implements an intelligent teleoperation system for the **Niryo Ned2 robotic arm** using **Leap Motion hand tracking** and **machine learning-based movement classification**. The system achieves human-like control precision while filtering unintentional movements through a two-factor authentication approach: gesture activation + real-time ML classification.

### The Problem
Traditional hand tracking systems suffer from noise and unintentional movements that can trigger unwanted robot actions, creating safety concerns and reducing control precision.

### Our Solution
A dual-layer safety system combining:
1. **Gesture-based activation** (pointing gesture to start, peace sign to stop)
2. **ML-powered movement classification** (99.51% accuracy) that filters intentional from unintentional hand movements in real-time

---

## ✨ Key Features

- 🎮 **Intuitive Hand Control** - Natural 6-DOF robot manipulation through hand gestures
- 🧠 **AI Movement Filtering** - Real-time classification with 99.51% accuracy
- 🔒 **Two-Factor Safety** - Gesture activation + ML verification for enhanced safety
- ⚡ **Low Latency** - Sub-second response time for seamless control
- 📊 **Comprehensive Dataset** - 400 labeled recordings (35,712 training windows)
- 🎯 **Gesture Recognition** - Point to activate, peace sign to deactivate
- 🔄 **Cross-Platform** - Windows (Leap Motion) → Network → Ubuntu (ROS2) → Robot

---

## 🏗️ System Architecture
```
┌─────────────────┐       UDP        ┌──────────────────┐      ROS2      ┌─────────────┐
│  Windows PC     │ ════════════════> │   Ubuntu/ROS2    │ ══════════════> │  Niryo Ned2 │
│                 │                   │                  │                 │  Robot Arm  │
│ • Leap Motion   │                   │ • Hand Receiver  │                 │             │
│ • Hand Tracking │                   │ • ML Classifier  │ <─ Calibration ─┤ IP: ...143  │
│ • Gesture Det.  │                   │ • Robot Control  │                 │             │
└─────────────────┘                   └──────────────────┘                 └─────────────┘
      ↓                                        ↓
  Hand Stream                          [FUTURE: ML Filter]
  hand_streamV2.py                     hand_classifier_node.py
```

**Current Flow:**
1. Leap Motion captures hand movements (Windows)
2. Hand data streamed via UDP to Ubuntu
3. ROS2 node publishes to topics
4. Robot controller subscribes and commands robot

**Next Step (In Progress):**
- ML classifier node filters movements before robot control

---

## 🛠️ Technologies Used

### Hardware
- **Leap Motion Controller (LM-010)** - Hand tracking sensor
- **Niryo Ned2** - 6-DOF collaborative robotic arm
- **Network** - UDP communication between Windows/Ubuntu

### Software Stack

#### Windows Side
![Python](https://img.shields.io/badge/Python-3.8-3776AB?logo=python&logoColor=white)
![Leap Motion SDK](https://img.shields.io/badge/Leap_Motion-Gemini_V5.20-00C853)

- Ultraleap Gemini SDK
- LeapC Python bindings
- NumPy for data processing

#### Ubuntu/ROS Side
![ROS2](https://img.shields.io/badge/ROS2-Jazzy-22314E?logo=ros&logoColor=white)
![Python](https://img.shields.io/badge/Python-3.12-3776AB?logo=python&logoColor=white)

- ROS2 Jazzy Jalisco
- rclpy (ROS2 Python client)
- Niryo ROS2 driver
- MoveIt2 (motion planning)

#### Machine Learning
![scikit-learn](https://img.shields.io/badge/scikit--learn-1.3-F7931E?logo=scikitlearn&logoColor=white)
![pandas](https://img.shields.io/badge/pandas-2.0-150458?logo=pandas&logoColor=white)
![NumPy](https://img.shields.io/badge/NumPy-1.24-013243?logo=numpy&logoColor=white)

- **Random Forest Classifier** - Best performer (99.51% accuracy)
- **Feature Engineering** - 18 engineered features per time window
- **Sliding Window** - 1-second windows with 0.5s stride
- Dataset: 35,712 labeled windows from 400 recordings

---

## 🚀 Quick Start

### Prerequisites
- Windows PC with Python 3.8+ and Leap Motion Controller
- Ubuntu machine/VM with ROS2 Jazzy installed
- Niryo Ned2 robot on same network
- Both systems on same network

### Installation

#### 1. Windows Setup
```powershell
# Clone repository
git clone https://github.com/Shuaibu-oluwatunmise/Ned2_leap_teleoperation.git
cd robotics-teleoperation/windows_side

# Create virtual environment
py -3.8 -m venv leap_env
leap_env\Scripts\activate

# Install Leap Motion bindings
git clone https://github.com/ultraleap/leapc-python-bindings.git
cd leapc-python-bindings
pip install -r requirements.txt
pip install -e leapc-python-api
cd ..

# Install dependencies
pip install numpy
```

#### 2. Ubuntu/ROS Setup
```bash
# Clone repository
cd ~
git clone https://github.com/Shuaibu-oluwatunmise/Ned2_leap_teleoperation.git

# Run automated setup (one-time, ~30 minutes)
cd robotics-teleoperation/ubuntu_ros_side
python3 setup_niryo.py

# Transfer model files to models/ directory
# (Only needed when ML integration is complete)
```

### Running the System

**See [Quick Start Guide](docs/instructions%20for%20quick%20use.txt) for detailed step-by-step instructions.**

---

## 📁 Project Structure
```
ROBOTICS/
├── windows_side/                    # Windows/Leap Motion code
│   ├── hand_streamV2.py            # ⭐ Main streaming script
│   ├── data_recorder.py            # Training data collection
│   └── ml_training/                # ML pipeline
│       ├── feature_extractor.py    # JSON → Features
│       ├── train_model.py          # Model training
│       ├── real_time_classifier.py # Local testing
│       └── training_features.csv   # Extracted features
│
├── ubuntu_ros_side/                # Ubuntu/ROS2 code
│   ├── hand_receiverV2.py         # UDP → ROS2 topics
│   ├── robot_controllerV3.py      # ⭐ Best robot controller
│   ├── setup_niryo.py             # Automated setup script
│   └── models/                    # Trained ML models
│       ├── hand_classifier_*.pkl  # Random Forest model
│       └── feature_scaler_*.pkl   # Feature normalization
│
├── DATA/                          # Training data archives
│   ├── intentional_200.zip        # 200 intentional recordings
│   └── unintentional_200.zip      # 200 unintentional recordings
│
└── docs/                          # Documentation
    ├── Document.txt               # Full project roadmap
    ├── DataRequirements.txt       # ML data collection guide
    └── Training Results.txt       # Model performance metrics
```

---

## 🧪 Machine Learning Pipeline

### 1. Data Collection
- **Duration:** 3 weeks
- **Recordings:** 400 files (200 intentional + 200 unintentional)
- **Recording Length:** 45 seconds each
- **Participants:** 5 people (diverse movement patterns)
- **Scenarios:** 
  - Intentional: Deliberate robot control gestures
  - Unintentional: Typing, reaching, phone use, scratching, fidgeting

### 2. Feature Engineering
Extracted **18 features** per 1-second window:
- Hand presence ratio & frame count
- Displacement, velocity statistics (mean, std, max)
- Acceleration patterns
- Direction consistency (movement smoothness)
- 3D position statistics
- Grab/pinch strength
- Finger extension patterns

### 3. Model Training
Tested 3 algorithms:
- ✅ **Random Forest** - 99.51% accuracy (selected)
- Gradient Boosting - 99.50% accuracy
- Logistic Regression - 98.49% accuracy

**Training Split:**
- Train: 70% (24,997 windows)
- Validation: 10% (3,572 windows)
- Test: 20% (7,143 windows)

---

## 📊 Results

### Model Performance

| Metric | Score |
|--------|-------|
| **Test Accuracy** | **99.51%** |
| Precision (Intentional) | 99.22% |
| Recall (Intentional) | 99.80% |
| F1-Score | 99.51% |
| False Positive Rate | 0.78% |
| False Negative Rate | 0.20% |

### Confusion Matrix
```
                 Predicted
               Int.    Unint.
Actual  Int.  │ 3564     7   │
       Unint. │  28   3544   │
```

**Interpretation:**
- Out of 1000 intentional gestures, 998 correctly recognized
- Out of 1000 unintentional movements, 992 correctly ignored
- Only 8 per 1000 false triggers

### Top 3 Most Important Features
1. **Hand Presence Ratio** (27%) - Tracking continuity
2. **Finger Mean** (15%) - Finger extension patterns
3. **Hand Present Frames** (15%) - Confirms tracking stability

📄 **Full results:** [Training Results.txt](docs/Training%20Results.txt)

---

## 🔮 Next Steps

### Phase 6: ML Integration (In Progress)
- [ ] Create `hand_classifier_node.py` ROS2 node
- [ ] Integrate classifier between receiver and controller
- [ ] Real-world testing and validation
- [ ] Performance optimization for <100ms latency
- [ ] Edge case handling and robustness testing

### Future Enhancements
- [ ] Multi-hand control support
- [ ] Gripper control via grab strength
- [ ] Adaptive learning from user corrections
- [ ] Extended gesture vocabulary
- [ ] Visual feedback system for classification status
- [ ] Support for additional robot platforms

---

## 📚 Documentation

| Document | Description |
|----------|-------------|
| [Quick Start Guide](docs/instructions%20for%20quick%20use.txt) | Step-by-step usage instructions |
| [Project Roadmap](docs/Document.txt) | Complete development phases |
| [Data Requirements](docs/DataRequirements.txt) | ML data collection methodology |
| [Training Results](docs/Training%20Results.txt) | Detailed model performance |
| [Setup Instructions](ubuntu_ros_side/setupinstructions.txt) | Full installation guide |

---

## 🤝 Contributing

Contributions, feedback and suggestions are welcome!

### If You're Interested:
1. Fork the repository
2. Create a feature branch
3. Submit a pull request with detailed description

---

## 📝 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

## 🙏 Acknowledgments

- **Middlesex University London** - Academic support
- **Anthropic's Claude** - Development assistance and debugging
- **Ultraleap** - Leap Motion SDK and documentation
- **Niryo Robotics** - ROS2 driver and robot platform
- **Open Source Community** - ROS2, scikit-learn, and Python ecosystems

---

## 👨‍💻 Author

**Raph (Oluwatunmise Shuaibu)** - Final Year BEng Mechatronics and Robotics Student  
Middlesex University London (Graduating July 2025)

- 🔗 GitHub: [@Shuaibu-oluwatunmise](https://github.com/Shuaibu-oluwatunmise)
- 📧 Email: shuaibuoluwatunmise@gmail.com
- 💼 LinkedIn: [Oluwatunmise Shuaibu](https://linkedin.com/in/oluwatunmise-shuaibu-881519257)

---

## 📸 Demo

> *Coming soon: Video demonstration of the system in action*

---

<div align="center">

**⭐ If you find this project interesting, please consider giving it a star! ⭐**

Made with ❤️ and lots of ☕

</div>
