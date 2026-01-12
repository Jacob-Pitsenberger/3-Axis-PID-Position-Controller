# **3‑Axis PID Position Controller**

A modular, extensible **3‑axis position‑hold controller** currently supporting the DJI Tello drone, featuring:

- World‑frame PID control for **X, Y, Z, and yaw**
- A custom **state estimator** with altitude fusion, velocity transforms, and yaw‑rate estimation
- **Velocity‑based damping** for drift suppression
- **Yaw‑rate damping** for stable heading hold
- High‑frequency **CSV logging** for tuning and analysis
- Clean, maintainable architecture designed for iterative refinement

This project implements a full hover‑control stack on top of the Tello SDK using Python, with a focus on clarity, modularity, and real‑world flight performance.

> Future iterations are planned to support additional STEM‑focused drones with Python‑accessible SDKs, such as the CoDrone EDU.
> 
---

## **📦 Project Structure**

```
3-Axis-PID-Position-Controller/
│
├── main.py
├── config/
│   └── pid_config.json
│
├── control/
│   └── pid/
│       ├── pid_base.py
│       ├── pid_x.py
│       ├── pid_y.py
│       ├── pid_z.py
│       └── pid_yaw.py
│
├── controller/
│   ├── controller.py
│   └── state_estimator.py
│
├── drone/
│   ├── drone_interface.py
│   └── drone_state.py
│
└── utils/
    ├── config_loader.py
    ├── filters.py
    ├── logger.py
    └── transforms.py
```

---

## **🚀 Quick Start**

### **1. Install dependencies**
```
pip install djitellopy
```

### **2. Power on the Tello and connect to its Wi‑Fi network**

### **3. Run the controller**
```
python main.py
```

The drone will:

1. Connect  
2. Take off  
3. Stabilize  
4. Enter the PID hover loop  
5. Log flight data to `/logs/`  

---

## **📘 Documentation**

Full documentation is available in the GitHub Wiki:

- **Architecture Overview**  
- **Module‑Level Deep Dives**  
- **Control Theory Primer**  
- **Tuning Guide**  
- **Flight Log Interpretation Guide**  
- **How the Estimator Works**  
- **How Damping Works**

These pages explain the full control stack, estimator math, tuning workflow, and how to interpret logs for iterative refinement.

---

## **🧠 Key Features**

### **World‑Frame PID Control**
Each axis uses a dedicated PID controller with:

- Anti‑windup  
- Derivative filtering  
- Output clamping  
- Configurable gains via JSON  

### **Custom State Estimator**
- Altitude fusion (ToF + barometer + height)  
- Body‑to‑world velocity transform  
- Horizontal position integration  
- Yaw‑rate estimation with wrap‑around handling  

### **Damping Layers**
- Velocity damping for X/Y drift suppression  
- Yaw‑rate damping for stable heading hold  

### **High‑Frequency Logging**
- Structured CSV logs  
- Throttled logging to keep file sizes manageable  
- Ideal for tuning and analysis  

---

## **🛠️ Requirements**

- Python 3.8+  
- `djitellopy`  
- A DJI Tello drone  
- A stable indoor environment for hover testing  

---

## **📈 Development Status**

The controller is still in an active tuning phase and has not yet reached a fully stable hover state.

---

## **📄 License**

This project is released under the [MIT License](LICENSE.md).


