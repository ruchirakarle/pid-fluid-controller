# PID Fluid Pressure Controller

A real-time fluid dispensing pressure control system built in **C++** and **Python**, simulating the kind of electromechanical control software used in industrial fluid dispensing equipment. The project implements a full PID (Proportional-Integral-Derivative) control loop with a live GUI dashboard for monitoring and tuning.

---

## What It Does

In industrial dispensing systems (like those used in electronics manufacturing, medical devices, and packaging), precise fluid pressure control is critical. Too much pressure damages components; too little causes incomplete fills. A PID controller continuously measures the error between the target pressure (setpoint) and the actual pressure, then adjusts the pump output to correct it in real time.

This project:
- Simulates a fluid pump system responding to a PID control signal
- Lets you tune the controller live and see the effect instantly
- Triggers alarms when pressure goes outside a safe operating range
- Logs all session data to CSV for analysis

---

## Features

- **C++ PID Algorithm** — clean implementation of Proportional, Integral, and Derivative control with anti-windup clamping to prevent integral saturation
- **Live Python Dashboard** — real-time scrolling graph of pressure vs setpoint, built with Tkinter and Matplotlib
- **Adjustable Gain Sliders** — tune Kp, Ki, Kd live without restarting; see the effect on the graph instantly
- **Setpoint Control** — drag the setpoint slider to change the target pressure at any time
- **Industrial Alarm System** — flashing red alert banner when pressure exceeds safe thresholds; adjustable low/high alarm limits
- **Dotted Threshold Lines** — alarm boundaries drawn directly on the pressure graph
- **Status Indicator** — live SAFE / ALARM status label with color coding
- **Dark / Light Mode Toggle** — switch between themes with one click
- **CSV Data Logger** — every session logs time, setpoint, pressure, and control signal to a timestamped CSV file
- **Disturbance Simulation** — setpoint jumps at t=10s to simulate a real valve event

---

## Tech Stack

| Component | Technology |
|---|---|
| Control algorithm | C++ |
| GUI dashboard | Python, Tkinter |
| Data visualization | Matplotlib |
| Data logging | Python CSV, Pandas |
| Build | g++ (GCC) |

---

## Project Structure

```
pid-fluid-controller/
├── pid_controller.cpp   # C++ PID algorithm + fluid system simulation
├── dashboard.py         # Python live GUI dashboard
├── plot_pid.py          # Static plot generator from CSV
├── pid_output.csv       # Sample session output
├── pid_plot.png         # Sample output graph
└── README.md
```

---

## How to Run

### C++ Controller
```bash
g++ -o pid pid_controller.cpp
./pid.exe               # Windows
./pid                   # Mac / Linux
```
This prints a pressure table and writes `pid_output.csv`.

### Python Dashboard
```bash
pip install matplotlib pandas
python dashboard.py
```

### Static Plot
```bash
python plot_pid.py
```

---

## PID Tuning Guide

| Gain | Effect when increased |
|---|---|
| Kp (Proportional) | Faster response, but may overshoot |
| Ki (Integral) | Eliminates steady-state error, may oscillate |
| Kd (Derivative) | Dampens overshoot, slows response |

**Anti-windup** is implemented to prevent the integral term from accumulating during large setpoint changes, which would cause excessive overshoot in a real system.


---

## Author

Ruchira Karle
MS Computer Science — Northeastern University
linkedin.com/in/ruchira-karle

**Ruchira Karle**  
MS Computer Science — Northeastern University  
[linkedin.com/in/ruchira-karle](https://linkedin.com/in/ruchira-karle)
