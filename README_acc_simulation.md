# Autonomous Adaptive Cruise Control (ACC) Simulation

A dynamic 2D simulation of an Autonomous Vehicle's Adaptive Cruise Control (ACC) and Autonomous Emergency Braking (AEB) systems. Built using Python and Pygame, this project demonstrates longitudinal control logic, sensor perception, and real-time vehicle telemetry.

## 🚀 Features

* **Adaptive Cruise Control (ACC):** Automatically adjusts speed to maintain a safe following distance from the vehicle ahead using proportional gap-error control.
* **Autonomous Emergency Braking (AEB):** Calculates the Time-To-Collision (TTC) continuously and applies emergency maximum braking if a collision is imminent.
* **Simulated Radar Perception:** Simulates a forward-facing radar cone with Gaussian noise to mimic real-world sensor inaccuracies.
* **Dynamic Lead Vehicle Scenario:** The lead vehicle is programmed to perform a series of maneuvers (cruising, moderate braking, accelerating, and sudden hard braking) to test the robustness of the Ego vehicle's controller.
* **Interactive Telemetry Dashboard:** Displays real-time data including vehicle speed, acceleration, radar gap distance, Time-to-Collision (TTC), and the current operating state (CRUISE, FOLLOWING, or EMERGENCY BRAKE).
* **Immersive Visuals:** Includes moving background scenery, functioning brake lights, and visual radar beams for an intuitive understanding of the system's operation.

## 🛠️ Technology Stack
* **Language:** Python 3.11+
* **Libraries:** `pygame` (2D Rendering & Game Loop), `numpy` (Mathematics & Noise Simulation)

## ⚙️ How It Works

The simulation uses a longitudinal control algorithm based on **Spacing Policy**. 
1. The desired gap is calculated dynamically based on the Ego vehicle's current velocity and a fixed safe time gap. 
2. If the lead vehicle is out of radar range, the Ego vehicle defaults to standard cruise control. 
3. If the lead vehicle is detected, the controller computes the necessary acceleration/deceleration by combining distance gap error and relative velocity damping.

## 💻 Installation & Usage

1. **Clone the repository:**
   ```bash
   git clone https://github.com/YourUsername/your-repo-name.git
   cd your-repo-name
