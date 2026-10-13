# Overtake Safety System

An advanced autonomous vehicle overtaking simulation and safety decision prediction system.

---

## Project Structure

The project is cleanly separated into **Frontend** and **Backend** components:

```text
overtake-main/
├── README.md                 # Project documentation
├── index.html                # Root portal (redirects to frontend/index.html)
│
├── backend/                  # Python simulation & physics engine
│   ├── master.py             # 10,000-step overtaking safety analysis simulation
│   └── requirements.txt      # Python dependencies
│
└── frontend/                 # Interactive web interface & visualization
    ├── index.html            # Hub landing page
    ├── simulation.html       # 2D horizontal overtake simulator & telemetry
    ├── backend.html          # Dynamic overtaking safety predictor tool
    ├── css/
    │   ├── index.css         # Styling for landing page
    │   ├── simulation.css    # Styling for simulation UI
    │   └── backend.css       # Styling for safety predictor UI
    └── js/
        ├── simulation.js     # Canvas rendering & real-time simulation logic
        └── backend.js        # Numerical integration & safety metric algorithms
```

---

## Backend Usage

The backend runs a physics simulation analyzing relative kinematics, safe distance polynomials, Time-To-Collision (TTC), abort braking distances, and merging gap conditions.

### Prerequisites

- Python 3.8+
- NumPy

```bash
cd backend
pip install -r requirements.txt
```

### Running the Backend Simulation

```bash
python3 master.py
```

The script will simulate 10,000 time steps and output real-time kinematics and safety evaluations to the console.

---

## Frontend Usage

The frontend contains interactive visual applications:
1. **Overtake Simulation (`simulation.html`)**: Interactive real-time vehicle movement on a roadway, camera following ego car, live speed controls, and dynamic safety condition indicators.
2. **Safety Predictor (`backend.html`)**: Input custom vehicle speeds and distances to compute dynamic clearance, TTC, and abort windows.

### Running the Frontend

You can simply open `frontend/index.html` (or the root `index.html`) directly in any modern web browser:

```bash
# Option 1: Direct file opening (macOS)
open frontend/index.html

# Option 2: Local HTTP server
python3 -m http.server 8000
# Then visit http://localhost:8000/
```