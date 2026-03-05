# Surgical Robot Simulation

A 3D surgical robot arm simulation built with [BrowserBotics](https://browserbotics.com). A Panda robotic arm picks and places surgical instruments (scalpel, forceps, suture kit) from an instrument tray onto an operating table inside a fully modeled operating room.

## Features

- **7-DOF Panda robot arm** with inverse kinematics and smooth motion
- **3 surgical instruments** — scalpel, forceps, and suture kit
- **Interactive controls** — pick, drop, return home, and reset via GUI buttons
- **Joint sliders** — manually control each joint when the arm is idle
- **Detailed OR environment** — operating table, anaesthesia machine, vital signs monitor, IV pole, crash cart, surgical lamp, and wall clock

## Requirements

- Python 3
- [browserbotics](https://pypi.org/project/browserbotics/)

## Installation

```bash
pip install browserbotics
```

## Usage

```bash
python surgry.py
```

Use the GUI buttons to control the robot:

| Button | Action |
|---|---|
| **Pick Scalpel** | Pick up the scalpel from the tray |
| **Pick Forceps** | Pick up the forceps from the tray |
| **Pick Suture** | Pick up the suture kit from the tray |
| **Drop on Table** | Place the held instrument on the operating table |
| **Return Home** | Move the arm back to its home position |
| **Reset All** | Return all instruments to the tray and reset the arm |

When no instrument is held, use the joint sliders (J1–J7) to move the arm manually.
