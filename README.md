# CookieProject - CPi Class

## Overview

This project uses a Raspberry Pi with OpenCV to detect markers and track object positions in real-time. It employs ArUco markers for precise location tracking and uses a serial interface to communicate coordinates to an external device. The main application is designed to detect cookies on a conveyor belt and transmit position data to control an external robotic system.

## Features

- **Marker Detection:** Utilizes OpenCV’s ArUco library to detect and track specific markers on the camera feed.
- **Real-Time Position Tracking:** Continuously reads from a video feed, detecting markers and calculating the center point of objects of interest.
- **Contour Detection:** Applies HSV filtering and morphological transformations to isolate regions of interest and detect contours.
- **Serial Communication:** Sends detected position coordinates over a serial connection to an external device for further processing.
- **Customizable Settings:** Provides various configurable parameters, such as HSV color thresholds, camera intrinsics, and serial port settings.

## Code Structure

- **CPi Class:** Handles marker detection, contour analysis, and serial communication.
  - **`__init__`:** Initializes camera, serial port, and configurable parameters (e.g., camera matrix, distortion coefficients, HSV range).
  - **`get_position`:** Captures frames, detects markers, and draws offset dots for visualization. Detects ArUco markers and draws contours around the object.
  - **`draw_offset_dots`:** Calculates and draws dots with specified offsets for visual feedback on marker positions.
  - **`get_contour`:** Processes a region of interest, applies HSV filtering, detects contours, and calculates the object center.
  - **`send_command`:** Sends the coordinates of the detected object’s center over a serial connection.
- **`main` function:** Instantiates `CPi`, continuously calls `get_position` to process frames, and ensures cleanup on exit.

## Installation

1. **Prerequisites:**
   - Python 3.x
   - OpenCV (with ArUco module)
   - Numpy
   - PySerial

2. **Setup:**
   ```bash
   pip install opencv-python-headless numpy pyserial

## Usage
**Run the main script:**
   ```bash
     python CookieProjectPi.py 
  ```
**Press q to exit the application.**

## Configuration
- **Serial Port:** Default is /dev/ttyAMA0 at 9600 baud rate.
- **HSV Thresholds:** Set in the CPi class, allowing customization for target object detection.

