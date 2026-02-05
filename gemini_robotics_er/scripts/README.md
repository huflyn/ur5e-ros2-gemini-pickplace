# Gemini API Scripts
This folder contains scripts to test and demonstrate the capabilities of the Gemini Robotics API.

---

### ``gemini_api_test.py``
A script to verify the API is working properly and your API key is set up correctly.

Run the script using:
```bash
python3 gemini_api_test.py 
```
Response (example):
```bash
Yes, I am here and ready to assist. What can I help you with today?
```

---

**``gemini_api_request.py``**: A script to make a custom request to the Gemini API over the command line.

Run the script using:
```bash
python3 gemini_api_request.py "Your custom prompt here"
```
Response (example):
```bash
Using prompt: 'Explain very briefly your capabilities and use cases in a project with a ur5e and a realsense camera and apick an place task (lego bricks)'
---
Based on the UR5e and RealSense camera integration:

**Capabilities:**

1.  **3D Pose Estimation:** Process RealSense depth data to identify the precise 3D location and orientation of individual Lego bricks, even in unstructured piles.
2.  **Pick Planning:** Generate accurate motion paths and calculate inverse kinematics for the UR5e arm to approach, grasp, and move specific bricks without collisions.

**Use Cases:**

*   **Unstructured Pick-and-Place:** Sorting Lego bricks from a jumbled pile into designated containers (e.g., by color or size).
*   **Assembly Automation:** Picking specific bricks from various locations and placing them precisely for automated construction or kitting tasks.
```
