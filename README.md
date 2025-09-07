\# Project - Robotics Programming (Partial Exam 2)



This repository contains the implementation of the \*\*Second Partial Exam\*\* for the \*Robotics Programming\* course using \*\*Webots\*\*.



\## 📌 Description

The project consists of building a \*\*4x4 meters maze\*\* in Webots, including:

\- A floor with sand texture and a perimeter wall with brick texture.

\- Internal walls with a different but also rough texture.

\- Three main objects:

&nbsp; 1. A composite object (slide).

&nbsp; 2. A pre-existing object in Webots.

&nbsp; 3. A bridge (stone) with enough clearance for the robot to go underneath.



In addition, a \*\*mobile robot\*\* was added with a camera and distance sensors. The robot:

\- Detects direction arrows placed strategically in the maze.

\- Turns left or right based on the detected arrow color.

\- Avoids collisions with walls using distance sensors.



\## ⚙️ Controller

The controller implements the robot’s autonomous navigation logic:

\- Uses \*\*distance sensors\*\* to avoid collisions.

\- Uses the \*\*camera\*\* to identify direction arrows.

\- Moves autonomously until it exits the maze.



\## 📁 Repository Structure

\- `worlds/Parcial2.wbt` → Maze world.

\- `worlds/textures/` → Folder containing the textures used.

\- `controllers/driving\_robot/driving\_robot.cpp` → Robot controller (with explanatory comments).



\## ▶️ How to Run

1\. Open the project in \*\*Webots\*\*.

2\. Load the world `Parcial2.wbt`.

3\. Run the controller `driving\_robot.cpp` (Remove intermediate build files and build the current project).

4\. Watch the robot navigate autonomously through the maze.



\## 🎥 Demo Video

The simulation video can be found at the following link:  

👉 \* https://youtu.be/uGfmg9Skfok \*



---



✍️ \*\*Author:\*\* \*Patricia Sarahi Jimenez-Leura\*  

📅 \*\*Due date:\*\* April 3, 2025



