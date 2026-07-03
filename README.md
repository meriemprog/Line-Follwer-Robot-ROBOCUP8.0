<img width="282" height="751" alt="map" src="https://github.com/user-attachments/assets/90800433-9444-409d-9c55-69cb765ee236" />
Readme · MDPID Line-Follower Robot (ESP32)

An autonomous line-following robot built for competition, running on an ESP32 microcontroller. The robot uses a 6-sensor IR array with a PID control loop to track a line, handles inverted-color (color-swap) track sections, and includes timed maneuver phases tuned for a specific competition course.

Features


6-channel IR sensor array with automatic calibration (white surface / black line) at startup
PID line-tracking control (tunable Kp, Ki, Kd) with anti-windup protection on the integral term
Weighted error calculation — sensor weights can be reconfigured per track segment for sharper or gentler corrections
Timed maneuver sequencing — the main loop uses millis()-based time windows to switch between different speed/PID/weight profiles for different sections of the course (start, straights, curves, etc.)
Inverted-line detection & handling — a dedicated routine (readSensorsinv) detects and follows color-swapped track sections where black/white are reversed
Automatic stop condition — the robot halts when it detects a full stop-line (all 6 sensors active) after completing the course
Push-button start — robot waits for a button press before beginning its run, allowing manual placement on the track
Onboard LED status indicator — used during calibration and inverted-line phases


Hardware

ComponentDescriptionMicrocontrollerESP32Line sensors6x IR/reflectance sensors (analog)Motor driver2x DC motors, H-bridge style (forward/reverse PWM pins per motor)Push buttonUsed to trigger the start of the runOnboard LEDStatus indicator during calibration

Pinout

SignalPinLeft Motor ForwardGPIO 4Left Motor BackwardGPIO 16Right Motor ForwardGPIO 17Right Motor BackwardGPIO 18Status LEDGPIO 2Start ButtonGPIO 34IR Sensors (0–5)GPIO 32, 33, 25, 26, 27, 13

How It Works


Calibration — On boot, the robot is placed on a white surface, then on the black line, so each sensor's min/max reflectance values are recorded (~13s process). Calibration values are printed over Serial for debugging.
Start trigger — The robot waits for a button press before beginning its run.
Line tracking loop — Each cycle:

Reads and binarizes all 6 sensors against the calibrated threshold
Computes a weighted position error from active sensors
Runs the PID controller on the error to produce a correction output
Converts the PID output into differential left/right motor speeds



Course-specific phases — The main loop() uses elapsed time (millis() - t0) to switch between different weight sets, base speeds, and Kp values for different segments of the known competition track.
Inverted-color handling — If the sensor pattern matches a color-inverted section (outer sensors triggered, center sensors not), the robot switches to an inverted read/response routine until it exits that section.
Finish detection — Once the robot detects a full black stop-line (all 6 sensors active) after completing the inverted section, it stops both motors and halts execution.


Configuration

Key tunable parameters near the top of the file:

cppfloat Kp = 43;
float Ki = 0.0005;
float Kd = 8;

int baseSpeed = 150;   // base cruising speed (0–255)
int maxSpeed  = 255;
int minSpeed  = 30;

These — along with the per-segment weights arrays inside loop() — are the main values to retune when adapting the robot to a new track layout.

Usage


Flash competition_code.ino to the ESP32 via the Arduino IDE (select the correct ESP32 board profile).
Open the Serial Monitor at 115200 baud to view calibration output and debug logs.
Power on the robot with it placed on a white area of the track; wait for the LED to turn on/off (white calibration phase).
Move the robot onto the black line when prompted; wait for calibration to complete.
Press the start button to begin the run.
The robot will follow the line autonomously and stop automatically at the finish/stop-line.


Notes


Some code comments and variable names are in French/Darija (e.g. t_lekhra, khrouj, test_tri9_twil), reflecting the original development context — functionally these track timing and state flags for the inverted-line exit sequence.
The timed phases in loop() are tuned for a specific competition track and will need to be re-calibrated (both timings and sensor weights) if reused on a different course. [Uploading CDC-SUIVEUR.pdf…]()


https://github.com/user-attachments/assets/bd51bef6-24f9-4c43-a9d5-c52722ca410f


