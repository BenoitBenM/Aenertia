# Website concept

There are two websites. 

- One simple website allowing remote tuning of the PID without having to constantly connect the ESP32 to a laptop.
  
- The main website which should allow a wide range of features and possibly implement a cloud.

<br>

## PID turning website

This is a simple version of the website that should allow to tune the PID by transmitting PID values to the Raspberry Pi.   


## Main website

This design should implement a manual and an autonomous mode, as well as a PID control tab (Same logic as the above design).

**It should have tabs, for different control modes and PID tuning**

  - The website should always display the current battery level and power consumption

  - When clicking on a separate tab it would be good to have a warning message to confirm you want to switch mode (avoids misclicks)

<br>
    
**<ins>In manual mode:</ins> The robot should be able to go in any direction:**

  - Add left right forward and backward buttons. Link those buttons to a keyboard control. 
                
  - Display the mapping on the UI (Advanced feature). Optionally display the camera view
                
  - Have a "Register Location" button which stores the current coordinates on the map and allows the user to type a label for it.

<br>

**<ins>In autonomous mode:</ins> The robot should  be able to enter two modes: Follower and guide.**

  - **In follow mode,** it follows a person and keeps on mapping the area.
      - Display the mapping on the UI (Advanced feature). Optionally display the camera view   

  - **In guide mode,** the user is presented with the key locations stored and able to click on any of them.
      - Display the mapping on the UI (Advanced feature). Optionally display the camera view   
      - When clicking on one of these options the information should be transmitted to the PI.
        
<br>

### Server and data transmission

The server should be able to transmit much more than high or low code. It should be able to send strings to the Raspberry Pi and receive strings from it in inverted communication. 

```
                                            ┌────────────────────────────┐
                                            │       Web UI (Browser)     │
                                            │      HTML/JS interface     │
                                            │    Sends string commands   │
                                            │      e.g., "Set_Manual"    │
                                            │                            │
                                            └────────────┬───────────────┘
                                                         │ MQTT (publish)
                                                         ▼
                                        ┌────────────────────────────────────┐
                                        │       MQTT Broker (Mosquitto)      │
                                        │     Running on Raspberry Pi 5      │
                                        │       Topic: "robot/command"       │
                                        │     Also handles "robot/status"    │
                                        └────────────┬───────────────────────┘
                                                     │ MQTT (subscribe)
                                                     ▼
                                    ┌──────────────────────────────────────────────┐
                                    │         Raspberry Pi 5 (Robot Brain)         │
                                    │        Subscribes to "robot/command"         │
                                    │          Parses received strings:            │
                                    │            e.g., "go_to kitchen"             │
                                    │        Executes ROS2 logic: (BENOIT)         │
                                    │             - nav2 navigation                │
                                    │             - person tracking                │
                                    │             - SLAM                           │
                                    │     Sends motor/sensor commands to ESP32     │
                                    │     via USB Serial                           │
                                    └────────────┬─────────────────────────────────┘
                                                 │ Serial (USB / UART)      ** I ll take care of the (BENOIT)
                                                 ▼
                                    ┌──────────────────────────────────────────────┐
                                    │             ESP32 Microcontroller            │
                                    │        Runs a single general firmware        │
                                    │           Parses string commands:            │
                                    │            e.g., "SET_PWM 40 40"             │
                                    │           "STOP"                             │
                                    │           "SET_PID 1.2 0.1 0.01"             │
                                    │        Controls motors, reads sensors        │
                                    │           Sends feedback if needed           │
                                    └──────────────────────────────────────────────┘
```


The opposite communication should also work from the PI to the UI

```
                                    ┌──────────────────────────────────────────────┐
                                    │              ESP32 Microcontroller           │
                                    │                                              │
                                    │           Sends to Raspberry Pi:             │
                                    │        • IMU readings                        │
                                    │        • Ultrasonic/ToF sensor data          │
                                    │        • Battery voltage (analog read)       │
                                    │                                              │
                                    │         Receives from Raspberry Pi:          │
                                    │        • Motor commands (e.g., SET_PWM)      │
                                    │        • PID tuning values                   │
                                    │        • Mode switches (e.g., STOP, FOLLOW)  │
                                    │                                              │
                                    │                                              │
                                    └────────────┬─────────────────────────────────┘
                                                 │ USB Serial (bi-directional)
                                                 ▼
                                    ┌──────────────────────────────────────────────┐
                                    │           Raspberry Pi 5 (Robot Brain)       │
                                    │                                              │
                                    │        Publishes robot state info:           │
                                    │        • Live SLAM map updates               │
                                    │        • Camera video + OpenCV detection     │
                                    │        • Battery status (from ESP32)         │
                                    │        • Sensor readings                     │
                                    │        • Current robot mode                  │
                                    │                                              │
                                    │  Uses ROS2 + MQTT/WebSocket bridge           │
                                    └────────────┬─────────────────────────────────┘
                                                 │ MQTT/WebSocket (publish)
                                                 ▼
                                    ┌──────────────────────────────────────────────┐
                                    │           MQTT Broker (Mosquitto)            │
                                    │                                              │
                                    │       Running on Pi or external server       │
                                    │            Topics published:                 │
                                    │        • robot/status/map                    │
                                    │        • robot/status/video                  │
                                    │        • robot/status/battery                │
                                    │        • robot/status/sensors                │
                                    │        • robot/status/mode                   │
                                    └────────────┬─────────────────────────────────┘
                                                 │ MQTT/WebSocket (subscribe)
                                                 ▼
                                    ┌──────────────────────────────────────────────┐
                                    │              Web UI (Browser)                │
                                    │                                              │
                                    │              Displays live:                  │
                                    │        • SLAM map (canvas)                   │
                                    │        • Camera feed (MJPEG/WebSocket)       │
                                    │        • Battery level                       │
                                    │        • Sensor readings                     │
                                    │        • Current robot mode                  │
                                    └──────────────────────────────────────────────┘


```

MQTT is better than HTTP for robot communication because it's:

Faster (low latency)

Lighter (small overhead)

It is Publish and subscribe drivent (event driven) instead of request driven which is much better for a real time system



### Notes:

* The Web UI never talks directly to the ESP32.
* The Raspberry Pi 5 handles all logic and command parsing.
* Communication flows from UI → MQTT → Pi → ESP32.
* Commands are sent as strings (not binary or analog values).
* Pi uses USB serial to send commands to ESP32.
* ESP32 firmware handles conditionally reacting to each command.
