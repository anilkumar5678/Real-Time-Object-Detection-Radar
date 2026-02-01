# Real-Time Object Detection Radar

<div align="center">

![License](https://img.shields.io/badge/license-MIT-blue.svg)
![Arduino](https://img.shields.io/badge/Arduino-00979D?logo=arduino&logoColor=white)
![MATLAB](https://img.shields.io/badge/MATLAB-0076A8?logo=mathworks&logoColor=white)

A real-time 180° radar system integrating Arduino and MATLAB for precise object detection and visualization.

</div>

## 📋 Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Hardware Requirements](#hardware-requirements)
- [Software Requirements](#software-requirements)
- [Circuit Diagram](#circuit-diagram)
- [Installation](#installation)
- [Setup Instructions](#setup-instructions)
- [Usage](#usage)
- [How It Works](#how-it-works)
- [Project Structure](#project-structure)
- [Troubleshooting](#troubleshooting)
- [Future Enhancements](#future-enhancements)
- [Contributing](#contributing)
- [License](#license)
- [Acknowledgments](#acknowledgments)

## 🎯 Overview

The Real-Time Object Detection Radar is an innovative project that combines Arduino hardware with MATLAB software to create a functional radar system. Using an HC-SR04 ultrasonic sensor mounted on a servo motor, the system performs a 180° sweep to detect objects in its surroundings. The data is transmitted via serial communication to a MATLAB GUI that displays real-time object positions on a polar plot, providing an intuitive radar-like visualization.

This project is perfect for:
- Robotics enthusiasts and students
- Educational demonstrations of sensor integration
- Object detection and proximity sensing applications
- Learning about serial communication between hardware and software
- Understanding radar systems and data visualization

## ✨ Features

- **180° Scanning Range**: Full semicircular coverage for comprehensive object detection
- **Real-Time Visualization**: Instant polar plot display using MATLAB's powerful graphing capabilities
- **High-Performance Graphics**: Flicker-free visualization with optimized rendering
- **Serial Communication**: Efficient data streaming between Arduino and MATLAB
- **Hardware Alerts**: Integrated buzzer and vibration motor for proximity warnings
- **Distance Measurement**: Accurate object distance detection using ultrasonic technology
- **Customizable Parameters**: Adjustable detection range, sweep speed, and alert thresholds
- **User-Friendly Interface**: Intuitive MATLAB GUI for easy monitoring and control

## 🔧 Hardware Requirements

| Component | Specification | Quantity | Purpose |
|-----------|--------------|----------|---------|
| **Arduino Board** | Arduino Uno/Nano/Mega | 1 | Main microcontroller |
| **HC-SR04 Ultrasonic Sensor** | Range: 2cm - 400cm | 1 | Distance measurement |
| **Servo Motor** | SG90 or similar (180°) | 1 | Sensor rotation |
| **Buzzer** | 5V Active/Passive Buzzer | 1 | Audio alert |
| **Vibration Motor** | 3V-5V DC Motor | 1 | Haptic feedback (optional) |
| **Breadboard** | Standard size | 1 | Circuit prototyping |
| **Jumper Wires** | Male-to-Male, Male-to-Female | As needed | Connections |
| **Power Supply** | 5V/9V (USB or Battery) | 1 | Power source |
| **Resistors** | 220Ω (for LED indicators) | Optional | Circuit protection |

### Optional Components
- LED indicators for visual feedback
- Push buttons for manual control
- LCD display for standalone operation
- External power supply for stable operation

## 💻 Software Requirements

- **Arduino IDE** (Version 1.8.x or later)
  - Download: [https://www.arduino.cc/en/software](https://www.arduino.cc/en/software)
  
- **MATLAB** (Version R2016b or later recommended)
  - Required Toolboxes:
    - Instrument Control Toolbox (for serial communication)
    - Signal Processing Toolbox (optional, for data filtering)
  
- **Arduino Libraries** (if using advanced features):
  - Servo.h (built-in)
  - NewPing.h (optional, for enhanced ultrasonic control)

- **USB Drivers**:
  - CH340/CH341 drivers (for Arduino clones)
  - FTDI drivers (for FTDI-based Arduino boards)

## 🔌 Circuit Diagram

### Pin Connections

#### HC-SR04 Ultrasonic Sensor
```
HC-SR04          Arduino
--------         -------
VCC       -----> 5V
TRIG      -----> Digital Pin 9
ECHO      -----> Digital Pin 10
GND       -----> GND
```

#### Servo Motor
```
Servo            Arduino
-----            -------
VCC (Red)  ----> 5V
GND (Brown)----> GND
Signal (Orange)-> Digital Pin 3
```

#### Buzzer
```
Buzzer           Arduino
------           -------
Positive   ----> Digital Pin 7
Negative   ----> GND
```

#### Vibration Motor (Optional)
```
Motor            Arduino
-----            -------
Positive   ----> Digital Pin 8
Negative   ----> GND
```

### Circuit Schematic

```
                    +5V
                     |
          +----------+----------+
          |          |          |
      [HC-SR04]   [Servo]   [Buzzer]
          |          |          |
      +---+---+      |          |
      |       |      |          |
   Trig(9) Echo(10) Sig(3)   Pin(7)
      |       |      |          |
      +-------+------+----------+
                     |
                    GND
```

**Important Notes:**
- Ensure all components share a common ground
- The servo motor may require external power supply for stable operation under load
- Add a 100µF capacitor across the power supply for noise reduction (recommended)

## 📦 Installation

### Step 1: Install Arduino IDE

1. Download Arduino IDE from the official website
2. Install the software following the platform-specific instructions
3. Connect your Arduino board via USB
4. Select the correct board and port from Tools menu

### Step 2: Install MATLAB

1. Install MATLAB with the required toolboxes
2. Verify Instrument Control Toolbox installation:
   ```matlab
   ver('instrument')
   ```

### Step 3: Set Up Hardware

1. Assemble the circuit according to the circuit diagram
2. Mount the HC-SR04 sensor on the servo motor arm
3. Secure all connections on the breadboard
4. Connect the Arduino to your computer via USB

## 🚀 Setup Instructions

### Arduino Configuration

1. **Upload the Arduino Code:**
   ```arduino
   // Sample Arduino code structure
   #include <Servo.h>
   
   Servo radarServo;
   const int trigPin = 9;
   const int echoPin = 10;
   const int servoPin = 3;
   const int buzzerPin = 7;
   
   void setup() {
     Serial.begin(9600);
     pinMode(trigPin, OUTPUT);
     pinMode(echoPin, INPUT);
     pinMode(buzzerPin, OUTPUT);
     radarServo.attach(servoPin);
   }
   
   void loop() {
     // Scanning and measurement code
   }
   ```

2. **Verify Serial Communication:**
   - Open Serial Monitor (Tools > Serial Monitor)
   - Set baud rate to 9600
   - Check for incoming data

### MATLAB Configuration

1. **Find Arduino COM Port:**
   ```matlab
   serialportlist("available")
   ```

2. **Configure Serial Communication:**
   ```matlab
   % Replace 'COM3' with your Arduino's port
   arduinoPort = serialport('COM3', 9600);
   configureTerminator(arduinoPort, "LF");
   flush(arduinoPort);
   ```

3. **Initialize Radar Display:**
   ```matlab
   % Create polar plot figure
   figure('Name', 'Object Detection Radar', 'NumberTitle', 'off');
   ax = polaraxes;
   ax.ThetaLim = [0 180];
   ax.RLim = [0 100];
   ```

## 🎮 Usage

### Starting the Radar System

1. **Power up the Arduino:**
   - Connect Arduino to power source
   - Verify LED indicators (if installed)

2. **Launch MATLAB GUI:**
   ```matlab
   % Run the main radar script
   run('radar_main.m')
   ```

3. **Begin Scanning:**
   - The servo will start rotating from 0° to 180°
   - Objects will appear as points on the polar plot
   - Distance is shown on the radial axis
   - Angle is shown on the angular axis

4. **Monitoring:**
   - Watch the real-time visualization
   - Listen for buzzer alerts when objects are detected nearby
   - Check distance readings in the MATLAB console

### Adjusting Parameters

- **Detection Range:** Modify the maximum distance threshold in Arduino code
- **Sweep Speed:** Adjust servo delay between angles
- **Alert Distance:** Change buzzer trigger distance
- **Display Refresh Rate:** Modify MATLAB plot update frequency

### Stopping the System

1. Close the MATLAB GUI or press Ctrl+C
2. The Arduino will continue running until powered off
3. Disconnect serial port in MATLAB:
   ```matlab
   delete(arduinoPort);
   clear arduinoPort;
   ```

## 🔍 How It Works

### System Architecture

The radar system operates through a coordinated sequence of actions:

1. **Sensor Positioning:**
   - Servo motor rotates the ultrasonic sensor incrementally (typically 1-5° steps)
   - Position angles range from 0° to 180°

2. **Distance Measurement:**
   - HC-SR04 sends ultrasonic pulse (40 kHz)
   - Pulse reflects off objects and returns to sensor
   - Time-of-flight is measured
   - Distance = (Time × Speed of Sound) / 2

3. **Data Transmission:**
   - Arduino formats data as: `angle,distance`
   - Data sent via serial at 9600 baud
   - Example: `45,32` means 32cm at 45°

4. **Visualization:**
   - MATLAB receives and parses serial data
   - Converts polar coordinates (angle, distance)
   - Updates polar plot in real-time
   - Old data points fade or are cleared for clarity

5. **Alert System:**
   - Arduino monitors distance readings
   - If distance < threshold: activate buzzer/vibration
   - Provides immediate feedback without MATLAB dependency

### Mathematical Foundation

**Distance Calculation:**
```
Distance (cm) = (Echo_Time × 343 m/s × 100 cm/m) / 2
              = Echo_Time × 0.01715 cm/µs
```

**Coordinate Conversion:**
```
Cartesian X = Distance × cos(Angle)
Cartesian Y = Distance × sin(Angle)
```

### Performance Considerations

- **Scan Rate:** Approximately 1-2 seconds per full sweep (depends on servo speed and measurements per angle)
- **Accuracy:** ±3cm (HC-SR04 specification)
- **Range:** 2cm to 400cm (practical range: 2cm to 200cm)
- **Resolution:** Limited by servo step size (typically 1-5°)

## 📁 Project Structure

```
Real-Time-Object-Detection-Radar/
│
├── arduino/
│   ├── radar_scanner.ino          # Main Arduino sketch
│   ├── config.h                    # Configuration parameters
│   └── README.md                   # Arduino-specific documentation
│
├── matlab/
│   ├── radar_main.m                # Main MATLAB script
│   ├── radar_gui.m                 # GUI interface
│   ├── serial_communication.m      # Serial port handling
│   ├── plot_radar.m                # Visualization functions
│   └── README.md                   # MATLAB-specific documentation
│
├── docs/
│   ├── circuit_diagram.png         # Visual circuit diagram
│   ├── assembly_guide.pdf          # Step-by-step assembly
│   ├── troubleshooting.md          # Common issues and solutions
│   └── user_manual.pdf             # Complete user guide
│
├── examples/
│   ├── basic_radar.ino             # Simplified Arduino example
│   ├── enhanced_features.ino       # Advanced features demo
│   └── matlab_examples/            # MATLAB code samples
│
├── LICENSE                         # MIT License
└── README.md                       # This file
```

## 🛠️ Troubleshooting

### Common Issues and Solutions

#### 1. No Serial Connection
**Problem:** MATLAB cannot find Arduino port

**Solutions:**
- Verify Arduino is connected via USB
- Check Device Manager (Windows) or `ls /dev/tty*` (Mac/Linux)
- Install correct USB drivers (CH340 or FTDI)
- Try different USB cable or port
- Restart Arduino IDE and MATLAB

#### 2. Inaccurate Distance Readings
**Problem:** Sensor shows erratic or incorrect distances

**Solutions:**
- Check sensor connections (especially Echo and Trig pins)
- Ensure sensor has clear line of sight
- Avoid soft/angled surfaces that absorb ultrasonic waves
- Add capacitor for power stabilization
- Limit maximum range to 200cm for better accuracy

#### 3. Servo Jittering
**Problem:** Servo movement is unstable or jittery

**Solutions:**
- Use external 5V power supply for servo
- Add 100µF capacitor across servo power pins
- Reduce servo speed in code
- Check for loose connections
- Ensure sufficient current supply (500mA minimum)

#### 4. MATLAB Plot Not Updating
**Problem:** Visualization freezes or doesn't refresh

**Solutions:**
- Add `drawnow` command in MATLAB loop
- Reduce plot update frequency
- Clear old data points periodically
- Check for MATLAB warnings in console
- Verify serial buffer isn't full

#### 5. No Buzzer Sound
**Problem:** Buzzer doesn't activate despite objects nearby

**Solutions:**
- Verify buzzer polarity
- Check pin connections (ensure using correct digital pin)
- Test buzzer separately with `digitalWrite(buzzerPin, HIGH)`
- Try different buzzer (some require specific PWM frequencies)
- Adjust distance threshold in code

#### 6. Serial Buffer Overflow
**Problem:** Data transmission becomes slow or stops

**Solutions:**
```matlab
% Clear buffer before reading
flush(arduinoPort);
% Limit data rate in Arduino
delay(50); // Add delay between transmissions
```

### Debugging Tips

1. **Use Serial Monitor:**
   ```arduino
   Serial.print("Angle: ");
   Serial.print(angle);
   Serial.print(" Distance: ");
   Serial.println(distance);
   ```

2. **Test Components Individually:**
   - Test servo sweep without sensor
   - Test sensor readings at fixed position
   - Verify serial communication with simple messages

3. **Check Power Supply:**
   - Measure voltage with multimeter (should be 4.8-5.2V)
   - Ensure adequate current capacity
   - Look for brownout symptoms

## 🚀 Future Enhancements

### Planned Features

- [ ] **360° Full Coverage:** Add second servo for complete scanning
- [ ] **Object Tracking:** Implement algorithm to track moving objects
- [ ] **Data Logging:** Save scan data to file for later analysis
- [ ] **Mobile App Integration:** Control and monitor via smartphone
- [ ] **Multiple Sensor Support:** Use sensor array for improved accuracy
- [ ] **Machine Learning:** Object classification using ML algorithms
- [ ] **3D Visualization:** Add elevation angle for 3D mapping
- [ ] **Wireless Communication:** Replace serial with Bluetooth/WiFi
- [ ] **Autonomous Navigation:** Integration with robot chassis
- [ ] **Web Interface:** Browser-based control and monitoring

### Possible Improvements

- **Enhanced GUI:** Add controls for parameters, start/stop buttons, and settings
- **Data Filtering:** Implement Kalman filter for noise reduction
- **Multi-Color Display:** Color-code objects by distance or size
- **Sound Effects:** Different tones for different distance ranges
- **Recording Mode:** Capture and replay scan sessions
- **Alert Zones:** Configure custom alert regions
- **Performance Metrics:** Display scan rate, detection count, etc.

## 🤝 Contributing

Contributions are welcome! Here's how you can help:

### Ways to Contribute

1. **Report Bugs:** Open an issue with detailed description
2. **Suggest Features:** Share your ideas for improvements
3. **Submit Pull Requests:** 
   - Fork the repository
   - Create a feature branch (`git checkout -b feature/AmazingFeature`)
   - Commit your changes (`git commit -m 'Add some AmazingFeature'`)
   - Push to the branch (`git push origin feature/AmazingFeature`)
   - Open a Pull Request

### Contribution Guidelines

- Follow existing code style and conventions
- Comment your code clearly
- Test your changes thoroughly
- Update documentation as needed
- Add examples for new features

### Areas Needing Help

- Documentation improvements
- Code optimization
- Additional language support
- Testing on different hardware configurations
- Creating tutorial videos
- Developing example projects

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

### MIT License Summary

- ✅ Commercial use
- ✅ Modification
- ✅ Distribution
- ✅ Private use
- ❌ Liability
- ❌ Warranty

## 🙏 Acknowledgments

- **Arduino Community:** For extensive documentation and support
- **MathWorks:** For MATLAB's powerful visualization capabilities
- **Open Source Contributors:** For libraries and code examples
- **Robotics Community:** For inspiration and project ideas

### Inspiration and References

- Radar systems in aviation and maritime navigation
- Ultrasonic sensor applications in robotics
- MATLAB data visualization techniques
- Arduino servo control best practices

## 📞 Contact and Support

- **Issues:** Report bugs or request features via [GitHub Issues](https://github.com/anilkumar5678/Real-Time-Object-Detection-Radar/issues)
- **Discussions:** Join conversations in [GitHub Discussions](https://github.com/anilkumar5678/Real-Time-Object-Detection-Radar/discussions)
- **Email:** dhulipallaanil5191@gmail.com

## ⭐ Show Your Support

If you found this project helpful, please consider:
- Giving it a ⭐ on GitHub
- Sharing it with others
- Contributing to its development
- Using it in your own projects

---

<div align="center">

**Made with ❤️ for robotics enthusiasts**

[Report Bug](https://github.com/anilkumar5678/Real-Time-Object-Detection-Radar/issues) · [Request Feature](https://github.com/anilkumar5678/Real-Time-Object-Detection-Radar/issues) · [Documentation](https://github.com/anilkumar5678/Real-Time-Object-Detection-Radar/wiki)

</div>
