# Ultrasonic Radar System

> A low‑cost, Arduino‑based ultrasonic radar that scans a 180° field and visualizes object positions in real time using Processing.

---

## 📋 Table of Contents

1. [Features](#features)  
2. [Repository Structure](#repository-structure)  
3. [Hardware Setup](#hardware-setup)  
4. [Arduino Code](#arduino-code)  
5. [Processing Visualization](#processing-visualization)  
6. [Documentation](#documentation)  
7. [License](#license)  

---

## 🚀 Features

- **Distance Measurement**  
  Detects objects between 2 cm and 400 cm using an HC‑SR04 ultrasonic sensor.
- **Rotational Scanning**  
  A servo motor sweeps across 0°–180° in 1° increments.
- **Real‑time Visualization**  
  A Processing sketch renders radar-style displays with fading trails and color-coding by distance.
- **Modular Design**  
  Easily extensible for 360° scanning, wireless communication, or multiple sensors.

---

## 🗂 Repository Structure

```
ultrasonic-radar/
├── docs/                  # Thesis PDF & design documentation
│   └── RTP-final.pdf
├── hardware/              # Circuit diagrams & 3D mounts
│   ├── circuit_diagram.png
│   └── servo_bracket.stl
├── arduino/               # Arduino sketch
│   └── ultrasonic_radar.ino
├── processing/            # Processing visualization code
│   └── radar_visualization.pde
├── images/                # Project photos & diagrams
│   ├── system_photo.jpg
│   └── pcb_layout.png
├── LICENSE                # MIT License
└── README.md              # Project overview
```

---

## ⚙️ Hardware Setup

1. **Arduino Uno**  
2. **HC‑SR04 Ultrasonic Sensor**  
   - VCC → 5 V  
   - GND → GND  
   - Trig → D9  
   - Echo → D10  
3. **SG90 Servo Motor**  
   - VCC → External 5 V (common GND)  
   - Signal → D6  
4. **Power Stabilization**  
   Place a 220 µF capacitor across the servo’s power rails to smooth current spikes.

![image](https://github.com/user-attachments/assets/17a80fe0-577e-4033-9dbb-7d12aa6875c7)


---

## 💻 Arduino Code

**File:** `arduino/ultrasonic_radar.ino`

```cpp
#include <Servo.h>

const int trigPin = 9;
const int echoPin = 10;
const int servoPin = 6;

Servo sweepServo;

void setup() {
  Serial.begin(115200);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  sweepServo.attach(servoPin);
}

long measureDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000);  // timeout: 30 ms
  return (duration > 0) ? (duration / 58.2) : -1;  // distance in cm
}

void sweep() {
  for (int angle = 0; angle <= 180; angle++) {
    sweepServo.write(angle);
    delay(15);  // allow servo movement
    long dist = measureDistance();
    if (dist < 0 || dist > 400) dist = 0;  // sanitize
    Serial.print("A:");
    Serial.print(angle);
    Serial.print(" D:");
    Serial.println(dist);
  }
}

void loop() {
  sweep();                  // forward sweep
  reverse();                // backward sweep
}

void reverse() {
  for (int angle = 180; angle >= 0; angle--) {
    sweepServo.write(angle);
    delay(15);
    long dist = measureDistance();
    if (dist < 0 || dist > 400) dist = 0;
    Serial.print("A:");
    Serial.print(angle);
    Serial.print(" D:");
    Serial.println(dist);
  }
}
```

---

## 📊 Processing Visualization

**File:** `processing/radar_visualization.pde`

```java
import processing.serial.*;

Serial myPort;
final int MAX_RANGE = 400;  // centimeters
PVector center;
int currentAngle = 0;

void setup() {
  size(600, 600);
  center = new PVector(width/2, height/2);
  println(Serial.list());
  myPort = new Serial(this, Serial.list()[0], 115200);
  myPort.bufferUntil('
');
  frameRate(30);
}

void draw() {
  background(0);
  drawRadarGrid();
  stroke(0, 255, 0);
  line(center.x, center.y,
       center.x + cos(radians(currentAngle)) * width/2,
       center.y + sin(radians(currentAngle)) * height/2);
}

void serialEvent(Serial p) {
  String line = p.readStringUntil('\n').trim();
  if (line.startsWith("A:")) {
    String[] parts = split(line, ' ');
    int angle = int(split(parts[0], ':')[1]);
    int dist  = int(split(parts[1], ':')[1]);
    plotPoint(angle, dist);
    currentAngle = angle;
  }
}

void drawRadarGrid() {
  stroke(80);
  noFill();
  ellipse(center.x, center.y, width*0.8, height*0.8);
  for (int a = 0; a < 360; a += 30) {
    float x2 = center.x + cos(radians(a)) * width*0.4;
    float y2 = center.y + sin(radians(a)) * height*0.4;
    line(center.x, center.y, x2, y2);
  }
}

void plotPoint(int angle, int dist) {
  float r = map(dist, 0, MAX_RANGE, 0, width*0.4);
  float x = center.x + cos(radians(angle-90)) * r;
  float y = center.y + sin(radians(angle-90)) * r;
  noStroke();
  fill(map(dist,0,MAX_RANGE,0,255), 255-map(dist,0,MAX_RANGE,0,255), 0);
  ellipse(x, y, 8, 8);
}
```

---

## 📄 Documentation

- **Project report & design rationale:** `docs/RTP-final.pdf`

---

## ⚖️ License

This project is licensed under the MIT License. See [LICENSE](LICENSE) for details.
