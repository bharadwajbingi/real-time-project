# Ultrasonic Radar System

A low‑cost, Arduino‑based ultrasonic radar that scans a 180° field and visualizes object positions in real time using Processing.

![system_photo.jpg](images/system_photo.jpg)

## 🚀 Features

- **Distance Measurement**  
  Uses HC‑SR04 ultrasonic sensor to detect objects from 2 cm to 400 cm.
- **Rotational Scanning**  
  Servo motor sweeps from 0°–180° in 1° increments.
- **Real‑time Visualization**  
  Processing sketch renders a radar‑style display with fading trails and color‑coding by range.
- **Modular Design**  
  Easily extendable to 360° scanning, wireless comms, or multi‑sensor fusion.

## 📁 Repository Structure

```
ultrasonic‑radar/
├── README.md
├── docs/                  ← Thesis PDF & design docs  
├── hardware/              ← Circuit diagrams, 3D‑printed mounts  
├── arduino/               ← Arduino sketch  
├── processing/            ← Processing visualization code  
├── images/                ← Photos and diagrams  
└── LICENSE                ← MIT License  
```

## ⚙️ Hardware Setup

1. **Arduino Uno**  
2. **HC‑SR04 Ultrasonic Sensor**  
   - VCC → 5 V  
   - GND → GND  
   - Trig → D9  
   - Echo → D10  
3. **SG90 Servo Motor**  
   - VCC → External 5 V (shared GND)  
   - Signal → D6  
4. Add a 220 µF capacitor across the servo power rails to smooth out current spikes.

![circuit_diagram.png](hardware/circuit_diagram.png)

## 💻 Arduino Code

File: `arduino/ultrasonic_radar.ino`

\`\`\`cpp
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
  long duration = pulseIn(echoPin, HIGH, 30000); // timeout 30 ms
  return duration > 0 ? duration / 58.2 : -1;    // cm
}

void loop() {
  for (int angle = 0; angle <= 180; angle += 1) {
    sweepServo.write(angle);
    delay(15);                          // allow servo to reach position
    long dist = measureDistance();     // in cm
    if (dist < 0 || dist > 400) dist = 0; // sanitize out‑of‑range
    Serial.print("A:"); Serial.print(angle);
    Serial.print(" D:"); Serial.println(dist);
  }
  for (int angle = 180; angle >= 0; angle -= 1) {
    sweepServo.write(angle);
    delay(15);
    long dist = measureDistance();
    if (dist < 0 || dist > 400) dist = 0;
    Serial.print("A:"); Serial.print(angle);
    Serial.print(" D:"); Serial.println(dist);
  }
}
\`\`\`

## 📊 Processing Visualization

File: `processing/radar_visualization.pde`

\`\`\`java
import processing.serial.*;

Serial myPort;
int maxRange = 400;    // cm
PVector center;
int sweepAngle = 0;

void setup() {
  size(600, 600);
  center = new PVector(width/2, height/2);
  println(Serial.list());
  // change the index [0] to your COM port
  myPort = new Serial(this, Serial.list()[0], 115200);
  myPort.bufferUntil('
');
  frameRate(30);
}

void draw() {
  background(0);
  drawGrid();
  stroke(0,255,0);
  line(center.x, center.y,
       center.x + cos(radians(sweepAngle))*width/2,
       center.y + sin(radians(sweepAngle))*height/2);
}

void serialEvent(Serial p) {
  String line = p.readStringUntil('
').trim();
  if (line.startsWith("A:")) {
    String[] parts = split(line, ' ');
    int angle = int(split(parts[0], ':')[1]);
    int dist  = int(split(parts[1], ':')[1]);
    plotPoint(angle, dist);
    sweepAngle = angle;
  }
}

void drawGrid() {
  stroke(80);
  noFill();
  ellipse(center.x, center.y, width*0.8, height*0.8);
  for (int a = 0; a < 360; a += 30) {
    float x2 = center.x + cos(radians(a))*width*0.4;
    float y2 = center.y + sin(radians(a))*height*0.4;
    line(center.x, center.y, x2, y2);
  }
}

void plotPoint(int angle, int dist) {
  float r = map(dist, 0, maxRange, 0, width*0.4);
  float x = center.x + cos(radians(angle-90)) * r;
  float y = center.y + sin(radians(angle-90)) * r;
  noStroke();
  fill(map(dist,0,maxRange,0,255), 255-map(dist,0,maxRange,0,255), 0);
  ellipse(x, y, 8, 8);
}
\`\`\`

## 📄 Documentation

- Full project report and design rationale in `docs/RTP‑final.pdf`.

## ⚖️ License

This project is released under the MIT License. See [LICENSE](LICENSE) for details.
