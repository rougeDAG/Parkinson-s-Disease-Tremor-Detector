# 🧠 **Parkinson’s Tremor Detection System**

Developed by [rougeDAG](https://github.com/rougeDAG/)
  
**Under the guidance of:** Dr. Gugapriya, School of Electronics Engineering, VIT Chennai  

---

## 📜 **Table of Contents**
- [Overview](https://github.com/rougeDAG/Parkinson-s-Disease-Tremor-Detector?tab=readme-ov-file#-overview) 
- [Problem Statement](https://github.com/rougeDAG/Parkinson-s-Disease-Tremor-Detector?tab=readme-ov-file#-problem-statement)  
- [Proposed Solution](http://github.com/rougeDAG/Parkinson-s-Disease-Tremor-Detector?tab=readme-ov-file#-proposed-solution)
- [Hardware Design & Circuit](https://github.com/rougeDAG/Parkinson-s-Disease-Tremor-Detector?tab=readme-ov-file#%EF%B8%8F-hardware-design--circuit)
- [Working Principle](https://github.com/rougeDAG/Parkinson-s-Disease-Tremor-Detector?tab=readme-ov-file#-working-principle)
- [Algorithm Flow](https://github.com/rougeDAG/Parkinson-s-Disease-Tremor-Detector?tab=readme-ov-file#-algorithm-flow)
- [ESP32 Code](https://github.com/rougeDAG/Parkinson-s-Disease-Tremor-Detector?tab=readme-ov-file#-esp32-code-snippet)
- [Bill of Materials](https://github.com/rougeDAG/Parkinson-s-Disease-Tremor-Detector?tab=readme-ov-file#-bill-of-materials)
- [Results](https://github.com/rougeDAG/Parkinson-s-Disease-Tremor-Detector?tab=readme-ov-file#-results)
- [Future Scope](https://github.com/rougeDAG/Parkinson-s-Disease-Tremor-Detector?tab=readme-ov-file#-future-scope)
- [References](https://github.com/rougeDAG/Parkinson-s-Disease-Tremor-Detector?tab=readme-ov-file#-references)

---

## 🎯 **Overview**
The **Parkinson’s Tremor Detection System** is an IoT-based biomedical prototype designed to detect involuntary tremors in individuals suffering from Parkinson’s disease. The system measures angular velocity using an **MPU6050 gyroscope**, processes the data via **Fast Fourier Transform (FFT)** on an **ESP32**, and identifies tremor frequencies between **4–6 Hz**.  

Real-time results are displayed on an **LCD display**, while a **buzzer** provides an audible alert during detection.  

---

## 🚨 **Problem Statement**
Parkinson’s disease causes rhythmic, involuntary hand movements (tremors) typically in the 4–6 Hz range. Manual diagnosis requires medical-grade sensors, making continuous monitoring difficult and expensive.  
This project aims to create a **low-cost, real-time system** to automatically detect tremor patterns and assist in early diagnosis or therapy tracking.

---

## 💡 **Proposed Solution**
The proposed system uses the **MPU6050** to capture motion data, which the **ESP32** analyzes through FFT to identify dominant tremor frequencies. If the detected frequency lies within 4–6 Hz and exceeds a certain magnitude threshold, the system classifies it as a **tremor event** and activates both visual (LCD) and audible (buzzer) alerts.

---

## ⚙️ **Hardware Design & Circuit**

**Connections:**  
| Component | ESP32 Pin | Function |
|------------|------------|----------|
| MPU6050 SDA | GPIO 21 | I2C Data |
| MPU6050 SCL | GPIO 22 | I2C Clock |
| LCD SDA | GPIO 19 | I2C Data |
| LCD SCL | GPIO 23 | I2C Clock |
| Buzzer | GPIO 5 | Output alert |
| Power | 3.3V & GND | Power lines |
<img width="900" height="444" alt="image" src="https://github.com/user-attachments/assets/4d6a46b7-4b76-4e77-8939-d453e738c9e9" />


---

## 🧩 **Working Principle**
1. The **MPU6050 sensor** captures hand motion data in the form of angular velocity.  
2. The **ESP32 microcontroller** samples 128 readings at 100 Hz.  
3. FFT is applied to convert time-domain data into the frequency domain.  
4. The **dominant frequency** is identified.  
5. If frequency lies within **4–6 Hz** and magnitude exceeds the threshold, the tremor is detected.  
6. The **LCD** displays messages such as “Normal”, “Pre-Tremor Detected”, or “Tremor Detected”.  
7. The **Serial Monitor** logs detection data, and a **buzzer** alerts the user.  

---

## 🔁 **Algorithm Flow **
**Input → MPU6050 Data → FFT Processing → Frequency Detection → Tremor Validation → Output (LCD + Buzzer)**
<img width="472" height="557" alt="image" src="https://github.com/user-attachments/assets/c9f8019e-3433-4fac-b0b3-f77a5051687e" />


---

## 🧠 **ESP32 Code Snippet**
```cpp
ARDUINO CODE
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <arduinoFFT.h>
#include <LiquidCrystal_I2C.h>
#define BUZZER_PIN 25
#define MPU_SDA 21
#define MPU_SCL 22
#define LCD_SDA 18
#define LCD_SCL 19

Adafruit_MPU6050 mpu;
TwoWire I2CMPU = TwoWire(0);
TwoWire I2CLCD = TwoWire(1);
LiquidCrystal_I2C lcd(0x27, 16, 2);
#define SAMPLES 128
#define SAMPLING_FREQUENCY 100.0
ArduinoFFT<double> FFT = ArduinoFFT<double>();
double vReal[SAMPLES];
double vImag[SAMPLES];
#define TREMOR_MIN_HZ 4.0
#define TREMOR_MAX_HZ 6.0
#define TREMOR_THRESHOLD 10.0
unsigned long lastSampleMicros = 0;
int sampleCount = 0;
const unsigned long samplingInterval = 1000000 / SAMPLING_FREQUENCY;

void setup() {
  Serial.begin(115200);
  pinMode(BUZZER_PIN, OUTPUT);
  ledcSetup(0, 2000, 8);
  ledcAttachPin(BUZZER_PIN, 0);
  ledcWrite(0, 0);
  I2CMPU.begin(MPU_SDA, MPU_SCL, 100000);
  I2CLCD.begin(LCD_SDA, LCD_SCL, 100000);
  lcd.begin(16, 2);
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Parkinson's Test");
  lcd.setCursor(0, 1);
  lcd.print("Initializing...");
  delay(1500);
  lcd.clear();
  if (!mpu.begin(0x68, &I2CMPU)) {
    Serial.println("MPU6050 not found. Check wiring!");
    lcd.print("MPU Error!");
    while (1);
  }
  Serial.println("MPU6050 Connected!");
  lcd.print("MPU Ready");
  delay(1000);
  lcd.clear();
}
void loop() {
  if (sampleCount < SAMPLES) {
    if (micros() - lastSampleMicros >= samplingInterval) {
      lastSampleMicros = micros();
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);
      vReal[sampleCount] = g.gyro.x;
      vImag[sampleCount] = 0;
      sampleCount++;
    }
    return;
  }
  FFT.windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.compute(vReal, vImag, SAMPLES, FFT_FORWARD);
  FFT.complexToMagnitude(vReal, vImag, SAMPLES);
  double peakFrequency = 0;
  double peakMagnitude = 0;
  for (int i = 1; i < (SAMPLES / 2); i++) {
    double freq = (i * SAMPLING_FREQUENCY) / SAMPLES;
    if (vReal[i] > peakMagnitude) {
      peakMagnitude = vReal[i];
      peakFrequency = freq;
    }
  }
  Serial.print("Dominant Freq: ");
  Serial.print(peakFrequency, 2);
  Serial.print(" Hz  |  Magnitude: ");
  Serial.println(peakMagnitude, 2);
  if (peakFrequency >= TREMOR_MIN_HZ && peakFrequency <= TREMOR_MAX_HZ && peakMagnitude > TREMOR_THRESHOLD) {
    Serial.println("Tremor Detected!");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("TREMOR DETECTED!");
    ledcWriteTone(0, 2000);
  } else if (peakFrequency >= TREMOR_MIN_HZ - 1 && peakFrequency <= TREMOR_MAX_HZ + 1 && peakMagnitude > (TREMOR_THRESHOLD / 2)) {
    Serial.println("Pre-Tremor Detected!");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Pre-Tremor");
    lcd.setCursor(0, 1);
    lcd.print("Detected");
    ledcWriteTone(0, 1000);
  } else {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Normal");
    ledcWriteTone(0, 0);
  }
  sampleCount = 0;
  delay(1000);
}

```

---

## 🧾 **Bill of Materials**
| Component | Quantity | Description |
|------------|-----------|--------------|
| ESP32 | 1 | Main processing microcontroller |
| MPU6050 | 1 | Accelerometer + Gyroscope module |
| 16x2 LCD Display (I2C) | 1 | Status output display |
| Buzzer | 1 | Audio alert output |
| Jumper Wires | - | Circuit connections |
| Breadboard | 1 | Prototyping board |
| USB Cable | 1 | Power + Upload connection |

---

## 📊 **Results**
The system successfully identifies tremor frequencies in the **4–6 Hz range**, displaying detection results in real time. The **LCD display** provides clear visual feedback, and the **buzzer alert** ensures user awareness.

---

## 🔮 **Future Scope**
- Integrate **mobile app support** via Bluetooth or Wi-Fi for live tremor tracking.  
- Add **data logging and cloud analysis** using Firebase or ThingSpeak.  
- Miniaturize the hardware for **wearable wristband design**.  
- Train an **AI model** to classify tremor severity automatically.

---

## 📚 **References**
1. Adafruit MPU6050 Library Documentation  
2. ArduinoFFT Library – Enrique Condes  
3. Parkinson’s Foundation – Tremor Frequency Research  
4. VIT Chennai Embedded Systems Lab Resources  
