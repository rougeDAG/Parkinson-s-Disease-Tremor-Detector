# 🧠 **Parkinson’s Tremor Detection System**

Developed by [Hari Sai Senthil Kumar](https://github.com/rougeDAG)  
**Roll No:** 24BLC1108  
**Under the guidance of:** Dr. Gugapriya, School of Electronics Engineering, VIT Chennai  

---

## 📜 **Table of Contents**
- [Overview](#overview)  
- [Problem Statement](#problem-statement)  
- [Proposed Solution](#proposed-solution)  
- [Hardware Design & Circuit](#hardware-design--circuit)  
- [Working Principle](#working-principle)  
- [Algorithm Flow](#algorithm-flow)  
- [ESP32 Code](#esp32-code)  
- [Bill of Materials](#bill-of-materials)  
- [Results](#results)  
- [Future Scope](#future-scope)  
- [References](#references)  

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

## 🔁 **Algorithm Flow**
**Input → MPU6050 Data → FFT Processing → Frequency Detection → Tremor Validation → Output (LCD + Buzzer)**

---

## 🧠 **ESP32 Code Snippet**
```cpp
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ArduinoFFT.h>
#include <LiquidCrystal_I2C.h>

#define TREMOR_FREQ_MIN 4.0
#define TREMOR_FREQ_MAX 6.0
#define TREMOR_THRESHOLD 10.0
#define SAMPLES 128
#define SAMPLING_FREQUENCY 100

Adafruit_MPU6050 mpu;
LiquidCrystal_I2C lcd(0x27, 16, 2);
double vReal[SAMPLES];
double vImag[SAMPLES];
unsigned long lastSampleTime = 0;
int sampleCounter = 0;
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, SAMPLES, SAMPLING_FREQUENCY);

void setup() {
  Serial.begin(115200);
  lcd.init();
  lcd.backlight();
  pinMode(5, OUTPUT);
  if (!mpu.begin()) while (1);
}

void loop() {
  if (sampleCounter < SAMPLES) {
    if (micros() - lastSampleTime >= 1000000 / SAMPLING_FREQUENCY) {
      lastSampleTime = micros();
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);
      vReal[sampleCounter] = g.gyro.x;
      vImag[sampleCounter] = 0;
      sampleCounter++;
    }
  } else {
    FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.compute(FFT_FORWARD);
    FFT.complexToMagnitude();
    double peakMagnitude = 0, peakFrequency = 0;
    for (int i = 1; i < SAMPLES / 2; i++) {
      double freq = (i * SAMPLING_FREQUENCY) / SAMPLES;
      if (vReal[i] > peakMagnitude) {
        peakMagnitude = vReal[i];
        peakFrequency = freq;
      }
    }
    if (peakFrequency >= TREMOR_FREQ_MIN && peakFrequency <= TREMOR_FREQ_MAX && peakMagnitude > TREMOR_THRESHOLD) {
      lcd.clear(); lcd.print("TREMOR DETECTED");
      digitalWrite(5, HIGH);
      Serial.println("TREMOR DETECTED");
    } else {
      lcd.clear(); lcd.print("NORMAL");
      digitalWrite(5, LOW);
    }
    sampleCounter = 0;
  }
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
