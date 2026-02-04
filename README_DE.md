# BNO055 Arduino Bibliothek - Dokumentation

## Inhaltsverzeichnis
1. [Einführung](#einführung)
2. [Was ist der BNO055?](#was-ist-der-bno055)
3. [Hardware-Anforderungen](#hardware-anforderungen)
4. [Verkabelung](#verkabelung)
5. [Installation](#installation)
6. [Erste Schritte](#erste-schritte)
7. [Betriebsmodi](#betriebsmodi)
8. [Sensordaten-Typen](#sensordaten-typen)
9. [Kalibrierung](#kalibrierung)
10. [API-Referenz](#api-referenz)
11. [Beispiele](#beispiele)
12. [Fehlerbehebung](#fehlerbehebung)
13. [Kompatibilität](#kompatibilität)

---

## Einführung

Diese Bibliothek ermöglicht die einfache Verwendung des BNO055 9-Achsen-Orientierungssensors mit Arduino und ESP32. Der BNO055 ist ein intelligenter Sensor, der Beschleunigungsmesser, Gyroskop und Magnetometer kombiniert und die Sensorfusion direkt auf dem Chip durchführt.

**Autor:** ROBERT BOSCH GMBH  
**Lizenz:** GNU General Public License v3.0  
**Version:** 1.2.1

---

## Was ist der BNO055?

Der **BNO055** ist ein absoluter Orientierungssensor von Bosch Sensortec. Er kombiniert:

- **3-Achsen-Beschleunigungsmesser** (Accelerometer) - misst Beschleunigung
- **3-Achsen-Gyroskop** - misst Drehgeschwindigkeit
- **3-Achsen-Magnetometer** - misst Magnetfeld (wie ein Kompass)
- **Integrierter Mikroprozessor** - führt Sensorfusion durch

### Hauptvorteile:
- **Sensorfusion auf dem Chip**: Der BNO055 berechnet die Orientierung selbst, was die Arbeit mit einem Mikrocontroller erheblich vereinfacht
- **Mehrere Ausgabeformate**: Euler-Winkel, Quaternionen, Rohdaten
- **Automatische Kalibrierung**: Der Sensor kann sich selbst kalibrieren
- **Niedrige CPU-Last**: Da die Berechnungen auf dem Sensor stattfinden

---

## Hardware-Anforderungen

### Mindestanforderungen:
- **Arduino-Board** (z.B. Arduino Uno, Due, Mega) oder **ESP32**
- **BNO055 Sensor-Modul** (z.B. Adafruit BNO055 Breakout Board)
- **Verbindungskabel** (Jumper-Kabel)
- **Spannungsversorgung**: 3.3V (wichtig!)

### Wichtige Hinweise:
- Es gibt mehrere Varianten des Sensors: Der einfache IC-Chip und der Chip mit Platine.
- ⚠️ **Der BNO055-Chip arbeitet mit 3.3V** - die Verwendung von 5V kann den Sensor beschädigen!  
- ⚠️ Wenn Sie einen 5V Arduino verwenden, benötigen Sie Level-Shifter für die I2C-Leitungen oder einen Sensor mit integriertem Spannungsregler.

⚠️ Dies gilt allerdings nur für den Chip, manche Platinen beinhalten einen Level-Shifter für die Verwendung mit 5V, bitte überprüfen sie dazu das Datenblatt.

---

## Verkabelung

### Standard I2C-Verbindung:

| BNO055 Pin | Arduino Due/ESP32 | Arduino Uno/Mega (mit Level-Shifter) | Beschreibung |
|------------|-------------------|---------------------------------------|--------------|
| VIN        | 3.3V              | 3.3V oder 5V (je nach Modul)         | Spannungsversorgung |
| GND        | GND               | GND                                   | Masse |
| SDA/Tx     | SDA (Pin 20)      | A4 (Uno) / Pin 20 (Mega)             | I2C Daten (SDA im I2C-Modus) |
| SCL/Rx     | SCL (Pin 21)      | A5 (Uno) / Pin 21 (Mega)             | I2C Takt (SCL im I2C-Modus) |
| ADD        | GND               | GND (für Adresse 0x28) oder 3.3V (für Adresse 0x29) | Adressauswahl |
| INT        | nicht verbunden   | nicht verbunden                       | Interrupt-Pin (optional) |
| BOOT       | nicht verbunden   | nicht verbunden                       | Boot-Modus (optional) |
| RST        | nicht verbunden   | nicht verbunden                       | Reset-Pin (optional) |

### ESP32 Verkabelung:

| BNO055 Pin | ESP32                       | Beschreibung |
|------------|-----------------------------|--------------|
| VIN        | 3.3V                        | Spannungsversorgung |
| GND        | GND                         | Masse |
| SDA/Tx     | GPIO 21 (oder frei wählbar) | I2C Daten |
| SCL/Rx     | GPIO 22 (oder frei wählbar) | I2C Takt |
| ADD        | GND                         | Adresse 0x28 (Standard) |
| INT        | nicht verbunden             | Interrupt-Pin (optional) |
| BOOT       | nicht verbunden             | Boot-Modus (optional) |
| RST        | nicht verbunden             | Reset-Pin (optional) |

**Hinweise:** 
- Der ADD-Pin bestimmt die I2C-Adresse: GND = 0x28 (Standard), 3.3V = 0x29
- Die Pins SDA/Tx und SCL/Rx funktionieren im I2C-Modus als SDA und SCL
- INT, BOOT und RST sind optionale Pins und müssen für den Normalbetrieb nicht verbunden werden

---

## Installation

### Methode 1: Arduino Library Manager (empfohlen)
1. Öffnen Sie die Arduino IDE
2. Gehen Sie zu **Sketch** → **Bibliothek einbinden** → **Bibliotheken verwalten...**
3. Suchen Sie nach "BNO055"
4. Klicken Sie auf **Installieren**

### Methode 2: Manuelle Installation
1. Laden Sie die Bibliothek von [GitHub](https://github.com/arduino-libraries/BNO055) herunter
2. Entpacken Sie die ZIP-Datei
3. Verschieben Sie den Ordner in Ihr Arduino-Bibliotheksverzeichnis:
   - Windows: `C:\Users\<Benutzername>\Documents\Arduino\libraries\`
   - Mac: `~/Documents/Arduino/libraries/`
   - Linux: `~/Arduino/libraries/`
4. Starten Sie die Arduino IDE neu

### Für ESP32:
- Stellen Sie sicher, dass das ESP32 Board-Paket installiert ist
- Die BNO055-Bibliothek funktioniert auch mit ESP32 über die Wire-Bibliothek

---

## Erste Schritte

### Einfachstes Beispiel:

```cpp
#include "BNO055_support.h"
#include <Wire.h>

struct bno055_t myBNO;

void setup() {
  // I2C-Kommunikation initialisieren
  Wire.begin();
  
  // Serielle Kommunikation starten
  Serial.begin(115200);
  
  // BNO055 initialisieren
  BNO_Init(&myBNO);
  
  // Betriebsmodus setzen (NDOF = 9-Achsen-Fusion)
  bno055_set_operation_mode(OPERATION_MODE_NDOF);
  
  delay(100);
  
  Serial.println("BNO055 bereit!");
}

void loop() {
  // Ihr Code hier
}
```

### Schritt-für-Schritt-Erklärung:

1. **Wire.begin()**: Startet die I2C-Kommunikation
2. **BNO_Init(&myBNO)**: Initialisiert den Sensor und speichert Geräteinformationen in der Struktur
3. **bno055_set_operation_mode()**: Legt fest, welche Sensoren verwendet werden und wie die Daten verarbeitet werden
4. **delay(100)**: Wartet, bis der Sensor bereit ist

---

## Betriebsmodi

Der BNO055 hat verschiedene Betriebsmodi. Jeder Modus verwendet unterschiedliche Sensorkombinationen:

### Konfigurationsmodus
- **OPERATION_MODE_CONFIG** (0x00)
  - Wird für die Konfiguration verwendet
  - Keine Sensordaten verfügbar

### Nicht-Fusion Modi (Rohdaten)
- **OPERATION_MODE_ACCONLY** (0x01)
  - Nur Beschleunigungsmesser aktiv
  
- **OPERATION_MODE_MAGONLY** (0x02)
  - Nur Magnetometer aktiv
  
- **OPERATION_MODE_GYRONLY** (0x03)
  - Nur Gyroskop aktiv
  
- **OPERATION_MODE_ACCMAG** (0x04)
  - Beschleunigungsmesser + Magnetometer
  
- **OPERATION_MODE_ACCGYRO** (0x05)
  - Beschleunigungsmesser + Gyroskop
  
- **OPERATION_MODE_MAGGYRO** (0x06)
  - Magnetometer + Gyroskop
  
- **OPERATION_MODE_AMG** (0x07)
  - Alle drei Sensoren (ohne Fusion)

### Fusion Modi (empfohlen)
- **OPERATION_MODE_IMUPLUS** (0x08)
  - IMU-Modus: Beschleunigungsmesser + Gyroskop mit Fusion
  - Kein Magnetometer (weniger Störanfällig)
  - ✅ Gut für Bewegungserkennung
  
- **OPERATION_MODE_COMPASS** (0x09)
  - Kompass-Modus: Beschleunigungsmesser + Magnetometer
  - ✅ Gut für Orientierung relativ zum Magnetfeld der Erde
  
- **OPERATION_MODE_M4G** (0x0A)
  - Magnetometer für Gyro
  - Ähnlich wie IMU, aber mit Magnetometer statt Gyroskop
  
- **OPERATION_MODE_NDOF_FMC_OFF** (0x0B)
  - 9-Achsen-Fusion ohne Schnellmagnetkalibrierung
  
- **OPERATION_MODE_NDOF** (0x0C) ⭐ **EMPFOHLEN**
  - 9-Achsen-Fusion: Alle Sensoren mit vollständiger Fusion
  - ✅ Beste Genauigkeit für absolute Orientierung
  - ✅ Am häufigsten verwendet

### Beispiel Modus-Wechsel:
```cpp
// Zu NDOF-Modus wechseln (beste Genauigkeit)
bno055_set_operation_mode(OPERATION_MODE_NDOF);
delay(20); // Kurze Pause nach Moduswechsel

// Zu IMU-Modus wechseln (ohne Magnetometer)
bno055_set_operation_mode(OPERATION_MODE_IMUPLUS);
delay(20);
```

---

## Sensordaten-Typen

Der BNO055 liefert verschiedene Arten von Daten:

### 1. Beschleunigungsdaten (Accelerometer)
Misst die Beschleunigung in drei Achsen (X, Y, Z).

**Einheit:** m/s² oder mg (milli-g)

```cpp
struct bno055_accel accel;
bno055_read_accel_xyz(&accel);

Serial.print("X: "); Serial.print(accel.x);
Serial.print(" Y: "); Serial.print(accel.y);
Serial.print(" Z: "); Serial.println(accel.z);
```

**Verwendung:** Erkennung von Bewegung, Neigung, Stößen

---

### 2. Gyroskop-Daten (Gyroscope)
Misst die Drehgeschwindigkeit in drei Achsen.

**Einheit:** Grad/Sekunde (dps) oder Radiant/Sekunde (rps)

```cpp
struct bno055_gyro gyro;
bno055_read_gyro_xyz(&gyro);

Serial.print("X: "); Serial.print(gyro.x);
Serial.print(" Y: "); Serial.print(gyro.y);
Serial.print(" Z: "); Serial.println(gyro.z);
```

**Verwendung:** Erkennung von Rotation, Drohnen-Stabilisierung

---

### 3. Magnetometer-Daten (Magnetometer)
Misst das Magnetfeld in drei Achsen.

**Einheit:** µT (Mikrotesla)

```cpp
struct bno055_mag mag;
bno055_read_mag_xyz(&mag);

Serial.print("X: "); Serial.print(mag.x);
Serial.print(" Y: "); Serial.print(mag.y);
Serial.print(" Z: "); Serial.println(mag.z);
```

**Verwendung:** Kompass, Richtungsbestimmung

---

### 4. Euler-Winkel ⭐ EINFACHSTE METHODE
Beschreibt die Orientierung als drei Winkel.

**Komponenten:**
- **Heading (h)**: Gier (Yaw) - Rotation um die Z-Achse (0-360°)
- **Roll (r)**: Rollen - Rotation um die X-Achse (-180 bis +180°)
- **Pitch (p)**: Nicken - Rotation um die Y-Achse (-90 bis +90°)

**Einheit:** Grad (°)

```cpp
struct bno055_euler euler;
bno055_read_euler_hrp(&euler);

// Rohdaten in Grad umrechnen (geteilt durch 16)
float heading = euler.h / 16.0;
float roll = euler.r / 16.0;
float pitch = euler.p / 16.0;

Serial.print("Heading: "); Serial.print(heading);
Serial.print(" Roll: "); Serial.print(roll);
Serial.print(" Pitch: "); Serial.println(pitch);
```

**Verwendung:** Einfachste Methode zur Orientierungsbestimmung, ideal für Anfänger

**Anschauliches Beispiel:**
- **Heading**: Wie ein Kompass - in welche Richtung zeigt das Objekt?
- **Roll**: Wie ein Flugzeug - kippt es nach links oder rechts?
- **Pitch**: Wie ein Flugzeug - steigt oder sinkt die Nase?

---

### 5. Quaternionen
Mathematische Darstellung der Orientierung ohne Gimbal-Lock-Problem.

**Komponenten:** w, x, y, z

```cpp
struct bno055_quaternion quat;
bno055_read_quaternion_wxyz(&quat);

// Werte in echte Quaternionen umrechnen (geteilt durch 16384)
float w = quat.w / 16384.0;
float x = quat.x / 16384.0;
float y = quat.y / 16384.0;
float z = quat.z / 16384.0;
```

**Verwendung:** Fortgeschrittene Anwendungen, 3D-Grafik, Vermeidung von Gimbal-Lock

---

### 6. Lineare Beschleunigung
Beschleunigung ohne Schwerkraft.

```cpp
struct bno055_linear_accel lin_accel;
bno055_read_linear_accel_xyz(&lin_accel);
```

**Verwendung:** Erkennung von tatsächlicher Bewegung (ohne Einfluss der Schwerkraft)

---

### 7. Schwerkraftvektor
Nur die Schwerkraft-Komponente.

```cpp
struct bno055_gravity gravity;
bno055_read_gravity_xyz(&gravity);
```

**Verwendung:** Neigungsmessung

---

### 8. Temperatur
Sensortemperatur.

```cpp
BNO055_S16 temp;
bno055_read_temperature_data(&temp);

Serial.print("Temperatur: ");
Serial.print(temp);
Serial.println(" °C");
```

---

## Kalibrierung

Der BNO055 kalibriert sich automatisch, aber Sie können den Fortschritt überwachen.

### Kalibrierungsstatus

Jeder Sensor hat einen Kalibrierungsstatus von **0** (nicht kalibriert) bis **3** (vollständig kalibriert).

```cpp
unsigned char sys_calib, gyro_calib, accel_calib, mag_calib;

// Kalibrierungsstatus auslesen
bno055_get_syscalib_status(&sys_calib);
bno055_get_gyrocalib_status(&gyro_calib);
bno055_get_accelcalib_status(&accel_calib);
bno055_get_magcalib_status(&mag_calib);

Serial.print("System: "); Serial.print(sys_calib);
Serial.print(" Gyro: "); Serial.print(gyro_calib);
Serial.print(" Accel: "); Serial.print(accel_calib);
Serial.print(" Mag: "); Serial.println(mag_calib);
```

### Kalibrierungsanleitung:

#### 1. Gyroskop
- **Sensor muss ruhig liegen**
- Legen Sie den Sensor auf eine stabile Oberfläche
- Warten Sie einige Sekunden ohne Bewegung

#### 2. Beschleunigungsmesser
- **Sensor in verschiedene Positionen bringen**
- Halten Sie den Sensor in verschiedenen Orientierungen (flach, auf der Seite, aufrecht)
- Halten Sie jede Position für einige Sekunden ruhig

#### 3. Magnetometer (am wichtigsten!)
- **Sensor in Form einer "8" bewegen**
- Bewegen Sie den Sensor langsam in verschiedenen Orientierungen
- Beschreiben Sie eine liegende "8" (Unendlichkeitszeichen) in der Luft
- Drehen und neigen Sie dabei den Sensor in alle Richtungen
- **Achtung:** Magnetometer sind anfällig für:
  - Metallische Objekte in der Nähe
  - Elektrische Geräte (Motoren, Lautsprecher)
  - Stahlbeton-Gebäude
  - Magnete

### Vollständige Kalibrierung:
Alle Sensoren müssen Status **3** erreichen, damit auch der System-Status **3** wird.

### Kalibrierungsdaten speichern (fortgeschritten):
Sie können Kalibrierungsdaten speichern und später wiederherstellen, um die Kalibrierung zu beschleunigen. Details dazu finden Sie im Datenblatt des BNO055.

---

## API-Referenz

### Initialisierung

#### `BNO_Init(struct bno055_t *bno055)`
Initialisiert den BNO055-Sensor.

**Parameter:**
- `bno055`: Zeiger auf die BNO055-Struktur

**Rückgabewert:** Erfolgscode (0 = Erfolg)

**Beispiel:**
```cpp
struct bno055_t myBNO;
BNO_Init(&myBNO);
```

---

### Betriebsmodus

#### `bno055_set_operation_mode(unsigned char mode)`
Setzt den Betriebsmodus.

**Parameter:**
- `mode`: Betriebsmodus (siehe [Betriebsmodi](#betriebsmodi))

**Beispiel:**
```cpp
bno055_set_operation_mode(OPERATION_MODE_NDOF);
```

#### `bno055_get_operation_mode(unsigned char *mode)`
Liest den aktuellen Betriebsmodus.

---

### Sensordaten lesen

#### Beschleunigung
```cpp
// Einzelne Achsen
bno055_read_accel_x(BNO055_S16 *accel_x);
bno055_read_accel_y(BNO055_S16 *accel_y);
bno055_read_accel_z(BNO055_S16 *accel_z);

// Alle Achsen gleichzeitig
bno055_read_accel_xyz(struct bno055_accel *accel);
```

#### Gyroskop
```cpp
// Einzelne Achsen
bno055_read_gyro_x(BNO055_S16 *gyro_x);
bno055_read_gyro_y(BNO055_S16 *gyro_y);
bno055_read_gyro_z(BNO055_S16 *gyro_z);

// Alle Achsen gleichzeitig
bno055_read_gyro_xyz(struct bno055_gyro *gyro);
```

#### Magnetometer
```cpp
// Einzelne Achsen
bno055_read_mag_x(BNO055_S16 *mag_x);
bno055_read_mag_y(BNO055_S16 *mag_y);
bno055_read_mag_z(BNO055_S16 *mag_z);

// Alle Achsen gleichzeitig
bno055_read_mag_xyz(struct bno055_mag *mag);
```

#### Euler-Winkel ⭐
```cpp
// Einzelne Winkel
bno055_read_euler_h(BNO055_S16 *euler_h);  // Heading
bno055_read_euler_r(BNO055_S16 *euler_r);  // Roll
bno055_read_euler_p(BNO055_S16 *euler_p);  // Pitch

// Alle Winkel gleichzeitig
bno055_read_euler_hrp(struct bno055_euler *euler);
```

**Umrechnung in Grad:**
```cpp
float heading_grad = euler.h / 16.0;
float roll_grad = euler.r / 16.0;
float pitch_grad = euler.p / 16.0;
```

#### Quaternionen
```cpp
// Einzelne Komponenten
bno055_read_quaternion_w(BNO055_S16 *quat_w);
bno055_read_quaternion_x(BNO055_S16 *quat_x);
bno055_read_quaternion_y(BNO055_S16 *quat_y);
bno055_read_quaternion_z(BNO055_S16 *quat_z);

// Alle Komponenten gleichzeitig
bno055_read_quaternion_wxyz(struct bno055_quaternion *quat);
```

**Umrechnung:**
```cpp
float w = quat.w / 16384.0;
float x = quat.x / 16384.0;
float y = quat.y / 16384.0;
float z = quat.z / 16384.0;
```

#### Lineare Beschleunigung
```cpp
bno055_read_linear_accel_xyz(struct bno055_linear_accel *lin_accel);
```

#### Schwerkraft
```cpp
bno055_read_gravity_xyz(struct bno055_gravity *gravity);
```

#### Temperatur
```cpp
bno055_read_temperature_data(BNO055_S16 *temp);
```

---

### Kalibrierung

```cpp
// Kalibrierungsstatus abrufen (0-3)
bno055_get_syscalib_status(unsigned char *sys_calib);
bno055_get_gyrocalib_status(unsigned char *gyro_calib);
bno055_get_accelcalib_status(unsigned char *accel_calib);
bno055_get_magcalib_status(unsigned char *mag_calib);
```

---

### Einheitenauswahl

```cpp
// Beschleunigungseinheit: 0 = m/s², 1 = mg
bno055_set_accel_unit(unsigned char unit);

// Gyroskop-Einheit: 0 = dps, 1 = rps
bno055_set_gyro_unit(unsigned char unit);

// Euler-Einheit: 0 = Grad, 1 = Radiant
bno055_set_euler_unit(unsigned char unit);

// Temperatur-Einheit: 0 = Celsius, 1 = Fahrenheit
bno055_set_temperature_unit(unsigned char unit);
```

---

### Weitere Funktionen

```cpp
// System-Status und Fehler
bno055_get_system_status_code(unsigned char *sys_status);
bno055_get_system_error_code(unsigned char *sys_error);

// Stromsparmodus
bno055_set_powermode(unsigned char powermode);

// System-Reset
bno055_set_reset_sys(unsigned char rst_sys);
```

---

## Beispiele

### Beispiel 1: Geräteinformationen auslesen

```cpp
#include "BNO055_support.h"
#include <Wire.h>

struct bno055_t myBNO;

void setup() {
  Wire.begin();
  Serial.begin(115200);
  
  BNO_Init(&myBNO);
  bno055_set_operation_mode(OPERATION_MODE_NDOF);
  delay(100);
  
  // Geräteinformationen ausgeben
  Serial.println("=== BNO055 Geräteinformationen ===");
  Serial.print("Chip ID: ");
  Serial.println(myBNO.chip_id);
  
  Serial.print("Software Revision: ");
  Serial.println(myBNO.sw_revision_id);
  
  Serial.print("Beschleunigungsmesser Revision: ");
  Serial.println(myBNO.accel_revision_id);
  
  Serial.print("Gyroskop Revision: ");
  Serial.println(myBNO.gyro_revision_id);
  
  Serial.print("Magnetometer Revision: ");
  Serial.println(myBNO.mag_revision_id);
  
  Serial.print("I2C-Adresse: 0x");
  Serial.println(myBNO.dev_addr, HEX);
}

void loop() {
  // Leer
}
```

---

### Beispiel 2: Euler-Winkel ausgeben (einfach)

```cpp
#include "BNO055_support.h"
#include <Wire.h>

struct bno055_t myBNO;
struct bno055_euler euler;

void setup() {
  Wire.begin();
  Serial.begin(115200);
  
  BNO_Init(&myBNO);
  bno055_set_operation_mode(OPERATION_MODE_NDOF);
  
  delay(100);
  Serial.println("Starte Orientierungsmessung...");
}

void loop() {
  // Euler-Winkel auslesen
  bno055_read_euler_hrp(&euler);
  
  // In Grad umrechnen
  float heading = euler.h / 16.0;
  float roll = euler.r / 16.0;
  float pitch = euler.p / 16.0;
  
  // Ausgabe
  Serial.print("Heading: ");
  Serial.print(heading, 2);
  Serial.print("°  Roll: ");
  Serial.print(roll, 2);
  Serial.print("°  Pitch: ");
  Serial.print(pitch, 2);
  Serial.println("°");
  
  delay(100);  // 10 Hz
}
```

---

### Beispiel 3: Kalibrierungsstatus überwachen

```cpp
#include "BNO055_support.h"
#include <Wire.h>

struct bno055_t myBNO;

void setup() {
  Wire.begin();
  Serial.begin(115200);
  
  BNO_Init(&myBNO);
  bno055_set_operation_mode(OPERATION_MODE_NDOF);
  
  delay(100);
  Serial.println("=== Kalibrierung ===");
  Serial.println("Bewegen Sie den Sensor zur Kalibrierung");
}

void loop() {
  unsigned char sys_calib, gyro_calib, accel_calib, mag_calib;
  
  // Kalibrierungsstatus auslesen
  bno055_get_syscalib_status(&sys_calib);
  bno055_get_gyrocalib_status(&gyro_calib);
  bno055_get_accelcalib_status(&accel_calib);
  bno055_get_magcalib_status(&mag_calib);
  
  // Ausgabe
  Serial.print("System: ");
  Serial.print(sys_calib);
  Serial.print("/3  Gyro: ");
  Serial.print(gyro_calib);
  Serial.print("/3  Accel: ");
  Serial.print(accel_calib);
  Serial.print("/3  Mag: ");
  Serial.print(mag_calib);
  Serial.print("/3");
  
  // Visuelle Anzeige
  if (sys_calib == 3) {
    Serial.println("  ✓ VOLLSTÄNDIG KALIBRIERT!");
  } else {
    Serial.println("  ⚠ Kalibrierung läuft...");
  }
  
  delay(500);  // 2 Hz
}
```

---

### Beispiel 4: Alle Sensordaten ausgeben

```cpp
#include "BNO055_support.h"
#include <Wire.h>

struct bno055_t myBNO;
struct bno055_accel accel;
struct bno055_gyro gyro;
struct bno055_mag mag;
struct bno055_euler euler;

void setup() {
  Wire.begin();
  Serial.begin(115200);
  
  BNO_Init(&myBNO);
  bno055_set_operation_mode(OPERATION_MODE_NDOF);
  
  delay(100);
  Serial.println("=== BNO055 Alle Sensordaten ===");
}

void loop() {
  // Alle Daten auslesen
  bno055_read_accel_xyz(&accel);
  bno055_read_gyro_xyz(&gyro);
  bno055_read_mag_xyz(&mag);
  bno055_read_euler_hrp(&euler);
  
  // Beschleunigung
  Serial.println("--- Beschleunigung (m/s²) ---");
  Serial.print("X: "); Serial.print(accel.x / 100.0);
  Serial.print("  Y: "); Serial.print(accel.y / 100.0);
  Serial.print("  Z: "); Serial.println(accel.z / 100.0);
  
  // Gyroskop
  Serial.println("--- Gyroskop (dps) ---");
  Serial.print("X: "); Serial.print(gyro.x / 16.0);
  Serial.print("  Y: "); Serial.print(gyro.y / 16.0);
  Serial.print("  Z: "); Serial.println(gyro.z / 16.0);
  
  // Magnetometer
  Serial.println("--- Magnetometer (µT) ---");
  Serial.print("X: "); Serial.print(mag.x / 16.0);
  Serial.print("  Y: "); Serial.print(mag.y / 16.0);
  Serial.print("  Z: "); Serial.println(mag.z / 16.0);
  
  // Euler
  Serial.println("--- Orientierung (Grad) ---");
  Serial.print("Heading: "); Serial.print(euler.h / 16.0);
  Serial.print("  Roll: "); Serial.print(euler.r / 16.0);
  Serial.print("  Pitch: "); Serial.println(euler.p / 16.0);
  
  Serial.println();
  delay(1000);  // 1 Hz
}
```

---

### Beispiel 5: ESP32 mit BNO055

```cpp
#include "BNO055_support.h"
#include <Wire.h>

// ESP32 verwendet standardmäßig GPIO 21 (SDA) und GPIO 22 (SCL)
// Sie können auch benutzerdefinierte Pins verwenden:
// #define SDA_PIN 21
// #define SCL_PIN 22

struct bno055_t myBNO;
struct bno055_euler euler;

void setup() {
  // Für benutzerdefinierte I2C-Pins:
  // Wire.begin(SDA_PIN, SCL_PIN);
  
  Wire.begin();  // Standard-Pins
  Serial.begin(115200);
  
  delay(1000);  // ESP32 braucht etwas Zeit zum Starten
  
  Serial.println("ESP32 mit BNO055");
  
  BNO_Init(&myBNO);
  bno055_set_operation_mode(OPERATION_MODE_NDOF);
  
  delay(100);
  Serial.println("BNO055 initialisiert!");
}

void loop() {
  bno055_read_euler_hrp(&euler);
  
  float heading = euler.h / 16.0;
  float roll = euler.r / 16.0;
  float pitch = euler.p / 16.0;
  
  Serial.printf("Heading: %.2f°  Roll: %.2f°  Pitch: %.2f°\n", 
                heading, roll, pitch);
  
  delay(100);
}
```

---

## Fehlerbehebung

### Problem: Sensor wird nicht erkannt

**Mögliche Ursachen:**
1. **Falsche Verkabelung**
   - Überprüfen Sie alle Verbindungen
   - Stellen Sie sicher, dass SDA/Tx und SCL/Rx nicht vertauscht sind
   
2. **Falsche I2C-Adresse**
   - Standard ist 0x28 (wenn ADD auf GND)
   - Kann auch 0x29 sein (wenn ADD auf VCC/3.3V)
   - Ändern Sie in BNO055.h:
     ```cpp
     #define BNO055_I2C_ADDR BNO055_I2C_ADDR1  // oder ADDR2
     ```

3. **Spannungsprobleme**
   - BNO055 benötigt 3.3V
   - Überprüfen Sie die Stromversorgung
   
4. **I2C-Bus-Probleme**
   - Testen Sie mit einem I2C-Scanner:
     ```cpp
     #include <Wire.h>
     void setup() {
       Wire.begin();
       Serial.begin(115200);
       Serial.println("I2C Scanner");
     }
     void loop() {
       for(byte addr = 1; addr < 127; addr++) {
         Wire.beginTransmission(addr);
         if(Wire.endTransmission() == 0) {
           Serial.print("Gefunden: 0x");
           Serial.println(addr, HEX);
         }
       }
       delay(5000);
     }
     ```

---

### Problem: Unplausible Werte

**Mögliche Ursachen:**
1. **Sensor nicht kalibriert**
   - Überprüfen Sie den Kalibrierungsstatus
   - Führen Sie die Kalibrierungsprozedur durch
   
2. **Falscher Betriebsmodus**
   - Stellen Sie sicher, dass der richtige Modus gesetzt ist
   - Für Orientierung: OPERATION_MODE_NDOF
   
3. **Magnetische Störungen**
   - Halten Sie Magnete und Motoren fern
   - Kalibrieren Sie weg von metallischen Objekten
   
4. **Falsche Umrechnung**
   - Überprüfen Sie Ihre Umrechnungsfaktoren
   - Euler: Division durch 16
   - Quaternionen: Division durch 16384

---

### Problem: Werte springen oder sind instabil

**Lösungen:**
1. **Kalibrierung verbessern**
   - Führen Sie eine vollständige Kalibrierung durch
   - Besonders das Magnetometer benötigt sorgfältige Kalibrierung
   
2. **Daten filtern**
   - Implementieren Sie einen gleitenden Durchschnitt
   - Beispiel:
     ```cpp
     // Globale Variable für geglätteten Wert
     float smoothedHeading = 0.0;
     
     void loop() {
       struct bno055_euler euler;
       bno055_read_euler_hrp(&euler);
       float newHeading = euler.h / 16.0;
       
       // Exponentieller gleitender Durchschnitt (90% alt, 10% neu)
       smoothedHeading = 0.9 * smoothedHeading + 0.1 * newHeading;
       
       Serial.println(smoothedHeading);
     }
     ```
   
3. **IMU-Modus verwenden**
   - Wenn das Magnetometer Probleme macht, verwenden Sie OPERATION_MODE_IMUPLUS
   - Dieser Modus nutzt kein Magnetometer

---

### Problem: Sensor reagiert langsam

**Lösungen:**
1. **Datenrate erhöhen**
   ```cpp
   // FASTEST_MODE_1 = 100 Hz (höchste Datenrate)
   // FASTEST_MODE_2 = 100 Hz
   // GAME_MODE = 100 Hz
   // UI_MODE = 50 Hz
   // NORMAL_MODE = 100 Hz (empfohlen für die meisten Anwendungen)
   bno055_set_data_output_rate(FASTEST_MODE_1);  // 100 Hz
   ```
   
2. **I2C-Taktrate erhöhen**
   ```cpp
   Wire.setClock(400000);  // 400 kHz Fast Mode
   ```

---

### Problem: ESP32-spezifische Probleme

1. **Watchdog-Reset**
   - Fügen Sie `yield()` in lange Schleifen ein
   - Oder deaktivieren Sie den Watchdog
   
2. **I2C-Timing**
   - ESP32 kann manchmal I2C-Timing-Probleme haben
   - Versuchen Sie:
     ```cpp
     Wire.setTimeout(1000);  // 1 Sekunde Timeout (in Millisekunden)
     ```

---

## Kompatibilität

### Getestete Boards:

✅ **Arduino:**
- Arduino Uno (mit Level-Shifter oder 3.3V-Modul)
- Arduino Due (nativ 3.3V)
- Arduino Mega 2560 (mit Level-Shifter)
- Arduino Nano (mit Level-Shifter)
- Arduino Leonardo (mit Level-Shifter)

✅ **ESP32:**
- ESP32 DevKit V1
- ESP32-WROOM-32
- ESP-WROOM-32D
- Alle ESP32-Boards mit I2C-Unterstützung

✅ **Andere:**
- Teensy 3.x/4.x
- STM32 (mit Arduino-Core)

### Anforderungen:
- **Arduino IDE:** Version 1.6.0 oder höher
- **ESP32 Board Package:** Version 1.0.0 oder höher (für ESP32)
- **Wire-Bibliothek:** Enthalten in Arduino

### I2C-Kommunikation:
- **Standard-Taktrate:** 100 kHz
- **Schnelle Taktrate:** 400 kHz (empfohlen)
- **Adresse:** 0x28 oder 0x29 (konfigurierbar)

---

## Häufig gestellte Fragen (FAQ)

### 1. Wie schnell kann ich Daten auslesen?
Der BNO055 kann bis zu 100 Hz Daten liefern. In der Praxis sind 10-50 Hz für die meisten Anwendungen ausreichend.

### 2. Kann ich mehrere BNO055 an einem Arduino verwenden?
Ja, aber Sie benötigen einen I2C-Multiplexer (z.B. TCA9548A), da der BNO055 nur zwei I2C-Adressen unterstützt.

### 3. Wie genau ist der BNO055?
- **Heading:** ±2.5° (mit vollständiger Kalibrierung)
- **Roll/Pitch:** ±1° (statisch)
- Genauigkeit hängt stark von der Kalibrierung ab!

### 4. Funktioniert der Sensor in Gebäuden?
Ja, aber das Magnetometer kann durch Stahlbeton und elektronische Geräte gestört werden. Für Innenräume ist der IMU-Modus oft besser.

### 5. Wie lange dauert die Kalibrierung?
- **Gyroskop:** 5-10 Sekunden (Sensor ruhig halten)
- **Beschleunigungsmesser:** 30-60 Sekunden (verschiedene Positionen)
- **Magnetometer:** 1-2 Minuten ("8"-Bewegung)

### 6. Kann ich die Bibliothek mit MicroPython verwenden?
Diese Bibliothek ist für C/C++ (Arduino) geschrieben. Für MicroPython gibt es separate Bibliotheken.

### 7. Wie viel Strom verbraucht der Sensor?
- **Normal-Modus:** ca. 12 mA
- **Low-Power-Modus:** ca. 3 mA
- **Suspend-Modus:** ca. 40 µA

---

## Nützliche Ressourcen

- **Bosch BNO055 Datenblatt:** [https://www.bosch-sensortec.com](https://www.bosch-sensortec.com)
- **GitHub Repository:** [https://github.com/arduino-libraries/BNO055](https://github.com/arduino-libraries/BNO055)
- **Arduino Forum:** [https://forum.arduino.cc](https://forum.arduino.cc)
- **Adafruit BNO055 Tutorial:** [https://learn.adafruit.com](https://learn.adafruit.com)

---

## Lizenz

Diese Bibliothek steht unter der **GNU General Public License v3.0**.

Copyright (C) 2014 Bosch Sensortec GmbH  
Alle Rechte vorbehalten von ROBERT BOSCH GMBH

---

## Unterstützung

Bei Fragen oder Problemen:
1. Überprüfen Sie die [Fehlerbehebung](#fehlerbehebung)
2. Sehen Sie sich die [Beispiele](#beispiele) an
3. Suchen Sie im Arduino-Forum
4. Öffnen Sie ein Issue auf GitHub

---

## Changelog

**Version 1.2.1:**
- Stabilität verbessert
- Beispiele aktualisiert
- Dokumentation erweitert

**Version 1.2:**
- Erste öffentliche Version
- Grundfunktionalität implementiert

---

**Viel Erfolg mit Ihrem BNO055-Projekt! 🚀**

Wenn diese Dokumentation hilfreich war, geben Sie dem Repository einen Stern ⭐ auf GitHub!
