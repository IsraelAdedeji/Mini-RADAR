#include <ESP32Servo.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include <SPI.h>

#define TRIG_PIN 5
#define ECHO_PIN 19
#define SERVO_PIN 13

#define LED_PIN 2
#define BUZZER_PIN 4

#define TFT_CS   15
#define TFT_DC   21
#define TFT_RST  26
#define TFT_SCK  18
#define TFT_MOSI 33

#define OLED_SDA 16
#define OLED_SCL 14
#define OLED_RESET -1

Adafruit_ILI9341 tft = Adafruit_ILI9341(
  TFT_CS,
  TFT_DC,
  TFT_MOSI,
  TFT_SCK,
  TFT_RST
);

Adafruit_SSD1306 oled(128, 64, &Wire, OLED_RESET);

Servo radarServo;

const int minAngle = 15;
const int maxAngle = 165;
const int stepAngle = 2;

const int maxDistance = 100;
const int alertDistance = 35;

const int centerX = 160;
const int centerY = 230;
const int radarRadius = 150;

int currentAngle = minAngle;
int direction = 1;

int lastSweepAngle = -1;
int lastDotX = -1;
int lastDotY = -1;

float currentDistance = -1;
bool objectDetected = false;
bool alertActive = false;

unsigned long lastScanTime = 0;
unsigned long lastOledUpdate = 0;

const unsigned long scanInterval = 45;
const unsigned long oledInterval = 200;

void setup() {
  Serial.begin(115200);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  digitalWrite(LED_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);

  radarServo.setPeriodHertz(50);
  radarServo.attach(SERVO_PIN, 500, 2400);

  tft.begin();
  tft.setRotation(1);
  tft.fillScreen(ILI9341_BLACK);
  drawRadarBase();

  Wire.begin(OLED_SDA, OLED_SCL);

  if (!oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED failed");
  } else {
    oled.clearDisplay();
    oled.display();
  }

  updateOLED();
}

void loop() {
  unsigned long now = millis();

  if (now - lastScanTime >= scanInterval) {
    lastScanTime = now;
    performScanStep();
  }

  if (now - lastOledUpdate >= oledInterval) {
    lastOledUpdate = now;
    updateOLED();
  }
}

void performScanStep() {
  radarServo.write(currentAngle);
  delay(8);

  currentDistance = getDistance();

  objectDetected = currentDistance > 0 && currentDistance <= maxDistance;
  alertActive = currentDistance > 0 && currentDistance <= alertDistance;

  updateAlertOutputs();

  eraseLastSweep();
  eraseLastDot();

  drawSweep(currentAngle);

  if (objectDetected) {
    drawObjectDot(currentAngle, currentDistance);
  }

  lastSweepAngle = currentAngle;

  currentAngle += direction * stepAngle;

  if (currentAngle >= maxAngle) {
    currentAngle = maxAngle;
    direction = -1;
  }

  if (currentAngle <= minAngle) {
    currentAngle = minAngle;
    direction = 1;
  }
}

float getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);

  if (duration == 0) return -1;

  return duration * 0.034 / 2;
}

void drawRadarBase() {
  tft.fillScreen(ILI9341_BLACK);

  tft.drawCircle(centerX, centerY, radarRadius, ILI9341_GREEN);
  tft.drawCircle(centerX, centerY, radarRadius * 2 / 3, ILI9341_DARKGREEN);
  tft.drawCircle(centerX, centerY, radarRadius / 3, ILI9341_DARKGREEN);

  tft.drawLine(centerX - radarRadius, centerY, centerX + radarRadius, centerY, ILI9341_GREEN);

  for (int a = 30; a <= 150; a += 30) {
    float rad = radians(a);
    int x = centerX + radarRadius * cos(rad);
    int y = centerY - radarRadius * sin(rad);
    tft.drawLine(centerX, centerY, x, y, ILI9341_DARKGREEN);
  }
}

void drawSweep(int angle) {
  float rad = radians(angle);
  int x = centerX + radarRadius * cos(rad);
  int y = centerY - radarRadius * sin(rad);

  tft.drawLine(centerX, centerY, x, y, ILI9341_GREEN);
}

void eraseLastSweep() {
  if (lastSweepAngle < 0) return;

  float rad = radians(lastSweepAngle);
  int x = centerX + radarRadius * cos(rad);
  int y = centerY - radarRadius * sin(rad);

  tft.drawLine(centerX, centerY, x, y, ILI9341_BLACK);
  redrawRadarGrid();
}

void redrawRadarGrid() {
  tft.drawCircle(centerX, centerY, radarRadius, ILI9341_GREEN);
  tft.drawCircle(centerX, centerY, radarRadius * 2 / 3, ILI9341_DARKGREEN);
  tft.drawCircle(centerX, centerY, radarRadius / 3, ILI9341_DARKGREEN);
  tft.drawLine(centerX - radarRadius, centerY, centerX + radarRadius, centerY, ILI9341_GREEN);

  for (int a = 30; a <= 150; a += 30) {
    float rad = radians(a);
    int x = centerX + radarRadius * cos(rad);
    int y = centerY - radarRadius * sin(rad);
    tft.drawLine(centerX, centerY, x, y, ILI9341_DARKGREEN);
  }
}

void drawObjectDot(int angle, float distance) {
  int mappedDistance = map((int)distance, 0, maxDistance, 0, radarRadius);

  float rad = radians(angle);
  int x = centerX + mappedDistance * cos(rad);
  int y = centerY - mappedDistance * sin(rad);

  tft.fillCircle(x, y, 5, ILI9341_RED);

  lastDotX = x;
  lastDotY = y;
}

void eraseLastDot() {
  if (lastDotX < 0 || lastDotY < 0) return;

  tft.fillCircle(lastDotX, lastDotY, 6, ILI9341_BLACK);
  redrawRadarGrid();

  lastDotX = -1;
  lastDotY = -1;
}

void updateOLED() {
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(1);

  oled.setCursor(0, 0);
  oled.println("Radar Status");

  oled.setCursor(0, 14);
  oled.print("Angle: ");
  oled.println(currentAngle);

  oled.setCursor(0, 28);
  oled.print("Dist : ");
  if (currentDistance < 0) {
    oled.println("None");
  } else {
    oled.print(currentDistance, 1);
    oled.println(" cm");
  }

  oled.setCursor(0, 42);
  oled.print("Object: ");
  oled.println(objectDetected ? "YES" : "NO");

  oled.setCursor(0, 54);
  oled.print("Alert : ");
  oled.println(alertActive ? "YES" : "NO");

  oled.display();
}

void updateAlertOutputs() {
  if (alertActive) {
    digitalWrite(LED_PIN, HIGH);
    digitalWrite(BUZZER_PIN, HIGH);
  } else {
    digitalWrite(LED_PIN, LOW);
    digitalWrite(BUZZER_PIN, LOW);
  }
}
