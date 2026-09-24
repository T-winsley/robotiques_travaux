#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define ECHO_PIN 11
#define TRIGGER_PIN 12

Adafruit_SSD1306 display(128, 64, &Wire, -1);

const int CAR_WIDTH = 30;
const int CAR_Y = 42; 

int carX = (128 - CAR_WIDTH) / 2;

const int distMin = 10;
const int distMax = 50;

const int mapMin = 0;
const int mapMax = 128 - CAR_WIDTH; 

float lastDistance = distMin;

void setup() {
  Serial.begin(9600);

  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();

  display.setTextSize(1);          
  display.setTextColor(SSD1306_WHITE); 
  display.display();
}

void loop() {
  unsigned long currentTime = millis();

  sonarTask(currentTime);
  displayTask(currentTime);
}

void sonarTask(unsigned long ct) {
  const int interval = 250;
  static unsigned long previousTime = 0;

  if (ct - previousTime < interval) {
    return;
  }
  previousTime = ct;

  long duration;
  float distance;

  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);

  digitalWrite(TRIGGER_PIN, LOW);

  duration = pulseIn(ECHO_PIN, HIGH);
  distance = duration * 0.034 / 2;

  if (distance >= 2 && distance <= 400) {
    lastDistance = distance;
  }

  Serial.print("Distance : ");
  Serial.print(lastDistance);
  Serial.println(" cm");

  float clamped = constrain(lastDistance, distMin, distMax);

  carX = map(clamped, distMin, distMax, mapMin, mapMax);
}

void displayTask(unsigned long ct) {
  const int interval = 100;
  static unsigned long previousTime = 0;

  if (ct - previousTime < interval) {
    return;
  }
  previousTime = ct;

  display.clearDisplay();

  drawSun(15, 15);

  drawCar(carX, CAR_Y);

  display.setCursor(80, 0);
  display.print("Winsley");

  display.display();
}

void drawSun(int x, int y) {
  display.fillCircle(x, y, 6, SSD1306_WHITE);

  display.drawLine(x, y - 10, x, y - 14, SSD1306_WHITE);       
  display.drawLine(x, y + 10, x, y + 14, SSD1306_WHITE);       
  display.drawLine(x - 10, y, x - 14, y, SSD1306_WHITE);       
  display.drawLine(x + 10, y, x + 14, y, SSD1306_WHITE);      

  display.drawLine(x - 7, y - 7, x - 10, y - 10, SSD1306_WHITE);
  display.drawLine(x + 7, y - 7, x + 10, y - 10, SSD1306_WHITE);
  display.drawLine(x - 7, y + 7, x - 10, y + 10, SSD1306_WHITE);
  display.drawLine(x + 7, y + 7, x + 10, y + 10, SSD1306_WHITE);
}

void drawCar(int x, int y) {
  display.drawLine(x + 7,  y,      x + 10, y - 6, SSD1306_WHITE);
  display.drawLine(x + 10, y - 6,  x + 20, y - 6, SSD1306_WHITE);
  display.drawLine(x + 20, y - 6,  x + 23, y,     SSD1306_WHITE);

  display.drawLine(x,      y,      x + 7,  y,     SSD1306_WHITE);
  display.drawLine(x + 23, y,      x + 30, y,     SSD1306_WHITE);
  display.drawLine(x,      y,      x + 3,  y + 6, SSD1306_WHITE);
  display.drawLine(x + 30, y,      x + 27, y + 6, SSD1306_WHITE);

  display.drawLine(x + 7, y + 6, x + 23, y + 6, SSD1306_WHITE);

  display.drawCircle(x + 7,  y + 6, 3, SSD1306_WHITE);
  display.drawCircle(x + 23, y + 6, 3, SSD1306_WHITE);
}