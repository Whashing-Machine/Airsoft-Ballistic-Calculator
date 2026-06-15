#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <svp_sonne.h>
#include <math.h>
#include <ReefwingMPU6x00.h>
#include <Reefwing_imutypes.h>
#include <JC02-1Rangefinder.h>
#include <ReefwingAHRS.h>
#include <sym-lib.h>
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define BLACK 0x0000
#define WHITE 0xFFFF
#define OLED_RESET -1
#define y0 (SCREEN_HEIGHT / 2)   
#define x0  0
#define w  SCREEN_WIDTH      
#define GAMERHEIGHT 180
int dotX, dotY;
static const uint8_t MOSI_PIN = 8;
static const uint8_t MISO_PIN = 10;
static const uint8_t SCLK_PIN = 9;
static const uint8_t CS_PIN  = 7;
static const uint8_t INT_PIN = 1;
static const uint8_t LED0_PIN = A4;
static const uint8_t LED1_PIN = A5;
static MPU6500 imu = MPU6500(SPI, CS_PIN);
static Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
SoftwareSerial rfSerial(10,11); 
Rangefinder rf(rfSerial);

void startupcal() {
  imu.calibrateAccelGyro();

  BiasOffsets gyroCal = imu.getGyroOffsets();   // correct method

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);

  // Check the actual BiasOffsets field names in Reefwing_imuTypes.h
  // e.g. gyroCal.x, or gyroCal.bias[0] — adjust accordingly
  if (gyroCal.x != 0) {
    display.print("Gyroscope Calibrated!");
  } else {
    display.print("Gyroscope Error!");
  }
  display.display();
  delay(3000);
void setup() {
  Serial.begin(115200);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;); 
  }

  pinMode(INT_PIN,  INPUT);
  pinMode(LED0_PIN, OUTPUT);
  pinMode(LED1_PIN, OUTPUT);
  imu.begin();
  display.clearDisplay();
  display.drawBitmap(0, 0, svp_sonne_bitmap, SCREEN_WIDTH, SCREEN_HEIGHT, WHITE);
  display.display();
  delay(3000); 
  startupcal();
}
unsigned long drawreflines() {
  display.drawFastHLine(x0,y0,w,WHITE);
 display.drawEllipse( w/2,y0,SCREEN_WIDTH /2,y0,WHITE);
}
void rollpitchupdate() {
  if (imu.dataAvailable()) {
    imu.readSensor();
    SensorData data = imu.getSensorData();
    ahrs.update(data);
  }

  EulerAngles angles = ahrs.getAngles();
  float angleRad = angles.roll * DEG_TO_RAD;

  const int rx = SCREEN_WIDTH / 2;
  const int ry = y0;

  dotX = (SCREEN_WIDTH / 2) + (int)(rx * cos(angleRad));
  dotY = y0                 + (int)(ry * sin(angleRad));

  display.drawLine(dotX, dotY, x0, y0, WHITE);
  display.drawCircle(dotX, dotY, 6, WHITE);
}

void rangedisplay() {

  float distance = 120;

  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(100, 0);
  display.print(distance);
}

void distancecomp(float range, float result_z) {
  float compensation = (0.24455 * result_z)/(0.00171875 * float range));

  int compX = (int)((dotX + x0) /2);
  int compY = (int)((dotY + y0) /2) - (int)compensation;

  display.drawCircle(compX, compY, 6, WHITE);
 }
void loop() {
  display.clearDisplay();
  drawreflines();
  rollpitchupdate();
  rangedisplay();
  distancecomp(52, 21.2);
  }