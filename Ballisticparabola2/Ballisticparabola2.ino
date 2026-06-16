#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <svp_sonne.h>
#include <math.h>
#include <ReefwingMPU6050.h>
#include <Reefwing_imutypes.h>
//#include <JC02-1Rangefinder.h>
//#include <ReefwingAHRS.h>
#include <sym-lib.h>
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
//#define BLACK 0x0000
//#define WHITE 0xFFFF
#define OLED_RESET -1
#define y0 (SCREEN_HEIGHT / 2)   
#define x0  0
#define w  SCREEN_WIDTH      
#define GAMERHEIGHT 180
int dotX, dotY;
static const uint8_t MOSI_PIN = 4;
//static const uint8_t MISO_PIN = 10;
static const uint8_t SCLK_PIN = 3;
//static const uint8_t CS_PIN  = 7;
static const uint8_t INT_PIN = 1;
static const uint8_t LED0_PIN = A4;
static const uint8_t LED1_PIN = A5;
//static MPU6050 imu = MPU6050(SPI, CS_PIN);
static Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
//SoftwareSerial rfSerial(10,11); 
//Rangefinder rf(rfSerial);
//ReefwingAHRS ahrs; 
ReefwingMPU6050 imu;
float timeStep = 0.01;

void startupcal() {
  Serial.println("start up call started");
  Serial.println(imu.connected());
  imu.calibrateGyro();


  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);

  // Check the actual BiasOffsets field names in Reefwing_imuTypes.h
  // e.g. gyroCal.x, or gyroCal.bias[0] — adjust accordingly
  /*
  if (gyroCal.x != 0) {
    display.print("Gyroscope Calibrated!");
  } else {
    display.print("Gyroscope Error!");
  }
  */
  display.display();
  delay(3);
  Serial.println("ended start up");
}
void setup() {
  Serial.begin(115200);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;); 
  }

  pinMode(INT_PIN,  INPUT);
  pinMode(LED0_PIN, OUTPUT);
  pinMode(LED1_PIN, OUTPUT);
  imu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G);
  /*
  ahrs.setDOF(DOF::DOF_6);
  ahrs.setImuType(ImuType::MPU6050);
  ahrs.setFusionAlgorithm(SensorFusion::CLASSIC);
  */
  //ahrs.setDeclination(12.717);      
  display.clearDisplay();
  display.drawBitmap(0, 0, svp_sonne_bitmap, SCREEN_WIDTH, SCREEN_HEIGHT, SSD1306_WHITE);
  display.display();
  delay(3); 
  startupcal();
}
void drawreflines() {
  display.drawFastHLine(x0,y0,w,SSD1306_WHITE);
 display.drawEllipse( w/2,y0,SCREEN_WIDTH /2,y0,SSD1306_WHITE);
}
float angleRad = 0;
void rollpitchupdate() {
  Serial.println("1");
  /*
  ScaledData data ;
  if (imu.dataAvailable()) {
      Serial.println("2");

    imu.readNormalizeGyro();
      Serial.println("3");

    data = imu.readNormalizeGyro();
    /*
    ScaledData data = imu.getSensorData();
    ahrs.setData(data);
    ahrs.update();
    
  }
  */
  //imu.readNormalizeGyro();
  ScaledData data = imu.readNormalizeGyro();
  Serial.println("4");

/*
  EulerAngles angles = ahrs.angles;
  float angleRad = angles.roll * DEG_TO_RAD;
*/

  const int rx = SCREEN_WIDTH / 2;
  const int ry = y0;
    Serial.println("5");

  angleRad = angleRad + data.sx * timeStep;
  dotX = (SCREEN_WIDTH / 2) + (int)(rx * cos(angleRad));
  dotY = y0                 + (int)(ry * sin(angleRad));
  Serial.println(angleRad);
  display.drawLine(dotX, dotY, x0, y0, SSD1306_WHITE);
  display.drawCircle(dotX, dotY, 6, SSD1306_WHITE);
}

void rangedisplay() {

  float distance = 120;

  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(100, 0);
  display.print(distance);
}

void distancecomp(float range, float result_z) {
  float compensation = (0.24455 * result_z)/(0.00171875 * float(range));

  int compX = (int)((dotX + x0) /2);
  int compY = (int)((dotY + y0) /2) - (int)compensation;

  display.drawCircle(compX, compY, 6, SSD1306_WHITE);
}
void loop() {
  Serial.println("looped");
  display.clearDisplay();
  drawreflines();
  rollpitchupdate();
  rangedisplay();
  distancecomp(52, 21.2);
}