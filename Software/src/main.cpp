/*
A high speed balancing robot, running on an ESP32.

Wouter Klop
wouter@elexperiment.nl
For updates, see elexperiment.nl

Use at your own risk. This code is far from stable.

This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License.
To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/
This basically means: if you use my code, acknowledge it.
Also, you have to publish all modifications.

*/
#include <Arduino.h>
#include <FlySkyIBus.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <Streaming.h>
#include <MPU6050.h>
#include <PID.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WebSocketsServer.h>
#include <FS.h>
#include <SPIFFS.h>
#include <SPIFFSEditor.h>
#include <fastStepper.h>
// #include <par.h>
#include <Preferences.h>  // for storing settings

// Define parameter
#ifndef M_PI
  #define M_PI 3.14159265358979323846
#endif

// Define state variable
#define RUNNING 1
#define STANDBY 0
bool state = STANDBY;
// ----- Input method

// Driving behaviour
float speedFactor = 0.5;  // how strong it reacts to inputs, lower = softer (limits max speed) (between 0 and 1)
float steerFactor = 1.0;  // how strong it reacts to inputs, lower = softer (limits max speed) (between 0 and 1)
float speedFilterConstant = 0.9;  // how fast it reacts to inputs, higher = softer (between 0 and 1, but not 0 or 1)
float steerFilterConstant = 0.9;  // how fast it reacts to inputs, higher = softer (between 0 and 1, but not 0 or 1)

// PPM (called CPPM, PPM-SUM) signal containing 8 RC-Channels in 1 PIN ("RX" on board)
// Channel 1 = steer, Channel 2 = speed
#define INPUT_PPM
#define PPM_PIN 16  // GPIO-Number
#define minPPM 990  // minimum PPM-Value (Stick down)
#define maxPPM 2015  // maximum PPM-Value (Stick up)

// FlySkyIBus signal containing 8 RC-Channels in 1 PIN ("RX" on board)
// #define INPUT_IBUS

// ----- Type definitions
typedef union {
  struct {
    float val; // Float (4 bytes) comes first, as otherwise padding will be applied
    uint8_t cmd;
    uint8_t checksum;
  };
  uint8_t array[6];
} command;

// Plot settings
struct {
  boolean enable = 0; // Enable sending data
  uint8_t prescaler = 4;
} plot;

#define FORMAT_SPIFFS_IF_FAILED true

// ----- Function prototypes
void sendWifiList(void);
void parseSerial();
void parseCommand(char* data, uint8_t length);
void calculateGyroOffset(uint8_t nSample);
void readSensor();
void initSensor(uint8_t n);
void setMicroStep(uint8_t uStep);
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length);
void sendConfigurationData(uint8_t num);

void IRAM_ATTR motLeftTimerFunction();
void IRAM_ATTR motRightTimerFunction();

// ----- Definitions and variables
// -- Web server
const char* http_username = "admin";
const char* http_password = "admin";
AsyncWebServer httpServer(80);
WebSocketsServer wsServer = WebSocketsServer(81);

// -- EEPROM
Preferences preferences;

// -- Stepper motors
#define motEnablePin 27
#define motUStepPin1 14
#define motUStepPin2 12
#define motUStepPin3 13

fastStepper motLeft(5, 4, 0, motLeftTimerFunction);
fastStepper motRight(2, 15, 1, motRightTimerFunction);

uint8_t microStep = 32;
uint8_t motorCurrent = 150;
float maxStepSpeed = 1500;

// -- PID control
#define dT_MICROSECONDS 5000
#define dT dT_MICROSECONDS/1000000.0

#define PID_ANGLE 0
#define PID_POS 1
#define PID_SPEED 2

#define PID_ANGLE_MAX 20
PID pidAngle(cPD, dT, PID_ANGLE_MAX, -PID_ANGLE_MAX);
#define PID_POS_MAX 35
PID pidPos(cPD, dT, PID_POS_MAX, -PID_POS_MAX);
PID pidSpeed(cP, dT, PID_POS_MAX, -PID_POS_MAX);

uint8_t controlMode = 1; // 0 = only angle, 1 = angle+position, 2 = angle+speed

// -- IMU
MPU6050 imu;

#define GYRO_SENSITIVITY 65.5

int16_t gyroOffset[3];
float accAngle = 0;
float filterAngle = 0;
float angleOffset = 2.0;
float gyroFilterConstant = 0.996;
float gyroGain = 1.0;

// -- Others
#define ledPin 2
#define motorCurrentPin 25
#define battVoltagePin 34

// -- WiFi
const char host[] = "balancingrobot";

// Noise source (for system identification)
boolean noiseSourceEnable = 0;
float noiseSourceAmplitude = 1;

// ----- Parameter definitions -----
// void updatePIDParameters() {
//   pidAngle.updateParameters();
//   pidSpeed.updateParameters();
//   pidPos.updateParameters();
// }
// par pidPar[] = {&pidAngle.K, &pidAngle.Ti, &pidAngle.Td, &pidAngle.N, &pidAngle.R, &pidAngle.minOutput, &pidAngle.maxOutput, &pidAngle.controllerType,
//   &pidPos.K, &pidPos.Ti, &pidPos.Td, &pidPos.N, &pidPos.R, &pidPos.minOutput, &pidPos.maxOutput, &pidPos.controllerType,
//   &pidSpeed.K, &pidSpeed.Ti, &pidSpeed.Td, &pidSpeed.N, &pidSpeed.R, &pidSpeed.minOutput, &pidSpeed.maxOutput, &pidSpeed.controllerType, &updatePIDParameters};
//
// parList pidParList(pidPar);

// par motorPar[] = {&motorCurrent, &maxStepSpeed};
// par wifiPar[] = {&wifiMode, &wifiSSID, &wifiKey};
// par sensorPar[] = {&gyroOffset, &gyroGain, &angleOffset, &updateGyroOffset, &updateAngleOffset};
// par controlPar[] = {&remoteType, &controlMode};

// struct {
//   struct {
//     uint8_t mode;
//     char ssid[30];
//     char key[30];
//   } wifi;
// } settings;

// ----- Interrupt functions -----
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR motLeftTimerFunction() {
  portENTER_CRITICAL_ISR(&timerMux);
  motLeft.timerFunction();
  portEXIT_CRITICAL_ISR(&timerMux);
}
void IRAM_ATTR motRightTimerFunction() {
  portENTER_CRITICAL_ISR(&timerMux);
  motRight.timerFunction();
  portEXIT_CRITICAL_ISR(&timerMux);
}


void setMotorCurrent() {
  dacWrite(motorCurrentPin, motorCurrent);
}

void sendData(uint8_t *b, uint8_t l) {
  wsServer.sendBIN(0,b,l);
}

void wirelessTask(void * parameters) {
  while (1) {
    #ifdef INPUT_IBUS
    IBus.loop();
    #endif
    wsServer.loop();
    delay(2);
  }
}

// -- PPM Input
#ifdef INPUT_PPM
volatile int interruptCounter = 0;
int numberOfInterrupts = 0;

// portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

//Array in which the channel values are stored
volatile int32_t rxData[] = {0,0,0,0,0,0,0,0};
//int in which the time difference to the last pulse is stored
volatile uint32_t rxPre = 1;
//int in which the current channel number to read out is stored
volatile uint8_t channelNr = 0; 
//indicates if the data in the channel value array are reliable e.g. there have been two sync breaks, so in the array there are only "real" values synced to the correspondinc channel number
volatile uint8_t firstRoundCounter = 2;
volatile boolean firstRoundPassed = false;
volatile boolean validRxValues = false;

void rxFalling() {  // will be called when the ppm peak is over
  if(micros()-rxPre > 6000) {  // if the current peak is the first peak after the syncro break of 10ms
    rxPre = micros();
    channelNr = 0;  // reset the channel number to syncronize again
    firstRoundCounter--;//the values in the first round are complete rubish, since there would'nt have been a first channel sync, this var indicates for the other functions, if the values are reliable
    if(firstRoundCounter == 0) firstRoundPassed = true;
  }
  else {
    rxData[channelNr] = micros()-rxPre;
    rxPre = micros();
    channelNr++;
  } 
  if(!validRxValues) {
    if(firstRoundPassed) {
        validRxValues = true;
    }
  }
}
#endif

// ----- Main code
void setup() {

  Serial.begin(115200);
  #ifdef INPUT_IBUS
  IBus.begin(Serial2);
  #endif
  preferences.begin("settings", false);  // false = RW-mode
  // preferences.clear();  // Remove all preferences under the opened namespace

  pinMode(motEnablePin, OUTPUT);
  pinMode(motUStepPin1, OUTPUT);
  pinMode(motUStepPin2, OUTPUT);
  pinMode(motUStepPin3, OUTPUT);
  digitalWrite(motEnablePin, 1); // Disable steppers during startup
  setMicroStep(microStep);

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, 0);

  motLeft.init();
  motRight.init();
  motLeft.microStep = microStep;
  motRight.microStep = microStep;

  // SPIFFS setup (to flash the board ota)
  if(!SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED)){
    Serial.println("SPIFFS mount failed");
    return;
  } else {
    Serial.println("SPIFFS mount success");
  }

  // Gyro setup
  delay(200);
  Wire.begin(21,22,400000);
  imu.initialize();
  imu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);
  // Calculate and store gyro offsets
  delay(50);

  // Init EEPROM, if not done before
  #define PREF_VERSION 1  // if setting structure has been changed, count this number up to delete all settings
  if (preferences.getUInt("pref_version", 0) != PREF_VERSION) {
    preferences.clear();  // Remove all preferences under the opened namespace
    preferences.putUInt("pref_version", PREF_VERSION);
    Serial << "EEPROM init complete, all preferences deleted, new pref_version: " << PREF_VERSION << "\n";
  }

  // Read gyro offsets
  Serial << "Gyro calibration values: ";
  for (uint8_t i=0; i<3; i++) {
    char buf[16];
    sprintf(buf, "gyro_offset_%u", i);
    gyroOffset[i] = preferences.getShort(buf, 0);
    Serial << gyroOffset[i] << "\t";
  }
  Serial << endl;

  // Read angle offset
  angleOffset = preferences.getFloat("angle_offset", 0.0);

  // Perform initial gyro measurements
  initSensor(50);

  // Connect to Wifi and setup OTA if known Wifi network cannot be found
  boolean wifiConnected = 0;
  if (preferences.getUInt("wifi_mode", 0)==1) {
    char ssid[63];
    char key[63];
    preferences.getBytes("wifi_ssid", ssid, 63);
    preferences.getBytes("wifi_key", key, 63);
    Serial << "Connecting to '" << ssid << "'" << endl;
    // Serial << "Connecting to '" << ssid << "', '" << key << "'" << endl;
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, key);
    if (!(WiFi.waitForConnectResult() != WL_CONNECTED)) {
      Serial.print("Connected to WiFi with IP address: ");
      Serial.println(WiFi.localIP());
      wifiConnected = 1;
    } else {
      Serial.println("Could not connect to known WiFi network");
    }
  }
  if (!wifiConnected) {
    Serial.println("Starting AP...");
    WiFi.mode(WIFI_AP_STA);
    // WiFi.softAPConfig(apIP, apIP, IPAddress(192,168,178,24));
    WiFi.softAP("balancingRobot", "turboturbo");
    Serial.print("AP started with IP address: ");
    Serial.println(WiFi.softAPIP());
  }

  ArduinoOTA.setHostname(host);
  ArduinoOTA
  .onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";
    Serial.println("Start updating " + type);
  })
  .onEnd([]() {
    Serial.println("\nEnd");
  })
  .onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r\n", (progress / (total / 100)));
  })
  .onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });

  ArduinoOTA.begin();

  // Start DNS server
  if (MDNS.begin(host)) {
    Serial.print("MDNS responder started, name: ");
    Serial.println(host);
  } else {
    Serial.println("Could not start MDNS responder");
  }

  httpServer.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    Serial.println("Loading index.htm");
    request->send(SPIFFS, "/index.htm");
  });

  httpServer.serveStatic("/", SPIFFS, "/");
  httpServer.onNotFound([](AsyncWebServerRequest *request){
      request->send(404, "text/plain", "FileNotFound");
  });

  httpServer.addHandler(new SPIFFSEditor(SPIFFS,http_username,http_password));
  httpServer.begin();

  wsServer.begin();
  wsServer.onEvent(webSocketEvent);

  MDNS.addService("http", "tcp", 80);
  MDNS.addService("ws", "tcp", 81);

  // Make some funny sounds
  // for (uint8_t i=0; i<150; i++) {
  //   motRight.speed = 500 + i*10;
  //   updateStepper(&motRight);
  //   delay(5);
  // }

  dacWrite(motorCurrentPin, motorCurrent);

  pidAngle.setParameters(0.65,0,0.075,15);
  pidPos.setParameters(1,0,1.2,20);
  pidSpeed.setParameters(6,5,0,20);

  // pidParList.read();

  // PPM
  #ifdef INPUT_PPM
  pinMode(PPM_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PPM_PIN), rxFalling, FALLING);
  #endif

  // Run wireless related tasks on core 0
  // xTaskCreatePinnedToCore(
  //                   wirelessTask,   /* Function to implement the task */
  //                   "wirelessTask", /* Name of the task */
  //                   10000,      /* Stack size in words */
  //                   NULL,       /* Task input parameter */
  //                   0,          /* Priority of the task */
  //                   NULL,       /* Task handle. */
  //                   0);  /* Core where the task should run */

  Serial.println("Ready");

}

float steerInput, speedInput;

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
 return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Define variable for the process
static unsigned long tLast = 0;
float pidAngleOutput = 0;
float avgMotSpeed;
float steer = 0;
static float avgSteer;
static float avgSpeed;
static boolean enableControl = 0;
static float avgMotSpeedSum = 0;
int32_t avgMotStep;
float pidPosOutput = 0, pidSpeedOutput = 0;
static uint8_t k = 0;
static float avgBatteryVoltage = 0;
static uint32_t lastInputTime = 0;
uint32_t tNowMs;
float absSpeed = 0;
float noiseValue = 0;

void running() {
  // Read receiver inputs

  #ifdef INPUT_IBUS
    if (IBus.isActive())
    {                                                                       // Check if receiver is active
      speedInput = ((float)IBus.readChannel(1) - 1500) / 5.0 * speedFactor; // Normalise between -100 and 100
      steerInput = ((float)IBus.readChannel(0) - 1500) / 5.0 * steerFactor;
    }
  #endif

  #ifdef INPUT_PPM
    if (rxData[1] == 0 || rxData[0] == 0)
    { // no ppm signal (tx off || rx set to no signal in failsave || no reciever connected (use 100k pulldown))
      speedInput = 0.0;
      steerInput = 0.0;
    }
    else
    {                                                                                                                                // normal ppm signal
      speedInput = mapfloat((float)constrain(rxData[1], minPPM, maxPPM), (float)minPPM, (float)maxPPM, -100.0, 100.0) * speedFactor; // Normalise between -100 and 100
      steerInput = mapfloat((float)constrain(rxData[0], minPPM, maxPPM), (float)minPPM, (float)maxPPM, -100.0, 100.0) * steerFactor;
    }
  #endif

    avgSpeed = speedFilterConstant * avgSpeed + (1 - speedFilterConstant) * speedInput / 5.0;
    avgSteer = steerFilterConstant * avgSteer + (1 - steerFilterConstant) * steerInput;
    // uint8_t lastControlMode = controlMode;
    // controlMode = (2000-IBus.readChannel(5))/450;

    if (abs(avgSpeed) < 0.2)
    {
      // speedInput = 0;
    }
    else
    {
      lastInputTime = tNowMs;
      if (controlMode == 1)
      {
        controlMode = 2;
        motLeft.setStep(0);
        motRight.setStep(0);
        pidSpeed.reset();
      }
    }

    steer = avgSteer;
    // if (abs(avgSteer)>1) {
    //   steer = avgSteer * (1 - abs(avgSpeed)/150.0);
    // } else {
    //   steer = 0;
    // }

    // }

    if (tNowMs - lastInputTime > 2000 && controlMode == 2)
    {
      controlMode = 1;
      motLeft.setStep(0);
      motRight.setStep(0);
      pidPos.reset();
    }

    if (controlMode == 0)
    {
      pidAngle.setpoint = avgSpeed * 2;
    }
    else if (controlMode == 1)
    {
      avgMotStep = (motLeft.getStep() + motRight.getStep()) / 2;
      pidPos.setpoint = avgSpeed;
      pidPos.input = -((float)avgMotStep) / 1000.0;
      pidPosOutput = pidPos.calculate();
      pidAngle.setpoint = pidPosOutput;
    }
    else if (controlMode == 2)
    {
      pidSpeed.setpoint = avgSpeed;
      pidSpeed.input = -avgMotSpeedSum / 100.0;
      pidSpeedOutput = pidSpeed.calculate();
      pidAngle.setpoint = pidSpeedOutput;
    }

    // Optionally, add some noise to angle for system identification purposes
    // if (noiseSourceEnable) {
    //   pidAngle.input = filterAngle + noiseSourceAmplitude*((random(1000)/1000.0)-0.5);
    // } else {
    pidAngle.input = filterAngle;
    // }

    pidAngleOutput = pidAngle.calculate();

    if (noiseSourceEnable)
    {
      noiseValue = noiseSourceAmplitude * ((random(1000) / 1000.0) - 0.5);
      pidAngleOutput += noiseValue;
    }

    avgMotSpeedSum += pidAngleOutput / 2;
    if (avgMotSpeedSum > maxStepSpeed)
    {
      avgMotSpeedSum = maxStepSpeed;
    }
    else if (avgMotSpeedSum < -maxStepSpeed)
    {
      avgMotSpeedSum = -maxStepSpeed;
    }
    avgMotSpeed = avgMotSpeedSum;
    motLeft.speed = avgMotSpeed + steer;
    motRight.speed = avgMotSpeed - steer;

    // Switch microstepping
    absSpeed = abs(avgMotSpeed);
    uint8_t lastMicroStep = microStep;

    if (absSpeed > (150 * 32 / microStep) && microStep > 1)
      microStep /= 2;
    if (absSpeed < (130 * 32 / microStep) && microStep < 32)
      microStep *= 2;

    if (microStep != lastMicroStep)
    {
      motLeft.microStep = microStep;
      motRight.microStep = microStep;
      setMicroStep(microStep);
    }

    // Disable control if robot is almost horizontal. Re-enable if upright.
    if (abs(filterAngle) > 70)
    {
      enableControl = 0;
      motLeft.speed = 0;
      motRight.speed = 0;
      digitalWrite(motEnablePin, 1); // Inverted action on enable pin
    }
}

void reset() {
  if (abs(filterAngle) < 0.5)
  { // (re-)enable and reset stuff
    enableControl = 1;
    controlMode = 1;
    avgMotSpeedSum = 0;
    motLeft.setStep(0);
    motRight.setStep(0);
    pidAngle.reset();
    pidPos.reset();
    pidSpeed.reset();
    digitalWrite(motEnablePin, 0); // Inverted action on enable pin
    // delay(1);
  }
}

void loop() {
/*
  static unsigned long tLast = 0;
  float pidAngleOutput = 0;
  float avgMotSpeed;
  float steer = 0;
  static float avgSteer;
  static float avgSpeed;
  static boolean enableControl = 0;
  static float avgMotSpeedSum = 0;
  int32_t avgMotStep;
  float pidPosOutput = 0, pidSpeedOutput = 0;
  static uint8_t k = 0;
  static float avgBatteryVoltage = 0;
  static uint32_t lastInputTime = 0;
  uint32_t tNowMs;
  float absSpeed = 0;
  float noiseValue = 0;
*/

  unsigned long tNow = micros();
  tNowMs = millis();

  // checking the system every 0.005s
  if (tNow-tLast > dT_MICROSECONDS) {
    // read the sensor then changing the state
    readSensor();
    switch (enableControl) {
      case 1:
        state = RUNNING;
        break;
      case 0:
        state = STANDBY;
        break;
    }
    switch (state) {
      case RUNNING:
        running();
        break;
      case STANDBY:
        reset();
        break;
    }
      
/*
    if (enableControl) {
      // Read receiver inputs

      #ifdef INPUT_IBUS
      if (IBus.isActive()) { // Check if receiver is active
        speedInput = ((float) IBus.readChannel(1)-1500)/5.0 * speedFactor; // Normalise between -100 and 100
        steerInput = ((float) IBus.readChannel(0)-1500)/5.0 * steerFactor;
      }
      #endif

      #ifdef INPUT_PPM
      if (rxData[1] == 0 || rxData[0] == 0) {  // no ppm signal (tx off || rx set to no signal in failsave || no reciever connected (use 100k pulldown))
        speedInput = 0.0;
        steerInput = 0.0;
      } else {  // normal ppm signal
        speedInput = mapfloat((float)constrain(rxData[1], minPPM, maxPPM), (float)minPPM, (float)maxPPM, -100.0, 100.0) * speedFactor;  // Normalise between -100 and 100
        steerInput = mapfloat((float)constrain(rxData[0], minPPM, maxPPM), (float)minPPM, (float)maxPPM, -100.0, 100.0) * steerFactor;
      }
      #endif

      avgSpeed = speedFilterConstant*avgSpeed + (1-speedFilterConstant)*speedInput/5.0;
      avgSteer = steerFilterConstant*avgSteer + (1-steerFilterConstant)*steerInput;
      // uint8_t lastControlMode = controlMode;
      // controlMode = (2000-IBus.readChannel(5))/450;

      if (abs(avgSpeed)<0.2) {
        // speedInput = 0;
      } else {
        lastInputTime = tNowMs;
        if (controlMode==1) {
          controlMode = 2;
          motLeft.setStep(0);
          motRight.setStep(0);
          pidSpeed.reset();
        }
      }

      steer = avgSteer;
      // if (abs(avgSteer)>1) {
      //   steer = avgSteer * (1 - abs(avgSpeed)/150.0);
      // } else {
      //   steer = 0;
      // }

      // }

      if (tNowMs-lastInputTime>2000 && controlMode == 2) {
        controlMode = 1;
        motLeft.setStep(0);
        motRight.setStep(0);
        pidPos.reset();
      }

      if (controlMode == 0) {
        pidAngle.setpoint = avgSpeed*2;
      } else if (controlMode == 1) {
        avgMotStep = (motLeft.getStep() + motRight.getStep())/2;
        pidPos.setpoint = avgSpeed;
        pidPos.input = -((float) avgMotStep) / 1000.0;
        pidPosOutput = pidPos.calculate();
        pidAngle.setpoint = pidPosOutput;
      } else if (controlMode == 2) {
        pidSpeed.setpoint = avgSpeed;
        pidSpeed.input = -avgMotSpeedSum/100.0;
        pidSpeedOutput = pidSpeed.calculate();
        pidAngle.setpoint = pidSpeedOutput;
      }


      // Optionally, add some noise to angle for system identification purposes
      // if (noiseSourceEnable) {
      //   pidAngle.input = filterAngle + noiseSourceAmplitude*((random(1000)/1000.0)-0.5);
      // } else {
        pidAngle.input = filterAngle;
      // }

      pidAngleOutput = pidAngle.calculate();

      if (noiseSourceEnable) {
        noiseValue = noiseSourceAmplitude*((random(1000)/1000.0)-0.5);
        pidAngleOutput += noiseValue;
      }

      avgMotSpeedSum += pidAngleOutput/2;
      if (avgMotSpeedSum>maxStepSpeed) {
        avgMotSpeedSum  = maxStepSpeed;
      } else if (avgMotSpeedSum<-maxStepSpeed) {
        avgMotSpeedSum  = -maxStepSpeed;
      }
      avgMotSpeed = avgMotSpeedSum;
      motLeft.speed = avgMotSpeed + steer;
      motRight.speed = avgMotSpeed - steer;

      // Switch microstepping
      absSpeed = abs(avgMotSpeed);
      uint8_t lastMicroStep = microStep;

      if (absSpeed > (150 * 32 / microStep) && microStep > 1) microStep /= 2;
      if (absSpeed < (130 * 32 / microStep) && microStep < 32) microStep *= 2;

      if (microStep!=lastMicroStep) {
        motLeft.microStep = microStep;
        motRight.microStep = microStep;
        setMicroStep(microStep);
      }

      // Disable control if robot is almost horizontal. Re-enable if upright.
      if (abs(filterAngle)>70) {
        enableControl = 0;
        motLeft.speed = 0;
        motRight.speed = 0;
        digitalWrite(motEnablePin, 1); // Inverted action on enable pin
      }
    } else {
      if (abs(filterAngle)<0.5) { // (re-)enable and reset stuff
        enableControl = 1;
        controlMode = 1;
        avgMotSpeedSum = 0;
        motLeft.setStep(0);
        motRight.setStep(0);
        pidAngle.reset();
        pidPos.reset();
        pidSpeed.reset();
        digitalWrite(motEnablePin, 0); // Inverted action on enable pin
        // delay(1);
      }
    }
*/

    motLeft.update();
    motRight.update();
    // updateStepper(&motLeft);
    // updateStepper(&motRight);

    avgBatteryVoltage = avgBatteryVoltage*0.995 + analogRead(battVoltagePin)*0.0293*0.005;
    
    // plotting the data
    if (k==plot.prescaler) {
      k = 0;

      if (wsServer.connectedClients(0)>0 && plot.enable) {
        union plotData{
          struct {
            uint8_t cmd = 255;
            uint8_t fill1;
            uint8_t fill2;
            uint8_t fill3;
            float f[13];
          };
          uint8_t b[56];
          plotData() {}  // Explicit constructor definition
          ~plotData(){}; // Explicit destructor definition
        } plotData;

        plotData.f[0] = micros()/1000000.0;
        plotData.f[1] = accAngle;
        plotData.f[2] = filterAngle;
        plotData.f[3] = pidAngle.setpoint;
        plotData.f[4] = pidAngle.input;
        plotData.f[5] = pidAngleOutput;
        plotData.f[6] = pidPos.setpoint;
        plotData.f[7] = pidPos.input;
        plotData.f[8] = pidPosOutput;
        plotData.f[9] = pidSpeed.setpoint;
        plotData.f[10] = pidSpeed.input;
        plotData.f[11] = pidSpeedOutput;
        plotData.f[12] = noiseValue;
        wsServer.sendBIN(0, plotData.b, sizeof(plotData.b));
      }
    }
    k++;

    // for (uint8_t i=0; i<6; i++) {
    //   Serial << IBus.readChannel(i) << "\t";
    // }
    // Serial << endl;

    // Serial << speedInput << "\t" << steerInput << endl;

    // Serial << microStep << "\t" << absSpeed << "\t" << endl;

    parseSerial();

    // Serial << micros()-tNow << "\t";

    tLast = tNow;

      // Run other tasks
    ArduinoOTA.handle();
    #ifdef INPUT_IBUS
      IBus.loop();
    #endif
    wsServer.loop();

    // Serial << micros()-tNow << endl;
  }

  // delay(1);
}

void parseSerial() {
  static char serialBuf[10];
  static uint8_t pos = 0;
  char currentChar;

  while (Serial.available()) {
    currentChar = Serial.read();
    serialBuf[pos++] = currentChar;
    if (currentChar == 'x') {
      parseCommand(serialBuf, pos);
      pos = 0;
    }
  }

}

void parseCommand(char* data, uint8_t length) {
  float val2;
  if ((data[length-1]=='x') && length>=3) {
    switch (data[0]) {
      case 'c': { // Change controller parameter
        uint8_t controllerNumber = data[1] - '0';
        char cmd2 = data[2];
        float val = atof(data+3);

        // Make a temporary pid object, in which parameters are updated
        PID pidTemp = pidAngle;
        switch (controllerNumber) {
          case 1: pidTemp = pidAngle; break;
          case 2: pidTemp = pidPos;   break;
          case 3: pidTemp = pidSpeed; break;
        }

        switch (cmd2) {
          case 'p': pidTemp.K = val;  break;
          case 'i': pidTemp.Ti = val; break;
          case 'd': pidTemp.Td = val; break;
          case 'n': pidTemp.N = val; break;
          case 't': pidTemp.controllerType = (uint8_t) val; break;
          case 'm': pidTemp.maxOutput = val; break;
          case 'o': pidTemp.minOutput = -val; break;
        }
        pidTemp.updateParameters();

        // Store temporary pid object in correct pid object
        switch (controllerNumber) {
          case 1: pidAngle = pidTemp; break;
          case 2: pidPos = pidTemp;   break;
          case 3: pidSpeed = pidTemp; break;
        }

        Serial << controllerNumber << "\t" << pidTemp.K << "\t" << pidTemp.Ti << "\t" << pidTemp.Td << "\t" << pidTemp.N << "\t" << pidTemp.controllerType << endl;
        break;
      }
      case 'a': // Change angle offset
        angleOffset = atof(data+1);
        Serial << angleOffset << endl;
        break;
      case 'f':
        gyroFilterConstant = atof(data+1);
        Serial << gyroFilterConstant << endl;
        break;
      case 'v':
        motorCurrent = atof(data+1);
        Serial << motorCurrent << endl;
        dacWrite(motorCurrentPin, motorCurrent);
        break;
      case 'm':
        val2 = atof(data+1);
        Serial << val2 << endl;
        controlMode = val2;
        break;
      case 'u':
        microStep = atoi(data+1);
        setMicroStep(microStep);
        break;
      case 'g':
        gyroGain = atof(data+1);
        break;
      case 'p': {
        switch (data[1]) {
          case 'e':
            plot.enable = atoi(data+2);
            break;
          case 'p':
            plot.prescaler = atoi(data+2);
            break;
          case 'n': // Noise source enable
            noiseSourceEnable = atoi(data+2);
            break;
          case 'a': // Noise source amplitude
            noiseSourceAmplitude = atof(data+2);
            break;
        }
        break;
      }
      // case 'h':
      //   plot.enable = atoi(data+1);
      //   break;
      // case 'i':
      //   plot.prescaler = atoi(data+1);
      //   break;
      case 'j':
        gyroGain = atof(data+1);
        break;
      case 'k': {
        uint8_t cmd2 = atoi(data+1);
        if (cmd2==1) {  // calibrate gyro
          calculateGyroOffset(100);
        } else if (cmd2==2) {  // calibrate acc
          Serial << "Updating angle offset from " << angleOffset;
          angleOffset = filterAngle;
          Serial << " to " << angleOffset << endl;
          preferences.putFloat("angle_offset", angleOffset);
        }
        break;}
      case 'l':
        maxStepSpeed = atof(&data[1]);
        break;
      case 'n':
        gyroFilterConstant = atof(&data[1]);
        break;
      case 'w': {
        char cmd2 = data[1];
        char buf[63];
        uint8_t len;

        switch (cmd2) {
          case 'r':
            Serial.println("Rebooting...");
            ESP.restart();
            // pidParList.sendList(&wsServer);
            break;
          case 'l': // Send wifi networks to WS client
            sendWifiList();
            break;
          case 's': // Update WiFi SSID
            len = length-3;
            memcpy(buf, &data[2], len);
            buf[len] = 0;
            preferences.putBytes("wifi_ssid", buf, 63);
            break;
          case 'k': // Update WiFi key
            len = length-3;
            memcpy(buf, &data[2], len);
            buf[len] = 0;
            preferences.putBytes("wifi_key", buf, 63);
            break;
          case 'm': // WiFi mode (0=AP, 1=use SSID)
            Serial.println(atoi(&data[2]));
            preferences.putUInt("wifi_mode", atoi(&data[2]));
          }
        break;}
    }
  }
}

void sendWifiList(void) {
  char wBuf[200];
  uint8_t n;
  uint16_t pos = 2;

  wBuf[0] = 'w';
  wBuf[1] = 'l';

  Serial.println("Scan started");
  n = WiFi.scanNetworks();

  if (n>5) n = 5; // Limit to first 5 SSIDs

  // Make concatenated list, separated with commas
  for (uint8_t i=0; i<n; i++) {
    pos += sprintf(wBuf + pos, "%s,", WiFi.SSID(i).c_str());
  }
  wBuf[pos-1] = 0;

  Serial.println(wBuf);
  wsServer.sendTXT(0, wBuf);
}

void calculateGyroOffset(uint8_t nSample) {
  int32_t sumX = 0, sumY = 0, sumZ = 0;
  int16_t x, y, z;

  for (uint8_t i=0; i<nSample; i++) {
    imu.getRotation(&x, &y, &z);
    sumX += x;
    sumY += y;
    sumZ += z;
    delay(5);
  }

  gyroOffset[0] = sumX/nSample;
  gyroOffset[1] = sumY/nSample;
  gyroOffset[2] = sumZ/nSample;

  for (uint8_t i=0; i<3; i++) {
    char buf[16];
    sprintf(buf, "gyro_offset_%u", i);
    preferences.putShort(buf, gyroOffset[i]);
  }

  Serial << "New gyro calibration values: " << gyroOffset[0] << "\t" << gyroOffset[1] << "\t" << gyroOffset[2] << endl;
}

void readSensor() {
  int16_t ax, ay, az, gx, gy, gz;
  float deltaGyroAngle;

  imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // accAngle = atan2f((float) ax, (float) az) * 180.0/M_PI;
  // deltaGyroAngle = -((float)((gy - gyroOffset[1])) / GYRO_SENSITIVITY) * dT * gyroGain;
    accAngle = atan2f((float) ay, (float) az) * 180.0/M_PI - angleOffset;
    deltaGyroAngle = ((float)((gx - gyroOffset[0])) / GYRO_SENSITIVITY) * dT * gyroGain;

  filterAngle = gyroFilterConstant * (filterAngle + deltaGyroAngle) + (1 - gyroFilterConstant) * (accAngle);

  // Serial << ay/1000.0 << "\t" << az/1000.0 << "\t" << accAngle << "\t" << filterAngle << endl;
}

void initSensor(uint8_t n) {
  float gyroFilterConstantBackup = gyroFilterConstant;
  gyroFilterConstant = 0.8;
  for (uint8_t i=0; i<n; i++) {
    readSensor();
  }
  gyroFilterConstant = gyroFilterConstantBackup;

}

void setMicroStep(uint8_t uStep) {
  // input:                     1 2 4 8 16 32
  // uStep table corresponds to 0 1 2 3 4  5  in binary on uStep pins
  // So, we need to take the log2 of input
  uint8_t uStepPow = 0;
  while (uStep >>= 1) uStepPow++;

  digitalWrite(motUStepPin1, uStepPow&0x01);
  digitalWrite(motUStepPin2, uStepPow&0x02);
  digitalWrite(motUStepPin3, uStepPow&0x04);
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {

    switch(type) {
        case WStype_DISCONNECTED:
            Serial.printf("[%u] Disconnected!\n", num);
            break;
        case WStype_CONNECTED: {
                IPAddress ip = wsServer.remoteIP(num);
                Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);
                sendConfigurationData(num);
            }
            break;
        case WStype_TEXT:
            Serial.printf("[%u] get Text: %s\n", num, payload);
            parseCommand((char*) payload, length);
            break;
        case WStype_BIN: {
            // Serial.printf("[%u] get binary length: %u\n", num, length);

            // if (length==6) {
            //   cmd c;
            //   memcpy(c.arr, payload, 6);
            //   Serial << "Binary: " << c.grp << "\t" << c.cmd << "\t" << c.val << "\t" << sizeof(cmd) << endl;
            //
            //   if (c.grp<parList::groupCounter) {
            //     if (c.grp==0 && c.cmd<100) {
            //       pidParList.set(c.cmd,c.val);
            //
            //       // pidPar[c.cmd].setFloat(c.val);
            //     }
            //     if (c.cmd==253) {
            //       pidParList.sendList(&wsServer);
            //     }
            //     if (c.cmd==254) {
            //       pidParList.read();
            //     }
            //     if (c.cmd==255) {
            //       pidParList.write();
            //     }
            //   } else if (c.grp==100) {
            //     if (c.cmd==0) {
            //       speedInput = c.val;
            //     } else if (c.cmd==1) {
            //       steerInput = c.val;
            //     }
            //   }
            // }

            break;
          }
		case WStype_ERROR:
		case WStype_FRAGMENT_TEXT_START:
		case WStype_FRAGMENT_BIN_START:
		case WStype_FRAGMENT:
		case WStype_FRAGMENT_FIN:
			break;
    }

}

void sendConfigurationData(uint8_t num) {
  // send message to client
  char wBuf[63];
  char buf[63];
  sprintf(wBuf, "c%dp%.4f", 1, pidAngle.K);
  wsServer.sendTXT(num, wBuf);
  sprintf(wBuf, "c%di%.4f", 1, pidAngle.Ti);
  wsServer.sendTXT(num, wBuf);
  sprintf(wBuf, "c%dd%.4f", 1, pidAngle.Td);
  wsServer.sendTXT(num, wBuf);
  sprintf(wBuf, "c%dn%.4f", 1, pidAngle.N);
  wsServer.sendTXT(num, wBuf);
  sprintf(wBuf, "c%dr%.4f", 1, pidAngle.R);
  wsServer.sendTXT(num, wBuf);
  sprintf(wBuf, "c%dm%.4f", 1, pidAngle.maxOutput);
  wsServer.sendTXT(num, wBuf);
  sprintf(wBuf, "c%do%.4f", 1, -pidAngle.minOutput);
  wsServer.sendTXT(num, wBuf);
  sprintf(wBuf, "c%dp%.4f", 2, pidPos.K);
  wsServer.sendTXT(num, wBuf);
  sprintf(wBuf, "c%di%.4f", 2, pidPos.Ti);
  wsServer.sendTXT(num, wBuf);
  sprintf(wBuf, "c%dd%.4f", 2, pidPos.Td);
  wsServer.sendTXT(num, wBuf);
  sprintf(wBuf, "c%dn%.4f", 2, pidPos.N);
  wsServer.sendTXT(num, wBuf);
  sprintf(wBuf, "c%dr%.4f", 2, pidPos.R);
  wsServer.sendTXT(num, wBuf);
  sprintf(wBuf, "c%dm%.4f", 2, pidPos.maxOutput);
  wsServer.sendTXT(num, wBuf);
  sprintf(wBuf, "c%do%.4f", 2, -pidPos.minOutput);
  wsServer.sendTXT(num, wBuf);
  sprintf(wBuf, "c%dp%.4f", 3, pidSpeed.K);
  wsServer.sendTXT(num, wBuf);
  sprintf(wBuf, "c%di%.4f", 3, pidSpeed.Ti);
  wsServer.sendTXT(num, wBuf);
  sprintf(wBuf, "c%dd%.4f", 3, pidSpeed.Td);
  wsServer.sendTXT(num, wBuf);
  sprintf(wBuf, "c%dn%.4f", 3, pidSpeed.N);
  wsServer.sendTXT(num, wBuf);
  sprintf(wBuf, "c%dr%.4f", 3, pidSpeed.R);
  wsServer.sendTXT(num, wBuf);
  sprintf(wBuf, "c%dm%.4f", 3, pidSpeed.maxOutput);
  wsServer.sendTXT(num, wBuf);
  sprintf(wBuf, "c%do%.4f", 3, -pidSpeed.minOutput);
  wsServer.sendTXT(num, wBuf);
  sprintf(wBuf, "h%.4f", speedFilterConstant);
  wsServer.sendTXT(num, wBuf);
  sprintf(wBuf, "i%.4f", steerFilterConstant);
  wsServer.sendTXT(num, wBuf);
  sprintf(wBuf, "v%d", motorCurrent);
  wsServer.sendTXT(num, wBuf);
  sprintf(wBuf, "j%.4f", gyroGain);
  wsServer.sendTXT(num, wBuf);
  sprintf(wBuf, "n%.4f", gyroFilterConstant);
  wsServer.sendTXT(num, wBuf);
  sprintf(wBuf, "l%.4f", maxStepSpeed);
  wsServer.sendTXT(num, wBuf);
  sprintf(wBuf, "wm%d", preferences.getUInt("wifi_mode", 0));  // 0=AP, 1=Client
  wsServer.sendTXT(num, wBuf);
  preferences.getBytes("wifi_ssid", buf, 63);
  sprintf(wBuf, "ws%s", buf);
  wsServer.sendTXT(num, wBuf);
}
