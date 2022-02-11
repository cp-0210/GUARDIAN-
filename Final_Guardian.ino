#include <AWS_IOT.h>

const char apn[]      = "airtelgprs.com";
const char gprsUser[] = ""; // GPRS User
const char gprsPass[] = ""; // GPRS Password

// Setting serial for debug console
#define SerialMon Serial
// Set serial for AT commands (to SIM800 module)
#define SerialAT Serial1

// Module pins
#define MODEM_TX             17
#define MODEM_RX             16

//MAX30100 I2C connection
#define I2C_SDA              21
#define I2C_SCL              22

//MQTT message topics
char CLIENT_ID[] = "Guardian_ESP32";
#define cnx_sts "testmsg"
#define hr "hr"
#define oxy "oxy"
#define battery "battery"
#define map "map"
char AWS_HOST[] = "a3pckw1b6djb2y-ats.iot.us-east-1.amazonaws.com";

//Battery level monitor config
#define R2 100
#define R3 10
#define VOLTAGE_OUT(Vin) (((Vin) * R3) / (R2 + R3))
#define VOLTAGE_MAX 4200
#define VOLTAGE_MIN 3300
#define ADC_REFERENCE 1100
#define VOLTAGE_TO_ADC(in) ((ADC_REFERENCE * (in)) / 4096)
#define BATTERY_MAX_ADC VOLTAGE_TO_ADC(VOLTAGE_OUT(VOLTAGE_MAX))
#define BATTERY_MIN_ADC VOLTAGE_TO_ADC(VOLTAGE_OUT(VOLTAGE_MIN))
const int Analog_channel_pin = 34;                                    // Pin for battery power check
int ADC_VALUE = 0;

uint32_t tsLastReport = 0;
#define REPORTING_PERIOD_MS     1000

// Configure TinyGSM library
#define TINY_GSM_MODEM_SIM800      // Modem is SIM800
#define TINY_GSM_RX_BUFFER   1024  // Set RX buffer to 1Kb

#include <Wire.h>
#include <TinyGsmClient.h>

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif

#include <AWS_IOT.h>
#include "MAX30100_PulseOximeter.h"

// I2C for SIM800 (to keep it running when powered from battery)
TwoWire I2CPower = TwoWire(0);

// I2C for BME280 sensor
TwoWire I2CBME = TwoWire(1);

PulseOximeter pox;
AWS_IOT var_aws;

// TinyGSM Client for Internet connection
TinyGsmClient client(modem);

#define uS_TO_S_FACTOR 1000000UL   /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  3600        /* Time ESP32 will go to sleep (in seconds) 3600 seconds = 1 hour */

#define IP5306_ADDR          0x75
#define IP5306_REG_SYS_CTL0  0x00

bool setPowerBoostKeepOn(int en) {
  I2CPower.beginTransmission(IP5306_ADDR);
  I2CPower.write(IP5306_REG_SYS_CTL0);
  if (en) {
    I2CPower.write(0x37); // Set bit1: 1 enable 0 disable boost keep on
  } else {
    I2CPower.write(0x35); // 0x37 is default reg value
  }
  return I2CPower.endTransmission() == 0;
}

String heart_rate() {
  Serial.print("Heart rate:");
  Serial.print(pox.getHeartRate());
  return String(pox.getHeartRate(), 2);
}

String spo2_rate() {
  Serial.print("bpm / SpO2:");
  Serial.print(pox.getSpO2());
  Serial.println("%");
  return String(pox.getSpO2(), 2);
}

String calc_battery_percentage(int adc)  {
  int battery_percentage = 100 * (adc - BATTERY_MIN_ADC) / (BATTERY_MAX_ADC - BATTERY_MIN_ADC);

  if (battery_percentage < 0)
    battery_percentage = 0;
  if (battery_percentage > 100)
    battery_percentage = 100;

  return String(battery_percentage);
}


void setup() {
  // Set serial monitor debugging window baud rate to 115200
  SerialMon.begin(115200);

  // Start I2C communication
  I2CPower.begin(I2C_SDA, I2C_SCL, 400000);

  // Keep power when running from battery
  bool isOk = setPowerBoostKeepOn(1);
  SerialMon.println(String("IP5306 KeepOn ") + (isOk ? "OK" : "FAIL"));

  // Set GSM module baud rate and UART pins
  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(3000);

  SerialMon.println("Initializing modem...");
  modem.restart();

  // Configure the wake up source as timer wake up
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);

   Wire.begin(I2C_SDA, I2C_SCL);
    Serial.print("Initializing pulse oximeter..");
  if (!pox.begin()) {
    Serial.println("FAILED");
    for (;;);
  } else {
    Serial.println("SUCCESS");
  }

  Serial.println("\n  Initializing connetction to AWS....");
  if (var_aws.connect(AWS_HOST,CLIENT_ID)==0) { // connects to host and returns 0 upon success
    Serial.println("Connected to AWS");
  }

  else {
    Serial.println(" AWS Connection failed!!");

  }
  Serial.println("  Done.\n\nDone.\n");

  Serial.flush();
}

void loop() {
  SerialMon.print("Connecting to APN: ");
  SerialMon.print(apn);
  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    SerialMon.println(" fail");
  }
  else {
    SerialMon.println(" OK");

    char payload1[2];
    String tts, blt, location,bpmt,spo2t;
    char bl[3],bpm[5],spo2[5],loc[40];

    float tt = touchRead(T0);
    tts = String(tt);
    tts.toCharArray(payload1, 2);              // OR USE itoa(number, character, size of char array);

    ADC_VALUE = analogRead(Analog_channel_pin);
    blt = calc_battery_percentage(ADC_VALUE);
    blt.toCharArray(bl, 3);                                                // battery percent assigned


    pox.update();
     if (millis() - tsLastReport > REPORTING_PERIOD_MS) {
    bpmt = heart_rate();
    spo2t = spo2_rate();
    bpmt.toCharArray(bpm, 5);
    spo2t.toCharArray(spo2, 5);
    tsLastReport = millis();
   }
#if TINY_GSM_TEST_GSM_LOCATION && defined TINY_GSM_MODEM_HAS_GSM_LOCATION
    float lat      = 0;
    float lon      = 0;
    float accuracy = 0;
    int   year     = 0;
    int   month    = 0;
    int   day      = 0;
    int   hour     = 0;
    int   min      = 0;
    int   sec      = 0;
    for (int8_t i = 5; i; i--) {
      DBG("Requesting current GSM location");
      if (modem.getGsmLocation(&lat, &lon, &accuracy, &year, &month, &day, &hour,
                               &min, &sec)) {
        DBG("Latitude:", String(lat, 8), "\tLongitude:", String(lon, 8));
        DBG("Accuracy:", accuracy);
        DBG("Year:", year, "\tMonth:", month, "\tDay:", day);
        DBG("Hour:", hour, "\tMinute:", min, "\tSecond:", sec);
        break;
      } else {
        DBG("Couldn't get GSM location, retrying in 5s.");
        delay(5000L);
      }
    }
    DBG("Retrieving GSM location again as a string");
    String location = modem.getGsmLocation();
    DBG("GSM Based Location String:", location);
#endif

    location.toCharArray(loc, 40);
    
    Serial.println("Publishing payload1:- ");
    var_aws.publish(cnx_sts, "true");
    if (var_aws.publish(hr, bpm) == 0) { // publishes payload and returns 0 upon success
      Serial.print("hr.Success");
    }
    else {
      Serial.println("Failed hr");
    }
    if (var_aws.publish(oxy, spo2) == 0) { // publishes payload and returns 0 upon success
      Serial.print("oxy.Success");
    }
    else {
      Serial.println("Failed oxy");
    }
    if (var_aws.publish(battery, bl) == 0) { // publishes payload and returns 0 upon success
      Serial.print("battery.Success");
    }
    else {
      Serial.println("Failed battery");
    }
    if (var_aws.publish(map, loc) == 0) { // publishes payload and returns 0 upon success
      Serial.print("map.Success");
    }
    else {
      Serial.println("Failed map");
    }
    if (var_aws.publish(cnx_sts, "true") == 0) { // publishes payload and returns 0 upon success
      Serial.print("cnx.Success");
    }
    else {
      Serial.println("Failed sts update");
    }
    delay(1000);
  }

}
