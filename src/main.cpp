#include <Arduino.h>
#include <esp_assert.h>

/*
************************************
SD CARD
************************************
*/
#include <SPI.h>
#include <SD.h>
File root;

void printDirectory(File dir, int numTabs) {
  while (true) {

    File entry =  dir.openNextFile();
    if (! entry) {
      // no more files
      break;
    }
    for (uint8_t i = 0; i < numTabs; i++) {
      Serial.print('\t');
    }
    Serial.print(entry.name());
    if (entry.isDirectory()) {
      Serial.println("/");
      printDirectory(entry, numTabs + 1);
    } else {
      // files have sizes, directories do not
      Serial.print("\t\t");
      Serial.println(entry.size(), DEC);
    }
    entry.close();
  }
}

bool mount_sd_card(const uint8_t spi_bus_num = VSPI, const uint8_t attempts_max = 10)
{
  uint8_t count = 0;
  SPI = SPIClass(VSPI);
  while (!SD.begin(5, SPI, 400000U, "/sd", 10U) && count < attempts_max) {
    ESP_LOGE("SD card initialization failed!");
    ++count;
    delay(1000);
  }

  if(count < attempts_max)
  {
    return true;
  }
  return false;
}

void sdcard_test(const String& root_dir = "/")
{
  root = SD.open(root_dir);
  printDirectory(root, 0);
  ESP_LOGD(tag,"sd card test success!");
}

/*
************************************
I2C
************************************
*/

#include <Wire.h>

constexpr int SDA_WIRE0 = 21;
constexpr int SCL_WIRE0 = 22;
#define FREQ_WIRE0 400000L

constexpr int SDA_WIRE1 = 27;
constexpr int SCL_WIRE1 = 26;
#define FREQ_WIRE1 100000L

void scanner(TwoWire& instance)
{
	 byte error, address;
  int nDevices;
  Serial.println("Scanning...");
  nDevices = 0;
  for(address = 1; address < 127; address++ ) {
    instance.beginTransmission(address);
    error = instance.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
      nDevices++;
    }
    else if (error==4) {
      Serial.print("Unknow error at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0) {
    Serial.println("No I2C devices found\n");
  }
  else {
    Serial.println("done\n");
  }
  delay(2000);  
}


/*
************************************
Barometer
************************************
*/

#include <HP20x_dev.h>
#include "Arduino.h"
#include "Wire.h" 
#include <KalmanFilter.h>
unsigned char ret = 0;

/* Instance */
grove::KalmanFilter t_filter;    //temperature filter
grove::KalmanFilter p_filter;    //pressure filter
grove::KalmanFilter a_filter;    //altitude filter

HP20x_dev HP20x(1, SDA_WIRE1, SCL_WIRE1, FREQ_WIRE1);
#define ENABLE_I2C_DEBUG_BUFFER 1

bool barometer_init()
{
  bool found = false;

  Serial.println("****HP20x_dev demo by seeed studio****\n");
  Serial.println("Calculation formula: H = [8.5(101325-P)]/100 \n");
  Wire1.begin(SDA_WIRE1, SCL_WIRE1, FREQ_WIRE1);
  scanner(Wire1);
  /* Power up,delay 150ms,until voltage is stable */
  delay(150);
  /* Reset HP20x_dev */
  HP20x.begin();
  delay(500);
  
  /* Determine HP20x_dev is available or not */
  while(!found)
  {
    ret = HP20x.isAvailable();
    if(OK_HP20X_DEV == ret)
    {
      Serial.println("HP20x_dev is available.\n");   
      found = true; 
    }
    else
    {
      Serial.println("HP20x_dev isn't available.\n");
      delay(500);
      ret = HP20x.isAvailable();
    }
  }
  return found;
}

void barometer_test(uint8_t test_count = 5)
{
  for (size_t i = 0; i < test_count; i++)
  {
    if(OK_HP20X_DEV == ret)
    { 
      long Temper = HP20x.ReadTemperature();
      long Pressure = HP20x.ReadPressure();
      long Altitude = HP20x.ReadAltitude();

      configASSERT(Temper > 0);
      configASSERT(Pressure > 0);
      configASSERT(Altitude != 0);

      float t = Temper/100.0;
      float p = Pressure/100.0;
      float a = Altitude/100.0;

      configASSERT(t > 0.0);
      configASSERT(p > 0.0);
      configASSERT(a != 0.0);

      Serial.println("------------------\n");
      Serial.println("Temper:");
      Serial.print(t);	  
      Serial.println("C.\n");
      Serial.println("Filter:");
      Serial.print(t_filter.Filter(t));
      Serial.println("C.\n");

      Serial.println("Pressure:");
      Serial.print(p);
      Serial.println("hPa.\n");
      Serial.println("Filter:");
      Serial.print(p_filter.Filter(p));
      Serial.println("hPa\n");
      
      Serial.println("Altitude:");
      Serial.print(a);
      Serial.println("m.\n");
      Serial.println("Filter:");
      Serial.print(a_filter.Filter(a));
      Serial.println("m.\n");
      Serial.println("------------------\n");
      delay(1000);
    }
  }
}

/*
************************************
GPS
************************************
*/

/*
************************************
IMU
************************************
*/

#include <stdint.h>
#include <SparkFunMPU9250-DMP.h>
MPU9250_DMP imu;

const signed char orientationMatrix[9] = {
	1, 0, 0,
	0, 1, 0,
	0, 0, 1
};

constexpr int DMP_FIFO_RATE = 5;
constexpr int ACCEL_FSR = 2;
constexpr int mpu_int_pin = 4;
constexpr int SELF_TEST_SUCCESS = 0x07;
constexpr float DECLINATION_NORTH = 20.0;


bool imu_init()
{
  pinMode(mpu_int_pin, INPUT);
  digitalWrite(mpu_int_pin, LOW);

  Wire.begin(SDA_WIRE0, SCL_WIRE0, FREQ_WIRE0);
  scanner(Wire);

 // Call imu.begin() to verify communication and initialize
  if (imu.begin(SDA_WIRE0, SCL_WIRE0) != INV_SUCCESS)
  {
    while (imu.begin(SDA_WIRE0, SCL_WIRE0) != INV_SUCCESS)
    {
      Serial.println("Unable to communicate with MPU-9250");
      Serial.println("Check connections, and try again.");
      Serial.println();
      delay(2000);
    }
  }

  Serial.println("MPU-9250 init ok");
  if ( imu.setAccelFSR(ACCEL_FSR) != INV_SUCCESS)
  {
    ESP_LOGD(tag, "Error: dmpSetOrientation() Failed! \n");
    return false;
  }
  return true;
}

void imu_test()
{
  int self_test_rc = imu.selfTest() ;
  if (self_test_rc != SELF_TEST_SUCCESS)
  {
    ESP_LOGD(tag, "Error: imu selfTest() Failed! \n");

    int gyro_success = self_test_rc & 0x01;
    int accel_success = self_test_rc & 0x02 ;
    int mag_success = self_test_rc & 0x04;

    printf("Error: gyro %d, accel %d, mag %d\n", gyro_success, accel_success, mag_success);
    return;
  }

  if ( imu.dmpSetOrientation(orientationMatrix) != INV_SUCCESS)
  {
    ESP_LOGD(tag, "Error: dmpSetOrientation() Failed!\n");
    return;
  }
 
  if ( imu.dmpEnableFeatures(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_GYRO_CAL | DMP_FEATURE_SEND_RAW_ACCEL) != INV_SUCCESS)
  {
    ESP_LOGD(tag, "Error: dmpEnableFeatures() Failed!\n");
    return;
  }

  if ( imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT | // Enable 6-axis quat
               DMP_FEATURE_GYRO_CAL | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_RAW_GYRO, // Use gyro calibration 
              DMP_FIFO_RATE) != INV_SUCCESS )// Set DMP FIFO rate to 10 Hz
  {
    ESP_LOGD(tag, "Error: dmpSetOrientation() Failed!\n");
    return;
  }

  // DMP_FEATURE_LP_QUAT can also be used. It uses the 
  // accelerometer in low-power mode to estimate quat's.
  // DMP_FEATURE_LP_QUAT and 6X_LP_QUAT are mutually exclusive
  Serial.printf("MPU-9250 DMP ok DMP fifo rate = %d\n", DMP_FIFO_RATE);
  delay(2000);
}

const char * tag = __FILE__;



void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  log_v("Verbose");
  log_d("Debug");
  log_i("Info");
  log_w("Warning"); 
  log_e("Error");

   // wait for ready
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available()) {
    Serial.println(F("Send any character to start sketch.\n"));
    delay(1500);
  }
  while (Serial.available() && Serial.read()); // empty buffer again

  ESP_LOGI("Initializing SD card...");

  bool mount_success = mount_sd_card(VSPI);
  if(mount_success)
  {
    ESP_LOGI(tag, "SD card initialization done.");
    sdcard_test();
  }
  else
  {
    ESP_LOGE(tag, "sd card mount failed!");
  }

  bool baro_init = barometer_init();
  if(!baro_init)
  {
    ESP_LOGE(tag, "Barometer init failed!");
  } 
  else
  {
    barometer_test();
  }

  bool imu_is_init = imu_init();
  if(!imu_is_init)
  {
    ESP_LOGE(tag, "IMU init failed!");
  } 
  else
  {
    imu_test();
  }

  
}

void loop() {
  // nothing happens after setup finishes.
}



