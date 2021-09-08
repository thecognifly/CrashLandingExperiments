//
// Adafruit Feather M0 Express ADC data logging
//
// Based on the Adafruit SdFat_circuitpython example
//
// The files can be accessed by double pressing the reset button and
// copying the circuitpython uf2 file (https://circuitpython.org/board/feather_m0_express/)
// into the featherboot drive (must be connected to the computer USB port :P)
// If you want to keep your arduino code, save the CURRENT.UF2 file before copying
// the circuitpython one from the link above.
// After that, it will reboot and mount an USB drive called CIRCUITPY. 
// To go back to arduino mode, double press, but now use the CURRENT.UF2 or just upload
// the sketch using the arduino ide.

#include <SPI.h>
#include <SdFat.h>
#include <Adafruit_SPIFlash.h>

#include <wiring_private.h>


#define LED_BUILTIN 13 // the red LED next to the uUSB connector

// On-board external flash (QSPI or SPI) macros should already
// defined in your board variant if supported
// - EXTERNAL_FLASH_USE_QSPI
// - EXTERNAL_FLASH_USE_CS/EXTERNAL_FLASH_USE_SPI
#if defined(EXTERNAL_FLASH_USE_QSPI)
  Adafruit_FlashTransport_QSPI flashTransport;

#elif defined(EXTERNAL_FLASH_USE_SPI)
  Adafruit_FlashTransport_SPI flashTransport(EXTERNAL_FLASH_USE_CS, EXTERNAL_FLASH_USE_SPI);

#else
  #error No QSPI/SPI flash are defined on your board variant.h !
#endif

/*
Data structure used to save the data to a binary file (save space...)
It occupies only 10 bytes and can be read in Python using:
def read(filename):
  values = []
  with open(filename, 'rb') as fp:
    while True:
      data = fp.read(10)
      if (not data) or (len(data)<10):
        break
      values.append(struct.unpack('<IHHH',data)) # Q=4 and l=2... 4+3x2=10
  return values
*/
typedef struct {
  uint32_t  t;
  uint16_t  x;
  uint16_t  y;
  uint16_t  z;
}IHHH;

const uint32_t DELAY_ADC = 250;

const uint32_t SAMPLE_INTERVAL_US = 1000;

// Total time to be logged
uint32_t TOTAL_LOG_TIME = 2000000; //in uS 2s => 2000000

const char* FILE_BASE_NAME = "acc_";

File datafile;

// Time in micros for next data record.
uint32_t logTime;

uint32_t startTime=0;


Adafruit_SPIFlash flash(&flashTransport);

// file system object from SdFat
FatFileSystem fatfs;

const int _readResolution = 10;
const int _ADCResolution = 10;

// Wait for synchronization of registers between the clock domains
static __inline__ void syncADC() __attribute__((always_inline, unused));
static void syncADC() {
  while (ADC->STATUS.bit.SYNCBUSY == 1)
    ;
}

static inline uint32_t mapResolution(uint32_t value, uint32_t from, uint32_t to)
{
  if (from == to) {
    return value;
  }
  if (from > to) {
    return value >> (from-to);
  }
  return value << (to-from);
}

void vec_analogRead(uint32_t* valueRead, const uint32_t* pins, int size, int udelay1stread)
{
  for(int i=0; i<size; i++){
    pinPeripheral(pins[i], PIO_ANALOG);
  }
  
  syncADC();
  ADC->INPUTCTRL.bit.MUXPOS = g_APinDescription[pins[0]].ulADCChannelNumber; // Selection for the positive ADC input
  
  syncADC();
  ADC->CTRLA.bit.ENABLE = 0x01;             // Enable ADC

  // Start 1st conversion
  syncADC();
  ADC->SWTRIG.bit.START = 1;
  
  while (ADC->INTFLAG.bit.RESRDY == 0);   // Waiting for 1st conversion to complete
  
  // Clear the Data Ready flag
  ADC->INTFLAG.reg = ADC_INTFLAG_RESRDY;

  // These two sequential reads may be too fast for my sensor
  // and therefore the sensor will not charge its output capacitor fast enough.
  delayMicroseconds(udelay1stread);

  for(int i=0; i<size; i++){
  syncADC();
  ADC->INPUTCTRL.bit.MUXPOS = g_APinDescription[pins[i]].ulADCChannelNumber; // Selection for the positive ADC input

  // Start conversion again, since The first conversion after the reference is changed must not be used.
  syncADC();
  ADC->SWTRIG.bit.START = 1;

  // Store the value
  while (ADC->INTFLAG.bit.RESRDY == 0);   // Waiting for conversion to complete
  valueRead[i] = ADC->RESULT.reg;
  }

  syncADC();
  ADC->CTRLA.bit.ENABLE = 0x00;             // Disable ADC

  for(int i=0; i<size; i++){
    valueRead[i] = mapResolution(valueRead[i], _ADCResolution, _readResolution);
  }  
}


// Log a data record.
void logData() {
  IHHH data;
  const int size = 3;
  const uint32_t pins[] = {A1, A2, A3};
  uint32_t valueRead[size];

  data.t = logTime; // this time will be slightly delayed since it was saved later
                    // and the values will only be read after this...
                    // but the distance between reads is within the fs
  // avoid delay from writing to flash...

  vec_analogRead(valueRead, pins, size, DELAY_ADC);
  
  data.z = valueRead[0];//analogRead(A1);
  data.y = valueRead[1];//analogRead(A2);
  data.x = valueRead[2];//analogRead(A3);
  
  // analogRead has a bug in the Adafruit fork that must be fixed... easy to fix though.
  // https://github.com/adafruit/ArduinoCore-samd/issues/254
  
  
  // Write data to file.  
  if(datafile.write(&data, 10) < 10){
    datafile.close();
    error_blink(100);
  }
}

void openFile(char* filename) {
  // Create or append to a file
  // to the end of it.  CircuitPython code can later open and see this file too!
  datafile = fatfs.open(filename, FILE_WRITE); // #define FILE_WRITE (O_RDWR | O_CREAT | O_AT_END)

  if (datafile) {
    Serial.println("File was successfuly open!");
  }
  else {
    Serial.println("Error, failed to open data file for writing!");
    error_blink(100);
  }

}

void closeFile(){
  if (datafile) {
    datafile.flush();
    datafile.close();
    Serial.println("File was successfuly closed!");
  }
}

void error_blink(int dtime){
    while(1){
      digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
      delay(dtime);                       // wait for a while
      digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
      delay(dtime);                       // wait for a while
    }
}

void erase_fs(){
  // if the file system becomes corrupted...
  if (!flash.eraseChip()) {
    Serial.println("Failed to erase chip!");
    error_blink(100);
  }

  flash.waitUntilReady();
  Serial.println("Successfully erased chip!");
  error_blink(500);
}

void setup() {

  uint8_t i=0;

  char filename[11]; //acc_000.bin

  
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  //https://www.arduino.cc/reference/en/language/functions/zero-due-mkr-family/analogreadresolution/
  analogReadResolution(12);
  analogReference(AR_DEFAULT);
  
  // Initialize serial port
  Serial.begin(115200);
  delay(100);
//  while (!Serial) { // wait for the serial port to open before continuing.
//    delay(100);
//  }
  Serial.println("Adafruit M0 Express CircuitPython Flash Example");

  // Initialize flash library and check its chip ID.
  if (!flash.begin()) {
    Serial.println("Error, failed to initialize flash chip!");
    error_blink(100);
  }
  Serial.print("Flash chip JEDEC ID: 0x"); Serial.println(flash.getJEDECID(), HEX);

  // First call begin to mount the filesystem.  Check that it returns true
  // to make sure the filesystem was mounted.
  if (!fatfs.begin(&flash)) {
    Serial.println("Failed to mount filesystem!");
    Serial.println("Was CircuitPython loaded on the board first to create the filesystem?");
    error_blink(100);

  }
  Serial.println("Mounted filesystem!");

  // Look for the last filename
  for(i=0;i<255;i++){
    sprintf(filename, "%s%d", FILE_BASE_NAME, i);
    strcat(filename, ".bin");
    if (fatfs.exists(filename)){
      Serial.println("The filename ");
      Serial.println(filename);
      Serial.println(" already exists!");
    }
    else {
      Serial.println("The filename ");
      Serial.println(filename);
      Serial.println(" doesn't exist!");
      break;
      }
  }

  // Blinks the red LED to indicate it is about to start recording
  // Don't reset it before it reaches the fast bliking phase!!!
  for(int i=0; i<3; i++){
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
  }

  openFile(filename);
//  erase_fs();
  
  // While it is recording data it keeps the red LED on
  digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {
  if (!startTime){
    startTime = micros();
    // Start on a multiple of the sample interval.
    logTime = micros()/(SAMPLE_INTERVAL_US) + 1;
    logTime *= SAMPLE_INTERVAL_US;
  }
  
  // Time for next record.
  logTime += SAMPLE_INTERVAL_US;

  // Wait for log time.
  int32_t diff;

  diff = (micros()-startTime-TOTAL_LOG_TIME);
  if (diff<=0){
    do {
      diff = micros() - logTime + DELAY_ADC;
    } while (diff<0);
  
    logData();

    // flushing the data reduces the max sample rate,
    // but not flusing will probably reduce the maximum amount of time 
    // recording without interruptions...
    if (datafile.getWriteError()) {
      Serial.println("Write error!!!");
      error_blink(100);
    }
  } 
  else if (diff>0) {
    closeFile();
    Serial.print("Current micros(): ");
    Serial.println(micros());
    Serial.print("TOTAL_LOG_TIME: ");
    Serial.println(TOTAL_LOG_TIME);
    Serial.println("Done!");

    //
    // Blinks fast to indicate the recording is over...
    // just reset the board to record another trial.
    //
    digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
    delay(250);                       // wait for a while
    digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
    delay(250);                       // wait for a while
  }

}