/*******************************************************************************
* transmits lora messages only. RX is disabled.
* copied the DF Robot SHT20 source into this file to
* allow trimming some things down to reduce the dynamic memory usage
* otherwise the talk2 whisper node will crash on the first print statement
* roughly need around 130 bytes free of dynamic memory
 *******************************************************************************/

//#include <DFRobot_SHT20.h>
#include <Wire.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>




//DFRobot_SHT20 sht20;


// we formerly would check this configuration; but now there is a flag,
// in the LMIC, LMIC.noRXIQinversion;
// if we set that during init, we get the same effect.  If
// DISABLE_INVERT_IQ_ON_RX is defined, it means that LMIC.noRXIQinversion is
// treated as always set.
//
// #if !defined(DISABLE_INVERT_IQ_ON_RX)
// #error This example requires DISABLE_INVERT_IQ_ON_RX to be set. Update \
//        lmic_project_config.h in arduino-lmic/project_config to set it.
// #endif

// How often to send a packet. Note that this sketch bypasses the normal
// LMIC duty cycle limiting, so when you change anything in this sketch
// (payload length, frequency, spreading factor), be sure to check if
// this interval should not also be increased.
// See this spreadsheet for an easy airtime and duty cycle calculator:
// https://docs.google.com/spreadsheets/d/1voGAtQAjC1qBmaVuP1ApNKs1ekgUjavHuVQIXyYSvNc
#define TX_INTERVAL 30000

#define NODEID "YR7RRUhX"
#define ERROR_I2C_TIMEOUT                     998
#define ERROR_BAD_CRC                         999
#define SHT20_I2C_ADDR                        0x40

#define TRIGGER_TEMP_MEASURE_HOLD             0xE3
#define TRIGGER_HUMD_MEASURE_HOLD             0xE5
#define TRIGGER_TEMP_MEASURE_NOHOLD           0xF3
#define TRIGGER_HUMD_MEASURE_NOHOLD           0xF5
#define WRITE_USER_REG                        0xE6
#define READ_USER_REG                         0xE7
#define SOFT_RESET                            0xFE
#define USER_REGISTER_RESOLUTION_MASK         0x81
#define USER_REGISTER_RESOLUTION_RH12_TEMP14  0x00
#define USER_REGISTER_RESOLUTION_RH8_TEMP12   0x01
#define USER_REGISTER_RESOLUTION_RH10_TEMP13  0x80
#define USER_REGISTER_RESOLUTION_RH11_TEMP11  0x81
#define USER_REGISTER_END_OF_BATTERY          0x40
#define USER_REGISTER_HEATER_ENABLED          0x04
#define USER_REGISTER_DISABLE_OTP_RELOAD      0x02

#define MAX_WAIT                              100
#define DELAY_INTERVAL                        10
#define SHIFTED_DIVISOR                       0x988000
#define MAX_COUNTER                           (MAX_WAIT/DELAY_INTERVAL)

TwoWire *_pWire = &Wire;
uint8_t _addr = SHT20_I2C_ADDR;
//int VBat;
//int VIn;
uint8_t _frame = 0;

// Pin mapping
const lmic_pinmap lmic_pins = {
  .nss = 10,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 7,
  .dio = { 2,},
};





void onEvent (ev_t ev) {
}

osjob_t txjob;
//osjob_t timeoutjob;
static void tx_func (osjob_t* job);

// Transmit the given string and call the given function afterwards
void tx(const char *str, osjobcb_t func, uint8_t len) {
  os_radio(RADIO_RST); // Stop RX first
  delay(1); // Wait a bit, without this os_radio below asserts, apparently because the state hasn't changed yet
  LMIC.dataLen = 0;
  while (LMIC.dataLen < len)
    LMIC.frame[LMIC.dataLen++] = *str++;
  LMIC.osjob.func = func;
  os_radio(RADIO_TX);
  Serial.println("TX");
}

// Enable rx mode and call func when a packet is received
void rx(osjobcb_t func) {
  
  LMIC.osjob.func = func;
  LMIC.rxtime = os_getTime(); // RX _now_
  // Enable "continuous" RX (e.g. without a timeout, still stops after
  // receiving a packet)
  os_radio(RADIO_RXON);
  // Serial.println("RX");
}

static void rxtimeout_func(osjob_t *job) {
  digitalWrite(LED_BUILTIN, LOW); // off
}

static void rx_func (osjob_t* job) {
  // Blink once to confirm reception and then keep the led on
  digitalWrite(LED_BUILTIN, LOW); // off
  delay(10);
  digitalWrite(LED_BUILTIN, HIGH); // on

  // Timeout RX (i.e. update led status) after 3 periods without RX
  //os_setTimedCallback(&timeoutjob, os_getTime() + ms2osticks(3*TX_INTERVAL), rxtimeout_func);

  // Reschedule TX so that it should not collide with the other side's
  // next TX
  //os_setTimedCallback(&txjob, os_getTime() + ms2osticks(TX_INTERVAL/2), tx_func);

  // if(LMIC.dataLen > 0){
  // Serial.print("Got ");
  // Serial.print(LMIC.dataLen);
  // Serial.println(" bytes");
  // Serial.write(LMIC.frame, LMIC.dataLen);
  // Serial.println();
  // }

  // Restart RX
  rx(rx_func);
}

static void txdone_func (osjob_t* job) {
  LMIC.dataLen = 0;
  //rx(rx_func);
}

int16_t temp_prev = 0;
uint16_t rh_prev = 0;
bool temp_first = false;
bool rh_first = false;
// log text to USART and toggle LED
static void tx_func (osjob_t* job) {
 char txBuf[32];
 _frame = _frame + 1;
 uint8_t j = 0;
 int16_t temp = ReadTemp();
 int16_t rh = ReadHumidity();

 j += snprintf(txBuf+j,32-j,"#00#%d#%s#%d,%d,",_frame,NODEID,ReadVBat(),ReadVIn());
 if(temp != ERROR_I2C_TIMEOUT*10 && temp != ERROR_BAD_CRC*10 && (abs(temp - temp_prev)<50 || !temp_first)){
    temp_first = true;
    j += snprintf(txBuf+j,32-j,"%d,",temp);
 }
 if(rh != ERROR_I2C_TIMEOUT*10 && rh != ERROR_BAD_CRC*10 && (abs(rh - rh_prev)<50 || !rh_first)){
    rh_first = true;
    j += snprintf(txBuf+j,32-j,"%d",rh);
 }

 if(temp != ERROR_I2C_TIMEOUT*10 && temp != ERROR_BAD_CRC*10)
 {
   temp_prev = temp;
 }
 if(rh != ERROR_I2C_TIMEOUT*10 && rh != ERROR_BAD_CRC*10)
 {
   rh_prev = rh;
 }
  // say hello
  tx(txBuf, txdone_func,j);
  // reschedule job every TX_INTERVAL (plus a bit of random to prevent
  // systematic collisions), unless packets are received, then rx_func
  // will reschedule at half this time.
  os_setTimedCallback(job, os_getTime() + ms2osticks(TX_INTERVAL + random(500)), tx_func);
}

// application entry point
void setup() {
  
  Serial.begin(115200);
  //Serial.println("init");
  initSHT20();                         // Init SHT20 Sensor
  delay(100);
  //Serial.println("c2");
  checkSHT20(); 
  // Set up stdout
  //fdev_setup_stream(&serial_stdout, serial_putchar, NULL, _FDEV_SETUP_WRITE);
  //stdout = &serial_stdout;
  //Serial.println("Starting");
  //#ifdef VCC_ENABLE
  // For Pinoccio Scout boards
  //pinMode(VCC_ENABLE, OUTPUT);
  //digitalWrite(VCC_ENABLE, HIGH);
  //delay(1000);
  //#endif

  pinMode(LED_BUILTIN, OUTPUT);

  // initialize runtime env
  ////Serial.println("os init");
  os_init();
  //Serial.println("os init complete");


  // make it easier for test, by pull the parameters up to the top of the
  // block. Ideally, we'd use the serial port to drive this; or have
  // a voting protocol where one side is elected the controller and
  // guides the responder through all the channels, powers, ramps
  // the transmit power from min to max, and measures the RSSI and SNR.
  // Even more amazing would be a scheme where the controller could
  // handle multiple nodes; in that case we'd have a way to do
  // production test and qualification. However, using an RWC5020A
  // is a much better use of development time.

  // set fDownlink true to use a downlink channel; false
  // to use an uplink channel. Generally speaking, uplink
  // is more interesting, because you can prove that gateways
  // *should* be able to hear you.
  const static bool fDownlink = false;

  // the downlink channel to be used.
  const static uint8_t kDownlinkChannel = 3;

  // the uplink channel to be used.
  const static uint8_t kUplinkChannel = 8 + 3;

  // this is automatically set to the proper bandwidth in kHz,
  // based on the selected channel.
  uint32_t uBandwidth;

  if (! fDownlink)
        {
        if (kUplinkChannel < 64)
                {
                LMIC.freq = AU915_125kHz_UPFBASE +
                            kUplinkChannel * AU915_125kHz_UPFSTEP;
                uBandwidth = 125;
                }
        else
                {
                LMIC.freq = AU915_500kHz_UPFBASE +
                            (kUplinkChannel - 64) * AU915_500kHz_UPFSTEP;
                uBandwidth = 500;
                }
        }
  else
        {
        // downlink channel
        LMIC.freq = AU915_500kHz_DNFBASE +
                    kDownlinkChannel * AU915_500kHz_DNFSTEP;
        uBandwidth = 500;
        }

  // Use a suitable spreading factor
  if (uBandwidth < 500)
        LMIC.datarate = AU915_DR_SF7;         // DR4
  else
        LMIC.datarate = AU915_DR_SF12CR;      // DR8

  // default tx power for AU: 30 dBm
  LMIC.txpow = 30;


  // disable RX IQ inversion
  LMIC.noRXIQinversion = true;

  // This sets CR 4/5, BW125 (except for EU/AS923 DR_SF7B, which uses BW250)
  LMIC.rps = updr2rps(LMIC.datarate);

  Serial.print("Frequency: "); Serial.print(LMIC.freq / 1000000);
            Serial.print("."); Serial.print((LMIC.freq / 100000) % 10);
            Serial.print("MHz");
  Serial.print("  LMIC.datarate: "); Serial.print(LMIC.datarate);
  Serial.print("  LMIC.txpow: "); Serial.println(LMIC.txpow);

  // This sets CR 4/5, BW125 (except for DR_SF7B, which uses BW250)
  LMIC.rps = updr2rps(LMIC.datarate);

  // disable RX IQ inversion
  LMIC.noRXIQinversion = true;

    

  //Serial.println("Started");
  Serial.flush();

  // setup initial job
  os_setCallback(&txjob, tx_func);
}

void loop() {
  //ReadVoltageLevels();
  //ReadSht20();
  // using millis_t = decltype(millis());

  //  // state variables
  //   static millis_t tLastPrint;
  //   static bool fInit = false;

  //   // get the current time
  //   const  millis_t now = millis();

  //   // once each minute, print the current high-frequency time.
  //   // this will give us an indication that the LMIC is up, and also let us check
  //   // that the clock is working properly. Also print opmode and txrx
  //   if (! fInit || (now - tLastPrint) >= 60 * 1000) {
  //     fInit = true;
  //     tLastPrint = now;
  //     Serial.print(os_getTime(), HEX);
  //     Serial.print(" op=");  Serial.print(LMIC.opmode, HEX);
  //     Serial.print(" txrx="); Serial.print(LMIC.txrxFlags, HEX);
  //     Serial.println(" ");
  //  }
  // execute scheduled jobs and events
  
  os_runloop_once();

  
}
// void ReadVoltageLevels(){
//   int vBat = analogRead(A6);
//   int vIn = analogRead(A7);
//   VBat = (int)(1000*(7.282 * vBat) / 1024);
//   VIn = (int)(1000*(7.282 * vIn) / 1024);

// }
int16_t ReadVBat(){
  return (int16_t)((7282 * analogRead(A6)) / 1024);
}
int16_t ReadVIn(){
  return (int16_t)((7282 * analogRead(A7)) / 1024);
}
int16_t ReadHumidity(){
  return (int16_t)(readHumidity()*10);
}
int16_t ReadTemp(){
  return (int16_t)(readTemperature()*10);
}
void initSHT20()
{
    _pWire->begin();
}

float readHumidity(void)
{
    uint16_t rawHumidity = readValue(TRIGGER_HUMD_MEASURE_NOHOLD);
    if(rawHumidity == ERROR_I2C_TIMEOUT || rawHumidity == ERROR_BAD_CRC){
        return(rawHumidity);
    }
    float tempRH = rawHumidity * (125.0 / 65536.0) - 6.0;
    
    return (tempRH);
}

float readTemperature(void)
{
    uint16_t rawTemperature = readValue(TRIGGER_TEMP_MEASURE_NOHOLD);
    if(rawTemperature == ERROR_I2C_TIMEOUT || rawTemperature == ERROR_BAD_CRC){
        return(rawTemperature);
    }
    float tempTemperature = rawTemperature * (175.72 / 65536.0) - 46.85;
    
    return (tempTemperature);
}

void checkSHT20(void)
{
    byte reg = readUserRegister();
    //showReslut("End of battery: ", reg & USER_REGISTER_END_OF_BATTERY);
    //showReslut("Heater enabled: ", reg & USER_REGISTER_HEATER_ENABLED);
    //showReslut("Disable OTP reload: ", reg & USER_REGISTER_DISABLE_OTP_RELOAD);
}

void setResolution(byte resolution)
{
    byte userRegister = readUserRegister();
    userRegister &= B01111110;
    resolution &= B10000001;
    userRegister |= resolution;
    writeUserRegister(userRegister);
}

byte readUserRegister(void)
{
    byte userRegister;
    _pWire->beginTransmission(_addr);
    _pWire->write(READ_USER_REG);
    _pWire->endTransmission();
    _pWire->requestFrom(_addr, (uint8_t)1);
    userRegister = _pWire->read();
    return (userRegister);
}

void writeUserRegister(byte val)
{
    _pWire->beginTransmission(_addr);
    _pWire->write(WRITE_USER_REG);
    _pWire->write(val);
    _pWire->endTransmission();
}

void showReslut(const char *prefix, int val)
{
    Serial.print(prefix);
    if(val){
        Serial.println("yes");
    }else{
        Serial.println("no");
    }
}

byte checkCRC(uint16_t message_from_sensor, uint8_t check_value_from_sensor)
{
    uint32_t remainder = (uint32_t)message_from_sensor << 8;
    remainder |= check_value_from_sensor;
    uint32_t divsor = (uint32_t)SHIFTED_DIVISOR;
    for(int i = 0 ; i < 16 ; i++){
        if(remainder & (uint32_t)1 << (23 - i)){
            remainder ^= divsor;
        }
        divsor >>= 1;
    }
    return (byte)remainder;
}

uint16_t readValue(byte cmd)
{
    _pWire->beginTransmission(_addr);
    _pWire->write(cmd);
    if(0 != _pWire->endTransmission()){   // Used Wire.endTransmission() to end a slave transmission started by beginTransmission() and arranged by write().
      return (ERROR_I2C_TIMEOUT);
    }
    byte toRead;
    byte counter;
    for(counter = 0, toRead = 0 ; counter < MAX_COUNTER && toRead != 3; counter++){
        delay(DELAY_INTERVAL);
        toRead = _pWire->requestFrom(_addr, (uint8_t)3);
    }
    if(counter == MAX_COUNTER){
        return (ERROR_I2C_TIMEOUT);
    }
    byte msb, lsb, checksum;
    msb = _pWire->read();
    lsb = _pWire->read();
    checksum = _pWire->read();
    uint16_t rawValue = ((uint16_t) msb << 8) | (uint16_t) lsb;
    if(checkCRC(rawValue, checksum) != 0){
        return (ERROR_BAD_CRC);
    }
    return rawValue & 0xFFFC;
}
