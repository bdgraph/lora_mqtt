/*******************************************************************************
 * Copyright (c) 2015 Matthijs Kooijman
 * Copyright (c) 2018 Terry Moore, MCCI Corporation
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example transmits data on hardcoded channel and receives data
 * when not transmitting. Running this sketch on two nodes should allow
 * them to communicate.
 *******************************************************************************/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#	include <WiFi.h>
#include <PubSubClient.h>
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
#define TX_INTERVAL 15000


// Pin mapping
const lmic_pinmap lmic_pins = {
  .nss = 5,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 15,
  .dio = { 2,4,4},
};


#define SCK 18
#define MISO 32
#define MOSI 23
#define RST 36
#define SS 5



// Add your MQTT Broker IP address, example:
//const char* mqtt_server = "192.168.1.144";
const char* mqtt_server = "192.168.1.114";

WiFiClient wifiClient;
PubSubClient client(wifiClient);
// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in arduino-lmoc/project_config/lmic_project_config.h,
// otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

void onEvent (ev_t ev) {
}

osjob_t txjob;
osjob_t timeoutjob;
static void tx_func (osjob_t* job);

// Transmit the given string and call the given function afterwards
void tx(const char *str, osjobcb_t func) {
  rx(rx_func);
  return;
  os_radio(RADIO_RST); // Stop RX first
  delay(1); // Wait a bit, without this os_radio below asserts, apparently because the state hasn't changed yet
  LMIC.dataLen = 0;
  while (*str)
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
  Serial.println("RX");
}

static void rxtimeout_func(osjob_t *job) {
  //digitalWrite(LED_BUILTIN, LOW); // off
}

static void rx_func (osjob_t* job) {
  // Blink once to confirm reception and then keep the led on
  //digitalWrite(LED_BUILTIN, LOW); // off
  //delay(10);
  //digitalWrite(LED_BUILTIN, HIGH); // on

  // Timeout RX (i.e. update led status) after 3 periods without RX
  //os_setTimedCallback(&timeoutjob, os_getTime() + ms2osticks(3*TX_INTERVAL), rxtimeout_func);

  // Reschedule TX so that it should not collide with the other side's
  // next TX
 // os_setTimedCallback(&txjob, os_getTime() + ms2osticks(TX_INTERVAL/2), tx_func);
  if(LMIC.dataLen > 0){
  Serial.print("Got ");
  Serial.print(LMIC.dataLen);
  Serial.println(" bytes");
  Serial.write(LMIC.frame, LMIC.dataLen);
  Serial.println();

  if(LMIC.frame[0] == '#'
  && LMIC.frame[1] == '0'
  && LMIC.frame[2] == '0'
  && LMIC.frame[3] == '#'){
    Serial.println("Valid Frame Received");
    String value;
    int value_idx;
    char buffVal = LMIC.frame[4];
    int i = 5;
    int frame = 0;
    String nodeid;
    while(buffVal != '#' && i < LMIC.dataLen+1){
      value += buffVal;
      buffVal = LMIC.frame[i];
      i++;
    }

    frame = value.toInt();
    value = "";
    buffVal = LMIC.frame[i];
    i += 1;
    while(buffVal != '#' && i < LMIC.dataLen+1){
      value += buffVal;
      buffVal = LMIC.frame[i];
      i++;
    }

    nodeid = value;
    value = "";
    printf("Frame:%d Node:%s\n",frame,nodeid);
    

    publish(nodeid+"/"+ String(value_idx),String(frame));
    value_idx++;
    
    while(true){
      buffVal = LMIC.frame[i];
      i++;
      while(buffVal != ',' && i < LMIC.dataLen+1){
        value += buffVal;
        buffVal = LMIC.frame[i];
        i++;
      }
      if(value != ""){
        publish(nodeid+"/"+ String(value_idx),value);
      }
      value = "";
      value_idx++;
      
      if(i >= (LMIC.dataLen+1)){
        break;
      }
    }

  }
  }

  // Restart RX
  rx(rx_func);
}

static void txdone_func (osjob_t* job) {
  LMIC.dataLen = 0;
  //rx(rx_func);
}

// log text to USART and toggle LED
static void tx_func (osjob_t* job) {
  // say hello
  tx("goodbye, world!", txdone_func);
  // reschedule job every TX_INTERVAL (plus a bit of random to prevent
  // systematic collisions), unless packets are received, then rx_func
  // will reschedule at half this time.
  os_setTimedCallback(job, os_getTime() + ms2osticks(TX_INTERVAL + random(500)), tx_func);
}

void publish(String topic, String value){
  printf("%s = %s\n",topic,value);
  char topicBuffer[topic.length() + 1];
  char valueBuffer[value.length()+1];

  topic.toCharArray(topicBuffer, topic.length()+1);
  value.toCharArray(valueBuffer, value.length()+1);
  client.publish(topicBuffer,valueBuffer);
}

// application entry point
void setup() {
  Serial.begin(115200);
  // Set up stdout
  
  Serial.println("Starting");
  //#ifdef VCC_ENABLE
  // For Pinoccio Scout boards
  //pinMode(VCC_ENABLE, OUTPUT);
  //digitalWrite(VCC_ENABLE, HIGH);
  //delay(1000);
  //#endif

//  pinMode(LED_BUILTIN, OUTPUT);

  // initialize runtime env
  Serial.println("os init");
  os_init();
  Serial.println("os init complete");
#if defined(CFG_eu868)
  // Use a frequency in the g3 which allows 10% duty cycling.
  LMIC.freq = 869525000;
  // Use a medium spread factor. This can be increased up to SF12 for
  // better range, but then, the interval should be (significantly)
  // raised to comply with duty cycle limits as well.
  LMIC.datarate = DR_SF9;
  // Maximum TX power
  LMIC.txpow = 27;
#elif defined(CFG_us915)
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
                LMIC.freq = US915_125kHz_UPFBASE +
                            kUplinkChannel * US915_125kHz_UPFSTEP;
                uBandwidth = 125;
                }
        else
                {
                LMIC.freq = US915_500kHz_UPFBASE +
                            (kUplinkChannel - 64) * US915_500kHz_UPFSTEP;
                uBandwidth = 500;
                }
        }
  else
        {
        // downlink channel
        LMIC.freq = US915_500kHz_DNFBASE +
                    kDownlinkChannel * US915_500kHz_DNFSTEP;
        uBandwidth = 500;
        }

  // Use a suitable spreading factor
  if (uBandwidth < 500)
        LMIC.datarate = US915_DR_SF7;         // DR4
  else
        LMIC.datarate = US915_DR_SF12CR;      // DR8

  // default tx power for US: 21 dBm
  LMIC.txpow = 21;
#elif defined(CFG_au915)
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
#elif defined(CFG_as923)
// make it easier for test, by pull the parameters up to the top of the
// block. Ideally, we'd use the serial port to drive this; or have
// a voting protocol where one side is elected the controller and
// guides the responder through all the channels, powers, ramps
// the transmit power from min to max, and measures the RSSI and SNR.
// Even more amazing would be a scheme where the controller could
// handle multiple nodes; in that case we'd have a way to do
// production test and qualification. However, using an RWC5020A
// is a much better use of development time.
        const static uint8_t kChannel = 0;
        uint32_t uBandwidth;

        LMIC.freq = AS923_F1 + kChannel * 200000;
        uBandwidth = 125;

        // Use a suitable spreading factor
        if (uBandwidth == 125)
                LMIC.datarate = AS923_DR_SF7;         // DR7
        else
                LMIC.datarate = AS923_DR_SF7B;        // DR8

        // default tx power for AS: 21 dBm
        LMIC.txpow = 16;

        if (LMIC_COUNTRY_CODE == LMIC_COUNTRY_CODE_JP)
                {
                LMIC.lbt_ticks = us2osticks(AS923JP_LBT_US);
                LMIC.lbt_dbmax = AS923JP_LBT_DB_MAX;
                }
#elif defined(CFG_kr920)
// make it easier for test, by pull the parameters up to the top of the
// block. Ideally, we'd use the serial port to drive this; or have
// a voting protocol where one side is elected the controller and
// guides the responder through all the channels, powers, ramps
// the transmit power from min to max, and measures the RSSI and SNR.
// Even more amazing would be a scheme where the controller could
// handle multiple nodes; in that case we'd have a way to do
// production test and qualification. However, using an RWC5020A
// is a much better use of development time.
        const static uint8_t kChannel = 0;
        uint32_t uBandwidth;

        LMIC.freq = KR920_F1 + kChannel * 200000;
        uBandwidth = 125;

        LMIC.datarate = KR920_DR_SF7;         // DR7
        // default tx power for KR: 14 dBm
        LMIC.txpow = KR920_TX_EIRP_MAX_DBM;
        if (LMIC.freq < KR920_F14DBM)
          LMIC.txpow = KR920_TX_EIRP_MAX_DBM_LOW;

        LMIC.lbt_ticks = us2osticks(KR920_LBT_US);
        LMIC.lbt_dbmax = KR920_LBT_DB_MAX;
#elif defined(CFG_in866)
// make it easier for test, by pull the parameters up to the top of the
// block. Ideally, we'd use the serial port to drive this; or have
// a voting protocol where one side is elected the controller and
// guides the responder through all the channels, powers, ramps
// the transmit power from min to max, and measures the RSSI and SNR.
// Even more amazing would be a scheme where the controller could
// handle multiple nodes; in that case we'd have a way to do
// production test and qualification. However, using an RWC5020A
// is a much better use of development time.
        const static uint8_t kChannel = 0;
        uint32_t uBandwidth;

        LMIC.freq = IN866_F1 + kChannel * 200000;
        uBandwidth = 125;

        LMIC.datarate = IN866_DR_SF7;         // DR7
        // default tx power for IN: 30 dBm
        LMIC.txpow = IN866_TX_EIRP_MAX_DBM;
#else
# error Unsupported LMIC regional configuration.
#endif


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
  InitWifi();
  client.setServer(mqtt_server, 1883);
  Serial.println("Started");
  Serial.flush();

  // setup initial job
  os_setCallback(&txjob, tx_func);

  
}

void loop() {
  if (!client.connected()) {
    if (client.connect("lora_gateway")) {
      Serial.println("MQTT connected");
    }    
  }
  using millis_t = decltype(millis());

   // state variables
    static millis_t tLastPrint;
    static bool fInit = false;

    // get the current time
    const  millis_t now = millis();

    // once each minute, print the current high-frequency time.
    // this will give us an indication that the LMIC is up, and also let us check
    // that the clock is working properly. Also print opmode and txrx
    if (! fInit || (now - tLastPrint) >= 60 * 1000) {
      fInit = true;
      tLastPrint = now;
      Serial.print(os_getTime(), HEX);
      Serial.print(" op=");  Serial.print(LMIC.opmode, HEX);
      Serial.print(" txrx="); Serial.print(LMIC.txrxFlags, HEX);
      Serial.println(" ");
   }
  // execute scheduled jobs and events
  os_runloop_once();  
}

void InitWifi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin("TheWifi", "30cupheast");
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }
  Serial.println(WiFi.localIP());
}