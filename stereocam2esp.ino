#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include "stereoprotocol.h"
/* Set these to your desired credentials. */
const char *ssid = "MyAndroidPhone";
const char *password = "1234567890";

unsigned int localPort = 4243; // port to listen on

#define PPRZ_STX 0x99
#ifndef STEREO_BUF_SIZE
#define STEREO_BUF_SIZE 4*64
#endif

uint16_t insert_loc, extract_loc, msg_start;   // place holders for buffer read and write
// Stereo communication input protocol
  uint8_t ser_read_buf[STEREO_BUF_SIZE];           // circular buffer for incoming data
  uint8_t msg_buf[STEREO_BUF_SIZE];         // define local data
  


typedef struct {
  uint8_t len;
  uint8_t *data;
  uint8_t fresh;
  uint8_t matrix_width;
  uint8_t matrix_height;
} uint8array;

extern uint8array stereocam_data;


extern uint8array stereocam_data;




  uint8array stereocam_data; //= {.len = 0, .data = msg_buf, .data_new = 0, .height = 0}; // buffer used to contain image without line endings

/* PPRZ message parser states */
enum normal_parser_states {
  SearchingPPRZ_STX,
  ParsingLength,
  ParsingSenderId,
  ParsingMsgId,
  ParsingMsgPayload,
  CheckingCRCA,
  CheckingCRCB
};

struct normal_parser_t {
  enum normal_parser_states state;
  unsigned char length;
  int counter;
  unsigned char sender_id;
  unsigned char msg_id;
  unsigned char payload[100];
  unsigned char crc_a;
  unsigned char crc_b;
};

struct normal_parser_t parser;

char packetBuffer[255]; //buffer to hold incoming packet
char outBuffer[1255];    //buffer to hold outgoing data
uint16_t out_idx = 0;

WiFiUDP udp;

#define DEZE_IS_HOST 0

IPAddress myIP;
IPAddress broadcastIP(192,168,255,255);

void setup() {
  delay(1000);
    // Open udp socket
  //udp_socket_create(&video_sock, STRINGIFY(VIEWVIDEO_HOST), VIEWVIDEO_PORT_OUT, -1, VIEWVIDEO_BROADCAST);


    insert_loc = 0;
  extract_loc = 0;
  msg_start = 0;
  stereocam_data.len = 0;
  stereocam_data.data = msg_buf;
  stereocam_data.fresh = 0;
  stereocam_data.matrix_width = 0;
  stereocam_data.matrix_height = 0;
  Serial.begin(921600);
  Serial.println();
  Serial.print("Connnecting to ");
  Serial.println(ssid);
  ArduinoOTA.onStart([]() {
    Serial.println("Start");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("End");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\n", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
  /* You can remove the password parameter if you want the AP to be open. */
#if DEZE_IS_HOST
  WiFi.softAP(ssid, password);
  myIP = WiFi.softAPIP();
#else
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    //Serial.print(".");
  }
  myIP = WiFi.localIP();
#endif
  Serial.println(myIP);
  MDNS.begin("roland");
  udp.begin(localPort);
}

void loop() {
  //ArduinoOTA.handle();
  
  /* Check for Serial data from drone */
  /* Put all serial in_bytes in a buffer */
  while(Serial.available() > 0) {
    unsigned char inbyte = Serial.read();
    if (parse_single_byte(inbyte)) { // if complete message detected
      //msg_buf is the image... with length 
      udp.beginPacketMulticast(broadcastIP, 4242, myIP);
      udp.write(outBuffer, out_idx);
      udp.endPacket();
      out_idx=0;
    }
  }
//  udp.beginPacketMulticast(broadcastIP, 4242, myIP);
//  char test[] = "A";
//  udp.write(test, 1);
//  udp.endPacket();
  delay(10);
}

enum parser_state_t{
  first,
  second,
  third,
  fourth,
  counting,
} parser_state = first;

uint16_t linecounter = 0;

uint8_t parse_single_byte(unsigned char in_byte)
{  
  if (handleStereoPackage(in_byte, STEREO_BUF_SIZE, &insert_loc, &extract_loc, &msg_start, msg_buf, ser_read_buf,
                            &stereocam_data.fresh, &stereocam_data.len, &stereocam_data.matrix_width, &stereocam_data.matrix_height)) {
    return 1;  
  }
/*//  switch(parser_state) {
//    case first:
//      out_idx = 0;
//      if (in_byte == 0xFF) parser_state = second;
//      break;
//
//    case second:
//      if (in_byte == 0x00) {
//        parser_state = third;
//      }
//      else {
//        parser_state = first;
//      }
//      break;
//      
//    case third:
//      if (in_byte == 0x00) {
//        parser_state = fourth;
//      }
//      else {
//        parser_state = first;
//      }
//      break;
//
//    case fourth:
//      if (in_byte == 0x80) {
//        parser_state = counting;
//      }
//      else {
//        parser_state = first;
//      }
//      break;
//    case counting:
//      break;
//        
//  }
  
  outBuffer[out_idx++] = in_byte;
  if(out_idx>265){
    outBuffer[out_idx-4] = 255;
    outBuffer[out_idx-3] = 0;
    outBuffer[out_idx-2] = 0;
    outBuffer[out_idx-1] = 0xDA;
    parser_state = first;
    return 1;
  }*/
  return 0;
}
