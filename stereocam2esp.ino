#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include "stereoprotocol.h"
//#include "time.h"
/* Set these to your desired credentials. */
const char *ssid = "MyAndroidPhone";
const char *password = "1234567890";

#define PPRZ_STX 0x99
#ifndef STEREO_BUF_SIZE
#define STEREO_BUF_SIZE 250*30
#endif

uint16_t insert_loc, extract_loc, msg_start;   // place holders for buffer read and write
// Stereo communication input protocol
uint8_t ser_read_buf[STEREO_BUF_SIZE];           // circular buffer for incoming data
uint8_t msg_buf[STEREO_BUF_SIZE];         // define local data
  

typedef struct {
  uint16_t len;
  uint8_t *data;
  uint8_t fresh;
  uint16_t matrix_width;
  uint16_t matrix_height;
} uint8array;

uint8array stereocam_data; //= {.len = 0, .data = msg_buf, .data_new = 0, .height = 0}; // buffer used to contain image without line endings



/* The different type of images we currently support */
enum image_type {
  IMAGE_YUV422,     ///< UYVY format (uint16 per pixel)
  IMAGE_GRAYSCALE,  ///< Grayscale image with only the Y part (uint8 per pixel)
  IMAGE_JPEG,       ///< An JPEG encoded image (not per pixel encoded)
  IMAGE_GRADIENT    ///< An image gradient (int16 per pixel)
};

volatile uint32_t timeval;
/* Main image structure */
struct image_t {
  enum image_type type;   ///< The image type
  uint16_t w;             ///< Image width
  uint16_t h;             ///< Image height
//  struct timeval ts;      ///< The timestamp of creation

  uint8_t buf_idx;        ///< Buffer index for V4L2 freeing
  uint32_t buf_size;      ///< The buffer size
  void *buf;              ///< Image buffer (depending on the image_type)
};

/* Image point structure */
struct point_t {
  uint16_t x;             ///< The x coordinate of the point
  uint16_t y;             ///< The y coordinate of the point
};


char packetBuffer[255]; //buffer to hold incoming packet
char outBuffer[1255];    //buffer to hold outgoing data
uint16_t out_idx = 0;

WiFiUDP udp;

#define DEZE_IS_HOST 0

IPAddress myIP;
IPAddress broadcastIP(192,168,255,255);

void setup() {
  delay(1000);
  insert_loc = 0;
  extract_loc = 0;
  msg_start = 0;
  stereocam_data.len = 0;
  stereocam_data.data = msg_buf;
  stereocam_data.fresh = 0;
  stereocam_data.matrix_width = 0;
  stereocam_data.matrix_height = 0;
  Serial.begin(115200);
  Serial.println();
  Serial.println("Connnecting to ");
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
    Serial.print(".");
  }
  myIP = WiFi.localIP();
#endif
  Serial.println(myIP);
  MDNS.begin("roland");
}


void loop() {
  ArduinoOTA.handle();
  
  /* Check for Serial data from drone */
  /* Put all serial in_bytes in a buffer */
  while(Serial.available() > 0) {
    unsigned char inbyte = Serial.read();
      
    if (parse_single_byte(inbyte)) { // if complete message detected

      uint16_t lineNumber;
      for(lineNumber = 0; lineNumber < stereocam_data.matrix_height; lineNumber+=1){
          udp.beginPacketMulticast(broadcastIP, 5000, myIP);
          Serial.println(stereocam_data.matrix_width);
          udp.write(stereocam_data.data+lineNumber*stereocam_data.matrix_width+1, stereocam_data.matrix_width-1);
          udp.endPacket();  
          delay(5);
      }
      
      out_idx=0;
    }
  }
  delay(10);
}

uint8_t parse_single_byte(unsigned char in_byte)
{  
  if (handleStereoPackageSmall(in_byte, STEREO_BUF_SIZE, &insert_loc, &extract_loc, &msg_start, msg_buf, ser_read_buf,
                            &stereocam_data.fresh, &stereocam_data.len, &stereocam_data.matrix_width, &stereocam_data.matrix_height)) {
    return 1;  
  }
  return 0;
}
