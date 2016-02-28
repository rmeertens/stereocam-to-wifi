#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include "stereoprotocol.h"
#include "time.h"
/* Set these to your desired credentials. */
const char *ssid = "MyAndroidPhone";
const char *password = "1234567890";

unsigned int localPort = 4243; // port to listen on

#define PPRZ_STX 0x99
#ifndef STEREO_BUF_SIZE
#define STEREO_BUF_SIZE 4*64
#endif



#define VIEWVIDEO_QUALITY_FACTOR 50
// RTP time increment at 90kHz (default: 0 for automatic)
#define VIEWVIDEO_RTP_TIME_INC 0

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



/**
 * Create a new image
 * @param[out] *img The output image
 * @param[in] width The width of the image
 * @param[in] height The height of the image
 * @param[in] type The type of image (YUV422 or grayscale)
 */
void image_create(struct image_t *img, uint16_t width, uint16_t height)
{
  // Set the variables
//  img->type = type;
  img->w = width;
  img->h = height;

  // Depending on the type the size differs
//  if (type == IMAGE_YUV422) {
    img->buf_size = sizeof(uint8_t) * 2 * width * height;
 /* } else if (type == IMAGE_JPEG) {
    img->buf_size = sizeof(uint8_t) * 2 * width * height;  // At maximum quality this is enough
  } else if (type == IMAGE_GRADIENT) {
    img->buf_size = sizeof(int16_t) * width * height;
  } else {
    img->buf_size = sizeof(uint8_t) * width * height;
  }*/

  img->buf = malloc(img->buf_size);
}

/**
 * Free the image
 * @param[in] *img The image to free
 */
void image_free(struct image_t *img)
{
  if (img->buf != NULL) {
    free(img->buf);
    img->buf = NULL;
  }
}



void image_from_grayscale(struct image_t *input_grayscale, struct image_t *output_yuv422){
  uint8_t *source = (uint8_t*)input_grayscale->buf;
  uint8_t *dest = (uint8_t*)output_yuv422->buf;
  source++;

  // Copy the creation timestamp (stays the same)
//  memcpy(&output_yuv422->ts, &input_grayscale->ts, sizeof(struct timeval));

  // Copy the pixels
  for (int y = 0; y < output_yuv422->h; y++) {
  for (int x = 0; x < output_yuv422->w; x++) {
    if (output_yuv422->type == IMAGE_YUV422) {
    *dest++ = 127;  // U / V
    }
    *dest++ = *source;    // Y
    source += 1;
  }
  }
}




























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

struct image_t coloredImage;
void setup() {
  delay(1000);
    // Open udp socket
  //udp_socket_create(&video_sock, STRINGIFY(VIEWVIDEO_HOST), VIEWVIDEO_PORT_OUT, -1, VIEWVIDEO_BROADCAST);
  
  image_create(&coloredImage, 256,96);
  
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

/*
 * The RTP timestamp is in units of 90000Hz. The same timestamp MUST
 appear in each fragment of a given frame. The RTP marker bit MUST be
 set in the last packet of a frame.
 * @param[in] *udp The UDP socket to send the RTP packet over
 * @param[in] *Jpeg JPEG encoded image byte buffer
 * @param[in] JpegLen The length of the byte buffer
 * @param[in] m_SequenceNumber RTP sequence number
 * @param[in] m_Timestamp Timestamp of the image
 * @param[in] m_offset 3 byte fragmentation offset for fragmented images
 * @param[in] marker_bit RTP marker bit
 * @param[in] w The width of the JPEG image
 * @param[in] h The height of the image
 * @param[in] format_code 0 for YUV422 and 1 for YUV421
 * @param[in] quality_code The JPEG encoding quality
 * @param[in] has_dri_header Whether we have an DRI header or not
 */
static void rtp_packet_send(
  WiFiUDP myudp,
  uint8_t *Jpeg, int JpegLen,
  uint32_t m_SequenceNumber, uint32_t m_Timestamp,
  uint32_t m_offset, uint8_t marker_bit,
  int w, int h,
  uint8_t format_code, uint8_t quality_code,
  uint8_t has_dri_header)
{

#define KRtpHeaderSize 12           // size of the RTP header
#define KJpegHeaderSize 8           // size of the special JPEG payload header

  uint8_t     RtpBuf[2048];
  int         RtpPacketSize = JpegLen + KRtpHeaderSize + KJpegHeaderSize;

  memset(RtpBuf, 0x00, sizeof(RtpBuf));

  /*
   The RTP header has the following format:

    0                   1                   2                   3
    0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
   |V=2|P|X|  CC   |M|     PT      |       sequence number         |
   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
   |                           timestamp                           |
   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
   |           synchronization source (SSRC) identifier            |
   +=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+
   |            contributing source (CSRC) identifiers             |
   |                             ....                              |
   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
   * */

  // Prepare the 12 byte RTP header
  RtpBuf[0]  = 0x80;                               // RTP version
  RtpBuf[1]  = 0x1a + (marker_bit << 7);           // JPEG payload (26) and marker bit
  RtpBuf[2]  = m_SequenceNumber >> 8;
  RtpBuf[3]  = m_SequenceNumber & 0x0FF;           // each packet is counted with a sequence counter
  RtpBuf[4]  = (m_Timestamp & 0xFF000000) >> 24;   // each image gets a timestamp
  RtpBuf[5]  = (m_Timestamp & 0x00FF0000) >> 16;
  RtpBuf[6]  = (m_Timestamp & 0x0000FF00) >> 8;
  RtpBuf[7]  = (m_Timestamp & 0x000000FF);
  RtpBuf[8]  = 0x13;                               // 4 byte SSRC (sychronization source identifier)
  RtpBuf[9]  = 0xf9;                               // we just an arbitrary number here to keep it simple
  RtpBuf[10] = 0x7e;
  RtpBuf[11] = 0x67;

  /* JPEG header", are as follows:
   *
   * http://tools.ietf.org/html/rfc2435

    0                   1                   2                   3
    0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
   | Type-specific |              Fragment Offset                  |
   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
   |      Type     |       Q       |     Width     |     Height    |
   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
   */

  // Prepare the 8 byte payload JPEG header
  RtpBuf[12] = 0x00;                               // type specific
  RtpBuf[13] = (m_offset & 0x00FF0000) >> 16;      // 3 byte fragmentation offset for fragmented images
  RtpBuf[14] = (m_offset & 0x0000FF00) >> 8;
  RtpBuf[15] = (m_offset & 0x000000FF);
  RtpBuf[16] = 0x00;                             // type: 0 422 or 1 421
  RtpBuf[17] = 60;                               // quality scale factor
  RtpBuf[16] = format_code;                      // type: 0 422 or 1 421
  if (has_dri_header) {
    RtpBuf[16] |= 0x40;  // DRI flag
  }
  RtpBuf[17] = quality_code;                     // quality scale factor
  RtpBuf[18] = w / 8;                            // width  / 8 -> 48 pixel
  RtpBuf[19] = h / 8;                            // height / 8 -> 32 pixel
  // append the JPEG scan data to the RTP buffer
  memcpy(&RtpBuf[20], Jpeg, JpegLen);

  //udp_socket_send_dontwait(udp, RtpBuf, RtpPacketSize);
  myudp.beginPacketMulticast(broadcastIP, 4242, myIP);
  myudp.write(RtpBuf, RtpPacketSize);
  myudp.endPacket();
};

/**
 * Send an RTP frame
 * @param[in] *udp The UDP connection to send the frame over
 * @param[in] *img The image to send over the RTP connection
 * @param[in] format_code 0 for YUV422 and 1 for YUV421
 * @param[in] quality_code The JPEG encoding quality
 * @param[in] has_dri_header Whether we have an DRI header or not
 * @param[in] delta_t Time between images (if set to 0 or less it is calculated)
 */
void rtp_frame_send(WiFiUDP myudp, struct image_t *img, uint8_t format_code,
                    uint8_t quality_code, uint8_t has_dri_header, uint32_t delta_t)
{
  static uint32_t packetcounter = 0;
  static uint32_t timecounter = 0;
  uint32_t offset = 0;
  uint32_t jpeg_size = img->buf_size;
  uint8_t *jpeg_ptr = (uint8_t*)img->buf;

#define MAX_PACKET_SIZE 1400
/*
  if (delta_t <= 0) {
    //struct timeval tv;
    gettimeofday(&tv, 0);
    uint32_t t = (tv.tv_sec % (256 * 256)) * 90000 + tv.tv_usec * 9 / 100;
    timecounter = t;
  }*/
    timecounter = 0;

  // Split frame into packets
  for (; jpeg_size > 0;) {
    uint32_t len = MAX_PACKET_SIZE;
    uint8_t lastpacket = 0;

    if (jpeg_size <= len) {
      lastpacket = 1;
      len = jpeg_size;
    }

    rtp_packet_send(myudp, jpeg_ptr, len, packetcounter, timecounter, offset, lastpacket, img->w, img->h, format_code,
                    quality_code, has_dri_header);

    jpeg_size -= len;
    jpeg_ptr  += len;
    offset    += len;
    packetcounter++;
  }


  if (delta_t > 0) {
    // timestamp = 1 / 90 000 seconds
    timecounter += delta_t;
  }
}


void loop() {
  //ArduinoOTA.handle();
  
  /* Check for Serial data from drone */
  /* Put all serial in_bytes in a buffer */
  while(Serial.available() > 0) {
    unsigned char inbyte = Serial.read();
    if (parse_single_byte(inbyte)) { // if complete message detected
    struct image_t grayscaleImage; 

    grayscaleImage.type=IMAGE_GRAYSCALE; 
    grayscaleImage.w=256;
    grayscaleImage.h=96;
    grayscaleImage.buf=outBuffer;
       
    image_from_grayscale(&grayscaleImage, &coloredImage);
    
    rtp_frame_send(udp, &coloredImage,   0,                        // Format 422
        VIEWVIDEO_QUALITY_FACTOR, // Jpeg-Quality
        0,                        // DRI Header
        VIEWVIDEO_RTP_TIME_INC    // 90kHz time increment
      );
      //msg_buf is the image... with length 
//      udp.beginPacketMulticast(broadcastIP, 4242, myIP);
//      udp.write(outBuffer, out_idx);
//      udp.endPacket();
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
