//Indoor Positioning Tag Code
#include "dw3000.h"
#include "dw3000_mac_802_15_4.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp_now.h>
#include <string>

#define APP_NAME "SS TWR AES INIT v1.0"

// Enter WiFi credentials here
const char *ssid = "WiFi NAMe";
const char *password = "WiFi PASSWORD";
const char *host = "SERVER HOST ADDRESS"; // Enter IP Address of server, e.g. 192.168.1.32

// MAC Addresses used for ESPNow protocol. Update section to include the MAC Addresses of the devices being used.
uint8_t tag_mac_addr[] =      { 0xEC, 0x62, 0x60, 0xF1, 0xBF, 0xD8 };
uint8_t anchor_mac_addr_1[] = { 0x10, 0x97, 0xBD, 0x5C, 0xB8, 0x60 };
uint8_t anchor_mac_addr_2[] = { 0x10, 0x97, 0xBD, 0x5E, 0x06, 0x20 };
uint8_t anchor_mac_addr_3[] = { 0x10, 0x97, 0xBD, 0x5D, 0xF9, 0xC4 };
uint8_t esp_mac_addrs[4][6] = {{ 0xEC, 0x62, 0x60, 0xF1, 0xBF, 0xD8 },
                               { 0x10, 0x97, 0xBD, 0x5C, 0xB8, 0x60 },
                               { 0x10, 0x97, 0xBD, 0x5E, 0x06, 0x20 },
                               { 0x10, 0x97, 0xBD, 0x5D, 0xF9, 0xC4 }};

float anchor_matrix[3][3] = {
  //list of anchor coordinates, relative to chosen origin.
  { 0, 0, -1.33 },  //Anchor labeled #1
  { 0, 0, -1.33 },  //Anchor labeled #2
  { 0, 0, -1.33 },  //Anchor labeled #3
};

/* Initiator data */
#define ANCHOR_ADDR_1 0x1122334455667788 /* this is the address of the responder */
#define ANCHOR_ADDR_2 0x1122334455667789 /* this is the address of the responder */
#define ANCHOR_ADDR_3 0x112233445566778A /* this is the address of the responder */
#define TAG_ADDR 0x8877665544332211      /* this is the address of the initiator */
#define PAN_ID 0x4321                    /* this is the PAN ID used in this example */

#define RX_BUF_LEN 127 /* The received frame cannot be bigger than 127 if STD PHR mode is used */

/* Inter-ranging delay period, in milliseconds. */
#define RNG_DELAY_MS 300

/* Default antenna delay values for 64 MHz PRF. See NOTE 2 below. */
#define TX_ANT_DLY 16384
#define RX_ANT_DLY 16384

#define START_RECEIVE_DATA_LOCATION 8  //MAC payload user data starts at index 8 (e.g. 'R' - in above response message)

/* Indexes to access some of the fields in the frames defined above. */
#define ALL_MSG_SN_IDX 2           //sequence number byte index in MHR
#define RESP_MSG_POLL_RX_TS_IDX 0  //index in the MAC payload for Poll RX time
#define RESP_MSG_RESP_TX_TS_IDX 4  //index in the MAC payload for Response TX time
#define RESP_MSG_TS_LEN 4

/* Note, the key index of 0 is forbidden to send as key index. Thus index 1 is the first.
 * This example uses this index for the key table for the encryption of initiator's data */
#define INITIATOR_KEY_INDEX 1

/* Delay between frames, in UWB microseconds. See NOTE 1 below. */
#define POLL_TX_TO_RESP_RX_DLY_UUS 1720
/* Receive response timeout. See NOTE 5 below. */
#define RESP_RX_TIMEOUT_UUS 250

// connection pins
const uint8_t PIN_RST = 27;  // reset pin
const uint8_t PIN_IRQ = 34;  // irq pin
const uint8_t PIN_SS = 4;    // spi select pin

float current_tag_position[2] = { 0, 0 };
float current_distance_rmse = 0.0;

const int port = 80;
WiFiClient client;
WiFiUDP udp;

float d[3];  // distances from anchors

bool connected = false;

// Control Signals
bool msg_recved = false;  // true if message has been recieved.
int instr_success = -1;   // -1 means not applicable, 0 means instruction failed, 1 means instruction executed successfully
double data_recieved = -1;// -1 means data failed to arrive.
int data_delivered = -1; // -1 means data has not been delivered yet. 0 means data failed to deliver. 1 means data has been delivered successfully.
bool anchors_changed = false;
int communication_mode = 0; // 0 = wifi, 1 = ESP NOW
bool anchors_set = false;

typedef struct msg_data {
  int type;        // Instruction (0), Confirmation (1), Data (2)
  int sender;      // Device message was sent from
  int reciever;    // Device message is sent to
  int connect_to;  // Device to connect to.
  int arg1;        //Used as conformation when sending data back to tag. Confirmation: Failed (0)	Success(1). Also used as number of times to average ranging out.
  double data;     // Instruction, Confirmation, or Data. Instruction: Change to tag(0)	Change to anchor(1)	Get Range (2). Confirmation: Failed (0)	Success(1)
} msg_data;

msg_data in_message;

esp_now_peer_info_t peerInfo1;
esp_now_peer_info_t peerInfo2;
esp_now_peer_info_t peerInfo3;

mac_frame_802_15_4_format_t mac_frame = {
  { { 0x09, 0xEC },
    0x00,
    { 0x21, 0x43 },
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
    { 0x0F, { 0x00, 0x00, 0x00, 0x00 }, 0x00 } },
  0x00
};

static dwt_aes_config_t aes_config = {
  AES_key_RAM,
  AES_core_type_CCM,
  MIC_0,
  AES_KEY_Src_Register,
  AES_KEY_Load,
  0,
  AES_KEY_128bit,
  AES_Encrypt
};

dwt_aes_key_t keys_options[NUM_OF_KEY_OPTIONS] = {
  { 0x00010203, 0x04050607, 0x08090A0B, 0x0C0D0E0F, 0x00000000, 0x00000000, 0x00000000, 0x00000000 },
  { 0x11223344, 0x55667788, 0x99AABBCC, 0xDDEEFF00, 0x00000000, 0x00000000, 0x00000000, 0x00000000 },
  { 0xFFEEDDCC, 0xBBAA9988, 0x77665544, 0x33221100, 0x00000000, 0x00000000, 0x00000000, 0x00000000 }
};

class Device {
public:
  mac_frame_802_15_4_format_t mac_frame;
  long long unsigned int SRC_ADDR;
  long long unsigned int DEST_ADDR;
  long long unsigned int DEST_PAN_ID;
  uint32_t frame_cnt;
  uint8_t seq_cnt;
  uint32_t status_reg;
  int8_t status;
  uint8_t nonce[13]; /* 13-byte nonce used in this example as per IEEE802.15.4 */
  uint8_t rx_buffer[RX_BUF_LEN];
  dwt_aes_job_t aes_job_tx, aes_job_rx;
  dwt_aes_config_t aes_config;
  dwt_aes_key_t *keys_options;
  /* MAC payload data of the frames used in the ranging process. See NOTE 3 below. */
  /* Poll message from the initiator to the responder */
  uint8_t tx_poll_msg[12] = { 'P', 'o', 'l', 'l', ' ', 'm', 'e', 's', 's', 'a', 'g', 'e' };
  /* Response message to the initiator. The first 8 bytes are used for Poll RX time and Response TX time.*/
  uint8_t rx_resp_msg[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 'R', 'e', 's', 'p', 'o', 'n', 's', 'e' };
  Device(mac_frame_802_15_4_format_t mac, long long unsigned int pan, long long unsigned int addr, long long unsigned int src, dwt_aes_config_t aes_con, dwt_aes_key_t *key_op) {
    mac_frame = mac;
    DEST_ADDR = addr;
    DEST_PAN_ID = pan;
    SRC_ADDR = src;
    frame_cnt = 0;  /* See Note 13 */
    seq_cnt = 0x0A; /* Frame sequence number, incremented after each transmission. */
    aes_config = aes_con;
    keys_options = key_op;
  }
  Device(Device &t)  //copy constructor
  {
    mac_frame = t.mac_frame;
    DEST_ADDR = t.DEST_ADDR;
    DEST_PAN_ID = t.DEST_PAN_ID;
    SRC_ADDR = t.SRC_ADDR;
    frame_cnt = t.frame_cnt;
    seq_cnt = t.seq_cnt;
    aes_config = t.aes_config;
    keys_options = t.keys_options;
    aes_job_tx = t.aes_job_tx;
    aes_job_rx = t.aes_job_rx;
    status = t.status;
    status_reg = t.status_reg;
  }
  uint32_t inc_frame() {
    frame_cnt++;
    return frame_cnt;
  }
  uint8_t inc_seq() {
    seq_cnt++;
    return seq_cnt;
  }
};

Device anchor1(mac_frame, PAN_ID, ANCHOR_ADDR_1, TAG_ADDR, aes_config, keys_options);
Device anchor2(mac_frame, PAN_ID, ANCHOR_ADDR_2, TAG_ADDR, aes_config, keys_options);
Device anchor3(mac_frame, PAN_ID, ANCHOR_ADDR_3, TAG_ADDR, aes_config, keys_options);
Device devices[3] = { anchor1, anchor2, anchor3 };

/* Default communication configuration. We use default non-STS DW mode. */
static dwt_config_t config = {
  5,                /* Channel number. */
  DWT_PLEN_128,     /* Preamble length. Used in TX only. */
  DWT_PAC8,         /* Preamble acquisition chunk size. Used in RX only. */
  9,                /* TX preamble code. Used in TX only. */
  9,                /* RX preamble code. Used in RX only. */
  1,                /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
  DWT_BR_6M8,       /* Data rate. */
  DWT_PHRMODE_STD,  /* PHY header mode. */
  DWT_PHRRATE_STD,  /* PHY header rate. */
  (129 + 8 - 8),    /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
  DWT_STS_MODE_OFF, /* STS disabled */
  DWT_STS_LEN_64,   /* STS length see allowed values in Enum dwt_sts_lengths_e */
  DWT_PDOA_M0       /* PDOA mode off */
};

/* Values for the PG_DELAY and TX_POWER registers reflect the bandwidth and power of the spectrum at the current
 * temperature. These values can be calibrated prior to taking reference measurements. See NOTE 2 below. */
extern dwt_txconfig_t txconfig_options;

void clear_msg() {
  msg_recved = false;
  instr_success = -1;
  data_recieved = -1;
  data_delivered = -1;
  // clear in_message
  in_message.type = -1;
  in_message.sender = -1;
  in_message.reciever = -1;
  in_message.arg1 = -1;
  in_message.connect_to = -1;
  in_message.data = -1;
  return;
}

void msg_parser(msg_data in) {
  bool first = true;
  while (msg_recved == true) {  // message already in buffer, wait to handle new message.
    if (first) {
      Serial.println("Waiting for previous message to clear out");
      first = false;
    }
    delay(1);
  }
  msg_recved = true;                  // bool value turned to false after message has been fully processed
  if (in_message.type == 1) {         // Confirmation type.
    if ((int)in_message.type == 0) {  // Instruction failed to execute, handle failure.
      instr_success = 0;
      Serial.println("Instruction failed to execute.");
    } else {
      instr_success = 1;
      // Serial.println("Instruction Executed Properly.");
    }
  } else if (in_message.type == 2) {
    if (in_message.arg1 == 0) {
      Serial.println("Failed to get data");
      instr_success = 0;
    } else {
      data_recieved = in_message.data;
      instr_success = 1;
    }
  } else {
    Serial.println("Message type invalid");
    Serial.println("Type: " + String(in_message.type));
    clear_msg();
  }
  return;
}

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Serial.print("\r\nLast Packet Send Status:\t");
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status == ESP_NOW_SEND_SUCCESS) {
    data_delivered = 1;
  }
  else {
    data_delivered = 0;
  }
}

// Callback when data is received
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  // Serial.println("Message Recieved");
  memcpy(&in_message, incomingData, sizeof(in_message));
  msg_parser(in_message);
}

void connectToWiFi(const char *ssid, const char *pwd) {
  Serial.println("Connecting to WiFi network: " + String(ssid));

  // delete old config
  WiFi.disconnect(true);
  //register event handler
  WiFi.onEvent(WiFiEvent);

  //Initiate connection
  WiFi.begin(ssid, pwd);

  Serial.println("Waiting for WIFI connection...");
  return;
}

//wifi event handler
void WiFiEvent(WiFiEvent_t event) {
  switch (event) {
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      //When connected set
      Serial.print("WiFi connected! IP address: ");
      Serial.println(WiFi.localIP());
      //initializes the UDP state
      //This initializes the transfer buffer
      udp.begin(WiFi.localIP(), port);
      connected = true;
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      connected = false;
      break;
    default: break;
  }
}

double ranging(Device *dev, int n) {
  int count = 0;
  int count1 = 0;
  double avgdist = 0;
  static double tof;
  static double distance;
  while (count < n && count1 < n * 5) {
    count1++;
    /* Program the correct key to be used */
    dwt_set_keyreg_128(&dev->keys_options[INITIATOR_KEY_INDEX - 1]);
    /* Set the key index for the frame */
    MAC_FRAME_AUX_KEY_IDENTIFY_802_15_4(&dev->mac_frame) = INITIATOR_KEY_INDEX;

    /* Update MHR to the correct SRC and DEST addresses and construct the 13-byte nonce
         * (same MAC frame structure is used to store both received data and transmitted data - thus SRC and DEST addresses
         * need to be updated before each transmission */
    mac_frame_set_pan_ids_and_addresses_802_15_4(&dev->mac_frame, dev->DEST_PAN_ID, dev->DEST_ADDR, dev->SRC_ADDR);
    mac_frame_get_nonce(&dev->mac_frame, dev->nonce);

    dev->aes_job_tx.mic_size = mac_frame_get_aux_mic_size(&dev->mac_frame);
    dev->aes_config.mode = AES_Encrypt;
    dev->aes_config.mic = dwt_mic_size_from_bytes(dev->aes_job_tx.mic_size);
    dwt_configure_aes(&dev->aes_config);

    /* The AES job will take the TX frame data and and copy it to DW IC TX buffer before transmission. See NOTE 7 below. */
    dev->status = dwt_do_aes(&dev->aes_job_tx, dev->aes_config.aes_core_type);

    /* Check for errors */
    if (dev->status < 0) {
      test_run_info((unsigned char *)"AES length error");
      while (1)
        ; /* Error */
    } else if (dev->status & AES_ERRORS) {
      test_run_info((unsigned char *)"ERROR AES");
      while (1)
        ; /* Error */
    }


    /* configure the frame control and start transmission */
    dwt_writetxfctrl(dev->aes_job_tx.header_len + dev->aes_job_tx.payload_len + dev->aes_job_tx.mic_size + FCS_LEN, 0, 1); /* Zero offset in TX buffer, ranging. */

    /* Start transmission, indicating that a response is expected so that reception is enabled automatically after the frame is sent and the delay
         * set by dwt_setrxaftertxdelay() has elapsed. */
    dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

    /* We assume that the transmission is achieved correctly, poll for reception of a frame or error/timeout. See NOTE 8 below. */
    while (!((dev->status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR))) {};

    /* Increment frame sequence number (modulo 256) and frame counter, after transmission of the poll message . */

    MAC_FRAME_SEQ_NUM_802_15_4(&dev->mac_frame) = dev->inc_seq();
    mac_frame_update_aux_frame_cnt(&dev->mac_frame, dev->inc_frame());

    if (dev->status_reg & SYS_STATUS_RXFCG_BIT_MASK) { /* Got response */
      uint32_t frame_len;

      /* Clear good RX frame event in the DW IC status register. */
      dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

      /* Read data length that was received */
      frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;

      /* A frame has been received: firstly need to read the MHR and check this frame is what we expect:
             * the destination address should match our source address (frame filtering can be configured for this check,
             * however that is not part of this example); then the header needs to have security enabled.
             * If any of these checks fail the rx_aes_802_15_4 will return an error
             * */
      dev->aes_config.mode = AES_Decrypt;
      PAYLOAD_PTR_802_15_4(&dev->mac_frame) = dev->rx_buffer; /* Set the MAC pyload ptr */

      /* This example assumes that initiator and responder are sending encrypted data */
      dev->status = rx_aes_802_15_4(&dev->mac_frame, frame_len, &dev->aes_job_rx, sizeof(dev->rx_buffer), dev->keys_options, dev->DEST_ADDR, dev->SRC_ADDR, &dev->aes_config);
      if (dev->status != AES_RES_OK) {
        do {
          switch (dev->status) {
            case AES_RES_ERROR_LENGTH:
              test_run_info((unsigned char *)"Length AES error");
              break;
            case AES_RES_ERROR:
              test_run_info((unsigned char *)"ERROR AES");
              break;
            case AES_RES_ERROR_FRAME:
              test_run_info((unsigned char *)"Error Frame");
              break;
            case AES_RES_ERROR_IGNORE_FRAME:
              test_run_info((unsigned char *)"Frame not for us");
              continue;  //Got frame not for us
          }
        } while (1);
      }


      /* Check that the frame is the expected response from the companion "SS TWR AES responder" example.
             * ignore the 8 first bytes of the response message as they contain the poll and response timestamps */
      if (memcmp(&dev->rx_buffer[START_RECEIVE_DATA_LOCATION], &dev->rx_resp_msg[START_RECEIVE_DATA_LOCATION],
                 dev->aes_job_rx.payload_len - START_RECEIVE_DATA_LOCATION)
          == 0) {
        uint32_t poll_tx_ts, resp_rx_ts, poll_rx_ts, resp_tx_ts;
        int32_t rtd_init, rtd_resp;
        float clockOffsetRatio;

        /* Retrieve poll transmission and response reception timestamps. See NOTE 9 below. */
        poll_tx_ts = dwt_readtxtimestamplo32();
        resp_rx_ts = dwt_readrxtimestamplo32();

        /* Read carrier integrator value and calculate clock offset ratio. See NOTE 11 below. */
        clockOffsetRatio = ((float)dwt_readclockoffset()) / (uint32_t)(1 << 26);

        /* Get timestamps embedded in response message. */
        resp_msg_get_ts(&dev->rx_buffer[RESP_MSG_POLL_RX_TS_IDX], &poll_rx_ts);
        resp_msg_get_ts(&dev->rx_buffer[RESP_MSG_RESP_TX_TS_IDX], &resp_tx_ts);

        /* Compute time of flight and distance, using clock offset ratio to correct for differing local and remote clock rates */
        rtd_init = resp_rx_ts - poll_tx_ts;
        rtd_resp = resp_tx_ts - poll_rx_ts;

        tof = ((rtd_init - rtd_resp * (1 - clockOffsetRatio)) / 2.0) * DWT_TIME_UNITS;
        distance = tof * SPEED_OF_LIGHT * 3.28084;
        count++;
        avgdist += distance;
      }

    } else {
      /* Clear RX error/timeout events in the DW IC status register. */
      dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
    }
  }
  return avgdist / count;
}

#define timecalc 0

int get_position() {
  static bool first = true;  //first time through, some preliminary work
  if (anchors_changed) {
    first = true;
    anchors_changed = false;
  }
  float b[3];                //temp vector, distances from anchors

  static float Ainv[2][2], k[3];  //these are calculated only once

  int i;
  int n = 40;  // number of readings to average out
  d[0] = ranging(&anchor1, n);
  d[1] = ranging(&anchor2, n);
  d[2] = ranging(&anchor3, n);

  if (first) {  //intermediate fixed vectors
    first = false;

    float x[3], y[3];  //intermediate vectors
    float A[2][2];     //the A matrix for system of equations to solve

    for (i = 0; i < 3; i++) {
      x[i] = anchor_matrix[i][0];
      y[i] = anchor_matrix[i][1];
      k[i] = x[i] * x[i] + y[i] * y[i];
    }

    // set up least squares equation

    for (i = 1; i < 3; i++) {
      A[i - 1][0] = x[i] - x[0];
      A[i - 1][1] = y[i] - y[0];
    }
    //invert A
    float det = A[0][0] * A[1][1] - A[1][0] * A[0][1];
    if (fabs(det) < 1.0E-4) {
      Serial.println("***Singular matrix, check anchor coordinates***");
      while (1) delay(1);  //hang
    }

    det = 1.0 / det;
    //scale adjoint
    Ainv[0][0] = det * A[1][1];
    Ainv[0][1] = -det * A[0][1];
    Ainv[1][0] = -det * A[1][0];
    Ainv[1][1] = det * A[0][0];
  }

  for (i = 1; i < 3; i++) {
    b[i - 1] = d[0] * d[0] - d[i] * d[i] + k[i] - k[0];
  }

  //least squares solution for position
  //solve:  2 A rc = b
  current_tag_position[0] = 0.5 * (Ainv[0][0] * b[0] + Ainv[0][1] * b[1]);
  current_tag_position[1] = 0.5 * (Ainv[1][0] * b[0] + Ainv[1][1] * b[1]);

  // calculate rms error for distances
  float rmse = 0.0, dc0 = 0.0, dc1 = 0.0;
  for (i = 0; i < 3; i++) {
    dc0 = current_tag_position[0] - anchor_matrix[i][0];
    dc1 = current_tag_position[1] - anchor_matrix[i][1];
    dc0 = d[i] - sqrt(dc0 * dc0 + dc1 * dc1);
    rmse += dc0 * dc0;
  }
  current_distance_rmse = sqrt(rmse / 3.0);
  return 1;
}

void send_udp(char *msg) {
  if (connected) {
    //Send a packet
    udp.beginPacket(host, port);
    udp.printf(msg);
    udp.endPacket();
  }
  else {
    Serial.println("Failed to send UDP, Not connected to internet.");
  }
}

void send_pos() {
  String str_pos = String(String(current_tag_position[0]) + ','
                          + String(current_tag_position[1]) + ','
                          + String(current_distance_rmse) + ','
                          + String(d[0]) + ','
                          + String(d[1]) + ','
                          + String(d[2]) + ','
                          + String(anchor_matrix[0][0]) + ','
                          + String(anchor_matrix[0][1]) + ','
                          + String(anchor_matrix[1][0]) + ','
                          + String(anchor_matrix[1][1]) + ','
                          + String(anchor_matrix[2][0]) + ','
                          + String(anchor_matrix[2][1]));
  char str[1024];
  str_pos.toCharArray(str, 1024);
  send_udp(str);
}

double get_anchor_range(int dev1, int dev2, double &range) {
  int delay_1 = 10;
  if (communication_mode == 0) { // switch communication mode if it is set to wifi.
    switch_esp_now();
    communication_mode = 1;
  }
  bool ret = true; // turns false if function fails
  msg_data data_out;
  data_out.type = 0;           // type = instruction
  data_out.sender = 0;         // sent from tag
  data_out.reciever = dev2;    // sent to anchor 2
  data_out.connect_to = dev1;  // connect anchor2 to anchor 1.
  data_out.data = 1;           // instruction = change to anchor
  data_out.arg1 = -1;
  // Serial.println("Set dev2 to anchor");
  esp_now_send(esp_mac_addrs[dev2], (uint8_t *)&data_out, sizeof(data_out));
  // wait for confirmation
  while((msg_recved == false) && (data_delivered != 0)) {
    delay(1);
  }
  delay(5);
  if (instr_success != 1)  {
    Serial.println("get_anchor_range: Anchor " + String(dev2) + " failed to change to anchor and connect to anchor " + String(dev1));
    ret = false;
  }
  clear_msg();
  delay(delay_1);

  data_out.reciever = dev1;    // sent to anchor 1
  data_out.connect_to = dev2;  // connect anchor1 to anchor 2.
  data_out.data = 0;           // instruction = change to tag
  // Serial.println("Set dev1 to tag");
  esp_now_send(esp_mac_addrs[dev1], (uint8_t *)&data_out, sizeof(data_out));
  // wait for confirmation
  while((msg_recved == false) && (data_delivered != 0)) {
    delay(1);
  }
  delay(5);
  if (instr_success != 1)  {
    Serial.println("get_anchor_range: Anchor " + String(dev1) + " failed to change to tag and connect to anchor " + String(dev2));
    ret = false;
  }
  clear_msg();
  delay(delay_1);

  // Anchor 1 and Anchor 2 modes set, get range next.
  data_out.data = 2;        // instruction = get range
  data_out.arg1 = 100;      // average out 100 readings
  esp_now_send(esp_mac_addrs[dev1], (uint8_t *)&data_out, sizeof(data_out));
  while((msg_recved == false) && (data_delivered != 0)) {
    delay(1);
  }
  delay(10);
  if (instr_success != 1)  {
    Serial.println("get_anchor_range: Anchor " + String(dev1) + " failed to get range from anchor " + String(dev2));
    range = data_recieved;
    ret = false;
  }
  else {
    range = data_recieved;
  }
  clear_msg();  
  delay(delay_1);
  return ret;
}

bool get_anchor_coords() {
  int delay_1 = 1000;
  double anchor12, anchor13, anchor23;
  bool anc1 = get_anchor_range(1,2, anchor12);
  delay(delay_1);
  bool anc2 = get_anchor_range(1,3, anchor13);
  delay(delay_1);
  bool anc3 = get_anchor_range(2,3, anchor23);
  delay(delay_1);
  if (anc1 && anc2 && anc3) {
    anchors_changed = true;
    anchor_matrix[0][0] = 0;
    anchor_matrix[0][1] = 0;
    anchor_matrix[1][0] = anchor12;
    anchor_matrix[1][1] = 0;
    double cos_a = (anchor13 * anchor13 + anchor12*anchor12 - anchor23 * anchor23) / (2 * anchor13 * anchor12);
    anchor_matrix[2][0] = cos_a * anchor13;
    anchor_matrix[2][1] = anchor13 * sqrt(1-cos_a*cos_a);
    Serial.println("Anchor 1: (" + String(anchor_matrix[0][0]) + ", " + String(anchor_matrix[0][1]) + ")");
    Serial.println("Anchor 2: (" + String(anchor_matrix[1][0]) + ", " + String(anchor_matrix[1][1]) + ")");
    Serial.println("Anchor 3: (" + String(anchor_matrix[2][0]) + ", " + String(anchor_matrix[2][1]) + ")");
    
  }
  return anc1 && anc2 && anc3;
}

bool init_anchors() {
  int delay_1 = 1000;
  bool ret = true;
  if (communication_mode == 0) { // switch communication mode if it is set to wifi.
    switch_esp_now();
    communication_mode = 1;
  }
  msg_data data_out;
  data_out.type = 0;        // type = instruction
  data_out.sender = 0;      // sent from tag
  data_out.reciever = 1;    // sent to anchor 1
  data_out.connect_to = 0;        // connect anchor to tag.
  data_out.data = 1;        // instruction = change to anchor
  esp_now_send(anchor_mac_addr_1, (uint8_t *)&data_out, sizeof(data_out));
  // wait for confirmation
  while((msg_recved == false) && (data_delivered != 0)) {
    delay(1);
  }
  delay(5);
  if (instr_success != 1)  {
    Serial.println("Anchor 1 failed to change to anchor and connect to tag");
    ret = false;
  }
  clear_msg();
  delay(delay_1);

  data_out.reciever = 2;  // sent to anchor 2
  esp_now_send(anchor_mac_addr_2, (uint8_t *)&data_out, sizeof(data_out));
  // wait for confirmation
  while((msg_recved == false) && (data_delivered != 0)) {
    delay(1);
  }
  delay(5);
  if (instr_success != 1)  {
    Serial.println("Anchor 2 failed to change to anchor and connect to tag");
    ret = false;
  }
  clear_msg();
  delay(delay_1);

  data_out.reciever = 3;  // sent to anchor 3
  esp_now_send(anchor_mac_addr_3, (uint8_t *)&data_out, sizeof(data_out));
  // wait for confirmation
  while((msg_recved == false) && (data_delivered != 0)) {
    delay(1);
  }
  delay(5);
  if (instr_success != 1)  {
    Serial.println("Anchor 3 failed to change to anchor and connect to tag");
    ret = false;
  }
  clear_msg();
  delay(delay_1);

  return ret;
}

int switch_esp_now() { // Switch from WiFi to ESP NOW
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return -1;
  }
  esp_now_register_send_cb(OnDataSent);
  if (esp_now_add_peer(&peerInfo1) != ESP_OK) {
    Serial.println("Failed to add peer 1");
    return -1;
  }

  if (esp_now_add_peer(&peerInfo2) != ESP_OK) {
    Serial.println("Failed to add peer 2");
    return -1;
  }

  if (esp_now_add_peer(&peerInfo3) != ESP_OK) {
    Serial.println("Failed to add peer 3");
    return -1;
  }
  esp_now_register_recv_cb(OnDataRecv);
  delay(5);
  return 0;
}

void switch_wifi() { // switch from ESP NOW to WiFi
  esp_now_deinit();
  connectToWiFi(ssid, password);
  WiFi.mode(WIFI_AP_STA);
}

bool configure_anchors() {
  switch_esp_now();
  bool a = get_anchor_coords();
  Serial.println("initializing anchors now");
  clear_msg();
  bool b = init_anchors();
  clear_msg();
  Serial.println("Finished");
  switch_wifi();
  while (connected == false) {
    delay(1);
  }
  return a && b;
}


void setup() {
  UART_init();
  test_run_info((unsigned char *)APP_NAME);

  /* Configure SPI rate, DW3000 supports up to 38 MHz */
  /* Reset DW IC */
  spiBegin(PIN_IRQ, PIN_RST);
  spiSelect(PIN_SS);

  delay(2);  // Time needed for DW3000 to start up (transition from INIT_RC to IDLE_RC, or could wait for SPIRDY event)

  while (!dwt_checkidlerc())  // Need to make sure DW IC is in IDLE_RC before proceeding
  {
    UART_puts("IDLE FAILED\r\n");
    while (1)
      ;
  }

  if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
    UART_puts("INIT FAILED\r\n");
    while (1)
      ;
  }

  // Enabling LEDs here for debug so that for each TX the D1 LED will flash on DW3000 red eval-shield boards.
  dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

  /* Configure DW IC. See NOTE 14 below. */
  if (dwt_configure(&config)) /* if the dwt_configure returns DWT_ERROR either the PLL or RX calibration has failed the host should reset the device */
  {
    test_run_info((unsigned char *)"CONFIG FAILED     ");
    while (1) {};
  }

  /* Configure the TX spectrum parameters (power, PG delay and PG count) */
  dwt_configuretxrf(&txconfig_options);

  /* Apply default antenna delay value. See NOTE 2 below. */
  dwt_setrxantennadelay(RX_ANT_DLY);
  dwt_settxantennadelay(TX_ANT_DLY);

  /* Set expected response's delay and timeout. See NOTE 1 and 5 below.
     * This example is paired with the SS-TWR responder and if delays/timings need to be changed
     * they must be changed in both to match. */
  dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
  dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);

  /* Next can enable TX/RX states output on GPIOs 5 and 6 to help debug */
  dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);

  /*Configure the TX and RX AES jobs, the TX job is used to encrypt the Poll message,
     * the RX job is used to decrypt the Response message */
  anchor1.aes_job_tx.mode = AES_Encrypt;                                       /* this is encryption job */
  anchor1.aes_job_tx.src_port = AES_Src_Tx_buf;                                /* dwt_do_aes will take plain text to the TX buffer */
  anchor1.aes_job_tx.dst_port = AES_Dst_Tx_buf;                                /* dwt_do_aes will replace the original plain text TX buffer with encrypted one */
  anchor1.aes_job_tx.nonce = anchor1.nonce;                                    /* pointer to the nonce structure*/
  anchor1.aes_job_tx.header = (uint8_t *)MHR_802_15_4_PTR(&anchor1.mac_frame); /* plain-text header which will not be encrypted */
  anchor1.aes_job_tx.header_len = MAC_FRAME_HEADER_SIZE(&anchor1.mac_frame);
  anchor1.aes_job_tx.payload = anchor1.tx_poll_msg;             /* payload to be encrypted */
  anchor1.aes_job_tx.payload_len = sizeof(anchor1.tx_poll_msg); /* size of payload to be encrypted */

  anchor1.aes_job_rx.mode = AES_Decrypt;          /* this is decryption job */
  anchor1.aes_job_rx.src_port = AES_Src_Rx_buf_0; /* The source of the data to be decrypted is the IC RX buffer */
  anchor1.aes_job_rx.dst_port = AES_Dst_Rx_buf_0; /* Decrypt the encrypted data to the IC RX buffer : this will destroy original RX frame */
  anchor1.aes_job_rx.header_len = anchor1.aes_job_tx.header_len;
  anchor1.aes_job_rx.header = anchor1.aes_job_tx.header; /* plain-text header which will not be encrypted */
  anchor1.aes_job_rx.payload = anchor1.rx_buffer;        /* pointer to where the decrypted data will be copied to when read from the IC*/


  anchor2.aes_job_tx.mode = AES_Encrypt;                                       /* this is encryption job */
  anchor2.aes_job_tx.src_port = AES_Src_Tx_buf;                                /* dwt_do_aes will take plain text to the TX buffer */
  anchor2.aes_job_tx.dst_port = AES_Dst_Tx_buf;                                /* dwt_do_aes will replace the original plain text TX buffer with encrypted one */
  anchor2.aes_job_tx.nonce = anchor2.nonce;                                    /* pointer to the nonce structure*/
  anchor2.aes_job_tx.header = (uint8_t *)MHR_802_15_4_PTR(&anchor2.mac_frame); /* plain-text header which will not be encrypted */
  anchor2.aes_job_tx.header_len = MAC_FRAME_HEADER_SIZE(&anchor2.mac_frame);
  anchor2.aes_job_tx.payload = anchor2.tx_poll_msg;             /* payload to be encrypted */
  anchor2.aes_job_tx.payload_len = sizeof(anchor2.tx_poll_msg); /* size of payload to be encrypted */

  anchor2.aes_job_rx.mode = AES_Decrypt;          /* this is decryption job */
  anchor2.aes_job_rx.src_port = AES_Src_Rx_buf_0; /* The source of the data to be decrypted is the IC RX buffer */
  anchor2.aes_job_rx.dst_port = AES_Dst_Rx_buf_0; /* Decrypt the encrypted data to the IC RX buffer : this will destroy original RX frame */
  anchor2.aes_job_rx.header_len = anchor2.aes_job_tx.header_len;
  anchor2.aes_job_rx.header = anchor2.aes_job_tx.header; /* plain-text header which will not be encrypted */
  anchor2.aes_job_rx.payload = anchor2.rx_buffer;        /* pointer to where the decrypted data will be copied to when read from the IC*/

  anchor3.aes_job_tx.mode = AES_Encrypt;                                       /* this is encryption job */
  anchor3.aes_job_tx.src_port = AES_Src_Tx_buf;                                /* dwt_do_aes will take plain text to the TX buffer */
  anchor3.aes_job_tx.dst_port = AES_Dst_Tx_buf;                                /* dwt_do_aes will replace the original plain text TX buffer with encrypted one */
  anchor3.aes_job_tx.nonce = anchor3.nonce;                                    /* pointer to the nonce structure*/
  anchor3.aes_job_tx.header = (uint8_t *)MHR_802_15_4_PTR(&anchor3.mac_frame); /* plain-text header which will not be encrypted */
  anchor3.aes_job_tx.header_len = MAC_FRAME_HEADER_SIZE(&anchor3.mac_frame);
  anchor3.aes_job_tx.payload = anchor3.tx_poll_msg;             /* payload to be encrypted */
  anchor3.aes_job_tx.payload_len = sizeof(anchor3.tx_poll_msg); /* size of payload to be encrypted */

  anchor3.aes_job_rx.mode = AES_Decrypt;          /* this is decryption job */
  anchor3.aes_job_rx.src_port = AES_Src_Rx_buf_0; /* The source of the data to be decrypted is the IC RX buffer */
  anchor3.aes_job_rx.dst_port = AES_Dst_Rx_buf_0; /* Decrypt the encrypted data to the IC RX buffer : this will destroy original RX frame */
  anchor3.aes_job_rx.header_len = anchor3.aes_job_tx.header_len;
  anchor3.aes_job_rx.header = anchor3.aes_job_tx.header; /* plain-text header which will not be encrypted */
  anchor3.aes_job_rx.payload = anchor3.rx_buffer;        /* pointer to where the decrypted data will be copied to when read from the IC*/

  Serial.begin(38400);
  Serial.println("Serial starting");

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  Serial.println(WiFi.macAddress());
  connectToWiFi(ssid, password);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

  // Register peers
  memcpy(peerInfo1.peer_addr, anchor_mac_addr_1, 6);
  peerInfo1.channel = 0;
  peerInfo1.encrypt = false;

  memcpy(peerInfo2.peer_addr, anchor_mac_addr_2, 6);
  peerInfo2.channel = 0;
  peerInfo2.encrypt = false;

  memcpy(peerInfo3.peer_addr, anchor_mac_addr_3, 6);
  peerInfo3.channel = 0;
  peerInfo3.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo1) != ESP_OK) {
    Serial.println("Failed to add peer 1");
    // return;
  }

  if (esp_now_add_peer(&peerInfo2) != ESP_OK) {
    Serial.println("Failed to add peer 2");
    // return;
  }

  if (esp_now_add_peer(&peerInfo3) != ESP_OK) {
    Serial.println("Failed to add peer 3");
    // return;
  }
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);
}


void loop() {
  if (!anchors_set) { // if anchor positions have not been calibrated yet, start calibration
    anchors_set = configure_anchors();
    Serial.println("Anchors Set? " + String(anchors_set));
    if (anchors_set) {
      Serial.println("Anchor Initialization Complete");
    }
  }
  else { // start sending positions
    get_position();
    send_pos();
  }
}
