#include "dw3000.h"
#include "dw3000_mac_802_15_4.h"
#include <WiFi.h>
#include <esp_now.h>
#define APP_NAME "SS TWR AES RESP v1.0"

// Define what anchor you are uploading to.
#define anchornum 3
int this_device = anchornum;

// MAC Addresses used for ESPNow protocol. Update section to include the MAC Addresses of the devices being used.
uint8_t tag_mac_addr[] =      {0xEC, 0x62, 0x60, 0xF1, 0xBF, 0xD8};
uint8_t anchor_mac_addr_1[] = {0x10, 0x97, 0xBD, 0x5C, 0xB8, 0x60};
uint8_t anchor_mac_addr_2[] = {0x10, 0x97, 0xBD, 0x5E, 0x06, 0x20};
uint8_t anchor_mac_addr_3[] = {0x10, 0x97, 0xBD, 0x5D, 0xF9, 0xC4};

// connection pins
const uint8_t PIN_RST = 27;  // reset pin
const uint8_t PIN_IRQ = 34;  // irq pin
const uint8_t PIN_SS = 4;    // spi select pin

mac_frame_802_15_4_format_t mac_frame1 = {
  { { 0x09, 0xEC },
    0x00,
    { 0x21, 0x43 },
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
    { 0x0F, { 0x00, 0x00, 0x00, 0x00 }, 0x00 } },
  0x00
};

/* Index to access some of the fields in the frames involved in the process. */
#define ALL_MSG_SN_IDX 2           //sequence number byte index in MHR
#define RESP_MSG_POLL_RX_TS_IDX 0  //index in the MAC payload for Poll RX time
#define RESP_MSG_RESP_TX_TS_IDX 4  //index in the MAC payload for Response TX time
#define RESP_MSG_TS_LEN 4
#define START_RECEIVE_DATA_LOCATION 8  //MAC payload user data starts at index 8 (e.g. 'R' - in above response message)
#define INITIATOR_KEY_INDEX 1

/* Delay between frames, in UWB microseconds. */
#define POLL_TX_TO_RESP_RX_DLY_UUS 1720
/* Receive response timeout.*/
#define RESP_RX_TIMEOUT_UUS 250

/* Buffer to store received response message.
 * Its size is adjusted to longest frame that this example code can handle. */
#define RX_BUF_LEN 127 /* The received frame cannot be bigger than 127 if STD PHR mode is used */

/* Note, the key index of 0 is forbidden to send as key index. Thus index 1 is the first.
 * This example uses this index for the key table for the encryption of responder's data */
#define RESPONDER_KEY_INDEX 2

/* Delay between frames, in UWB microseconds. See NOTE 1 below.
 * this includes the poll frame length ~ 240 us*/
#define POLL_RX_TO_RESP_TX_DLY_UUS 2000

#define TAG_ADDR      0x8877665544332211 /* this is the address of the responder */
#define ANCHOR_ADDR_1 0x1122334455667788 /* this is the address of the responder */
#define ANCHOR_ADDR_2 0x1122334455667789 /* this is the address of the responder */
#define ANCHOR_ADDR_3 0x112233445566778A /* this is the address of the responder */
#define PAN_ID 0x4321                    /* this is the PAN ID used in this example */

#define DEFAULT_ANT_DLY 16384

/* This SS-TWR example will use sample MAC data frame format as defined by mac_frame_802_15_4_format_t structure */
#if anchornum == 1
#define SRC_ADDR1 0x1122334455667788  /* this is the address of the initiator */
/* Default antenna delay values for 64 MHz PRF. See NOTE 2 below. */
// OLD 16410
//16367
#define TX_ANT_DLY_TAG 16411
#define RX_ANT_DLY_TAG 16411

#define TX_ANT_DLY_ANCHOR1 16384
#define RX_ANT_DLY_ANCHOR1 16384

#define TX_ANT_DLY_ANCHOR2 16392
#define RX_ANT_DLY_ANCHOR2 16392

#define TX_ANT_DLY_ANCHOR3 16395
#define RX_ANT_DLY_ANCHOR3 16395




#elif anchornum == 2
#define SRC_ADDR1 0x1122334455667789  /* this is the address of the initiator */
/* Default antenna delay values for 64 MHz PRF. See NOTE 2 below. */
// OLD 16404, 16367
#define TX_ANT_DLY_TAG 16414
#define RX_ANT_DLY_TAG 16414

#define TX_ANT_DLY_ANCHOR1 16392
#define RX_ANT_DLY_ANCHOR1 16392

#define TX_ANT_DLY_ANCHOR2 16384
#define RX_ANT_DLY_ANCHOR2 16384

#define TX_ANT_DLY_ANCHOR3 16398
#define RX_ANT_DLY_ANCHOR3 16398


#else
#define SRC_ADDR1 0x112233445566778A  /* this is the address of the initiator */
/* Default antenna delay values for 64 MHz PRF. See NOTE 2 below. */
// OLD 16406
#define TX_ANT_DLY_TAG 16402
#define RX_ANT_DLY_TAG 16402

#define TX_ANT_DLY_ANCHOR1 16395
#define RX_ANT_DLY_ANCHOR1 16395

#define TX_ANT_DLY_ANCHOR2 16398
#define RX_ANT_DLY_ANCHOR2 16398

#define TX_ANT_DLY_ANCHOR3 16384
#define RX_ANT_DLY_ANCHOR3 16384

#endif

int operating_mode = 0; // 0 = running as anchor, 1 = running as tag.
int curr_device = 0;
bool gotrangeyet = true;
bool rangegot = false;
double range_data = -1;
int num_avg = 0;

typedef struct msg_data {
  int type;         // Instruction (0), Confirmation (1), Data (2)
  int sender;       // Device message was sent from
  int reciever;     // Device message is sent to
  int connect_to;  // Device to connect to.
  int arg1;        //Used as conformation when sending data back to tag. Confirmation: Failed (0)	Success(1). Also used as number of times to average ranging out.
  double data;      // Instruction, Confirmation, or Data. Instruction: Change to tag(0)	Change to anchor(1)	Get Range (2). Confirmation: Failed (0)	Success(1)
} msg_data;

msg_data in_message;

esp_now_peer_info_t peerInfo;

static dwt_aes_config_t aes_config1 = {
  AES_key_RAM,
  AES_core_type_CCM,
  MIC_0,
  AES_KEY_Src_Register,
  AES_KEY_Load,
  0,
  AES_KEY_128bit,
  AES_Encrypt
};

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

/* Optional keys according to the key index - In AUX security header*/
static dwt_aes_key_t keys_options1[NUM_OF_KEY_OPTIONS] = {
  { 0x00010203, 0x04050607, 0x08090A0B, 0x0C0D0E0F, 0x00000000, 0x00000000, 0x00000000, 0x00000000 },
  { 0x11223344, 0x55667788, 0x99AABBCC, 0xDDEEFF00, 0x00000000, 0x00000000, 0x00000000, 0x00000000 },
  { 0xFFEEDDCC, 0xBBAA9988, 0x77665544, 0x33221100, 0x00000000, 0x00000000, 0x00000000, 0x00000000 }
};

static uint64_t poll_rx_ts;
static uint64_t resp_tx_ts;

class Device {
public:
  int dev_id;
  mac_frame_802_15_4_format_t mac_frame;
  long long unsigned int SRC_ADDR;
  long long unsigned int DEST_ADDR;
  long long unsigned int DEST_PAN_ID;
  long long unsigned int TX_ANT_DLY;
  long long unsigned int RX_ANT_DLY;
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

  uint8_t rx_poll_msg[12] = {'P','o','l','l',' ','m','e','s','s','a','g','e'};
  /* Response message to the initiator. The first 8 bytes are used for Poll RX time and Response TX time.*/
  uint8_t tx_resp_msg[16] = {0,0,0,0,0,0,0,0,'R','e','s','p','o','n','s','e'};

  /* Timestamps of frames transmission/reception. */
  static uint64_t poll_rx_ts;
  static uint64_t resp_tx_ts;
  
  Device(int id, mac_frame_802_15_4_format_t mac, long long unsigned int pan, long long unsigned int addr, 
         long long unsigned int src, dwt_aes_config_t aes_con, dwt_aes_key_t *key_op,
         long long unsigned int tx_delay, long long unsigned int rx_delay) {
    dev_id = id;
    mac_frame = mac;
    DEST_ADDR = addr;
    DEST_PAN_ID = pan;
    SRC_ADDR = src;
    frame_cnt = 0;  /* See Note 13 */
    seq_cnt = 0x0A; /* Frame sequence number, incremented after each transmission. */
    aes_config = aes_con;
    keys_options = key_op;
    TX_ANT_DLY = tx_delay;
    RX_ANT_DLY = rx_delay;
  }
  Device(Device &t)  //copy constructor
  {
    dev_id = t.dev_id;
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
    // rx_buffer = t.rx_buffer;
    // nonce = t.nonce;
    status = t.status;
    status_reg = t.status_reg;
    // tx_poll_msg = t.tx_poll_msg;
    // rx_resp_msg = t.rx_resp_msg;
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

long long unsigned int addrs[4] = {TAG_ADDR, ANCHOR_ADDR_1, ANCHOR_ADDR_2, ANCHOR_ADDR_3};
int tx_delays[4] = {TX_ANT_DLY_TAG, TX_ANT_DLY_ANCHOR1, TX_ANT_DLY_ANCHOR2, TX_ANT_DLY_ANCHOR3};
int rx_delays[4] = {RX_ANT_DLY_TAG, RX_ANT_DLY_ANCHOR1, RX_ANT_DLY_ANCHOR2, RX_ANT_DLY_ANCHOR3};

Device tag(0, mac_frame1, PAN_ID, TAG_ADDR, SRC_ADDR1, aes_config1, keys_options1, TX_ANT_DLY_TAG, RX_ANT_DLY_TAG);
Device anchor1(1, mac_frame1, PAN_ID, ANCHOR_ADDR_1, SRC_ADDR1, aes_config1, keys_options1, TX_ANT_DLY_ANCHOR1, RX_ANT_DLY_ANCHOR1);
Device anchor2(2, mac_frame1, PAN_ID, ANCHOR_ADDR_2, SRC_ADDR1, aes_config1, keys_options1, TX_ANT_DLY_ANCHOR2, RX_ANT_DLY_ANCHOR2);
Device anchor3(3, mac_frame1, PAN_ID, ANCHOR_ADDR_3, SRC_ADDR1, aes_config1, keys_options1, TX_ANT_DLY_ANCHOR3, RX_ANT_DLY_ANCHOR3);

Device* devices[4] = {&tag, &anchor1, &anchor2, &anchor3};

/* Values for the PG_DELAY and TX_POWER registers reflect the bandwidth and power of the spectrum at the current
 * temperature. These values can be calibrated prior to taking reference measurements. See NOTE 5 below. */
extern dwt_txconfig_t txconfig_options;

void msg_parser(msg_data in) {
  msg_data data_out;
  data_out.type = 1;           // type = confirmation
  data_out.sender = this_device;         // sent from this device
  data_out.reciever = 0;    // sent to tag
  data_out.connect_to = this_device;
  data_out.arg1 = -1;  // arg1 = 0
  data_out.data = -1;           // instruction = change to anchor
  if (in_message.type == 0) {
    if ((int)in_message.data == 0) { // change to tag
      curr_device = in_message.connect_to;
      Serial.println("Device switched to " + String(curr_device));
      setup_as_tag(devices[in_message.connect_to]);
      data_out.data = 1; // Confirm instruction executed properly
      data_out.arg1 = 1;
      esp_now_send(tag_mac_addr, (uint8_t *)&data_out, sizeof(data_out));
    }
    else if ((int)in_message.data == 1) { // change to anchor
      curr_device = in_message.connect_to;
      setup_as_anchor(devices[in_message.connect_to]);
      data_out.data = 1; // Confirm instruction executed properly
      data_out.arg1 = 1;
      esp_now_send(tag_mac_addr, (uint8_t *)&data_out, sizeof(data_out));
    }
    else if ((int)in_message.data == 2) { // send ranging data
      Serial.println("parser: sending ranging data");
      curr_device = in_message.connect_to;
      data_out.type = 2;           // type = data
      int n = in_message.arg1;
      num_avg = n;
      double range = ranging(devices[curr_device], n);
      Serial.println("Range calculated");
      if (isnan(range)) {
        Serial.println("Error: Ranging Failed");
        data_out.arg1 = 0;
        data_out.data = -1;
      }
      else {
        data_out.arg1 = 1;
        data_out.data = range;
        Serial.println(range);
      }
      esp_now_send(tag_mac_addr, (uint8_t *)&data_out, sizeof(data_out));      
    }
    else {
      Serial.println("Error: Instruction not defined");
        data_out.arg1 = 0;
        data_out.data = 0;
    }
  }
  else {
    Serial.println("Error: Message recieved is not an instruction.");
  }
  return;
}

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
}

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  Serial.println("message recieved");
  memcpy(&in_message, incomingData, sizeof(in_message));
  msg_parser(in_message);

}

void setup_as_anchor(Device *dev) {
  operating_mode = 0;
  Serial.println("Operating mode: " + String(operating_mode));
  curr_device = dev->dev_id;
  dwt_setrxaftertxdelay(0);
  dwt_setrxtimeout(0);
  /*Configure the TX and RX AES jobs, the TX job is used to encrypt the Response message,
     * the RX job is used to decrypt the Poll message */
  dev->aes_job_rx.mode = AES_Decrypt;                               /* Mode is set to decryption */
  dev->aes_job_rx.src_port = AES_Src_Rx_buf_0;                      /* Take encrypted frame from the RX buffer */
  dev->aes_job_rx.dst_port = AES_Dst_Rx_buf_0;                      /* Decrypt the frame to the same RX buffer : this will destroy original RX frame */
  dev->aes_job_rx.header_len = MAC_FRAME_HEADER_SIZE(&dev->mac_frame);   /* Set the header length (mac_frame contains the MAC header) */
  dev->aes_job_rx.header = (uint8_t *)MHR_802_15_4_PTR(&dev->mac_frame); /* Set the pointer to plain-text header which will not be encrypted */
  dev->aes_job_rx.payload = dev->rx_buffer;                              /* the decrypted RX MAC frame payload will be read out of IC into this buffer */

  dev->aes_job_tx.mode = AES_Encrypt;        /* this is encyption job */
  dev->aes_job_tx.src_port = AES_Src_Tx_buf; /* dwt_do_aes will take plain text to the TX buffer */
  dev->aes_job_tx.dst_port = AES_Dst_Tx_buf; /* dwt_do_aes will replace the original plain text TX buffer with encrypted one */
  dev->aes_job_tx.header_len = dev->aes_job_rx.header_len;
  dev->aes_job_tx.header = dev->aes_job_rx.header;        /* plain-text header which will not be encrypted */
  dev->aes_job_tx.payload = dev->tx_resp_msg;             /* payload to be sent */
  dev->aes_job_tx.payload_len = sizeof(dev->tx_resp_msg); /* payload length */
  return;
}

void setup_as_tag(Device *dev) {
  operating_mode = 1;
  Serial.println("Operating mode: " + String(operating_mode));
  dwt_softreset();
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

  curr_device = dev->dev_id;
  /* Apply default antenna delay value. See NOTE 2 below. */
  dwt_setrxantennadelay(DEFAULT_ANT_DLY);
  dwt_settxantennadelay(DEFAULT_ANT_DLY);

  /* Set expected response's delay and timeout. See NOTE 1 and 5 below.
     * This example is paired with the SS-TWR responder and if delays/timings need to be changed
     * they must be changed in both to match. */
  dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
  dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);

  dev->aes_job_tx.mode = AES_Encrypt;                                       /* this is encryption job */
  dev->aes_job_tx.src_port = AES_Src_Tx_buf;                                /* dwt_do_aes will take plain text to the TX buffer */
  dev->aes_job_tx.dst_port = AES_Dst_Tx_buf;                                /* dwt_do_aes will replace the original plain text TX buffer with encrypted one */
  dev->aes_job_tx.nonce = dev->nonce;                                    /* pointer to the nonce structure*/
  dev->aes_job_tx.header = (uint8_t *)MHR_802_15_4_PTR(&dev->mac_frame); /* plain-text header which will not be encrypted */
  dev->aes_job_tx.header_len = MAC_FRAME_HEADER_SIZE(&dev->mac_frame);
  dev->aes_job_tx.payload = dev->tx_poll_msg;             /* payload to be encrypted */
  dev->aes_job_tx.payload_len = sizeof(dev->tx_poll_msg); /* size of payload to be encrypted */

  dev->aes_job_rx.mode = AES_Decrypt;          /* this is decryption job */
  dev->aes_job_rx.src_port = AES_Src_Rx_buf_0; /* The source of the data to be decrypted is the IC RX buffer */
  dev->aes_job_rx.dst_port = AES_Dst_Rx_buf_0; /* Decrypt the encrypted data to the IC RX buffer : this will destroy original RX frame */
  dev->aes_job_rx.header_len = dev->aes_job_tx.header_len;
  dev->aes_job_rx.header = dev->aes_job_tx.header; /* plain-text header which will not be encrypted */
  dev->aes_job_rx.payload = dev->rx_buffer;        /* pointer to where the decrypted data will be copied to when read from the IC*/
  return;
}


double ranging(Device *dev, int n) {
// double ranging(int device, int n) {

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
    // Serial.println("device: " +String(dev->SRC_ADDR));
    while (!((dev->status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR))) {
      vTaskDelay(1);
      if (dev->dev_id != curr_device) {
        Serial.println("Dev ID not equal to curr_device. Returning");
        return -1;
      }
    }
    // Serial.println("reached2");
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
      // Serial.println("failed");
    }
  }
  return avgdist / count;
}

void anchor(Device *dev) {
  /* Activate reception immediately. */
  
  dwt_rxenable(DWT_START_RX_IMMEDIATE);
  /* Poll for reception of a frame or error/timeout. See NOTE 6 below. */
  int c = 0;
  while (!((dev->status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR))) {
    if (dev->dev_id != curr_device) {
      Serial.println("Selected device is not the current device. Returning");
      return;
    }
    delay(1);
    c++;
    if (c > 100) return;
  };
  /* Once a frame has been received read the payload and decrypt*/
  if (dev->status_reg & SYS_STATUS_RXFCG_BIT_MASK) {
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
    dev->aes_config.mode = AES_Decrypt;                /* configure for decryption*/
    PAYLOAD_PTR_802_15_4(&dev->mac_frame) = dev->rx_buffer; /* Set the MAC frame structure payload pointer
                                                             (this will contain decrypted data if status below is AES_RES_OK) */

    dev->status = rx_aes_802_15_4(&dev->mac_frame, frame_len, &dev->aes_job_rx, sizeof(dev->rx_buffer), dev->keys_options, dev->DEST_ADDR, dev->SRC_ADDR, &dev->aes_config);
    
    if (dev->status != AES_RES_OK) {
      /* report any errors */
      do {
        switch (dev->status) {
          case AES_RES_ERROR_LENGTH:
            test_run_info((unsigned char *)"AES length error");
            break;
          case AES_RES_ERROR:
            test_run_info((unsigned char *)"ERROR AES");
            break;
          case AES_RES_ERROR_FRAME:
            test_run_info((unsigned char *)"Error Frame");
            break;
          case AES_RES_ERROR_IGNORE_FRAME:
            test_run_info((unsigned char *)"Frame not for us");
            return;  //Got frame with wrong destination address
        }
      } while (1);
    }
    /* Check that the payload of the MAC frame matches the expected poll message
             * as should be sent by "SS TWR AES initiator" example. */
    if (memcmp(dev->rx_buffer, dev->rx_poll_msg, dev->aes_job_rx.payload_len) == 0) {
      uint32_t resp_tx_time;
      int ret;
      uint8_t nonce[13];

      /* Retrieve poll reception timestamp. */
      poll_rx_ts = get_rx_timestamp_u64();

      /* Compute response message transmission time. See NOTE 7 below. */
      resp_tx_time = (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
      dwt_setdelayedtrxtime(resp_tx_time);

      /* Response TX timestamp is the transmission time we programmed plus the antenna delay. */
      resp_tx_ts = (((uint64_t)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + dev->TX_ANT_DLY;

      /* Write all timestamps in the final message. See NOTE 8 below. */
      resp_msg_set_ts(&dev->tx_resp_msg[RESP_MSG_POLL_RX_TS_IDX], poll_rx_ts);
      resp_msg_set_ts(&dev->tx_resp_msg[RESP_MSG_RESP_TX_TS_IDX], resp_tx_ts);

      /* Now need to encrypt the frame before transmitting*/

      /* Program the correct key to be used */
      dwt_set_keyreg_128(&dev->keys_options[RESPONDER_KEY_INDEX - 1]);
      /* Set the key index for the frame */
      MAC_FRAME_AUX_KEY_IDENTIFY_802_15_4(&dev->mac_frame) = RESPONDER_KEY_INDEX;

      /* Increment the sequence number */
      MAC_FRAME_SEQ_NUM_802_15_4(&dev->mac_frame)
      ++;

      /* Update the frame count */
      mac_frame_update_aux_frame_cnt(&dev->mac_frame, mac_frame_get_aux_frame_cnt(&dev->mac_frame) + 1);

      /* Configure the AES job */
      dev->aes_job_tx.mic_size = mac_frame_get_aux_mic_size(&dev->mac_frame);
      dev->aes_job_tx.nonce = nonce; /* set below once MHR is set*/
      dev->aes_config.mode = AES_Encrypt;
      dev->aes_config.mic = dwt_mic_size_from_bytes(dev->aes_job_tx.mic_size);
      dwt_configure_aes(&dev->aes_config);

      /* Update the MHR (reusing the received MHR, thus need to swap SRC/DEST addresses */
      mac_frame_set_pan_ids_and_addresses_802_15_4(&dev->mac_frame, dev->DEST_PAN_ID, dev->DEST_ADDR, dev->SRC_ADDR);

      /* construct the nonce from the MHR */
      mac_frame_get_nonce(&dev->mac_frame, nonce);

      /* perform the encryption, the TX buffer will contain a full MAC frame with encrypted payload*/
      dev->status = dwt_do_aes(&dev->aes_job_tx, dev->aes_config.aes_core_type);
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
      ret = dwt_starttx(DWT_START_TX_DELAYED);

      /* If dwt_starttx() returns an error, abandon this ranging exchange and proceed to the next one. See NOTE 10 below. */
      if (ret == DWT_SUCCESS) {
        /* Poll DW IC until TX frame sent event set. See NOTE 6 below. */
        while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK)) {};

        /* Clear TXFRS event. */
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
      }
    }
  } else {
    /* Clear RX error events in the DW IC status register. */
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
    Serial.println("Failed");
  }
}

TaskHandle_t Task1;

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

  /* Enabling LEDs here for debug so that for each TX the D1 LED will flash on DW3000 red eval-shield boards.
     * Note, in real low power applications the LEDs should not be used. */
  dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

  /* Configure DW IC. See NOTE 13 below. */
  if (dwt_configure(&config)) /* if the dwt_configure returns DWT_ERROR either the PLL or RX calibration has failed the host should reset the device */
  {
    test_run_info((unsigned char *)"CONFIG FAILED     ");
    while (1) {};
  }

  /* Configure the TX spectrum parameters (power, PG delay and PG count) */
  dwt_configuretxrf(&txconfig_options);
  /* Apply default antenna delay value. See NOTE 2 below. */
  dwt_setrxantennadelay(RX_ANT_DLY_TAG);
  dwt_settxantennadelay(TX_ANT_DLY_TAG);
  /* Next can enable TX/RX states output on GPIOs 5 and 6 to help debug, and also TX/RX LEDs
     * Note, in real low power applications the LEDs should not be used. */
  dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);

  Serial.begin(38400);
  Serial.println("Serial starting");

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  Serial.println(WiFi.macAddress());
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peers
  memcpy(peerInfo.peer_addr, tag_mac_addr, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer 1");
    // return;
  }

  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  if (operating_mode == 0) {
    // Serial.println("Device connected to " + String(curr_device));
    anchor(devices[curr_device]);
  }
  else if (operating_mode == 1) {
    vTaskDelay(1);
  }
  else {
    vTaskDelay(1);
  }
}