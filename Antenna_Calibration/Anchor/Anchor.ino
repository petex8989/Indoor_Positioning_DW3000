#include "dw3000.h"
#include "dw3000_mac_802_15_4.h"

#define APP_NAME "SS TWR AES INIT v1.0"

float this_anchor_target_distance = 3.048;  //measured distance to anchor in m
uint16_t this_anchor_Adelay = 16600;               //starting value
uint16_t Adelay_delta = 100;                       //initial binary search step size

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

mac_frame_802_15_4_format_t mac_frame2 = {
  { { 0x09, 0xEC },
    0x00,
    { 0x22, 0x43 },
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
    { 0x0F, { 0x00, 0x00, 0x00, 0x00 }, 0x00 } },
  0x00
};

mac_frame_802_15_4_format_t mac_frame3 = {
  { { 0x09, 0xEC },
    0x00,
    { 0x23, 0x43 },
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
    { 0x0F, { 0x00, 0x00, 0x00, 0x00 }, 0x00 } },
  0x00
};

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

static dwt_aes_config_t aes_config2 = {
  AES_key_RAM,
  AES_core_type_CCM,
  MIC_0,
  AES_KEY_Src_Register,
  AES_KEY_Load,
  0,
  AES_KEY_128bit,
  AES_Encrypt
};

static dwt_aes_config_t aes_config3 = {
  AES_key_RAM,
  AES_core_type_CCM,
  MIC_0,
  AES_KEY_Src_Register,
  AES_KEY_Load,
  0,
  AES_KEY_128bit,
  AES_Encrypt
};

dwt_aes_key_t keys_options1[NUM_OF_KEY_OPTIONS] = {
  { 0x00010203, 0x04050607, 0x08090A0B, 0x0C0D0E0F, 0x00000000, 0x00000000, 0x00000000, 0x00000000 },
  { 0x11223344, 0x55667788, 0x99AABBCC, 0xDDEEFF00, 0x00000000, 0x00000000, 0x00000000, 0x00000000 },
  { 0xFFEEDDCC, 0xBBAA9988, 0x77665544, 0x33221100, 0x00000000, 0x00000000, 0x00000000, 0x00000000 }
};

dwt_aes_key_t keys_options2[NUM_OF_KEY_OPTIONS] = {
  { 0x00010201, 0x04050601, 0x08090A01, 0x0C0D0E01, 0x00000000, 0x00000000, 0x00000000, 0x00000000 },
  { 0x11223341, 0x55667781, 0x99AABBC1, 0xDDEEFF01, 0x00000000, 0x00000000, 0x00000000, 0x00000000 },
  { 0xFFEEDDC1, 0xBBAA9981, 0x77665541, 0x33221101, 0x00000000, 0x00000000, 0x00000000, 0x00000000 }
};

dwt_aes_key_t keys_options3[NUM_OF_KEY_OPTIONS] = {
  { 0x00010202, 0x04050602, 0x08090A02, 0x0C0D0E02, 0x00000000, 0x00000000, 0x00000000, 0x00000000 },
  { 0x11223342, 0x55667782, 0x99AABBC2, 0xDDEEFF02, 0x00000000, 0x00000000, 0x00000000, 0x00000000 },
  { 0xFFEEDDC2, 0xBBAA9982, 0x77665542, 0x33221102, 0x00000000, 0x00000000, 0x00000000, 0x00000000 }
};

/* Initiator data */
#define DEST_ADDR1 0x1122334455667788 /* this is the address of the responder */
#define DEST_ADDR2 0x1122334455667789 /* this is the address of the responder */
#define DEST_ADDR3 0x112233445566778A /* this is the address of the responder */
#define SRC_ADDR1 0x8877665544332211  /* this is the address of the initiator */
#define SRC_ADDR2 0x8877665544332212  /* this is the address of the initiator */
#define SRC_ADDR3 0x8877665544332213  /* this is the address of the initiator */
#define DEST_PAN_ID1 0x4321           /* this is the PAN ID used in this example */
#define DEST_PAN_ID2 0x4322           /* this is the PAN ID used in this example */
#define DEST_PAN_ID3 0x4323           /* this is the PAN ID used in this example */

#define RX_BUF_LEN 127 /* The received frame cannot be bigger than 127 if STD PHR mode is used */

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

Device device1(mac_frame1, DEST_PAN_ID1, DEST_ADDR1, SRC_ADDR1, aes_config1, keys_options1);
Device device2(mac_frame2, DEST_PAN_ID2, DEST_ADDR2, SRC_ADDR1, aes_config2, keys_options1);
Device device3(mac_frame3, DEST_PAN_ID3, DEST_ADDR3, SRC_ADDR1, aes_config3, keys_options1);
Device devices[3] = { device1, device2, device3 };

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



/* Inter-ranging delay period, in milliseconds. */
#define RNG_DELAY_MS 300

/* Default antenna delay values for 64 MHz PRF. See NOTE 2 below. */
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385

#define START_RECEIVE_DATA_LOCATION 8  //MAC payload user data starts at index 8 (e.g. 'R' - in above response message)

/* Indexes to access some of the fields in the frames defined above. */
#define ALL_MSG_SN_IDX 2           //sequence number byte index in MHR
#define RESP_MSG_POLL_RX_TS_IDX 0  //index in the MAC payload for Poll RX time
#define RESP_MSG_RESP_TX_TS_IDX 4  //index in the MAC payload for Response TX time
#define RESP_MSG_TS_LEN 4

/* Note, the key index of 0 is forbidden to send as key index. Thus index 1 is the first.
 * This example uses this index for the key table for the encryption of initiator's data */
#define INITIATOR_KEY_INDEX 1

/* Buffer to store received response message.
 * Its size is adjusted to longest frame that this example code can handle. */

/* Delay between frames, in UWB microseconds. See NOTE 1 below. */
#define POLL_TX_TO_RESP_RX_DLY_UUS 1720
/* Receive response timeout. See NOTE 5 below. */
#define RESP_RX_TIMEOUT_UUS 250

/* Values for the PG_DELAY and TX_POWER registers reflect the bandwidth and power of the spectrum at the current
 * temperature. These values can be calibrated prior to taking reference measurements. See NOTE 2 below. */
extern dwt_txconfig_t txconfig_options;



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
  dwt_setrxantennadelay(this_anchor_Adelay);
  dwt_settxantennadelay(this_anchor_Adelay);

  /* Set expected response's delay and timeout. See NOTE 1 and 5 below.
     * This example is paired with the SS-TWR responder and if delays/timings need to be changed
     * they must be changed in both to match. */
  dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
  dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);

  /* Next can enable TX/RX states output on GPIOs 5 and 6 to help debug */
  dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);

  /*Configure the TX and RX AES jobs, the TX job is used to encrypt the Poll message,
     * the RX job is used to decrypt the Response message */
  device1.aes_job_tx.mode = AES_Encrypt;                                       /* this is encryption job */
  device1.aes_job_tx.src_port = AES_Src_Tx_buf;                                /* dwt_do_aes will take plain text to the TX buffer */
  device1.aes_job_tx.dst_port = AES_Dst_Tx_buf;                                /* dwt_do_aes will replace the original plain text TX buffer with encrypted one */
  device1.aes_job_tx.nonce = device1.nonce;                                    /* pointer to the nonce structure*/
  device1.aes_job_tx.header = (uint8_t *)MHR_802_15_4_PTR(&device1.mac_frame); /* plain-text header which will not be encrypted */
  device1.aes_job_tx.header_len = MAC_FRAME_HEADER_SIZE(&device1.mac_frame);
  device1.aes_job_tx.payload = device1.tx_poll_msg;             /* payload to be encrypted */
  device1.aes_job_tx.payload_len = sizeof(device1.tx_poll_msg); /* size of payload to be encrypted */

  device1.aes_job_rx.mode = AES_Decrypt;          /* this is decryption job */
  device1.aes_job_rx.src_port = AES_Src_Rx_buf_0; /* The source of the data to be decrypted is the IC RX buffer */
  device1.aes_job_rx.dst_port = AES_Dst_Rx_buf_0; /* Decrypt the encrypted data to the IC RX buffer : this will destroy original RX frame */
  device1.aes_job_rx.header_len = device1.aes_job_tx.header_len;
  device1.aes_job_rx.header = device1.aes_job_tx.header; /* plain-text header which will not be encrypted */
  device1.aes_job_rx.payload = device1.rx_buffer;        /* pointer to where the decrypted data will be copied to when read from the IC*/


  device2.aes_job_tx.mode = AES_Encrypt;                                       /* this is encryption job */
  device2.aes_job_tx.src_port = AES_Src_Tx_buf;                                /* dwt_do_aes will take plain text to the TX buffer */
  device2.aes_job_tx.dst_port = AES_Dst_Tx_buf;                                /* dwt_do_aes will replace the original plain text TX buffer with encrypted one */
  device2.aes_job_tx.nonce = device2.nonce;                                    /* pointer to the nonce structure*/
  device2.aes_job_tx.header = (uint8_t *)MHR_802_15_4_PTR(&device2.mac_frame); /* plain-text header which will not be encrypted */
  device2.aes_job_tx.header_len = MAC_FRAME_HEADER_SIZE(&device2.mac_frame);
  device2.aes_job_tx.payload = device2.tx_poll_msg;             /* payload to be encrypted */
  device2.aes_job_tx.payload_len = sizeof(device2.tx_poll_msg); /* size of payload to be encrypted */

  device2.aes_job_rx.mode = AES_Decrypt;          /* this is decryption job */
  device2.aes_job_rx.src_port = AES_Src_Rx_buf_0; /* The source of the data to be decrypted is the IC RX buffer */
  device2.aes_job_rx.dst_port = AES_Dst_Rx_buf_0; /* Decrypt the encrypted data to the IC RX buffer : this will destroy original RX frame */
  device2.aes_job_rx.header_len = device2.aes_job_tx.header_len;
  device2.aes_job_rx.header = device2.aes_job_tx.header; /* plain-text header which will not be encrypted */
  device2.aes_job_rx.payload = device2.rx_buffer;        /* pointer to where the decrypted data will be copied to when read from the IC*/

  device3.aes_job_tx.mode = AES_Encrypt;                                       /* this is encryption job */
  device3.aes_job_tx.src_port = AES_Src_Tx_buf;                                /* dwt_do_aes will take plain text to the TX buffer */
  device3.aes_job_tx.dst_port = AES_Dst_Tx_buf;                                /* dwt_do_aes will replace the original plain text TX buffer with encrypted one */
  device3.aes_job_tx.nonce = device3.nonce;                                    /* pointer to the nonce structure*/
  device3.aes_job_tx.header = (uint8_t *)MHR_802_15_4_PTR(&device3.mac_frame); /* plain-text header which will not be encrypted */
  device3.aes_job_tx.header_len = MAC_FRAME_HEADER_SIZE(&device3.mac_frame);
  device3.aes_job_tx.payload = device3.tx_poll_msg;             /* payload to be encrypted */
  device3.aes_job_tx.payload_len = sizeof(device3.tx_poll_msg); /* size of payload to be encrypted */

  device3.aes_job_rx.mode = AES_Decrypt;          /* this is decryption job */
  device3.aes_job_rx.src_port = AES_Src_Rx_buf_0; /* The source of the data to be decrypted is the IC RX buffer */
  device3.aes_job_rx.dst_port = AES_Dst_Rx_buf_0; /* Decrypt the encrypted data to the IC RX buffer : this will destroy original RX frame */
  device3.aes_job_rx.header_len = device3.aes_job_tx.header_len;
  device3.aes_job_rx.header = device3.aes_job_tx.header; /* plain-text header which will not be encrypted */
  device3.aes_job_rx.payload = device3.rx_buffer;        /* pointer to where the decrypted data will be copied to when read from the IC*/

  Serial.begin(38400);
  Serial.println("Serial starting");
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
        distance = tof * SPEED_OF_LIGHT;
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

void calibrate() {
  float last_delta = 0.0;
  while(1) {
    float dist = ranging(&device1, 100);
    if (Adelay_delta < 2) { // Antenna delay is close enough
      Serial.print("final Adelay ");
      Serial.println(this_anchor_Adelay);
      return;
    }
    // error in measured distance
    float this_delta = dist - this_anchor_target_distance;
    if (this_delta * last_delta < 0.0) {
      // sign changed, reduce step size
      Adelay_delta = Adelay_delta / 2;  
    }
    last_delta = this_delta;
    if (this_delta > 0.0) {
      //new trial Adelay
      this_anchor_Adelay += Adelay_delta;
    }
    else this_anchor_Adelay -= Adelay_delta;
    // Set antenna delay to new delay
    dwt_setrxantennadelay(this_anchor_Adelay);
    dwt_settxantennadelay(this_anchor_Adelay);
  }
}

void loop() {
  calibrate();
  this_anchor_Adelay = 16600;
  dwt_setrxantennadelay(this_anchor_Adelay);
  dwt_settxantennadelay(this_anchor_Adelay);
  Adelay_delta = 100;
  // int del = 16372;
  // // dwt_setrxantennadelay(del);
  // // dwt_settxantennadelay(del);
  // // Serial.println(ranging(&device1, 100));
}

/*****************************************************************************************************************************************************
 * NOTES:
 *
 * 1. The single-sided two-way ranging scheme implemented here has to be considered carefully as the accuracy of the distance measured is highly
 *    sensitive to the clock offset error between the devices and the length of the response delay between frames. To achieve the best possible
 *    accuracy, this response delay must be kept as low as possible. In order to do so, 6.8 Mbps data rate is used in this example and the response
 *    delay between frames is defined as low as possible. The user is referred to User Manual for more details about the single-sided two-way ranging
 *    process.  NB:SEE ALSO NOTE 11.
 *
 *    Initiator: |Poll TX| ..... |Resp RX|
 *    Responder: |Poll RX| ..... |Resp TX|
 *                   ^|P RMARKER|                    - time of Poll TX/RX
 *                                   ^|R RMARKER|    - time of Resp TX/RX
 *
 *                       <--TDLY->                   - POLL_TX_TO_RESP_RX_DLY_UUS (RDLY-RLEN)
 *                               <-RLEN->            - RESP_RX_TIMEOUT_UUS   (length of response frame)
 *                    <----RDLY------>               - POLL_RX_TO_RESP_TX_DLY_UUS (depends on how quickly responder can turn around and reply)
 *
 *
 * 2. The sum of the values is the TX to RX antenna delay, this should be experimentally determined by a calibration process. Here we use a hard coded
 *    value (expected to be a little low so a positive error will be seen on the resultant distance estimate. For a real production application, each
 *    device should have its own antenna delay properly calibrated to get good precision when performing range measurements.
 * 3. The frames used here are Decawave specific ranging frames, complying with the IEEE 802.15.4 standard data frame encoding. The frames are the
 *    following:
 *     - a poll message sent by the initiator to trigger the ranging exchange.
 *     - a response message sent by the responder to complete the exchange and provide all information needed by the initiator to compute the
 *       time-of-flight (distance) estimate.
 *    The first 10 bytes of those frame are common and are composed of the following fields:
 *     - byte 0/1: frame control (0x8841 to indicate a data frame using 16-bit addressing).
 *     - byte 2: sequence number, incremented for each new frame.
 *     - byte 3/4: PAN ID (0xDECA).
 *     - byte 5/6: destination address, see NOTE 4 below.
 *     - byte 7/8: source address, see NOTE 4 below.
 *     - byte 9: function code (specific values to indicate which message it is in the ranging process).
 *    The remaining bytes are specific to each message as follows:
 *    Poll message:
 *     - no more data
 *    Response message:
 *     - byte 0 -> 13: poll message reception timestamp.
 *     - byte 4 -> 17: response message transmission timestamp.
 *    All messages end with a 2-byte checksum automatically set by DW IC.
 * 4. Source and destination addresses are hard coded constants in this example to keep it simple but for a real product every device should have a
 *    unique ID. Here, 16-bit addressing is used to keep the messages as short as possible but, in an actual application, this should be done only
 *    after an exchange of specific messages used to define those short addresses for each device participating to the ranging exchange.
 * 5. This timeout is for complete reception of a frame, i.e. timeout duration must take into account the length of the expected frame. Here the value
 *    is arbitrary but chosen large enough to make sure that there is enough time to receive the complete response frame sent by the responder at the
 *    6.8M data rate used (around 200 �s).
 * 6. In a real application, for optimum performance within regulatory limits, it may be necessary to set TX pulse bandwidth and TX power, (using
 *    the dwt_configuretxrf API call) to per device calibrated values saved in the target system or the DW IC OTP memory.
 * 7. dwt_writetxdata() takes the full size of the message as a parameter but only copies (size - 2) bytes as the check-sum at the end of the frame is
 *    automatically appended by the DW IC. This means that our variable could be two bytes shorter without losing any data (but the sizeof would not
 *    work anymore then as we would still have to indicate the full length of the frame to dwt_writetxdata()).
 * 8. We use polled mode of operation here to keep the example as simple as possible but all status events can be used to generate interrupts. Please
 *    refer to DW IC User Manual for more details on "interrupts". It is also to be noted that STATUS register is 5 bytes long but, as the event we
 *    use are all in the first bytes of the register, we can use the simple dwt_read32bitreg() API call to access it instead of reading the whole 5
 *    bytes.
 * 9. The high order byte of each 40-bit time-stamps is discarded here. This is acceptable as, on each device, those time-stamps are not separated by
 *    more than 2**32 device time units (which is around 67 ms) which means that the calculation of the round-trip delays can be handled by a 32-bit
 *    subtraction.
 * 10. The user is referred to DecaRanging ARM application (distributed with EVK1000 product) for additional practical example of usage, and to the
 *     DW IC API Guide for more details on the DW IC driver functions.
 * 11. The use of the clock offset value to correct the TOF calculation, significantly improves the result of the SS-TWR where the remote
 *     responder unit's clock is a number of PPM offset from the local initiator unit's clock.
 *     As stated in NOTE 2 a fixed offset in range will be seen unless the antenna delay is calibrated and set correctly.
 * 12. In this example, the DW IC is put into IDLE state after calling dwt_initialise(). This means that a fast SPI rate of up to 20 MHz can be used
 *     thereafter.
 * 13. Frame counter was set to zero in this example. The frame counter should be incremented each frame. When frame counter gets to its max value (uint32_t),
 *     key should be replaced.
 * 14. Desired configuration by user may be different to the current programmed configuration. dwt_configure is called to set desired
 *     configuration.
 * 15. When CCM core type is used, AES_KEY_Load needs to be set prior to each encryption/decryption operation, even if the AES KEY used has not changed.
 ****************************************************************************************************************************************************/
