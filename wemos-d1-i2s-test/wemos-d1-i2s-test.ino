/**
 ******************************************************************************
 * @file    listener.ino
 * @author  Joe Todd
 * @version
 * @date    November 2017
 * @brief   I2S interface for ESP8266 and SPH0645 MEMS microphone.
 *
  ******************************************************************************/
// #include <ESP8266WiFi.h>

extern "C" {
#include "user_interface.h"
#include "i2s_reg.h"
#include "slc_register.h"
#include "esp8266_peri.h"
#include "sos-iir-filter.h"
  void rom_i2c_writeReg_Mask(int, int, int, int, int, int);
}

//
// IIR Filters
//

// DC-Blocker filter - removes DC component from I2S data
// See: https://www.dsprelated.com/freebooks/filters/DC_Blocker.html
// a1 = -0.9992 should heavily attenuate frequencies below 10Hz
SOS_IIR_Filter DC_BLOCKER = {
  gain: 1.0,
  sos: { { -1.0, 0.0, +0.9992, 0 } }
};

//
// Equalizer IIR filters to flatten microphone frequency response
// See respective .m file for filter design. Fs = 48Khz.
//
// Filters are represented as Second-Order Sections cascade with assumption
// that b0 and a0 are equal to 1.0 and 'gain' is applied at the last step
// B and A coefficients were transformed with GNU Octave:
// [sos, gain] = tf2sos(B, A)
// See: https://www.dsprelated.com/freebooks/filters/Series_Second_Order_Sections.html
// NOTE: SOS matrix 'a1' and 'a2' coefficients are negatives of tf2sos output
//

// TDK/InvenSense ICS-43434
// Datasheet: https://www.invensense.com/wp-content/uploads/2016/02/DS-000069-ICS-43434-v1.1.pdf
// B = [0.477326418836803, -0.486486982406126, -0.336455844522277, 0.234624646917202, 0.111023257388606];
// A = [1.0, -1.93073383849136326, 0.86519456089576796, 0.06442838283825100, 0.00111249298800616];
SOS_IIR_Filter ICS43434 = {
  gain: 0.477326418836803,
  sos: { // Second-Order Sections {b1, b2, -a1, -a2}
         { +0.96986791463971267, 0.23515976355743193, -0.06681948004769928, -0.00111521990688128 },
         { -1.98905931743624453, 0.98908924206960169, +1.99755331853906037, -0.99755481510122113 } }
};

// TDK/InvenSense ICS-43432
// Datasheet: https://www.invensense.com/wp-content/uploads/2015/02/ICS-43432-data-sheet-v1.3.pdf
// B = [-0.45733702338341309   1.12228667105574775  -0.77818278904413563, 0.00968926337978037, 0.10345668405223755]
// A = [1.0, -3.3420781082912949, 4.4033694320978771, -3.0167072679918010, 1.2265536567647031, -0.2962229189311990, 0.0251085747458112]
SOS_IIR_Filter ICS43432 = {
  gain: -0.457337023383413,
  sos: { // Second-Order Sections {b1, b2, -a1, -a2}
         { -0.544047931916859, -0.248361759321800, +0.403298891662298, -0.207346186351843 },
         { -1.909911869441421, +0.910830292683527, +1.790285722826743, -0.804085812369134 },
         { +0.000000000000000, +0.000000000000000, +1.148493493802252, -0.150599527756651 } }
};

// TDK/InvenSense INMP441
// Datasheet: https://www.invensense.com/wp-content/uploads/2015/02/INMP441.pdf
// B ~= [1.00198, -1.99085, 0.98892]
// A ~= [1.0, -1.99518, 0.99518]
SOS_IIR_Filter INMP441 = {
  gain: 1.00197834654696,
  sos: { // Second-Order Sections {b1, b2, -a1, -a2}
         { -1.986920458344451, +0.986963226946616, +1.995178510504166, -0.995184322194091 } }
};

// Infineon IM69D130 Shield2Go
// Datasheet: https://www.infineon.com/dgdl/Infineon-IM69D130-DS-v01_00-EN.pdf?fileId=5546d462602a9dc801607a0e46511a2e
// B ~= [1.001240684967527, -1.996936108836337, 0.995703101823006]
// A ~= [1.0, -1.997675693595542, 0.997677044195563]
// With additional DC blocking component
SOS_IIR_Filter IM69D130 = {
  gain: 1.00124068496753,
  sos: {
    { -1.0, 0.0, +0.9992, 0 },  // DC blocker, a1 = -0.9992
    { -1.994461610298131, 0.994469278738208, +1.997675693595542, -0.997677044195563 } }
};

// Knowles SPH0645LM4H-B, rev. B
// https://cdn-shop.adafruit.com/product-files/3421/i2S+Datasheet.PDF
// B ~= [1.001234, -1.991352, 0.990149]
// A ~= [1.0, -1.993853, 0.993863]
// With additional DC blocking component
SOS_IIR_Filter SPH0645LM4H_B_RB = {
  gain: 1.00123377961525,
  sos: {                             // Second-Order Sections {b1, b2, -a1, -a2}
         { -1.0, 0.0, +0.9992, 0 },  // DC blocker, a1 = -0.9992
         { -1.988897663539382, +0.988928479008099, +1.993853376183491, -0.993862821429572 } }
};

//
// Weighting filters
//

//
// A-weighting IIR Filter, Fs = 48KHz
// (By Dr. Matt L., Source: https://dsp.stackexchange.com/a/36122)
// B = [0.169994948147430, 0.280415310498794, -1.120574766348363, 0.131562559965936, 0.974153561246036, -0.282740857326553, -0.152810756202003]
// A = [1.0, -2.12979364760736134, 0.42996125885751674, 1.62132698199721426, -0.96669962900852902, 0.00121015844426781, 0.04400300696788968]
SOS_IIR_Filter A_weighting = {
  gain: 0.169994948147430,
  sos: { // Second-Order Sections {b1, b2, -a1, -a2}
         { -2.00026996133106, +1.00027056142719, -1.060868438509278, -0.163987445885926 },
         { +4.35912384203144, +3.09120265783884, +1.208419926363593, -0.273166998428332 },
         { -0.70930303489759, -0.29071868393580, +1.982242159753048, -0.982298594928989 } }
};

//
// C-weighting IIR Filter, Fs = 48KHz
// Designed by invfreqz curve-fitting, see respective .m file
// B = [-0.49164716933714026, 0.14844753846498662, 0.74117815661529129, -0.03281878334039314, -0.29709276192593875, -0.06442545322197900, -0.00364152725482682]
// A = [1.0, -1.0325358998928318, -0.9524000181023488, 0.8936404694728326   0.2256286147169398  -0.1499917107550188, 0.0156718181681081]
SOS_IIR_Filter C_weighting = {
  gain: -0.491647169337140,
  sos: {
    { +1.4604385758204708, +0.5275070373815286, +1.9946144559930252, -0.9946217070140883 },
    { +0.2376222404939509, +0.0140411206016894, -1.3396585608422749, -0.4421457807694559 },
    { -2.0000000000000000, +1.0000000000000000, +0.3775800047420818, -0.0356365756680430 } }
};



// #define DEBUG

#define I2S_CLK_FREQ 160000000  // Hz
#define I2S_24BIT 3             // I2S 24 bit half data
#define I2S_LEFT 2              // I2S RX Left channel

#define I2SI_DATA 12  // I2S data on GPIO12
#define I2SI_BCK 13   // I2S clk on GPIO13
#define I2SI_WS 14    // I2S select on GPIO14

#define SLC_BUF_CNT 8   // Number of buffers in the I2S circular buffer
#define SLC_BUF_LEN 64  // Length of one buffer, in 32-bit words.

/**
 * Convert I2S data.
 * Data is 18 bit signed, MSBit first, two's complement.
 * Note: We can only send 31 cycles from ESP8266 so we only
 * shift by 13 instead of 14.
 * The 240200 is a magic calibration number I haven't figured
 * out yet.
 */
#define convert(sample) (((int32_t)(sample) >> 13) - 240200)

typedef struct {
  uint32_t blocksize : 12;
  uint32_t datalen : 12;
  uint32_t unused : 5;
  uint32_t sub_sof : 1;
  uint32_t eof : 1;
  volatile uint32_t owner : 1;

  uint32_t *buf_ptr;
  uint32_t *next_link_ptr;
} sdio_queue_t;

static sdio_queue_t i2s_slc_items[SLC_BUF_CNT];  // I2S DMA buffer descriptors
static uint32_t *i2s_slc_buf_pntr[SLC_BUF_CNT];  // Pointer to the I2S DMA buffer data
static volatile uint32_t rx_buf_cnt = 0;
static volatile uint32_t rx_buf_idx = 0;
static volatile bool rx_buf_flag = false;

void i2s_init();
void slc_init();
void i2s_set_rate(uint32_t rate);
void slc_isr(void *para);

float readoutMaxScale = 30000;

/* Main -----------------------------------------------------------------------*/
void setup() {
  Serial.println("START");
  rx_buf_cnt = 0;

  pinMode(I2SI_WS, OUTPUT);
  pinMode(I2SI_BCK, OUTPUT);
  pinMode(I2SI_DATA, INPUT);

  // WiFi.forceSleepBegin();
  // delay(500);

  Serial.begin(115200);

  slc_init();
  i2s_init();
}

void loop() {
  int32_t value;
  char withScale[256];

  if (rx_buf_flag) {
    for (int x = 0; x < SLC_BUF_LEN; x++) {
      if (i2s_slc_buf_pntr[rx_buf_idx][x] > 0) {
#ifdef DEBUG
        Serial.print(i2s_slc_buf_pntr[rx_buf_idx][x], BIN);
        Serial.println("");
#else
        value = convert(i2s_slc_buf_pntr[rx_buf_idx][x]);
        // value = i2s_slc_buf_pntr[rx_buf_idx][x];
        // sprintf(withScale, "-1 %f 1", (float)value / 4096.0f);
        // Serial.println(withScale);

        Serial.print(readoutMaxScale * -1);  // To freeze the lower limit
        Serial.print(" ");
        Serial.print(readoutMaxScale);  // To freeze the upper limit
        Serial.print(" ");
        Serial.println(value);
#endif
      }
    }
    rx_buf_flag = false;
  }
}

/* Function definitions -------------------------------------------------------*/

/**
 * Initialise I2S as a RX master.
 */
void i2s_init() {
  // Config RX pin function
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDI_U, FUNC_I2SI_DATA);
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTCK_U, FUNC_I2SI_BCK);
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTMS_U, FUNC_I2SI_WS);

  // Enable a 160MHz clock
  I2S_CLK_ENABLE();

  // Reset I2S
  I2SC &= ~(I2SRST);
  I2SC |= I2SRST;
  I2SC &= ~(I2SRST);

  // Reset DMA
  I2SFC &= ~(I2SDE | (I2SRXFMM << I2SRXFM));

  // Enable DMA
  I2SFC |= I2SDE | (I2S_24BIT << I2SRXFM);

  // Set RX single channel (left)
  I2SCC &= ~((I2STXCMM << I2STXCM) | (I2SRXCMM << I2SRXCM));
  I2SCC |= (I2S_LEFT << I2SRXCM);
  i2s_set_rate(16667);

  // Set RX data to be received
  I2SRXEN = SLC_BUF_LEN;

  // Bits mode
  I2SC |= (15 << I2SBM);

  // Start receiver
  I2SC |= I2SRXS;
}

/**
 * Set I2S clock.
 * I2S bits mode only has space for 15 extra bits,
 * 31 in total. The
 */
void i2s_set_rate(uint32_t rate) {
  uint32_t i2s_clock_div = (I2S_CLK_FREQ / (rate * 31 * 2)) & I2SCDM;
  uint32_t i2s_bck_div = (I2S_CLK_FREQ / (rate * i2s_clock_div * 31 * 2)) & I2SBDM;

#ifdef DEBUG
  Serial.printf("Rate %u Div %u Bck %u Freq %u\n",
                rate, i2s_clock_div, i2s_bck_div, I2S_CLK_FREQ / (i2s_clock_div * i2s_bck_div * 31 * 2));
#endif

  // RX master mode, RX MSB shift, right first, msb right
  I2SC &= ~(I2STSM | I2SRSM | (I2SBMM << I2SBM) | (I2SBDM << I2SBD) | (I2SCDM << I2SCD));
  I2SC |= I2SRF | I2SMR | I2SRMS | (i2s_bck_div << I2SBD) | (i2s_clock_div << I2SCD);
}

/**
 * Initialize the SLC module for DMA operation.
 * Counter intuitively, we use the TXLINK here to
 * receive data.
 */
void slc_init() {
  for (int x = 0; x < SLC_BUF_CNT; x++) {
    i2s_slc_buf_pntr[x] = (uint32_t *)malloc(SLC_BUF_LEN * 4);
    for (int y = 0; y < SLC_BUF_LEN; y++) i2s_slc_buf_pntr[x][y] = 0;

    i2s_slc_items[x].unused = 0;
    i2s_slc_items[x].owner = 1;
    i2s_slc_items[x].eof = 0;
    i2s_slc_items[x].sub_sof = 0;
    i2s_slc_items[x].datalen = SLC_BUF_LEN * 4;
    i2s_slc_items[x].blocksize = SLC_BUF_LEN * 4;
    i2s_slc_items[x].buf_ptr = (uint32_t *)&i2s_slc_buf_pntr[x][0];
    i2s_slc_items[x].next_link_ptr = (uint32_t *)((x < (SLC_BUF_CNT - 1)) ? (&i2s_slc_items[x + 1]) : (&i2s_slc_items[0]));
  }

  // Reset DMA
  ETS_SLC_INTR_DISABLE();
  SLCC0 |= SLCRXLR | SLCTXLR;
  SLCC0 &= ~(SLCRXLR | SLCTXLR);
  SLCIC = 0xFFFFFFFF;

  // Configure DMA
  SLCC0 &= ~(SLCMM << SLCM);     // Clear DMA MODE
  SLCC0 |= (1 << SLCM);          // Set DMA MODE to 1
  SLCRXDC |= SLCBINR | SLCBTNR;  // Enable INFOR_NO_REPLACE and TOKEN_NO_REPLACE

  // Feed DMA the 1st buffer desc addr
  SLCTXL &= ~(SLCTXLAM << SLCTXLA);
  SLCTXL |= (uint32_t)&i2s_slc_items[0] << SLCTXLA;

  ETS_SLC_INTR_ATTACH(slc_isr, NULL);

  // Enable EOF interrupt
  SLCIE = SLCITXEOF;
  ETS_SLC_INTR_ENABLE();

  // Start transmission
  SLCTXL |= SLCTXLS;
}

/**
 * Triggered when SLC has finished writing
 * to one of the buffers.
 */
void ICACHE_RAM_ATTR
slc_isr(void *para) {
  uint32_t status;

  status = SLCIS;
  SLCIC = 0xFFFFFFFF;

  if (status == 0) {
    return;
  }

  if (status & SLCITXEOF) {
    // We have received a frame
    ETS_SLC_INTR_DISABLE();
    sdio_queue_t *finished = (sdio_queue_t *)SLCTXEDA;

    finished->eof = 0;
    finished->owner = 1;
    finished->datalen = 0;

    for (int i = 0; i < SLC_BUF_CNT; i++) {
      if (finished == &i2s_slc_items[i]) {
        rx_buf_idx = i;
      }
    }
    rx_buf_cnt++;
    rx_buf_flag = true;
    ETS_SLC_INTR_ENABLE();
  }
}
