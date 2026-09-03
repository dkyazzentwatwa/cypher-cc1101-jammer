//
// CC1101 interactive terminal tool
// allows for sending / receiving data over serial port
// on selected radio channel, modulation, ..
//
// (C) Adam Loboda '2023 , adam.loboda@wp.pl
//  
// based on great SmartRC library by Little_S@tan
// Please download ZIP from 
// https://github.com/LSatan/SmartRC-CC1101-Driver-Lib
// and attach it as ZIP library for Arduino
//
// Also uses Arduino Command Line interpreter by Edgar Bonet
// from https://gist.github.com/edgar-bonet/607b387260388be77e96
//
// This code will ONLY work with ESP8266 board
//

#include <ELECHOUSE_CC1101_SRC_DRV.h>
#include <EEPROM.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

#define CCBUFFERSIZE 64
#define RECORDINGBUFFERSIZE 4096   // Buffer for recording the frames
#define EPROMSIZE 4096              // Size of EEPROM in your Arduino chip. For  ESP8266 size is 4096
#define BUF_LENGTH 128             // Buffer for the incoming command.

// named constants (replacing magic numbers scattered through the code)
#define MAX_PAYLOAD 60             // max RF payload bytes per frame
#define MAX_HEXCHARS 120           // max hex input chars (2 * MAX_PAYLOAD)
#define HEXDUMP_CHUNK 32           // bytes per chunk in RAW hex dumps
#define SCAN_RSSI_MARK (-75)       // dBm threshold to consider a signal present
#define RSSI_NONE (-100)           // sentinel "no signal" rssi value

// ---- WiFi Access Point configuration (edit to taste) ----
#define AP_SSID     "cc1101"
#define AP_PASSWORD "cc1101"          // >= 8 chars for WPA2; "" for an open AP
IPAddress apIP(192, 168, 1, 100);
IPAddress apGateway(192, 168, 1, 1);
IPAddress apSubnet(255, 255, 255, 0);

ESP8266WebServer server(80);

// Command output target. Defaults to Serial; temporarily repointed at a
// String sink while a command runs from the web (/cmd).
Print* out = &Serial;

// Live values decoded from the CC1101 configuration registers.
float currentFreq = 433.92;
float currentRxBw = 812.50;

// defining PINs set for ESP8266 - WEMOS D1 MINI module
byte sck = 14;     // GPIO 14
byte miso = 12;  // GPIO 12
byte mosi = 13;  // GPIO 13
byte ss = 15;      // GPIO 15
int gdo0 = 5;     // GPIO 5
int gdo2 = 4;     // GPIO 4


// position in big recording buffer
int bigrecordingbufferpos = 0; 

// number of frames in big recording buffer
int framesinbigrecordingbuffer = 0; 

// CLI activity: exactly one mode active at a time.
enum Mode { MODE_IDLE, MODE_RX, MODE_JAM, MODE_REC, MODE_CHAT, MODE_SCAN, MODE_SNIFF, MODE_BRUTE, MODE_RECRAW, MODE_PLAYRAW };
Mode activeMode = MODE_IDLE;

// Shared live telemetry for serial feedback and the web status panel.
unsigned long modeStartedAt = 0;
unsigned long lastModeFeedbackAt = 0;
uint32_t modeActivityCount = 0;

// scan mode state (serviced one frequency step per loop pass)
float scanFrom = 0, scanTo = 0, scanCursor = 0;
float scanBestFreq = 0;  int scanBestRssi = -100;  long scanCompare = 0;

// sniffer state - captured in small chunks per loop pass so the web stays responsive
#define SNIFF_CHUNK 128          // bytes captured per loop pass (divides RECORDINGBUFFERSIZE)
int sniffInterval = 0;           // microseconds per sample while MODE_SNIFF is active
int sniffCursor = 0;             // write position in the recording buffer

// RAW record/replay state (non-blocking background modes)
int recrawInterval = 0;          // microseconds per sample while MODE_RECRAW captures
bool recrawWaiting = false;      // true while armed and waiting for the RF line to go high
int playrawInterval = 0;         // microseconds per sample while MODE_PLAYRAW replays
// Watchdog-safe caps for RAW per-sample intervals. Every RAW path clamps to one of these so a
// single delayMicroseconds() can never overrun the ESP8266 watchdog. RECRAW/PLAYRAW run as one
// continuous block, so their cap keeps 4096 B x 8 bits under the hardware WDT (150 us ~= 4.9 s).
// RXRAW/BRUTE return to loop() between chunks, so they tolerate a looser cap. Tuned on-device.
#define RAW_MAX_INTERVAL        150
#define RAW_CHUNK_MAX_INTERVAL  1000

// brute mode state (a batch of codes serviced per loop pass)
int bruteInterval = 0, bruteBits = 0;
uint32_t bruteCode = 0, bruteMax = 0;

// human-readable name for /status
static const char* modeName(void) {
    switch (activeMode) {
        case MODE_RX: return "rx";          case MODE_JAM: return "jam";
        case MODE_REC: return "rec";        case MODE_CHAT: return "chat";
        case MODE_SCAN: return "scan";      case MODE_SNIFF: return "sniff";
        case MODE_BRUTE: return "brute";    case MODE_RECRAW: return "recraw";
        case MODE_PLAYRAW: return "playraw"; default: return "idle";
    }
}

static bool do_echo = true;

// buffer for receiving  CC1101
byte ccreceivingbuffer[CCBUFFERSIZE] = {0};

// buffer for sending  CC1101
byte ccsendingbuffer[CCBUFFERSIZE] = {0};

// buffer for recording and replaying of many frames
byte bigrecordingbuffer[RECORDINGBUFFERSIZE] = {0};

// buffer for hex to ascii conversions
byte textbuffer[BUF_LENGTH];


// Forward declarations. Arduino's auto-prototype generator mangles the
// return type of static functions on this sketch (producing broken
// prototypes and bogus #line directives that fail to compile), so we
// declare every function explicitly to suppress that generation.
void asciitohex(byte *ascii_ptr, byte *hex_ptr, int len);
void hextoascii(byte *ascii_ptr, byte *hex_ptr, int len);
static void cc1101initialize(void);
static bool enterRadioIdle(void);
static bool calibrateRadio(void);
static bool waitForRadioState(byte wanted, unsigned long timeoutMs);
static float readRadioFrequencyMHz(void);
static float readRadioRxBwKHz(void);
static byte readRadioState(void);
static float nearestRxBwKHz(float requested);
static void configureReceiveForModulation(int mod);
static bool supportedPa(int pa);
static bool supportedFrequency(float mhz);
static byte radioBand(float mhz);
static bool parseFloatArg(const char *text, float *value);
static bool parseIntArg(const char *text, int *value);
static bool validRadioProfile(float mhz, int mod, float drate, float dev, float rxbw, int pa);
static bool applyRadioProfile(float mhz, int mod, float drate, float dev, float rxbw, int pa);
static void zeroRecordingBuffer(void);
static void enterRawMode(bool tx);
static void exitRawMode(bool tx);
static void dumpBufferHex(int start, int count);
static bool ingestHex(char *in, byte *out, int *outlen);
static const char* modeName(void);
static void exec(char *cmdline);
static void serviceActiveMode(void);
static void stopActiveMode(void);
static void startActiveMode(Mode mode);
static bool modeFeedbackDue(unsigned long intervalMs);
static unsigned long modeElapsedSeconds(void);
static int modeProgressPercent(void);
void setup();
void loop();
static void startAP(void);
static void setupWebServer(void);
static void handleRoot(void);
static void handleCmd(void);
static void handleStatus(void);


// convert bytes in table to string with hex numbers
void asciitohex(byte *ascii_ptr, byte *hex_ptr,int len)
{
    byte i,j,k;
    for(i = 0; i < len; i++)
    {
      // high byte first
      j = ascii_ptr[i] / 16;
      if (j>9) 
         { k = j - 10 + 65; }
      else 
         { k = j + 48; }
      hex_ptr[2*i] = k ;
      // low byte second
      j = ascii_ptr[i] % 16;
      if (j>9) 
         { k = j - 10 + 65; }
      else
         { k = j + 48; }
      hex_ptr[(2*i)+1] = k ;
    };
    // terminate right after the last written nibble (i == len here)
    hex_ptr[2*len] = '\0' ;
}


// convert string with hex numbers to array of bytes
 void  hextoascii(byte *ascii_ptr, byte *hex_ptr,int len)
{
    byte i,j;
    for(i = 0; i < (len/2); i++)
     {
     // start from zero so invalid nibbles leave a defined 0, not garbage
     ascii_ptr[i] = 0;
     j = hex_ptr[i*2];
     if ((j>47) && (j<58))  ascii_ptr[i] = (j - 48) * 16;
     if ((j>64) && (j<71))  ascii_ptr[i] = (j - 55) * 16;
     if ((j>96) && (j<103)) ascii_ptr[i] = (j - 87) * 16;
     j = hex_ptr[i*2+1];
     if ((j>47) && (j<58))  ascii_ptr[i] += (j - 48);
     if ((j>64) && (j<71))  ascii_ptr[i] += (j - 55);
     if ((j>96) && (j<103)) ascii_ptr[i] += (j - 87);
     };
    ascii_ptr[i] = '\0' ;
}

// Initialize CC1101 board with default settings, you may change your preferences here
static void cc1101initialize(void)
{
    // initializing library with custom pins selected
     ELECHOUSE_cc1101.setSpiPin(sck, miso, mosi, ss);
     ELECHOUSE_cc1101.setGDO(gdo0, gdo2);

    // Main part to tune CC1101 with proper frequency, modulation and encoding    
    ELECHOUSE_cc1101.Init();                // must be set to initialize the cc1101!
    ELECHOUSE_cc1101.setGDO0(gdo0);         // set lib internal gdo pin (gdo0). Gdo2 not use for this example.
    ELECHOUSE_cc1101.setCCMode(1);          // set config for internal transmission mode. value 0 is for RAW recording/replaying
    ELECHOUSE_cc1101.setModulation(2);      // set modulation mode. 0 = 2-FSK, 1 = GFSK, 2 = ASK/OOK, 3 = 4-FSK, 4 = MSK.
    ELECHOUSE_cc1101.setMHZ(433.92);        // Here you can set your basic frequency. The lib calculates the frequency automatically (default = 433.92).The cc1101 can: 300-348 MHZ, 387-464MHZ and 779-928MHZ. Read More info from datasheet.
    currentFreq = 433.92;                    // keep the web /status readout in sync
    ELECHOUSE_cc1101.setDeviation(47.60);   // Set the Frequency deviation in kHz. Value from 1.58 to 380.85. Default is 47.60 kHz.
    ELECHOUSE_cc1101.setChannel(0);         // Set the Channelnumber from 0 to 255. Default is cahnnel 0.
    ELECHOUSE_cc1101.setChsp(199.95);       // The channel spacing is multiplied by the channel number CHAN and added to the base frequency in kHz. Value from 25.39 to 405.45. Default is 199.95 kHz.
    ELECHOUSE_cc1101.setRxBW(812.50);       // Set the Receive Bandwidth in kHz. Value from 58.03 to 812.50. Default is 812.50 kHz.
    ELECHOUSE_cc1101.setDRate(9.6);         // Set the Data Rate in kBaud. Value from 0.02 to 1621.83. Default is 99.97 kBaud!
    ELECHOUSE_cc1101.setPA(10);             // Set TxPower. The following settings are possible depending on the frequency band.  (-30  -20  -15  -10  -6    0    5    7    10   11   12) Default is max!
    ELECHOUSE_cc1101.setSyncMode(2);        // Combined sync-word qualifier mode. 0 = No preamble/sync. 1 = 16 sync word bits detected. 2 = 16/16 sync word bits detected. 3 = 30/32 sync word bits detected. 4 = No preamble/sync, carrier-sense above threshold. 5 = 15/16 + carrier-sense above threshold. 6 = 16/16 + carrier-sense above threshold. 7 = 30/32 + carrier-sense above threshold.
    ELECHOUSE_cc1101.setSyncWord(211, 145); // Set sync word. Must be the same for the transmitter and receiver. Default is 211,145 (Syncword high, Syncword low)
    ELECHOUSE_cc1101.setAdrChk(0);          // Controls address check configuration of received packages. 0 = No address check. 1 = Address check, no broadcast. 2 = Address check and 0 (0x00) broadcast. 3 = Address check and 0 (0x00) and 255 (0xFF) broadcast.
    ELECHOUSE_cc1101.setAddr(0);            // Address used for packet filtration. Optional broadcast addresses are 0 (0x00) and 255 (0xFF).
    ELECHOUSE_cc1101.setWhiteData(0);       // Turn data whitening on / off. 0 = Whitening off. 1 = Whitening on.
    ELECHOUSE_cc1101.setPktFormat(0);       // Format of RX and TX data. 0 = Normal mode, use FIFOs for RX and TX. 1 = Synchronous serial mode, Data in on GDO0 and data out on either of the GDOx pins. 2 = Random TX mode; sends random data using PN9 generator. Used for test. Works as normal mode, setting 0 (00), in RX. 3 = Asynchronous serial mode, Data in on GDO0 and data out on either of the GDOx pins.
    ELECHOUSE_cc1101.setLengthConfig(1);    // 0 = Fixed packet length mode. 1 = Variable packet length mode. 2 = Infinite packet length mode. 3 = Reserved
    ELECHOUSE_cc1101.setPacketLength(0);    // Indicates the packet length when fixed packet length mode is enabled. If variable packet length mode is used, this value indicates the maximum packet length allowed.
    ELECHOUSE_cc1101.setCrc(0);             // 1 = CRC calculation in TX and CRC check in RX enabled. 0 = CRC disabled for TX and RX.
    ELECHOUSE_cc1101.setCRC_AF(0);          // Enable automatic flush of RX FIFO when CRC is not OK. This requires that only one packet is in the RXIFIFO and that packet length is limited to the RX FIFO size.
    ELECHOUSE_cc1101.setDcFilterOff(0);     // Disable digital DC blocking filter before demodulator. Only for data rates <= 250 kBaud The recommended IF frequency changes when the DC blocking is disabled. 1 = Disable (current optimized). 0 = Enable (better sensitivity).
    ELECHOUSE_cc1101.setManchester(0);      // Enables Manchester encoding/decoding. 0 = Disable. 1 = Enable.
    ELECHOUSE_cc1101.setFEC(0);             // Enable Forward Error Correction (FEC) with interleaving for packet payload (Only supported for fixed packet length mode. 0 = Disable. 1 = Enable.
    ELECHOUSE_cc1101.setPRE(0);             // Sets the minimum number of preamble bytes to be transmitted. Values: 0 : 2, 1 : 3, 2 : 4, 3 : 6, 4 : 8, 5 : 12, 6 : 16, 7 : 24
    ELECHOUSE_cc1101.setPQT(0);             // Preamble quality estimator threshold. The preamble quality estimator increases an internal counter by one each time a bit is received that is different from the previous bit, and decreases the counter by 8 each time a bit is received that is the same as the last bit. A threshold of 4*PQT for this counter is used to gate sync word detection. When PQT=0 a sync word is always accepted.
    ELECHOUSE_cc1101.setAppendStatus(0);    // When enabled, two status bytes will be appended to the payload of the packet. The status bytes contain RSSI and LQI values, as well as CRC OK.
    configureReceiveForModulation(2);       // sensitivity-first OOK AGC and continuous packet RX
    currentFreq = readRadioFrequencyMHz();
    currentRxBw = readRadioRxBwKHz();
}


// ---- shared helpers (extracted from repeated command code) ----

// zero the whole recording buffer and rewind its pointers
static void zeroRecordingBuffer(void)
{
    memset(bigrecordingbuffer, 0, RECORDINGBUFFERSIZE);
    bigrecordingbufferpos = 0;
    framesinbigrecordingbuffer = 0;
}

// Start a background mode with fresh counters used by both serial and web UI.
static void startActiveMode(Mode mode)
{
    activeMode = mode;
    modeStartedAt = millis();
    lastModeFeedbackAt = modeStartedAt;
    modeActivityCount = 0;
}

// Clamp a RAW per-sample interval to a watchdog-safe maximum, announcing when it clamps.
// Guarantees no RAW path ever hands delayMicroseconds() an unbounded value.
static int clampRawInterval(int usec, int maxUsec, const char *tag)
{
    if (usec > maxUsec) {
        out->print(F("\r\n["));
        out->print(tag);
        out->print(F("] interval clamped to "));
        out->print(maxUsec);
        out->print(F(" us (max)\r\n"));
        usec = maxUsec;
    }
    return usec;
}

static bool modeFeedbackDue(unsigned long intervalMs)
{
    unsigned long now = millis();
    if (now - lastModeFeedbackAt < intervalMs) return false;
    lastModeFeedbackAt = now;
    return true;
}

static unsigned long modeElapsedSeconds(void)
{
    return (activeMode == MODE_IDLE) ? 0 : (millis() - modeStartedAt) / 1000;
}

static int modeProgressPercent(void)
{
    if (activeMode == MODE_SCAN && scanTo > scanFrom) {
        float pct = ((scanCursor - scanFrom) * 100.0f) / (scanTo - scanFrom);
        if (pct < 0) pct = 0;
        if (pct > 100) pct = 100;
        return (int)pct;
    }
    if (activeMode == MODE_SNIFF)
        return (sniffCursor * 100) / RECORDINGBUFFERSIZE;
    if (activeMode == MODE_REC)
        return (bigrecordingbufferpos * 100) / RECORDINGBUFFERSIZE;
    if (activeMode == MODE_BRUTE && bruteMax)
        return (int)((bruteCode * 100UL) / bruteMax);
    return -1;
}

// put CC1101 into async (bit-banged GDO0) mode for RAW record/replay
static void enterRawMode(bool tx)
{
    enterRadioIdle();
    ELECHOUSE_cc1101.setCCMode(0);
    ELECHOUSE_cc1101.setPktFormat(3);
    if (tx) ELECHOUSE_cc1101.SetTx(); else ELECHOUSE_cc1101.SetRx();
}

// restore CC1101 to normal packet mode after RAW record/replay
static void exitRawMode(bool tx)
{
    // release GDO0: the TX raw modes (playraw/brute) drove it as an OUTPUT, but in
    // packet mode the CC1101 owns that line. Leaving it forced (brute ends it HIGH)
    // causes pin contention that hangs the next library call -> watchdog reset.
    pinMode(gdo0, INPUT);
    enterRadioIdle();
    ELECHOUSE_cc1101.setCCMode(1);
    ELECHOUSE_cc1101.setPktFormat(0);
    (void)tx;
}

// dump 'count' bytes of the recording buffer as hex, in HEXDUMP_CHUNK chunks
static void dumpBufferHex(int start, int count)
{
    for (int i = start; i < start + count; i += HEXDUMP_CHUNK)
       {
         asciitohex(&bigrecordingbuffer[i], textbuffer, HEXDUMP_CHUNK);
         out->print((char *)textbuffer);
         ESP.wdtFeed();
         yield();
       }
}

// parse a hex string (max MAX_HEXCHARS chars, even length) into out[],
// storing the resulting byte count in *outlen; returns false on bad input
static bool ingestHex(char *in, byte *out, int *outlen)
{
    if (!in) return false;
    int n = strlen(in);
    if ((n <= 0) || (n > MAX_HEXCHARS) || (n & 1)) return false;
    for (int i = 0; i < n; i++) {
        char c = in[i];
        bool hex = (c >= '0' && c <= '9') || (c >= 'A' && c <= 'F') ||
                   (c >= 'a' && c <= 'f');
        if (!hex) return false;
    }
    hextoascii(out, (byte *)in, n);
    *outlen = n / 2;
    return true;
}

// Configuration registers are only changed in IDLE. MARCSTATE 0x01 is IDLE.
static byte readRadioState(void)
{
    return ELECHOUSE_cc1101.SpiReadStatus(CC1101_MARCSTATE) & 0x1F;
}

static bool waitForRadioState(byte wanted, unsigned long timeoutMs)
{
    unsigned long started = millis();
    while (millis() - started < timeoutMs) {
        if (readRadioState() == wanted) return true;
        ESP.wdtFeed();
        delay(1);
    }
    return false;
}

static bool enterRadioIdle(void)
{
    ELECHOUSE_cc1101.setSidle();
    return waitForRadioState(0x01, 100);
}

static bool calibrateRadio(void)
{
    if (!enterRadioIdle()) return false;
    ELECHOUSE_cc1101.SpiStrobe(CC1101_SCAL);
    return waitForRadioState(0x01, 100);
}

static float readRadioFrequencyMHz(void)
{
    uint32_t word = ((uint32_t)ELECHOUSE_cc1101.SpiReadReg(CC1101_FREQ2) << 16) |
                    ((uint32_t)ELECHOUSE_cc1101.SpiReadReg(CC1101_FREQ1) << 8) |
                    ELECHOUSE_cc1101.SpiReadReg(CC1101_FREQ0);
    return (word * 26.0f) / 65536.0f;
}

static float readRadioRxBwKHz(void)
{
    byte mdmcfg4 = ELECHOUSE_cc1101.SpiReadReg(CC1101_MDMCFG4);
    byte e = (mdmcfg4 >> 6) & 0x03;
    byte m = (mdmcfg4 >> 4) & 0x03;
    return 26000.0f / (8.0f * (4.0f + m) * (1 << e));
}

static float nearestRxBwKHz(float requested)
{
    float best = 0;
    float bestError = 1000000.0f;
    for (byte e = 0; e < 4; e++) {
        for (byte m = 0; m < 4; m++) {
            float candidate = 26000.0f / (8.0f * (4.0f + m) * (1 << e));
            float error = fabsf(candidate - requested);
            if (error < bestError) {
                best = candidate;
                bestError = error;
            }
        }
    }
    return best;
}

static bool supportedPa(int pa)
{
    const int values[] = {-30, -20, -15, -10, -6, 0, 5, 7, 10, 11, 12};
    for (unsigned int i = 0; i < sizeof(values) / sizeof(values[0]); i++)
        if (pa == values[i]) return true;
    return false;
}

static void configureReceiveForModulation(int mod)
{
    if (mod == 2) {
        // Sensitivity-first OOK profile: restore all DVGA gain stages, retain
        // the slower OOK decision boundary, and stay in RX after a packet.
        ELECHOUSE_cc1101.SpiWriteReg(CC1101_AGCCTRL2, 0x03);
        ELECHOUSE_cc1101.SpiWriteReg(CC1101_AGCCTRL1, 0x40);
        ELECHOUSE_cc1101.SpiWriteReg(CC1101_AGCCTRL0, 0xB2);
        ELECHOUSE_cc1101.SpiWriteReg(CC1101_MCSM1, 0x3C);
    } else {
        ELECHOUSE_cc1101.SpiWriteReg(CC1101_AGCCTRL2, 0xC7);
        ELECHOUSE_cc1101.SpiWriteReg(CC1101_AGCCTRL1, 0x00);
        ELECHOUSE_cc1101.SpiWriteReg(CC1101_AGCCTRL0, 0xB2);
        ELECHOUSE_cc1101.SpiWriteReg(CC1101_MCSM1, 0x30);
    }
}

static bool supportedFrequency(float mhz)
{
    return radioBand(mhz) != 0;
}

static byte radioBand(float mhz)
{
    if (mhz >= 300.0f && mhz <= 348.0f) return 1;
    if (mhz >= 387.0f && mhz <= 464.0f) return 2;
    if (mhz >= 779.0f && mhz <= 928.0f) return 3;
    return 0;
}

static bool parseFloatArg(const char *text, float *value)
{
    if (!text || !text[0]) return false;
    char *end = NULL;
    double parsed = strtod(text, &end);
    if (end == text || *end != '\0' || !isfinite(parsed)) return false;
    *value = (float)parsed;
    return true;
}

static bool parseIntArg(const char *text, int *value)
{
    if (!text || !text[0]) return false;
    char *end = NULL;
    long parsed = strtol(text, &end, 10);
    if (end == text || *end != '\0' || parsed < -32768L || parsed > 32767L) return false;
    *value = (int)parsed;
    return true;
}

static bool validRadioProfile(float mhz, int mod, float drate, float dev, float rxbw, int pa)
{
    return supportedFrequency(mhz) && mod >= 0 && mod <= 4 &&
           drate >= 0.0247955f && drate <= 1621.83f &&
           dev >= 1.586914f && dev <= 380.859375f &&
           rxbw >= 58.0357f && rxbw <= 812.5f && supportedPa(pa);
}

static bool applyRadioProfile(float mhz, int mod, float drate, float dev, float rxbw, int pa)
{
    if (!validRadioProfile(mhz, mod, drate, dev, rxbw, pa)) return false;

    bool resumeRx = (activeMode == MODE_RX || activeMode == MODE_REC);
    if (!resumeRx && activeMode != MODE_IDLE) stopActiveMode();
    if (!enterRadioIdle()) return false;

    float appliedBw = nearestRxBwKHz(rxbw);
    ELECHOUSE_cc1101.setMHZ(mhz);
    ELECHOUSE_cc1101.setModulation(mod);
    ELECHOUSE_cc1101.setRxBW(appliedBw);
    ELECHOUSE_cc1101.setDRate(drate);
    ELECHOUSE_cc1101.setDeviation(dev);
    ELECHOUSE_cc1101.setPA(pa);
    configureReceiveForModulation(mod);

    // The library's Calibrate() helper does not issue SCAL, so do it explicitly.
    if (!calibrateRadio()) return false;
    currentFreq = readRadioFrequencyMHz();
    currentRxBw = readRadioRxBwKHz();
    if (resumeRx) ELECHOUSE_cc1101.SetRx();
    return true;
}


// Execute a complete CC1101 command.

static void exec(char *cmdline)
{ 
        
    char *command = strsep(&cmdline, " ");
    int setting, setting2, len;
    float settingf1;

    // A bit-bang RAW mode (sniff/brute) leaves the radio in async mode. Any new
    // command means the user wants something else, so cleanly stop that mode first
    // (restoring packet mode) - otherwise packet RX/TX would be silently broken.
    if (activeMode == MODE_SNIFF || activeMode == MODE_BRUTE ||
        activeMode == MODE_RECRAW || activeMode == MODE_PLAYRAW) stopActiveMode();

    // Every legacy one-register "set..." command is also guarded. The atomic
    // applyradio command below manages its own state and calibration sequence.
    bool guardedConfig = strncmp(command, "set", 3) == 0;
    bool resumeRxAfterConfig = guardedConfig && (activeMode == MODE_RX || activeMode == MODE_REC);
    if (guardedConfig) {
        if (!resumeRxAfterConfig && activeMode != MODE_IDLE) stopActiveMode();
        if (!enterRadioIdle()) {
            out->print(F("[ERROR] radio did not reach IDLE; no setting written\r\n"));
            return;
        }
    }

  // identification of the command & actions
      
    if (strcmp_P(command, PSTR("help")) == 0) {
        out->println(F(
          "\r\n+------------------------------------------------------+\r\n"
          "| CYPHER CC1101 FIELD CONSOLE - COMMAND MAP           |\r\n"
          "+------------------------------------------------------+\r\n"
          "\r\n  LIVE RADIO\r\n"
          "  rx                         packet monitor on/off\r\n"
          "  rec                        record packets on/off\r\n"
          "  scan <from> <to>           sweep a MHz range\r\n"
          "  rxraw <usec>               stream raw blocks (usec/sample, max 1000)\r\n"
          "  recraw <usec>              arm 4096-byte capture (usec/sample, max 150)\r\n"
          "  playraw <usec>             replay raw buffer (usec/sample, max 150)\r\n"
          "  brute <usec> <bits>        test raw bit codes (usec/symbol, max 1000)\r\n"
          "  jam                        random TX mode on/off\r\n"
          "  chat                       serial-over-RF chat mode\r\n"
          "  x                          stop active mode\r\n"
         ));
          yield();       
        out->println(F(
          "  PACKETS + BUFFER\r\n"
          "  tx <hex>                   transmit up to 60 bytes\r\n"
          "  add <hex>                  add one packet frame\r\n"
          "  addraw <hex>               add bytes to raw buffer\r\n"
          "  show | showraw | showbit   inspect buffer\r\n"
          "  play <0|frame>             replay all or one frame\r\n"
          "  save | load | flush        manage buffer storage\r\n"
         ));
          yield();
        out->println(F(
          "  RADIO SETUP\r\n"
          "  setmhz <MHz>               base frequency\r\n"
          "  setmodulation <0..4>        2-FSK/GFSK/ASK/4-FSK/MSK\r\n"
          "  setdrate <kBaud>            data rate\r\n"
          "  setdeviation <kHz>          frequency deviation\r\n"
          "  setrxbw <kHz>               receive bandwidth\r\n"
          "  setpa <dBm>                 transmit power\r\n"
          "  applyradio <MHz> <mod> <rate> <dev> <bw> <dBm>\r\n"
          "  setchannel <0..255>         channel number\r\n"
          "  setchsp <kHz>               channel spacing\r\n"
          "  setsyncmode <0..7>          sync qualifier\r\n"
          "  setsyncword <low> <high>    sync bytes\r\n"
         ));
          yield();
         out->println(F(
          "  ADVANCED SETUP\r\n"
          "  setadrchk setaddr setwhitedata setpktformat\r\n"
          "  setlengthconfig setpacketlength setcrc setcrcaf\r\n"
          "  setdcfilteroff setmanchester setfec setpre setpqt\r\n"
          "  setappendstatus\r\n"
            ));
          yield();
        out->println(F(
          "  CONSOLE\r\n"
          "  status                     live system dashboard\r\n"
          "  getrssi                    current RSSI + LQI\r\n"
          "  init                       reset radio defaults\r\n"
          "  echo <0|1>                 terminal input echo\r\n"
          "  help                       show this command map\r\n"
          "\r\n  Tip: active modes print live telemetry. Send x to stop.\r\n"
          "+------------------------------------------------------+\r\n"
            ));
          yield();

    // Handling STATUS command - compact terminal dashboard
    } else if (strcmp_P(command, PSTR("status")) == 0) {
        out->print(F("\r\n+---------------- CC1101 STATUS ----------------+\r\n"));
        out->print(F("  mode       ")); out->println(modeName());
        currentFreq = readRadioFrequencyMHz();
        currentRxBw = readRadioRxBwKHz();
        out->print(F("  frequency  ")); out->print(currentFreq, 4); out->println(F(" MHz (register readback)"));
        out->print(F("  rx bw      ")); out->print(currentRxBw, 4); out->println(F(" kHz"));
        out->print(F("  marcstate  0x")); out->println(readRadioState(), HEX);
        out->print(F("  activity   ")); out->print(modeActivityCount);
        out->print(F(" | elapsed ")); out->print(modeElapsedSeconds()); out->println(F("s"));
        int progress = modeProgressPercent();
        if (progress >= 0) {
            out->print(F("  progress   ")); out->print(progress); out->println(F("%"));
        }
        out->print(F("  buffer     ")); out->print(bigrecordingbufferpos);
        out->print(F("/")); out->print(RECORDINGBUFFERSIZE);
        out->print(F(" bytes | ")); out->print(framesinbigrecordingbuffer); out->println(F(" frames"));
        out->print(F("  web        http://")); out->print(WiFi.softAPIP());
        out->print(F(" | clients ")); out->println(WiFi.softAPgetStationNum());
        out->print(F("  uptime     ")); out->print(millis() / 1000);
        out->print(F("s | free heap ")); out->print(ESP.getFreeHeap()); out->println(F(" bytes"));
        out->print(F("+------------------------------------------------+\r\n"));

    // Apply a complete validated profile while the synthesizer is safely idle.
    } else if (strcmp_P(command, PSTR("applyradio")) == 0) {
        char *args[6];
        bool complete = true;
        for (byte i = 0; i < 6; i++) {
            args[i] = strsep(&cmdline, " ");
            if (!args[i] || !args[i][0]) complete = false;
        }
        if (!complete) {
            out->print(F("[ERROR] applyradio needs MHz mod rate dev bw dBm\r\n"));
        } else {
            float mhz, drate, dev, rxbw;
            int mod, pa;
            bool parsed = parseFloatArg(args[0], &mhz) && parseIntArg(args[1], &mod) &&
                          parseFloatArg(args[2], &drate) && parseFloatArg(args[3], &dev) &&
                          parseFloatArg(args[4], &rxbw) && parseIntArg(args[5], &pa);
            if (!parsed || !applyRadioProfile(mhz, mod, drate, dev, rxbw, pa)) {
                out->print(F("[ERROR] profile rejected or radio calibration failed; no cached status updated\r\n"));
            } else {
                out->print(F("[OK] applied and calibrated | "));
                out->print(currentFreq, 4);
                out->print(F(" MHz | RXBW "));
                out->print(currentRxBw, 4);
                out->print(F(" kHz | MARCSTATE 0x"));
                out->println(readRadioState(), HEX);
            }
        }

    // Handling SETMODULATION command 
    } else if (strcmp_P(command, PSTR("setmodulation")) == 0) {
        setting = atoi(cmdline);
        if (setting >= 0 && setting <= 4) {
            ELECHOUSE_cc1101.setModulation(setting);
            configureReceiveForModulation(setting);
        }
        else out->print(F("[ERROR] modulation must be 0..4\r\n"));
        out->print(F("\r\nModulation: "));
        if (setting == 0) { out->print(F("2-FSK")); }
        else if (setting == 1) { out->print(F("GFSK")); }
        else if (setting == 2) { out->print(F("ASK/OOK")); }
        else if (setting == 3) { out->print(F("4-FSK")); }
        else if (setting == 4) { out->print(F("MSK")); };  
        out->print(F(" \r\n"));
        yield();

    // Handling SETMHZ command 
    } else if (strcmp_P(command, PSTR("setmhz")) == 0) {
        settingf1 = atof(cmdline);
        if (supportedFrequency(settingf1)) ELECHOUSE_cc1101.setMHZ(settingf1);
        else out->print(F("[ERROR] MHz must be in 300-348, 387-464, or 779-928\r\n"));
        out->print(F("\r\nFrequency: "));
        out->print(settingf1);
        out->print(F(" MHz\r\n"));
        yield();
        
    // Handling SETDEVIATION command 
    } else if (strcmp_P(command, PSTR("setdeviation")) == 0) {
        settingf1 = atof(cmdline);
        if (settingf1 >= 1.586914f && settingf1 <= 380.859375f)
            ELECHOUSE_cc1101.setDeviation(settingf1);
        else out->print(F("[ERROR] deviation must be 1.586914..380.859375 kHz\r\n"));
        out->print(F("\r\nDeviation: "));
        out->print(settingf1);
        out->print(F(" KHz\r\n"));        
        yield();

    // Handling SETCHANNEL command       
    } else if (strcmp_P(command, PSTR("setchannel")) == 0) {
        setting = atoi(cmdline);
        ELECHOUSE_cc1101.setChannel(setting);
        out->print(F("\r\nChannel:"));
        out->print(setting);
        out->print(F("\r\n"));        
        yield();

    // Handling SETCHSP command 
    } else if (strcmp_P(command, PSTR("setchsp")) == 0) {
        settingf1 = atof(cmdline);
        ELECHOUSE_cc1101.setChsp(settingf1);
        out->print(F("\r\nChann spacing: "));
        out->print(settingf1);
        out->print(F(" kHz\r\n"));  
        yield();

    // Handling SETRXBW command         
    } else if (strcmp_P(command, PSTR("setrxbw")) == 0) {
        settingf1 = atof(cmdline);
        if (settingf1 >= 58.0357f && settingf1 <= 812.5f) {
            settingf1 = nearestRxBwKHz(settingf1);
            ELECHOUSE_cc1101.setRxBW(settingf1);
        } else out->print(F("[ERROR] RX bandwidth must be 58.0357..812.5 kHz\r\n"));
        out->print(F("\r\nRX bandwidth: "));
        out->print(settingf1);
        out->print(F(" kHz \r\n"));  
        yield();

    // Handling SETDRATE command         
    } else if (strcmp_P(command, PSTR("setdrate")) == 0) {
        settingf1 = atof(cmdline);
        if (settingf1 >= 0.0247955f && settingf1 <= 1621.83f)
            ELECHOUSE_cc1101.setDRate(settingf1);
        else out->print(F("[ERROR] data rate must be 0.0247955..1621.83 kBaud\r\n"));
        out->print(F("\r\nDatarate: "));
        out->print(settingf1);
        out->print(F(" kbaud\r\n"));  
        yield();

    // Handling SETPA command         
    } else if (strcmp_P(command, PSTR("setpa")) == 0) {
        setting = atoi(cmdline);
        if (supportedPa(setting)) ELECHOUSE_cc1101.setPA(setting);
        else out->print(F("[ERROR] unsupported dBm value\r\n"));
        out->print(F("\r\nTX PWR: "));
        out->print(setting);
        out->print(F(" dBm\r\n"));  
        yield();
        
    // Handling SETSYNCMODE command         
    } else if (strcmp_P(command, PSTR("setsyncmode")) == 0) {
        int setting = atoi(cmdline);
        ELECHOUSE_cc1101.setSyncMode(setting);
        out->print(F("\r\nSynchronization: "));
        if (setting == 0) { out->print(F("No preamble")); }
        else if (setting == 1) { out->print(F("16 sync bits")); }
        else if (setting == 2) { out->print(F("16/16 sync bits")); }
        else if (setting == 3) { out->print(F("30/32 sync bits")); }
        else if (setting == 4) { out->print(F("No preamble/sync, carrier-sense")); }
        else if (setting == 5) { out->print(F("15/16 + carrier-sense")); }
        else if (setting == 6) { out->print(F("16/16 + carrier-sense")); }
        else if (setting == 7) { out->print(F("30/32 + carrier-sense")); };
        out->print(F("\r\n"));  
        yield();
        
    // Handling SETSYNCWORD command         
    } else if (strcmp_P(command, PSTR("setsyncword")) == 0) {
        setting = atoi(strsep(&cmdline, " "));
        setting2 = atoi(cmdline);
        // args are entered LOW then HIGH; setSyncWord takes (high, low)
        ELECHOUSE_cc1101.setSyncWord(setting2, setting);
        out->print(F("\r\nSynchronization:\r\n"));
        out->print(F("high = "));
        out->print(setting2);
        out->print(F("\r\nlow = "));
        out->print(setting);
        out->print(F("\r\n"));
        yield();
    
    // Handling SETADRCHK command         
    } else if (strcmp_P(command, PSTR("setadrchk")) == 0) {
        setting = atoi(cmdline);
        ELECHOUSE_cc1101.setAdrChk(setting);
        out->print(F("\r\nAddress checking:"));
        if (setting == 0) { out->print(F("No adr chk")); }
        else if (setting == 1) { out->print(F("Adr chk, no bcast")); }
        else if (setting == 2) { out->print(F("Adr chk and 0 bcast")); }
        else if (setting == 3) { out->print(F("Adr chk and 0 and FF bcast")); };
        out->print(F("\r\n"));  
        yield();
        
    // Handling SETADDR command         
    } else if (strcmp_P(command, PSTR("setaddr")) == 0) {
        setting = atoi(cmdline);
        ELECHOUSE_cc1101.setAddr(setting);
        out->print(F("\r\nAddress: "));
        out->print(setting);
        out->print(F("\r\n"));  
        yield();

    // Handling SETWHITEDATA command         
    } else if (strcmp_P(command, PSTR("setwhitedata")) == 0) {
        setting = atoi(cmdline);
        ELECHOUSE_cc1101.setWhiteData(setting);
        out->print(F("\r\nWhitening "));
        if (setting == 0) { out->print(F("OFF")); }
        else if (setting == 1) { out->print(F("ON")); }
        out->print(F("\r\n"));  
        yield();
        
    // Handling SETPKTFORMAT command         
    } else if (strcmp_P(command, PSTR("setpktformat")) == 0) {
        setting = atoi(cmdline);
        ELECHOUSE_cc1101.setPktFormat(setting);
        out->print(F("\r\nPacket format: "));
        if (setting == 0) { out->print(F("Normal mode")); }
        else if (setting == 1) { out->print(F("Synchronous serial mode")); }
        else if (setting == 2) { out->print(F("Random TX mode")); }
        else if (setting == 3) { out->print(F("Asynchronous serial mode")); };
        out->print(F("\r\n"));  
        yield();
  
    // Handling SETLENGTHCONFIG command         
    } else if (strcmp_P(command, PSTR("setlengthconfig")) == 0) {
        setting = atoi(cmdline);
        ELECHOUSE_cc1101.setLengthConfig(setting);
        out->print(F("\r\nPkt length mode: "));
        if (setting == 0) { out->print(F("Fixed")); }
        else if (setting == 1) { out->print(F("Variable")); }
        else if (setting == 2) { out->print(F("Infinite")); }
        else if (setting == 3) { out->print(F("Reserved")); };
        out->print(F("\r\n"));  
  
    // Handling SETPACKETLENGTH command         
    } else if (strcmp_P(command, PSTR("setpacketlength")) == 0) {
        setting = atoi(cmdline);
        ELECHOUSE_cc1101.setPacketLength(setting);
        out->print(F("\r\nPkt length: "));
        out->print(setting);
        out->print(F(" bytes\r\n"));  
        yield();
        
    // Handling SETCRC command         
    } else if (strcmp_P(command, PSTR("setcrc")) == 0) {
        setting = atoi(cmdline);
        ELECHOUSE_cc1101.setCrc(setting);
        out->print(F("\r\nCRC checking: "));
        if (setting == 0) { out->print(F("Disabled")); }
        else if (setting == 1) { out->print(F("Enabled")); };
        out->print(F("\r\n")); 
        yield();
        
    // Handling SETCRCAF command         
    } else if (strcmp_P(command, PSTR("setcrcaf")) == 0) {
        setting = atoi(cmdline);
        ELECHOUSE_cc1101.setCRC_AF(setting);
        out->print(F("\r\nCRC Autoflush: "));
        if (setting == 0) { out->print(F("Disabled")); }
        else if (setting == 1) { out->print(F("Enabled")); };
         out->print(F("\r\n")); 
        
    // Handling SETDCFILTEROFF command         
     } else if (strcmp_P(command, PSTR("setdcfilteroff")) == 0) {
        setting = atoi(cmdline);
        ELECHOUSE_cc1101.setDcFilterOff(setting);
        out->print(F("\r\nDC filter: "));
        if (setting == 0) { out->print(F("Enabled")); }
        else if (setting == 1) { out->print(F("Disabled")); };
        out->print(F("\r\n")); 
        yield();

    // Handling SETMANCHESTER command         
     } else if (strcmp_P(command, PSTR("setmanchester")) == 0) {
        setting = atoi(cmdline);
        ELECHOUSE_cc1101.setManchester(setting);
        out->print(F("\r\nManchester coding: "));
        if (setting == 0) { out->print(F("Disabled")); }
        else if (setting == 1) { out->print(F("Enabled")); };
        out->print(F("\r\n")); 
        yield();

    // Handling SETFEC command         
     } else if (strcmp_P(command, PSTR("setfec")) == 0) {
        setting = atoi(cmdline);
        ELECHOUSE_cc1101.setFEC(setting);
        out->print(F("\r\nForward Error Correction: "));
        if (setting == 0) { out->print(F("Disabled")); }
        else if (setting == 1) { out->print(F("Enabled")); };
        out->print(F("\r\n")); 
        yield();
        
    // Handling SETPRE command         
     } else if (strcmp_P(command, PSTR("setpre")) == 0) {
        setting = atoi(cmdline);
        ELECHOUSE_cc1101.setPRE(setting);
        out->print(F("\r\nMinimum preamble bytes:"));
        out->print(setting);
        out->print(F(" means 0 = 2 bytes, 1 = 3b, 2 = 4b, 3 = 6b, 4 = 8b, 5 = 12b, 6 = 16b, 7 = 24 bytes\r\n")); 
        out->print(F("\r\n")); 
        yield();

  
    // Handling SETPQT command         
      } else if (strcmp_P(command, PSTR("setpqt")) == 0) {
        setting = atoi(cmdline);
        ELECHOUSE_cc1101.setPQT(setting);
        out->print(F("\r\nPQT: "));
        out->print(setting);
        out->print(F("\r\n")); 
        yield();

    // Handling SETAPPENDSTATUS command         
       } else if (strcmp_P(command, PSTR("setappendstatus")) == 0) {
        setting = atoi(cmdline);
        ELECHOUSE_cc1101.setAppendStatus(setting);
        out->print(F("\r\nStatus bytes appending: "));
        if (setting == 0) { out->print(F("Enabled")); }
        else if (setting == 1) { out->print(F("Disabled")); };
        out->print(F("\r\n")); 
        yield();

    // Handling GETRSSI command         
      } else if (strcmp_P(command, PSTR("getrssi")) == 0) {
        //Rssi Level in dBm
        out->print(F("Rssi: "));
        out->println(ELECHOUSE_cc1101.getRssi());
        //Link Quality Indicator
        out->print(F(" LQI: "));
        out->println(ELECHOUSE_cc1101.getLqi());        
        out->print(F("\r\n")); 
        yield();


    // Handling SCAN command - frequency scanner by Little S@tan !
    } else if (strcmp_P(command, PSTR("scan")) == 0) {
        char *scanStart = strsep(&cmdline, " ");
        if (!parseFloatArg(scanStart, &scanFrom) || !parseFloatArg(cmdline, &scanTo)) {
            out->print(F("[ERROR] scan needs numeric start and stop MHz\r\n"));
            return;
        }
        byte fromBand = radioBand(scanFrom);
        byte toBand = radioBand(scanTo);
        if (!fromBand || fromBand != toBand || scanFrom >= scanTo) {
            out->print(F("[ERROR] scan must increase within one supported RF band\r\n"));
            return;
        }
        out->print(F("\r\n[SCAN] started | "));
        out->print(scanFrom);
        out->print(F(" -> "));
        out->print(scanTo);
        out->print(F(" MHz | step 0.01 MHz | stop with x\r\n"));
        ELECHOUSE_cc1101.Init();
        ELECHOUSE_cc1101.setRxBW(58);
        ELECHOUSE_cc1101.SetRx();
        scanCursor = scanFrom;
        scanBestFreq = 0; scanBestRssi = RSSI_NONE; scanCompare = 0;
        startActiveMode(MODE_SCAN);


    // handling SAVE command
    } else if (strcmp_P(command, PSTR("save")) == 0) {
        //start saving recording buffer content into EEPROM non-volatile memory 
        out->print(F("\r\nSaving recording buffer content into the non-volatile memory...\r\n"));
        
        for (setting=0; setting<EPROMSIZE ; setting++)  
           {  // copying byte after byte from SRAM to EEPROM
            EEPROM.write(setting, bigrecordingbuffer[setting] );
           };
        // commit the writes (flash-simulated EEPROM needs this)
        EEPROM.commit();
        // print confirmation
        out->print(F("\r\nSaving complete.\r\n\r\n"));
        yield();
        
                 
    // handling LOAD command
    } else if (strcmp_P(command, PSTR("load")) == 0) {
        // flush the recording buffer and rewind its pointers first
        zeroRecordingBuffer();
        //start loading EEPROM non-volatile memory content into recording buffer
        out->print(F("\r\nLoading content from the non-volatile memory into the recording buffer...\r\n"));
        
        for (setting=0; setting<EPROMSIZE ; setting++)
           { // copying byte after byte from EEPROM to SRAM
            bigrecordingbuffer[setting] = EEPROM.read(setting);
           }
        // reconstruct the frame count by walking the length-prefixed frames,
        // otherwise 'show'/'play' would see 0 frames and do nothing.
        // (RAW captures have no framing - use 'showraw'/'playraw' for those.)
        framesinbigrecordingbuffer = 0;
        for (int p = 0; p < EPROMSIZE; )
           {
            int flen = bigrecordingbuffer[p];
            if ((flen <= 0) || (flen > 60) || (p + 1 + flen > EPROMSIZE)) break;
            framesinbigrecordingbuffer++;
            p += 1 + flen;
           }
        out->print(F("\r\nLoading complete. Enter 'show' or 'showraw' to see the buffer content.\r\n\r\n"));
        yield();
                  


    // Handling RX command         
       } else if (strcmp_P(command, PSTR("rx")) == 0) {
        if (activeMode == MODE_RX) {
            stopActiveMode();
        } else {
            ELECHOUSE_cc1101.SetRx();
            startActiveMode(MODE_RX);
            out->print(F("\r\n[RX] listening for packets | stop with x or rx\r\n"));
        }
        yield();


    // Handling CHAT command
       } else if (strcmp_P(command, PSTR("chat")) == 0) {
        out->print(F("\r\n[CHAT] live link active | serial text will transmit over RF\r\n"));
        startActiveMode(MODE_CHAT);
        yield();


    // Handling JAM command
       } else if (strcmp_P(command, PSTR("jam")) == 0) {
        if (activeMode == MODE_JAM) {
            stopActiveMode();
        } else {
            startActiveMode(MODE_JAM);
            out->print(F("\r\n[JAM] active | random 60-byte frames | stop with x or jam\r\n"));
        }
        yield();
    
    // handling BRUTE command
    } else if (strcmp_P(command, PSTR("brute")) == 0) {
        bruteInterval = clampRawInterval(atoi(strsep(&cmdline, " ")), RAW_CHUNK_MAX_INTERVAL, "BRUTE");
        bruteBits     = atoi(cmdline);
        if (bruteBits > 16) bruteBits = 16;
        bruteMax = (bruteBits > 0) ? (1UL << bruteBits) : 0;
        if ((bruteInterval > 0) && (bruteBits > 0)) {
            enterRawMode(true);
            out->print(F("\r\n[BRUTE] started | symbol "));
            out->print(bruteInterval);
            out->print(F(" us | bits "));
            out->print(bruteBits);
            out->print(F(" | codes "));
            out->print(bruteMax);
            out->print(F(" | stop with x\r\n"));
            pinMode(gdo0, OUTPUT);
            bruteCode = 0;
            startActiveMode(MODE_BRUTE);
        }
        else { out->print(F("Wrong parameters.\r\n")); }

    // Handling TX command
       } else if (strcmp_P(command, PSTR("tx")) == 0) {
        // convert hex array to set of bytes
        if ( ingestHex(cmdline, ccsendingbuffer, &len) )
        {
                ccsendingbuffer[len] = 0x00;
                out->print(F("\r\nTransmitting RF packets.\r\n"));
                // send these data to radio over CC1101
                ELECHOUSE_cc1101.SendData(ccsendingbuffer, (byte)len);
                // echo back the frame that was sent
                asciitohex(ccsendingbuffer, textbuffer, len);
                out->print(F("Sent frame: "));
                out->print((char *)textbuffer);
                out->print(F("\r\n")); }
         else { out->print(F("Wrong parameters.\r\n")); };
        yield();



    // handling RECRAW command
    } else if (strcmp_P(command, PSTR("recraw")) == 0) {
        // Non-blocking RAW capture: arm and return immediately. serviceActiveMode()
        // waits for the signal (one poll per loop pass) and then does the single
        // timing-coherent capture - off the /cmd path, so the web never hangs.
        recrawInterval = clampRawInterval(atoi(cmdline), RAW_MAX_INTERVAL, "RECRAW");
        if (recrawInterval > 0) {
            zeroRecordingBuffer();
            enterRawMode(false);
            pinMode(gdo0, INPUT);
            out->print(F("\r\n[RECRAW] armed | waiting for GDO0 HIGH | "));
            out->print(recrawInterval);
            out->print(F(" us/sample | ~"));
            out->print((float)recrawInterval * RECORDINGBUFFERSIZE * 8 / 1000000.0f, 1);
            out->print(F(" s window | 4096 bytes | stop with x\r\n"));
            recrawWaiting = true;
            startActiveMode(MODE_RECRAW);
        }
        else { out->print(F("Wrong parameters.\r\n")); }

   // handling RXRAW command - sniffer
    } else if (strcmp_P(command, PSTR("rxraw")) == 0) {
        sniffInterval = clampRawInterval(atoi(cmdline), RAW_CHUNK_MAX_INTERVAL, "RXRAW");
        if (sniffInterval > 0) {
            zeroRecordingBuffer();
            enterRawMode(false);
            out->print(F("\r\n[RXRAW] streaming | sample "));
            out->print(sniffInterval);
            out->print(F(" us | block 128 bytes | stop with x\r\n"));
            pinMode(gdo0, INPUT);
            sniffCursor = 0;
            startActiveMode(MODE_SNIFF);
        }
        else { out->print(F("Wrong parameters.\r\n")); }


    // handling PLAYRAW command
    } else if (strcmp_P(command, PSTR("playraw")) == 0) {
        // Non-blocking RAW replay: arm and return immediately. serviceActiveMode()
        // does the single timing-coherent replay off the /cmd path.
        playrawInterval = clampRawInterval(atoi(cmdline), RAW_MAX_INTERVAL, "PLAYRAW");
        if (playrawInterval > 0) {
            enterRawMode(true);
            pinMode(gdo0, OUTPUT);
            out->print(F("\r\n[PLAYRAW] started | 4096 bytes | "));
            out->print(playrawInterval);
            out->print(F(" us/sample | ~"));
            out->print((float)playrawInterval * RECORDINGBUFFERSIZE * 8 / 1000000.0f, 1);
            out->print(F(" s\r\n"));
            startActiveMode(MODE_PLAYRAW);
        }
        else { out->print(F("Wrong parameters.\r\n")); }

    // handling SHOWRAW command
    } else if (strcmp_P(command, PSTR("showraw")) == 0) {
    // show the content of recorded RAW signal as hex numbers
       out->print(F("\r\nRecorded RAW data:\r\n"));
       dumpBufferHex(0, RECORDINGBUFFERSIZE);
       out->print(F("\r\n\r\n"));
       // feed the watchdog
       ESP.wdtFeed();
       // needed for ESP8266
       yield();
      


    // handling SHOWBIT command
    } else if (strcmp_P(command, PSTR("showbit")) == 0) {
    // show the content of recorded RAW signal as hex numbers
       out->print(F("\r\nRecorded RAW data as bit stream:\r\n"));
       for (int i = 0; i < RECORDINGBUFFERSIZE ; i = i + 32)  
           {        // first convert to hex numbers
                    asciitohex((byte *)&bigrecordingbuffer[i], (byte *)textbuffer,  32);
                    // now decode as binary and print
                    for (setting = 0; setting < 64 ; setting++)
                        {
                        setting2 = textbuffer[setting];
                        switch( setting2 )
                              {
                              case '0':
                              out->print(F("____"));
                              break;
   
                              case '1':
                              out->print(F("___-"));
                              break;
   
                              case '2':
                              out->print(F("__-_"));
                              break;

                              case '3':
                              out->print(F("__--"));
                              break;

                              case '4':
                              out->print(F("_-__"));
                              break;

                              case '5':
                              out->print(F("_-_-"));
                              break;

                              case '6':
                              out->print(F("_--_"));
                              break;

                              case '7':
                              out->print(F("_---"));
                              break;

                              case '8':
                              out->print(F("-___"));
                              break;

                              case '9':
                              out->print(F("-__-"));
                              break;

                              case 'A':
                              out->print(F("-_-_"));
                              break;

                              case 'B':
                              out->print(F("-_--"));
                              break;

                              case 'C':
                              out->print(F("--__"));
                              break;

                              case 'D':
                              out->print(F("--_-"));
                              break;

                              case 'E':
                              out->print(F("---_"));
                              break;

                              case 'F':
                              out->print(F("----"));
                              break;
                              
                              }; // end of switch
                              
                        }; // end of for
              // feed the watchdog
              ESP.wdtFeed();
              // needed for ESP8266   
              yield();      
   
              } // end of for
              out->print(F("\r\n\r\n"));


    // Handling ADDRAW command         
       } else if (strcmp_P(command, PSTR("addraw")) == 0) {
        // getting hex numbers - the content of the  frame
        // convert hex array to set of bytes
        if ( ingestHex(cmdline, textbuffer, &len) )
        {
                // check if the frame fits into the buffer and store it
                if (( bigrecordingbufferpos + len) < RECORDINGBUFFERSIZE)
                     { // copy current frame and increase pointer for next frames
                      memcpy(&bigrecordingbuffer[bigrecordingbufferpos], &textbuffer, len );
                      // increase position in big recording buffer for next frame
                      bigrecordingbufferpos = bigrecordingbufferpos + len; 
                      out->print(F("\r\nChunk added to recording buffer\r\n\r\n"));
                    }   
               else                  
                   {   
                     out->print(F("\r\nBuffer is full. The frame does not fit.\r\n "));
                   };
        }  
        else { out->print(F("Wrong parameters.\r\n")); };
        // needed for ESP8266   
        yield();      


        
    // Handling REC command         
    } else if (strcmp_P(command, PSTR("rec")) == 0) {
        if (activeMode == MODE_REC) {
            stopActiveMode();
            bigrecordingbufferpos = 0;
        } else {
            ELECHOUSE_cc1101.SetRx();
            zeroRecordingBuffer();
            startActiveMode(MODE_REC);
            out->print(F("\r\n[REC] packet recording active | buffer 0/4096 bytes | stop with x or rec\r\n"));
        }
        yield();
 

    // Handling PLAY command         
       } else if (strcmp_P(command, PSTR("play")) == 0) {
        setting = atoi(strsep(&cmdline, " "));
        // if number of played frames is 0 it means play all frames
        if ((setting >= 0) && (setting <= framesinbigrecordingbuffer))
        {
          out->print(F("\r\nReplaying recorded frames.\r\n "));
          // rewind recording buffer position to the beginning
          bigrecordingbufferpos = 0;
          if (framesinbigrecordingbuffer >0)
          {

            // start reading and sending frames from the buffer : FIFO
            for (int i=1; i<=framesinbigrecordingbuffer ; i++)  
               { 
                 // read length of the recorded frame first from the buffer
                 len = bigrecordingbuffer[bigrecordingbufferpos];
                 if ( ((len<=60) and (len>0)) and ((i == setting) or (setting == 0))  )
                 { 
                    // take next frame from the buffer  for replay
                    memcpy(ccsendingbuffer, &bigrecordingbuffer[bigrecordingbufferpos + 1], len );      
                    // send these data to radio over CC1101
                    ELECHOUSE_cc1101.SendData(ccsendingbuffer, (byte)len);
                 };
                  // increase position to the buffer and check exception
                  bigrecordingbufferpos = bigrecordingbufferpos + 1 + len;
                  if ( bigrecordingbufferpos > RECORDINGBUFFERSIZE) break;
                  // feed the watchdog per frame (a big replay must not starve it)
                  ESP.wdtFeed();
                  // needed for ESP8266
                  yield();
               };

             }; // end of IF framesinrecordingbuffer
        
          // rewind buffer position
          bigrecordingbufferpos = 0;
          out->print(F("Done.\r\n"));       
        }
         else { out->print(F("Wrong parameters.\r\n")); };


    // Handling ADD command         
       } else if (strcmp_P(command, PSTR("add")) == 0) {
        // getting hex numbers - the content of the  frame
        // convert hex array to set of bytes
        if ( ingestHex(cmdline, textbuffer, &len) )
        {
                // check if the frame fits into the buffer and store it
                if (( bigrecordingbufferpos + len + 1) < RECORDINGBUFFERSIZE)
                     { // put info about number of bytes
                      bigrecordingbuffer[bigrecordingbufferpos] = len; 
                      bigrecordingbufferpos++;
                      // next - copy current frame and increase 
                      memcpy(&bigrecordingbuffer[bigrecordingbufferpos], &textbuffer, len );
                      // increase position in big recording buffer for next frame
                      bigrecordingbufferpos = bigrecordingbufferpos + len; 
                      // increase counter of frames stored
                      framesinbigrecordingbuffer++;
                      out->print(F("\r\nAdded frame number "));
                      out->print(framesinbigrecordingbuffer);
                      out->print(F("\r\n"));                  
                    }   
               else                  
                   {   
                     out->print(F("\r\nBuffer is full. The frame does not fit.\r\n "));
                   };
        }  
        else { out->print(F("Wrong parameters.\r\n")); };
        // needed for ESP8266   
        yield();      
       

    // Handling SHOW command         
       } else if (strcmp_P(command, PSTR("show")) == 0) {
         if (framesinbigrecordingbuffer>0)
        {
          out->print(F("\r\nFrames stored in the recording buffer:\r\n "));
          // rewind recording buffer position to the beginning
          bigrecordingbufferpos = 0;
          // start reading and sending frames from the buffer : FIFO
          for (setting=1; setting<=framesinbigrecordingbuffer; setting++)  
               { 
                 // read length of the recorded frame first from the buffer
                 len = bigrecordingbuffer[bigrecordingbufferpos];
                 if ((len<=60) and (len>0))
                 { 
                    // take next frame from the buffer  for replay
                    // flush textbuffer
                    for (setting2 = 0; setting2 < BUF_LENGTH; setting2++)
                        { textbuffer[setting2] = 0; };           
                    asciitohex(&bigrecordingbuffer[bigrecordingbufferpos + 1], textbuffer,  len);
                    out->print(F("\r\nFrame "));
                    out->print(setting);
                    out->print(F(" : "));                     
                    out->print((char *)textbuffer);
                    out->print(F("\r\n"));
                 };
                    // increase position to the buffer and check exception
                    bigrecordingbufferpos = bigrecordingbufferpos + 1 + len;
                    if ( bigrecordingbufferpos > RECORDINGBUFFERSIZE) break;
                    // feed the watchdog
                    ESP.wdtFeed();
                 // 
               };
          // rewind buffer position
          // bigrecordingbufferpos = 0;
          out->print(F("\r\n")); 
        }
         else { out->print(F("Wrong parameters.\r\n")); };
        // needed for ESP8266   
        yield();      


    // Handling FLUSH command         
    } else if (strcmp_P(command, PSTR("flush")) == 0) {
        // flush the recording buffer and rewind its pointers
        zeroRecordingBuffer();
        out->print(F("\r\nRecording buffer cleared.\r\n"));
        // needed for ESP8266   
        yield();      
          
       
    // Handling ECHO command         
    } else if (strcmp_P(command, PSTR("echo")) == 0) {
        do_echo = atoi(cmdline);

    // Handling X command         
    // command 'x' stops jamming, receiveing, recording...
    } else if (strcmp_P(command, PSTR("x")) == 0) {
        stopActiveMode();
        out->print(F("\r\n"));
        // needed for ESP8266
        yield();

    // Handling INIT command         
    // command 'init' initializes board with default settings
    } else if (strcmp_P(command, PSTR("init")) == 0) {
        // init cc1101
        cc1101initialize();
        // give feedback
        out->print(F("CC1101 initialized\r\n"));
          
    } else {
        out->print(F("Error: Unknown command: "));
        out->println(command);
        // needed for ESP8266
        yield();
    }

    if (guardedConfig) {
        // Frequency changes require explicit calibration. Calibrating all legacy
        // setters also gives them one predictable and safe completion contract.
        if (!calibrateRadio()) {
            out->print(F("[ERROR] radio calibration timed out\r\n"));
        }
        currentFreq = readRadioFrequencyMHz();
        currentRxBw = readRadioRxBwKHz();
        if (resumeRxAfterConfig) ELECHOUSE_cc1101.SetRx();
    }
}


// stop the active mode and restore normal packet mode if it was a raw mode
static void stopActiveMode(void) {
    if (activeMode == MODE_IDLE) {
        out->print(F("[MODE] already idle\r\n"));
        return;
    }
    const char *stoppedName = modeName();
    if (activeMode == MODE_SNIFF || activeMode == MODE_RECRAW) exitRawMode(false);
    if (activeMode == MODE_BRUTE || activeMode == MODE_PLAYRAW) exitRawMode(true);
    recrawWaiting = false;
    enterRadioIdle();
    out->print(F("\r\n[MODE] "));
    out->print(stoppedName);
    out->print(F(" stopped | elapsed "));
    out->print(modeElapsedSeconds());
    out->print(F("s | activity "));
    out->print(modeActivityCount);
    out->print(F("\r\n"));
    activeMode = MODE_IDLE;
}


// Advance whichever long-running mode is active by one small slice.
// Called every loop() pass. Keeps the web server responsive.
static void serviceActiveMode(void)
{
    if (activeMode == MODE_SCAN) {
        // SetRx(float) performs SIDLE -> frequency write -> SRX. MCSM0 then
        // auto-calibrates on every IDLE-to-RX transition.
        ELECHOUSE_cc1101.SetRx(scanCursor);
        int rssi = ELECHOUSE_cc1101.getRssi();
        if (rssi > SCAN_RSSI_MARK && rssi > scanBestRssi) {
            scanBestRssi = rssi; scanBestFreq = scanCursor;
        }
        scanCursor += 0.01;
        if (scanCursor > scanTo) {
            scanCursor = scanFrom;
            modeActivityCount++;
            if (scanBestRssi > SCAN_RSSI_MARK) {
                long fr = scanBestFreq * 100;
                if (fr == scanCompare) {
                    out->print(F("[SCAN] signal locked | "));
                    out->print(scanBestFreq);
                    out->print(F(" MHz | RSSI "));
                    out->print(scanBestRssi);
                    out->print(F(" dBm\r\n"));
                    scanBestRssi = RSSI_NONE; scanCompare = 0; scanBestFreq = 0;
                } else {
                    scanCompare = scanBestFreq * 100;
                    scanCursor = scanBestFreq - 0.10;
                    scanBestFreq = 0; scanBestRssi = RSSI_NONE;
                }
            }
        }
        if (modeFeedbackDue(750)) {
            out->print(F("[SCAN] "));
            out->print(scanCursor, 2);
            out->print(F(" MHz | "));
            out->print(modeProgressPercent());
            out->print(F("% | RSSI "));
            out->print(rssi);
            out->print(F(" dBm | sweeps "));
            out->print(modeActivityCount);
            if (scanBestRssi > RSSI_NONE) {
                out->print(F(" | best "));
                out->print(scanBestFreq, 2);
                out->print(F(" / "));
                out->print(scanBestRssi);
                out->print(F(" dBm"));
            }
            out->print(F("\r\n"));
        }
    }
    else if (activeMode == MODE_SNIFF) {
        // Capture one small chunk (timing-coherent within the chunk), dump it, advance.
        // Small chunks let loop() return to handleClient() ~10x/s so the web stays live.
        for (int n = 0; n < SNIFF_CHUNK; n++) {
            byte receivedbyte = 0;
            for (int j = 7; j > -1; j--) {
                bitWrite(receivedbyte, j, digitalRead(gdo0));
                delayMicroseconds(sniffInterval);
            }
            bigrecordingbuffer[sniffCursor + n] = receivedbyte;
            ESP.wdtFeed();
        }
        out->print(F("[RXRAW] block "));
        out->print((modeActivityCount / SNIFF_CHUNK) + 1);
        out->print(F(" | bytes "));
        out->print(modeActivityCount);
        out->print(F("-"));
        out->print(modeActivityCount + SNIFF_CHUNK - 1);
        out->print(F(" | buffer "));
        out->print(sniffCursor);
        out->print(F("/"));
        out->print(RECORDINGBUFFERSIZE);
        out->print(F("\r\n"));
        dumpBufferHex(sniffCursor, SNIFF_CHUNK);
        out->print(F("\r\n"));
        sniffCursor += SNIFF_CHUNK;
        modeActivityCount += SNIFF_CHUNK;
        if (sniffCursor >= RECORDINGBUFFERSIZE) sniffCursor = 0;
        bigrecordingbufferpos = (modeActivityCount < RECORDINGBUFFERSIZE)
            ? modeActivityCount : RECORDINGBUFFERSIZE;
    }
    else if (activeMode == MODE_RECRAW) {
        if (recrawWaiting) {
            // non-blocking arm: sample the line once per pass; start on first HIGH
            if (digitalRead(gdo0) == HIGH) {
                recrawWaiting = false;
                out->print(F("[RECRAW] signal detected | capturing 4096 bytes now\r\n"));
            }
            else if (modeFeedbackDue(1000)) {
                out->print(F("[RECRAW] waiting for signal | armed "));
                out->print(modeElapsedSeconds());
                out->print(F("s | GDO0 LOW\r\n"));
            }
        } else {
            // one timing-coherent capture: soft WDT off for the duration so the unbroken
            // block can't trip it; the interval clamp keeps the block under the hardware WDT.
            ESP.wdtDisable();
            for (int i = 0; i < RECORDINGBUFFERSIZE; i++) {
                byte receivedbyte = 0;
                for (int j = 7; j > -1; j--) {
                    bitWrite(receivedbyte, j, digitalRead(gdo0));
                    delayMicroseconds(recrawInterval);
                }
                bigrecordingbuffer[i] = receivedbyte;
                ESP.wdtFeed();
            }
            ESP.wdtEnable(5000);
            modeActivityCount = RECORDINGBUFFERSIZE;
            bigrecordingbufferpos = RECORDINGBUFFERSIZE;
            out->print(F("[RECRAW] complete | 4096 bytes captured | elapsed "));
            out->print(modeElapsedSeconds());
            out->print(F("s | use showraw, showbit, save, or playraw\r\n"));
            exitRawMode(false);
            activeMode = MODE_IDLE;
            out->print(F("\r\ncc1101> "));
        }
    }
    else if (activeMode == MODE_PLAYRAW) {
        // one timing-coherent replay: soft WDT off for the duration so the unbroken
        // block can't trip it; the interval clamp keeps the block under the hardware WDT.
        ESP.wdtDisable();
        for (int i = 1; i < RECORDINGBUFFERSIZE; i++) {
            byte receivedbyte = bigrecordingbuffer[i];
            for (int j = 7; j > -1; j--) {
                digitalWrite(gdo0, bitRead(receivedbyte, j));
                delayMicroseconds(playrawInterval);
            }
            ESP.wdtFeed();
        }
        ESP.wdtEnable(5000);
        modeActivityCount = RECORDINGBUFFERSIZE;
        out->print(F("[PLAYRAW] complete | 4096 bytes replayed | elapsed "));
        out->print(modeElapsedSeconds());
        out->print(F("s\r\n"));
        exitRawMode(true);
        activeMode = MODE_IDLE;
        out->print(F("\r\ncc1101> "));
    }
    else if (activeMode == MODE_BRUTE) {
        // send a batch of codes this pass (each code 5x), keep timing tight.
        // small batch keeps loop() returning so the web stays responsive.
        const uint32_t BATCH = 16;
        for (uint32_t n = 0; n < BATCH && bruteCode < bruteMax; n++, bruteCode++) {
            for (int k = 0; k < 5; k++) {
                for (int j = bruteBits - 1; j > -1; j--) {
                    digitalWrite(gdo0, bitRead(bruteCode, j));
                    delayMicroseconds(bruteInterval);
                }
                ESP.wdtFeed();
            }
        }
        modeActivityCount = bruteCode;
        if (modeFeedbackDue(1000)) {
            out->print(F("[BRUTE] code "));
            out->print(bruteCode);
            out->print(F("/"));
            out->print(bruteMax);
            out->print(F(" | "));
            out->print(modeProgressPercent());
            out->print(F("% | elapsed "));
            out->print(modeElapsedSeconds());
            out->print(F("s\r\n"));
        }
        if (bruteCode >= bruteMax) {
            out->print(F("[BRUTE] complete | "));
            out->print(bruteMax);
            out->print(F(" codes tested | elapsed "));
            out->print(modeElapsedSeconds());
            out->print(F("s\r\n"));
            exitRawMode(true);
            activeMode = MODE_IDLE;
            out->print(F("\r\ncc1101> "));
        }
    }
    else if (activeMode == MODE_RX && modeFeedbackDue(3000)) {
        out->print(F("[RX] listening | packets "));
        out->print(modeActivityCount);
        out->print(F(" | "));
        out->print(currentFreq, 2);
        out->print(F(" MHz | elapsed "));
        out->print(modeElapsedSeconds());
        out->print(F("s\r\n"));
    }
    else if (activeMode == MODE_REC && modeFeedbackDue(3000)) {
        out->print(F("[REC] waiting | frames "));
        out->print(framesinbigrecordingbuffer);
        out->print(F(" | buffer "));
        out->print(bigrecordingbufferpos);
        out->print(F("/"));
        out->print(RECORDINGBUFFERSIZE);
        out->print(F(" bytes\r\n"));
    }
}


void setup() {

     // initialize USB Serial Port CDC
     Serial.begin(115200);

     Serial.println(F("\r\n+======================================================+"));
     Serial.println(F("|        CYPHER CC1101 FIELD CONSOLE / ESP8266        |"));
     Serial.println(F("+======================================================+"));
     Serial.println(F("[BOOT] serial console online at 115200 baud"));

    // Init flash-simulated EEPROM (ESP8266)
     EEPROM.begin(EPROMSIZE);

     // seed the PRNG once for the 'jam' random payloads
     randomSeed(analogRead(0));

     // initialize CC1101 module with preffered parameters
     cc1101initialize();

      if (ELECHOUSE_cc1101.getCC1101()) {  // Check the CC1101 Spi connection.
      Serial.println(F("[RADIO] CC1101 SPI link online | default 433.92 MHz"));
      } else {
      Serial.println(F("[RADIO] ERROR: CC1101 not detected | check SPI wiring"));
      };
    
      // setup variables
     bigrecordingbufferpos = 0;

     // Enable the ESP8266 software watchdog as a safety net
     ESP.wdtEnable(5000);

     // bring up WiFi AP and the web control panel
     startAP();
     setupWebServer();
     Serial.println(F("[READY] type help for commands or status for dashboard"));
     Serial.print(F("\r\ncc1101> "));
}


void loop() {

  // index for serial port characters
  int i = 0;

   // feed the watchdog in ESP8266
   ESP.wdtFeed();

   // service pending HTTP requests
   server.handleClient();

   // any serial input stops a running background mode (preserves old UX)
   if (activeMode == MODE_SCAN || activeMode == MODE_SNIFF || activeMode == MODE_BRUTE ||
       activeMode == MODE_RECRAW || activeMode == MODE_PLAYRAW) {
       if (Serial.available() && Serial.peek() != 'x') { stopActiveMode(); }
   }

    /* Process incoming commands. */
    while (Serial.available()) {
        static char buffer[BUF_LENGTH];
        static int length = 0;


    // handling CHAT MODE     
    if (activeMode == MODE_CHAT)
       {
            
            // clear serial port buffer index
            i = 0;

            // something was received over serial port put it into radio sending buffer
            while (Serial.available() and (i<(CCBUFFERSIZE-1)) ) 
             {

              // read single character from Serial port         
              ccsendingbuffer[i] = Serial.read();

              // also put it as ECHO back to serial port
              Serial.write(ccsendingbuffer[i]);

              // if CR was received add also LF character and display it on Serial port
              if (ccsendingbuffer[i] == 0x0d )
                  {  
                    Serial.write( 0x0a );
                    i++;
                    ccsendingbuffer[i] = 0x0a;
                  }
              //
              
              // increase CC1101 TX buffer position
              i++;
             };

            // clamp: a trailing CR appends an extra LF and can push i past the
            // last index, so keep the NUL write inside ccsendingbuffer[]
            if (i > (CCBUFFERSIZE - 1)) i = CCBUFFERSIZE - 1;
            // put NULL at the end of CC transmission buffer
            ccsendingbuffer[i] = '\0';

            // send these data to radio over CC1101
            ELECHOUSE_cc1101.SendData((char *)ccsendingbuffer);

            // feed the watchdog
            ESP.wdtFeed();
            // needed for ESP8266   
            yield();      
                           
       }
    // handling CLI commands processing
    else
      {   
        int data = Serial.read();
        if (data == '\b' || data == '\177') {  // BS and DEL
            if (length) {
                length--;
                if (do_echo) Serial.write("\b \b");
              // feed the watchdog
              ESP.wdtFeed();
            }
        }
        else if (data == '\r' || data == '\n' ) {
            if (do_echo) Serial.write("\r\n");    // output CRLF
            buffer[length] = '\0';
            if (length) exec(buffer);
            length = 0;
            if (activeMode == MODE_IDLE) Serial.print(F("\r\ncc1101> "));
            // feed the watchdog
            ESP.wdtFeed();

        }
        else if (length < BUF_LENGTH - 1) {
            buffer[length++] = data;
            if (do_echo) Serial.write(data);
            // feed the watchdog
            ESP.wdtFeed();
        }
       };  
      // end of handling CLI processing
        
    };

  /* Process RF received packets */
   
   //Checks whether something has been received.
  if ((activeMode == MODE_RX || activeMode == MODE_REC || activeMode == MODE_CHAT) &&
      ELECHOUSE_cc1101.CheckReceiveFlag())
      {

       //CRC Check. If "setCrc(false)" crc returns always OK!
       if (ELECHOUSE_cc1101.CheckCRC())
          { 
            // feed the watchdog
            ESP.wdtFeed();

            //Get received Data and calculate length
            int len = ELECHOUSE_cc1101.ReceiveData(ccreceivingbuffer);

            // Actions for CHAT MODE
            if ( (activeMode == MODE_CHAT) && (len < CCBUFFERSIZE ) )
               {
                // put NULL at the end of char buffer
                ccreceivingbuffer[len] = '\0';
                //Print received in char format.
                Serial.print((char *) ccreceivingbuffer);
                // feed the watchdog
                // ESP.wdtFeed();
                // needed for ESP8266   
                yield();      

               };  // end of handling Chat mode

            // Actions for RECEIVNG MODE
            if ( (activeMode == MODE_RX) && (len < CCBUFFERSIZE ) )
               {
                   // put NULL at the end of char buffer
                   ccreceivingbuffer[len] = '\0';
                   // flush textbuffer
                   for (int i = 0; i < BUF_LENGTH; i++)
                    { textbuffer[i] = 0; };
                   
                   //Print received packet as set of hex values directly 
                   // not to loose any data in buffer
                   // asciitohex((byte *)ccreceivingbuffer, (byte *)textbuffer,  len);
                   asciitohex(ccreceivingbuffer, textbuffer,  len);
                   modeActivityCount++;
                   lastModeFeedbackAt = millis();
                   Serial.print(F("[RX] packet #"));
                   Serial.print(modeActivityCount);
                   Serial.print(F(" | "));
                   Serial.print(len);
                   Serial.print(F(" bytes | RSSI "));
                   Serial.print(ELECHOUSE_cc1101.getRssi());
                   Serial.print(F(" dBm | data "));
                   Serial.println((char *)textbuffer);
                   // set RX  mode again
                   ELECHOUSE_cc1101.SetRx();
                   // feed the watchdog
                   ESP.wdtFeed();
                   // needed for ESP8266   
                   yield();                        
                };   // end of handling receiving mode 

            // Actions for RECORDING MODE               
            if ( (activeMode == MODE_REC) && (len < CCBUFFERSIZE ) )
               { 
                // copy the frame from receiving buffer for replay - only if it fits
                if (( bigrecordingbufferpos + len + 1) < RECORDINGBUFFERSIZE) 
                     {
                      // put info about number of bytes
                      bigrecordingbuffer[bigrecordingbufferpos] = len; 
                      bigrecordingbufferpos++;
                      // next - copy current frame and increase 
                      memcpy(&bigrecordingbuffer[bigrecordingbufferpos], ccreceivingbuffer, len );
                      // increase position in big recording buffer for next frame
                      bigrecordingbufferpos = bigrecordingbufferpos + len; 
                      // increase counter of frames stored
                      framesinbigrecordingbuffer++;
                      modeActivityCount = framesinbigrecordingbuffer;
                      lastModeFeedbackAt = millis();
                      asciitohex(ccreceivingbuffer, textbuffer, len);
                      Serial.print(F("[REC] frame #"));
                      Serial.print(framesinbigrecordingbuffer);
                      Serial.print(F(" | "));
                      Serial.print(len);
                      Serial.print(F(" bytes | buffer "));
                      Serial.print(bigrecordingbufferpos);
                      Serial.print(F("/"));
                      Serial.print(RECORDINGBUFFERSIZE);
                      Serial.print(F(" | data "));
                      Serial.println((char *)textbuffer);
                      // set RX  mode again
                      ELECHOUSE_cc1101.SetRx();
                      // feed the watchdog
                      ESP.wdtFeed();
                      // needed for ESP8266   
                      yield();                            
                     }
                     
                else {
                    Serial.print(F("[REC] buffer full | stored "));
                    Serial.print(framesinbigrecordingbuffer);
                    Serial.println(F(" frames | recording stopped"));
                    bigrecordingbufferpos = 0;
                    activeMode = MODE_IDLE;
                    Serial.print(F("\r\ncc1101> "));
                    // feed the watchdog
                    ESP.wdtFeed();
                    // needed for ESP8266   
                    yield();                          
                     };
                
               };   // end of handling frame recording mode 
 
          };   // end of CRC check IF


      };   // end of Check receive flag if

      // if jamming mode activate continously send something over RF...
      if ( activeMode == MODE_JAM )
      {
        // populate cc1101 sending buffer with random values (PRNG seeded in setup)
        for (i = 0; i<MAX_PAYLOAD; i++)
           { ccsendingbuffer[i] = (byte)random(255);
             // feed the watchdog
             ESP.wdtFeed();
             // needed for ESP8266
             yield();
           };
        // send these data to radio over CC1101
        ELECHOUSE_cc1101.SendData(ccsendingbuffer,MAX_PAYLOAD);
        modeActivityCount++;
        if (modeFeedbackDue(1000)) {
            Serial.print(F("[JAM] active | frames "));
            Serial.print(modeActivityCount);
            Serial.print(F(" | elapsed "));
            Serial.print(modeElapsedSeconds());
            Serial.println(F("s"));
        }
        // feed the watchdog
        ESP.wdtFeed();
        // needed for ESP8266   
        yield();              
      };

   // advance any active long-running mode by one slice
   serviceActiveMode();

   // give control for other procedures in ESP8266
   yield();
 
 
}  // end of main LOOP
