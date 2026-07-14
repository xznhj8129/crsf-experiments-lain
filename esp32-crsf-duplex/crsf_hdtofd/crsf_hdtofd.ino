#include <Arduino.h>

#include <stdarg.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "rom/gpio.h"
#include "soc/gpio_sig_map.h"

static const char *NAME = "BRIDGE";

static const uart_port_t HD_UART = UART_NUM_1;
static const uart_port_t FD_UART = UART_NUM_2;

static const int HD_PIN = 4;

static const int FD_TX_PIN = 17;
static const int FD_RX_PIN = 16;

static const int BAUD = 115200;

static const int UART_RX_BUF_SIZE = 4096;
static const int UART_TX_BUF_SIZE = 4096;

static const int BUF_SIZE = 128;

static const bool DEBUG_STATS = true;

static const uint32_t UART_BYTE_US = ((1000000UL * 10UL) + BAUD - 1UL) / BAUD;

/*
  Pure bus-idle guard. Not CRSF frame parsing.
  At 115200, 4 byte times is about 348 us.
*/
static const uint32_t HD_IDLE_US = UART_BYTE_US * 4UL;

/*
  ESP32 GPIO matrix constants used to detach UART RX while the same pad is TX.
  Same idea as ExpressLRS CRSFHandset::duplex_set_TX().
*/
static const uint8_t MATRIX_DETACH_IN_LOW = 0x30;

static volatile bool hdTxActive = false;
static volatile uint32_t lastHdActivityUs = 0;

static volatile uint32_t extEdges = 0;
static volatile uint32_t selfEdges = 0;

static uint32_t hdInBytes = 0;
static uint32_t fdInBytes = 0;
static uint32_t hdOutBytes = 0;
static uint32_t fdOutBytes = 0;

static uint32_t hdInBursts = 0;
static uint32_t fdInBursts = 0;

static uint32_t hdTxStarts = 0;
static uint32_t hdTxReleases = 0;
static uint32_t hdTxDefers = 0;

static SemaphoreHandle_t printLock = nullptr;

static uint32_t nowUs()
{
  return (uint32_t)esp_timer_get_time();
}

static void IRAM_ATTR hdEdgeIsr(void *arg)
{
  (void)arg;

  if (hdTxActive) {
    selfEdges++;
  } else {
    extEdges++;
    lastHdActivityUs = (uint32_t)esp_timer_get_time();
  }
}

static void printLocked(const char *fmt, ...)
{
  if (printLock) {
    xSemaphoreTake(printLock, portMAX_DELAY);
  }

  va_list ap;
  va_start(ap, fmt);
  Serial.vprintf(fmt, ap);
  va_end(ap);

  if (printLock) {
    xSemaphoreGive(printLock);
  }
}

static void stopHere(esp_err_t err, const char *where)
{
  if (err == ESP_OK) {
    return;
  }

  printLocked("[%s] ERROR %s: %d\n", NAME, where, (int)err);

  while (true) {
    delay(1000);
  }
}

static void setupUartBase(uart_port_t port)
{
  uart_config_t cfg;
  memset(&cfg, 0, sizeof(cfg));

  cfg.baud_rate = BAUD;
  cfg.data_bits = UART_DATA_8_BITS;
  cfg.parity = UART_PARITY_DISABLE;
  cfg.stop_bits = UART_STOP_BITS_1;
  cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  cfg.rx_flow_ctrl_thresh = 0;

  stopHere(uart_driver_install(port, UART_RX_BUF_SIZE, UART_TX_BUF_SIZE, 0, nullptr, 0),
           "uart_driver_install");

  stopHere(uart_param_config(port, &cfg),
           "uart_param_config");

  stopHere(uart_set_mode(port, UART_MODE_UART),
           "uart_set_mode");

  uart_flush_input(port);
}

static void setupFdUart()
{
  setupUartBase(FD_UART);

  stopHere(uart_set_pin(FD_UART,
                        FD_TX_PIN,
                        FD_RX_PIN,
                        UART_PIN_NO_CHANGE,
                        UART_PIN_NO_CHANGE),
           "uart_set_pin FD");
}

static void hdSetRx()
{
  /*
    Physical CRSF bay UART is inverted:
      physical idle low
      UART logical idle high

    GPIO matrix input inversion handles this.
  */
  stopHere(gpio_set_direction((gpio_num_t)HD_PIN, GPIO_MODE_INPUT),
           "HD RX direction");

  gpio_matrix_in((gpio_num_t)HD_PIN, U1RXD_IN_IDX, true);

  gpio_pulldown_en((gpio_num_t)HD_PIN);
  gpio_pullup_dis((gpio_num_t)HD_PIN);
}

static void hdSetTx()
{
  /*
    Inverted UART physical idle is low.
    Set pad low before handing it to UART TX.
  */
  stopHere(gpio_set_pull_mode((gpio_num_t)HD_PIN, GPIO_FLOATING),
           "HD TX floating");

  stopHere(gpio_set_level((gpio_num_t)HD_PIN, 0),
           "HD TX idle level");

  stopHere(gpio_set_direction((gpio_num_t)HD_PIN, GPIO_MODE_OUTPUT),
           "HD TX direction");

  /*
    Detach RX while TX owns the single wire.
    This prevents self-echo and avoids RX garbage during our own TX.
  */
  gpio_matrix_in(MATRIX_DETACH_IN_LOW, U1RXD_IN_IDX, false);

  /*
    Output inversion handles inverted physical CRSF.
  */
  gpio_matrix_out((gpio_num_t)HD_PIN, U1TXD_OUT_IDX, true, false);
}

static void setupHdUart()
{
  setupUartBase(HD_UART);

  hdSetRx();
  uart_flush_input(HD_UART);
}

static bool hdBusIdle()
{
  return (uint32_t)(nowUs() - lastHdActivityUs) >= HD_IDLE_US;
}

static bool hdTxDone()
{
  return uart_wait_tx_done(HD_UART, 0) == ESP_OK;
}

static void hdStartTx()
{
  if (hdTxActive) {
    return;
  }

  hdTxActive = true;
  hdTxStarts++;

  uart_flush_input(HD_UART);
  hdSetTx();
}

static void hdReleaseTx()
{
  if (!hdTxActive) {
    return;
  }

  if (!hdTxDone()) {
    return;
  }

  uart_flush_input(HD_UART);

  hdSetRx();

  lastHdActivityUs = nowUs();

  hdTxActive = false;
  hdTxReleases++;
}

static bool writeHdBytes(const uint8_t *buf, int len)
{
  if (len <= 0) {
    return true;
  }

  if (!hdTxActive) {
    if (!hdBusIdle()) {
      hdTxDefers++;
      return false;
    }

    hdStartTx();
  }

  int written = uart_write_bytes(HD_UART, (const char *)buf, len);

  if (written > 0) {
    hdOutBytes += (uint32_t)written;
  }

  return written == len;
}

static void serviceHdToFd()
{
  if (hdTxActive) {
    return;
  }

  uint8_t buf[BUF_SIZE];

  int n = uart_read_bytes(HD_UART, buf, sizeof(buf), 0);

  if (n <= 0) {
    return;
  }

  hdInBytes += (uint32_t)n;
  hdInBursts++;
  lastHdActivityUs = nowUs();

  int written = uart_write_bytes(FD_UART, (const char *)buf, n);

  if (written > 0) {
    fdOutBytes += (uint32_t)written;
  }
}

static void serviceFdToHd()
{
  uint8_t buf[BUF_SIZE];

  size_t fdPending = 0;
  uart_get_buffered_data_len(FD_UART, &fdPending);

  if (fdPending == 0) {
    return;
  }

  /*
    If the HD bus is currently receiving external traffic, leave FD bytes in
    the ESP-IDF UART ring buffer until the one-wire bus becomes idle.
  */
  if (!hdTxActive && !hdBusIdle()) {
    hdTxDefers++;
    return;
  }

  int toRead = (int)fdPending;

  if (toRead > BUF_SIZE) {
    toRead = BUF_SIZE;
  }

  int n = uart_read_bytes(FD_UART, buf, toRead, 0);

  if (n <= 0) {
    return;
  }

  fdInBytes += (uint32_t)n;
  fdInBursts++;

  writeHdBytes(buf, n);
}

static void serviceHdTxRelease()
{
  if (!hdTxActive) {
    return;
  }

  /*
    Keep TX ownership if more FD bytes are already queued. This prevents
    pointless release/reacquire gaps inside one continuous forwarded stream.
  */
  size_t fdPending = 0;
  uart_get_buffered_data_len(FD_UART, &fdPending);

  if (fdPending > 0) {
    return;
  }

  hdReleaseTx();
}

static void statsOnce()
{
  if (!DEBUG_STATS) {
    return;
  }

  static uint32_t lastMs = 0;

  static uint32_t lastExtEdges = 0;
  static uint32_t lastSelfEdges = 0;

  static uint32_t lastHdInBytes = 0;
  static uint32_t lastFdInBytes = 0;
  static uint32_t lastHdOutBytes = 0;
  static uint32_t lastFdOutBytes = 0;

  static uint32_t lastHdInBursts = 0;
  static uint32_t lastFdInBursts = 0;

  static uint32_t lastHdTxStarts = 0;
  static uint32_t lastHdTxReleases = 0;
  static uint32_t lastHdTxDefers = 0;

  uint32_t ms = millis();

  if ((uint32_t)(ms - lastMs) < 1000) {
    return;
  }

  lastMs = ms;

  uint32_t e0 = extEdges;
  uint32_t e1 = selfEdges;

  uint32_t a = hdInBytes;
  uint32_t b = fdInBytes;
  uint32_t c = hdOutBytes;
  uint32_t d = fdOutBytes;

  uint32_t x = hdInBursts;
  uint32_t y = fdInBursts;

  uint32_t s = hdTxStarts;
  uint32_t r = hdTxReleases;
  uint32_t z = hdTxDefers;

  size_t hdPending = 0;
  size_t fdPending = 0;

  uart_get_buffered_data_len(HD_UART, &hdPending);
  uart_get_buffered_data_len(FD_UART, &fdPending);

  int level = gpio_get_level((gpio_num_t)HD_PIN);

  printLocked(
    "[%s] gpio4=%d active=%d ext_edges=%lu/s self_edges=%lu/s "
    "HD_IN=%lu/s FD_IN=%lu/s HD_OUT=%lu/s FD_OUT=%lu/s "
    "HD_BURSTS=%lu/s FD_BURSTS=%lu/s "
    "TX_START=%lu/s TX_RELEASE=%lu/s TX_DEFER=%lu/s "
    "HD_PENDING=%u FD_PENDING=%u\n",
    NAME,
    level,
    hdTxActive ? 1 : 0,
    (unsigned long)(e0 - lastExtEdges),
    (unsigned long)(e1 - lastSelfEdges),
    (unsigned long)(a - lastHdInBytes),
    (unsigned long)(b - lastFdInBytes),
    (unsigned long)(c - lastHdOutBytes),
    (unsigned long)(d - lastFdOutBytes),
    (unsigned long)(x - lastHdInBursts),
    (unsigned long)(y - lastFdInBursts),
    (unsigned long)(s - lastHdTxStarts),
    (unsigned long)(r - lastHdTxReleases),
    (unsigned long)(z - lastHdTxDefers),
    (unsigned int)hdPending,
    (unsigned int)fdPending
  );

  lastExtEdges = e0;
  lastSelfEdges = e1;

  lastHdInBytes = a;
  lastFdInBytes = b;
  lastHdOutBytes = c;
  lastFdOutBytes = d;

  lastHdInBursts = x;
  lastFdInBursts = y;

  lastHdTxStarts = s;
  lastHdTxReleases = r;
  lastHdTxDefers = z;
}

void setup()
{
  Serial.begin(921600);
  delay(300);

  printLock = xSemaphoreCreateMutex();

  printLocked("\n[%s] boot\n", NAME);
  printLocked("[%s] HD GPIO=%d UART1 matrix inverted RX/TX baud=%d\n", NAME, HD_PIN, BAUD);
  printLocked("[%s] FD TX=%d RX=%d UART2 normal baud=%d\n", NAME, FD_TX_PIN, FD_RX_PIN, BAUD);

  setupHdUart();
  setupFdUart();

  stopHere(gpio_set_intr_type((gpio_num_t)HD_PIN, GPIO_INTR_ANYEDGE),
           "gpio_set_intr_type");

  esp_err_t isrErr = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);

  if (isrErr != ESP_OK && isrErr != ESP_ERR_INVALID_STATE) {
    stopHere(isrErr, "gpio_install_isr_service");
  }

  stopHere(gpio_isr_handler_add((gpio_num_t)HD_PIN, hdEdgeIsr, nullptr),
           "gpio_isr_handler_add");

  lastHdActivityUs = nowUs();

  printLocked("[%s] running\n", NAME);
}

void loop()
{
  /*
    RX side first. Then FD-to-HD. Then release TX if hardware UART is empty.
    This is still raw byte forwarding, no CRSF parsing.
  */
  serviceHdToFd();
  serviceFdToHd();
  serviceHdTxRelease();
  statsOnce();

  delayMicroseconds(10);
}