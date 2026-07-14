#include <Arduino.h>

#include <string.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "rom/gpio.h"
#include "soc/gpio_pins.h"
#include "soc/gpio_sig_map.h"

static const uart_port_t HD_UART = UART_NUM_0;
static const uart_port_t FD_UART = UART_NUM_1;

static const int HD_PIN = 3;

static const int FD_TX_PIN = 21;
static const int FD_RX_PIN = 20;

static const int BAUD = 115200;

static const int UART_RX_BUF_SIZE = 4096;
static const int UART_TX_BUF_SIZE = 4096;

static const int IO_BUF_SIZE = 256;
static const int MAX_CHUNKS_PER_PASS = 8;

static const uint32_t UART_BYTE_US = ((1000000UL * 10UL) + BAUD - 1UL) / BAUD;

/*
  Pure bus-idle guard. Not CRSF frame parsing.
  At 115200, 4 byte times is about 348 us.
*/
static const uint32_t HD_IDLE_US = UART_BYTE_US * 4UL;

/*
  ESP32-C3 GPIO matrix constant used to detach UART RX while the same pad is TX.
  Keep this handling intact for the one-wire inverted half-duplex pad.
*/
static const uint32_t MATRIX_DETACH_IN_LOW = GPIO_MATRIX_CONST_ZERO_INPUT;

static volatile bool hdTxActive = false;
static volatile uint32_t lastHdActivityUs = 0;

static uint8_t hdToFdBuf[IO_BUF_SIZE];
static uint8_t fdToHdBuf[IO_BUF_SIZE];

static uint32_t nowUs()
{
  return (uint32_t)esp_timer_get_time();
}

static void IRAM_ATTR hdEdgeIsr(void *arg)
{
  (void)arg;

  if (!hdTxActive) {
    lastHdActivityUs = (uint32_t)esp_timer_get_time();
  }
}

static void stopHere(esp_err_t err)
{
  if (err == ESP_OK) {
    return;
  }

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

  stopHere(uart_driver_install(port, UART_RX_BUF_SIZE, UART_TX_BUF_SIZE, 0, nullptr, 0));
  stopHere(uart_param_config(port, &cfg));
  stopHere(uart_set_mode(port, UART_MODE_UART));

  /*
    Low-latency receive path. Short CRSF/Lua frames should not sit in the UART
    FIFO waiting for a large RX threshold before the driver wakes up.
  */
  stopHere(uart_set_rx_full_threshold(port, 1));
  stopHere(uart_set_rx_timeout(port, 1));

  uart_flush_input(port);
}

static void fdSetPins()
{
  /*
    ESP32-C3 GPIO20/GPIO21 are native UART0 pins.

    This bridge uses UART0 for HD on GPIO3 through the GPIO matrix.
    Therefore GPIO20/GPIO21 must be reset away from native UART0 IOMUX
    before UART1 is routed onto them.
  */
  stopHere(gpio_reset_pin((gpio_num_t)FD_TX_PIN));
  stopHere(gpio_reset_pin((gpio_num_t)FD_RX_PIN));

  stopHere(gpio_set_direction((gpio_num_t)FD_TX_PIN, GPIO_MODE_OUTPUT));
  stopHere(gpio_set_direction((gpio_num_t)FD_RX_PIN, GPIO_MODE_INPUT));

  gpio_pullup_dis((gpio_num_t)FD_TX_PIN);
  gpio_pulldown_dis((gpio_num_t)FD_TX_PIN);

  gpio_pullup_dis((gpio_num_t)FD_RX_PIN);
  gpio_pulldown_dis((gpio_num_t)FD_RX_PIN);

  gpio_matrix_out((gpio_num_t)FD_TX_PIN, U1TXD_OUT_IDX, false, false);
  gpio_matrix_in((gpio_num_t)FD_RX_PIN, U1RXD_IN_IDX, false);
}

static void setupFdUart()
{
  setupUartBase(FD_UART);

  fdSetPins();

  stopHere(uart_set_pin(FD_UART,
                        FD_TX_PIN,
                        FD_RX_PIN,
                        UART_PIN_NO_CHANGE,
                        UART_PIN_NO_CHANGE));

  fdSetPins();

  uart_flush_input(FD_UART);
}

static void hdSetRx()
{
  /*
    Physical CRSF bay UART is inverted:
      physical idle low
      UART logical idle high

    GPIO matrix input inversion handles this.
  */
  stopHere(gpio_set_direction((gpio_num_t)HD_PIN, GPIO_MODE_INPUT));

  gpio_matrix_in((gpio_num_t)HD_PIN, U0RXD_IN_IDX, true);

  gpio_pulldown_en((gpio_num_t)HD_PIN);
  gpio_pullup_dis((gpio_num_t)HD_PIN);
}

static void hdSetTx()
{
  /*
    Inverted UART physical idle is low.
    Set pad low before handing it to UART TX.
  */
  stopHere(gpio_set_pull_mode((gpio_num_t)HD_PIN, GPIO_FLOATING));
  stopHere(gpio_set_level((gpio_num_t)HD_PIN, 0));
  stopHere(gpio_set_direction((gpio_num_t)HD_PIN, GPIO_MODE_OUTPUT));

  /*
    Detach RX while TX owns the single wire.
    This prevents self-echo and avoids RX garbage during our own TX.
  */
  gpio_matrix_in(MATRIX_DETACH_IN_LOW, U0RXD_IN_IDX, false);

  /*
    Output inversion handles inverted physical CRSF.
  */
  gpio_matrix_out((gpio_num_t)HD_PIN, U0TXD_OUT_IDX, true, false);
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

static bool hdLineLooksIdle()
{
  /*
    With this inverted physical bus, idle on the pad is low.
    This is only an extra last-moment sanity check, not the whole idle detector.
  */
  return gpio_get_level((gpio_num_t)HD_PIN) == 0;
}

static size_t uartPending(uart_port_t port)
{
  size_t pending = 0;
  uart_get_buffered_data_len(port, &pending);
  return pending;
}

static bool hdTxDone()
{
  return uart_wait_tx_done(HD_UART, 0) == ESP_OK;
}

static bool hdTryStartTx()
{
  if (hdTxActive) {
    return true;
  }

  if (!hdBusIdle()) {
    return false;
  }

  if (uartPending(HD_UART) > 0) {
    return false;
  }

  if (!hdLineLooksIdle()) {
    lastHdActivityUs = nowUs();
    return false;
  }

  gpio_intr_disable((gpio_num_t)HD_PIN);

  if (!hdBusIdle() || uartPending(HD_UART) > 0 || !hdLineLooksIdle()) {
    gpio_intr_enable((gpio_num_t)HD_PIN);
    lastHdActivityUs = nowUs();
    return false;
  }

  hdTxActive = true;
  hdSetTx();
  return true;
}

static void hdReleaseTxIfDone()
{
  if (!hdTxActive) {
    return;
  }

  if (uartPending(FD_UART) > 0) {
    return;
  }

  if (!hdTxDone()) {
    return;
  }

  uart_flush_input(HD_UART);

  hdSetRx();
  lastHdActivityUs = nowUs();
  hdTxActive = false;

  gpio_intr_enable((gpio_num_t)HD_PIN);
}

static void writeAll(uart_port_t port, const uint8_t *buf, int len)
{
  int offset = 0;

  while (offset < len) {
    int written = uart_write_bytes(port, (const char *)(buf + offset), len - offset);

    if (written <= 0) {
      return;
    }

    offset += written;
  }
}

static void serviceHdToFd()
{
  if (hdTxActive) {
    return;
  }

  for (int chunk = 0; chunk < MAX_CHUNKS_PER_PASS; chunk++) {
    int n = uart_read_bytes(HD_UART, hdToFdBuf, sizeof(hdToFdBuf), 0);

    if (n <= 0) {
      return;
    }

    lastHdActivityUs = nowUs();
    writeAll(FD_UART, hdToFdBuf, n);
  }
}

static void serviceFdToHd()
{
  for (int chunk = 0; chunk < MAX_CHUNKS_PER_PASS; chunk++) {
    size_t pending = uartPending(FD_UART);

    if (pending == 0) {
      return;
    }

    if (!hdTxActive && !hdTryStartTx()) {
      return;
    }

    int toRead = (int)pending;

    if (toRead > IO_BUF_SIZE) {
      toRead = IO_BUF_SIZE;
    }

    int n = uart_read_bytes(FD_UART, fdToHdBuf, toRead, 0);

    if (n <= 0) {
      return;
    }

    writeAll(HD_UART, fdToHdBuf, n);
  }
}

void setup()
{
  /*
    Do not use Serial here.

    UART0 is the half-duplex CRSF side in this sketch. On ESP32-C3,
    GPIO20/GPIO21 are also native UART0 pins before this code remaps them.
    Serial logging from this sketch can corrupt either side of the bridge.
  */
  setupHdUart();
  setupFdUart();

  stopHere(gpio_set_intr_type((gpio_num_t)HD_PIN, GPIO_INTR_ANYEDGE));

  esp_err_t isrErr = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);

  if (isrErr != ESP_OK && isrErr != ESP_ERR_INVALID_STATE) {
    stopHere(isrErr);
  }

  stopHere(gpio_isr_handler_add((gpio_num_t)HD_PIN, hdEdgeIsr, nullptr));

  lastHdActivityUs = nowUs();
}

void loop()
{
  if (hdTxActive) {
    serviceFdToHd();
    hdReleaseTxIfDone();
    return;
  }

  serviceHdToFd();
  serviceFdToHd();
  hdReleaseTxIfDone();
}