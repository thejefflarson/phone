#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "soc/gpio_struct.h"
#include "esp_spi_flash.h"
#include "esp_spiffs.h"
#include "driver/gpio.h"
#define STB_TRUETYPE_IMPLEMENTATION
#include "stb_truetype.h"

#define HX8357_TFTWIDTH            320  ///< 320 pixels wide
#define HX8357_TFTHEIGHT           480  ///< 480 pixels tall
#define HX8357_NOP                0x00  ///< No op
#define HX8357_SWRESET            0x01  ///< software reset
#define HX8357_RDDID              0x04  ///< Read ID
#define HX8357_RDDST              0x09  ///< (unknown)
#define HX8357_RDPOWMODE          0x0A  ///< Read power mode Read power mode
#define HX8357_RDMADCTL           0x0B  ///< Read MADCTL
#define HX8357_RDCOLMOD           0x0C  ///< Column entry mode
#define HX8357_RDDIM              0x0D  ///< Read display image mode
#define HX8357_RDDSDR             0x0F  ///< Read dosplay signal mode
#define HX8357_SLPIN              0x10  ///< Enter sleep mode
#define HX8357_SLPOUT             0x11  ///< Exit sleep mode
#define HX8357_INVOFF             0x20  ///< Turn off invert
#define HX8357_INVON              0x21  ///< Turn on invert
#define HX8357_DISPOFF            0x28  ///< Display on
#define HX8357_DISPON             0x29  ///< Display off
#define HX8357_CASET              0x2A  ///< Column addr set
#define HX8357_PASET              0x2B  ///< Page addr set
#define HX8357_RAMWR              0x2C  ///< Write VRAM
#define HX8357_RAMRD              0x2E  ///< Read VRAm
#define HX8357_TEON               0x35  ///< Tear enable on
#define HX8357_TEARLINE           0x44  ///< (unknown)
#define HX8357_MADCTL             0x36  ///< Memory access control
#define HX8357_COLMOD             0x3A  ///< Color mode
#define HX8357_SETOSC             0xB0  ///< Set oscillator
#define HX8357_SETPOWER           0xB1  ///< Set power control
#define HX8357_SETRGB             0xB3  ///< Set RGB interface
#define HX8357_SETCOM             0xB6  ///< Set VCOM voltage
#define HX8357_SETCYC             0xB4  ///< Set display cycle reg
#define HX8357_SETEXTC            0xB9  ///< Enable extension command
#define HX8357_SETSTBA            0xC0  ///< Set source option
#define HX8357_SETPANEL           0xCC  ///< Set Panel
#define HX8357_SETGAMMA           0xE0  ///< Set Gamma

typedef struct {
  uint8_t cmd;
  uint8_t bytes; //No of data in data; bit 7 = delay after set; 0xFF = end of cmds.
  uint8_t data[34];
} lcd_init_cmd_t;

static const lcd_init_cmd_t init_cmds[] = {
    {HX8357_SWRESET, 0x80, {0}}, // Soft reset, then delay 10 ms
    {HX8357_SLPOUT, 0x80, {0}}, // Exit Sleep, then delay 50 ms
    // TODO: Not sure why this doesn't work
    /* {HX8357_COLMOD, 1, {0x85}}, // 16 bit */
    {HX8357_DISPON, 0x80, {0}}, // Main screen turn on, delay 50 ms
    {0xff, 0, {0}},              // END OF COMMAND LIST
};

#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 18
#define PIN_NUM_SCLK 5
#define PIN_NUM_CS   15
#define PIN_NUM_DC   33
#define PIN_NUM_RST  16

void lcd_set_dc(spi_transaction_t *t) {
  int dc = (int)t->user;
  gpio_set_level(PIN_NUM_DC, dc);
}

void lcd_cmd(spi_device_handle_t spi, uint8_t cmd) {
  esp_err_t ret;
  spi_transaction_t t = {0};
  t.length = 8;
  t.tx_buffer = &cmd;
  t.user = (void *)0;
  ret = spi_device_polling_transmit(spi, &t);
  assert(ret == ESP_OK);
}

void lcd_data(spi_device_handle_t spi, const uint8_t *data, size_t length) {
  if (length == 0) return;
  esp_err_t ret;
  spi_transaction_t t = {0};
  t.length = length * 8; // size in bits
  t.tx_buffer = data;
  ret = spi_device_polling_transmit(spi, &t);
  assert(ret == ESP_OK);
}

void lcd_init(spi_device_handle_t spi) {
  gpio_set_direction(PIN_NUM_DC, GPIO_MODE_OUTPUT);
  gpio_set_direction(PIN_NUM_RST, GPIO_MODE_OUTPUT);
  // reset
  gpio_set_level(PIN_NUM_RST, 0);
  vTaskDelay(100 / portTICK_RATE_MS);
  gpio_set_level(PIN_NUM_RST, 1);
  vTaskDelay(100 / portTICK_RATE_MS);
  uint8_t cmd = 0;
  while (init_cmds[cmd].cmd != 0xff) {
    if (init_cmds[cmd].cmd == 0) {
      cmd++;
      vTaskDelay(300 / portTICK_PERIOD_MS);
      continue;
    }

    lcd_cmd(spi, init_cmds[cmd].cmd);
    if (init_cmds[cmd].bytes & 0x80) {
      vTaskDelay(100 / portTICK_PERIOD_MS);
    } else {
      lcd_data(spi, init_cmds[cmd].data, init_cmds[cmd].bytes);
    }
    cmd++;
  }
}

// TODO: Not sure why this doesn't work
void lcd_info(spi_device_handle_t spi, uint8_t cmd) {
  lcd_cmd(spi, cmd);

  spi_transaction_t t;
  memset(&t, 0, sizeof(t));
  t.length = 8*3;
  t.flags = SPI_TRANS_USE_RXDATA;
  t.user = (void *)1;
  esp_err_t ret;
  ret = spi_device_polling_transmit(spi, &t);
  assert(ret == ESP_OK);

  printf("%x: %x\n", cmd, *(uint32_t*)t.rx_data);
}

void app_main() {
  printf("Hello world!\n");
  // init spiffs
  esp_err_t ret = esp_vfs_spiffs_register(&(esp_vfs_spiffs_conf_t){
    .base_path = "/assets",
    .partition_label = NULL,
    .max_files = 5,
    .format_if_mount_failed = false
  });
  ESP_ERROR_CHECK(ret);
  struct stat st = {0};
  stat("/assets/font.otf", &st);
  printf("font size: %li\n", st.st_size);
  uint8_t *font = malloc(st.st_size);
  int64_t start = esp_timer_get_time();
  FILE *fd = fopen("/assets/font.otf", "rb");
  fread(font, 1, st.st_size, fd);
  fclose(fd);
  stbtt_fontinfo info;
  stbtt_InitFont(&info, font, stbtt_GetFontOffsetForIndex(font, 0));
  int64_t end = esp_timer_get_time() - start;
  printf("Time %lld (%lld millis)\n", end, end / 1000L);

  /* Print chip information */
  esp_chip_info_t chip_info;
  esp_chip_info(&chip_info);

  printf("This is ESP32 chip with %d CPU cores, WiFi%s%s, ", chip_info.cores,
         (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
         (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

  printf("silicon revision %d, ", chip_info.revision);

  printf("%uMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
         (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded"
                                                       : "external");
  printf("Starting LCD.\n");

  ret = spi_bus_initialize(HSPI_HOST, &(spi_bus_config_t){
    .miso_io_num = PIN_NUM_MISO,
    .mosi_io_num = PIN_NUM_MOSI,
    .sclk_io_num = PIN_NUM_SCLK,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
    .max_transfer_sz = 320 * 3 * 40,
    .flags = SPICOMMON_BUSFLAG_MASTER | SPICOMMON_BUSFLAG_SCLK |
             SPICOMMON_BUSFLAG_MISO | SPICOMMON_BUSFLAG_MOSI
  }, 1);
  ESP_ERROR_CHECK(ret);

  spi_device_handle_t lcd = {0};
  ret = spi_bus_add_device(HSPI_HOST, &(spi_device_interface_config_t){
    .clock_speed_hz = 40*1000*1000,
    .mode = 0,
    .spics_io_num = PIN_NUM_CS,
    .queue_size = 6,
    .pre_cb = lcd_set_dc,
    .flags = SPI_DEVICE_HALFDUPLEX
  }, &lcd);
  lcd_init(lcd);

  ESP_ERROR_CHECK(ret);
  printf("It worked.\n");

  /*
    We use six transactions here.
    CASET and PASET take two bytes each a high octet and a low octet to form a uint16_t.
    1. CASET - which columns to write to.
    2. CASET bytes(2)
    3. PASET - the page adress to start and end at.
    4. PASET bytes(2)
    5. RAMWR - sending data
    6. RAMWR line 480 pixels * 20 lines.
  */
  spi_transaction_t trans[6];

  size_t lines = 40;
  uint8_t *line = heap_caps_malloc(320 * lines * 3, MALLOC_CAP_DMA);
  for (int i = 0; i < 320 * lines * 3; i += 3) {
    line[i] = 0x00;
    line[i + 1] = 0x00;
    line[i + 2] = 0x00;
  }

  for (int i = 0; i < 6; i++) {
    memset(&trans[i], 0, sizeof(spi_transaction_t));
    if ((i & 1) == 0) {
      trans[i].user = (void *)0; // set command bit
      trans[i].length = 8;       // 8 bit command
    } else {
      trans[i].user = (void *)1; // set command bit
      trans[i].length = 8 * 4;   // 8 bit command
    }
    trans[i].flags = SPI_TRANS_USE_TXDATA; // we use data for most of this.
  }

  start = esp_timer_get_time();
  for (int y = 0; y < 480; y += lines) {
    trans[0].tx_data[0] = HX8357_CASET;       // Column Address Set
    trans[1].tx_data[0] = 0;                  // Start Col High
    trans[1].tx_data[1] = 0;                  // Start Col Low
    trans[1].tx_data[2] = 320 >> 8;           // End Col High
    trans[1].tx_data[3] = 320 & 0xff;         // End Col Low
    trans[2].tx_data[0] = HX8357_PASET;       // Page address set
    trans[3].tx_data[0] = y >> 8;             // Start page high
    trans[3].tx_data[1] = y & 0xff;           // start page low
    trans[3].tx_data[2] = (y + lines) >> 8;   // end page high
    trans[3].tx_data[3] = (y + lines) & 0xff; // end page low
    trans[4].tx_data[0] = HX8357_RAMWR;       // memory write
    trans[5].tx_buffer = line;                // finally send the line data
    trans[5].length = 320 * lines * 3 * 8;    // Data length, in bits
    trans[5].flags = 0; // undo SPI_TRANS_USE_TXDATA
                        // flag
    for (int x = 0; x < 6; x++) {
      ret = spi_device_queue_trans(lcd, &trans[x], portMAX_DELAY);
      ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    }
    spi_transaction_t *spi;
    for (int x = 0; x < 6; x++) {
      ret = spi_device_get_trans_result(lcd, &spi, portMAX_DELAY);
      ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    }
  }
  free(line);
  end = esp_timer_get_time() - start;
  printf("Time %lld (%lld millis)\n", end, end / 1000L);

  start = esp_timer_get_time();
  int ascent = 0;
  float scale = stbtt_ScaleForPixelHeight(&info, 40);
  stbtt_GetFontVMetrics(&info, &ascent, 0, 0);
  int baseline = ascent * scale;
  uint8_t screen[40][40];
  memset(screen, 0, sizeof(uint8_t));
  int x0, y0, x1, y1;
  const char *hello = "H";
  stbtt_GetCodepointBitmapBoxSubpixel(
    &info, hello[0], scale, scale,
    0, 0, &x0, &y0, &x1, &y1
  );

  stbtt_MakeCodepointBitmapSubpixel(
    &info, &screen[0][0],
    40, 40, 40, scale, scale, 0, 0, hello[0]
  );
  uint8_t *pixels = heap_caps_malloc(40 * 40 * 3, MALLOC_CAP_DMA);
  for (int y = 0; y < 40; y++) {
    for (int x = 0; x < 40 * 3; x += 3) {
      pixels[y + x] = screen[y][x / 3];
      pixels[y + x + 1] = screen[y][x / 3];
      pixels[y + x + 2] = screen[y][x / 3];
    }
  }

  trans[0].tx_data[0] = HX8357_CASET;       // Column Address Set
  trans[1].tx_data[0] = (320 / 2 - 20) >> 8;            // Start Col High
  trans[1].tx_data[1] = (320 / 2 - 20) & 0xff;         // Start Col Low
  trans[1].tx_data[2] = (320 / 2 + 20) >> 8;           // End Col High
  trans[1].tx_data[3] = (320 / 2 + 20) & 0xff;         // End Col Low
  trans[2].tx_data[0] = HX8357_PASET;       // Page address set
  trans[3].tx_data[0] = (480 / 2 - 20) >> 8;             // Start page high
  trans[3].tx_data[1] = (480 / 2 - 20) & 0xff;           // start page low
  trans[3].tx_data[2] = (480 / 2 + 20) >> 8;   // end page high
  trans[3].tx_data[3] = (480 / 2 + 20) & 0xff; // end page low
  trans[4].tx_data[0] = HX8357_RAMWR;       // memory write
  trans[5].tx_buffer = pixels;                // finally send the line data
  trans[5].length = 40 * 40 * 3 * 8;    // Data length, in bits
  trans[5].flags = 0; // undo SPI_TRANS_USE_TXDATA
                        // flag

  for (int x = 0; x < 6; x++) {
    ret = spi_device_queue_trans(lcd, &trans[x], portMAX_DELAY);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
  }
  spi_transaction_t *spi;
  for (int x = 0; x < 6; x++) {
    ret = spi_device_get_trans_result(lcd, &spi, portMAX_DELAY);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
  }
  end = esp_timer_get_time() - start;
  printf("Rendered in %lld (%lld millis)\n", end, end / 1000);
  for (int j=0; j < 20; ++j) {
    for (int i=0; i < 20; ++i)
      putchar(" .:ioVM@"[screen[j][i]>>5]);
    putchar('\n');
  }
  free(pixels);
  free(font);

  for (int i = 10; i >= 0; i--) {
    printf("Restarting in %d seconds...\n", i);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
  printf("Restarting now.\n");
  fflush(stdout);
  esp_vfs_spiffs_unregister(NULL);
  esp_restart();
}
