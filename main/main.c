#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "soc/gpio_struct.h"
#include "esp_spi_flash.h"
#include "esp_spiffs.h"
#include "driver/gpio.h"
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

#define MADCTL_MY  0x80 ///< Bottom to top
#define MADCTL_MX  0x40 ///< Right to left
#define MADCTL_MV  0x20 ///< Reverse Mode
#define MADCTL_ML  0x10 ///< LCD refresh Bottom to top
#define MADCTL_RGB 0x00 ///< Red-Green-Blue pixel order
#define MADCTL_BGR 0x08 ///< Blue-Green-Red pixel order
#define MADCTL_MH  0x04 ///< LCD refresh right to left

typedef struct {
  uint8_t cmd;
  uint8_t bytes; //No of data in data; bit 7 = delay after set; 0xFF = end of cmds.
  uint8_t data[34];
} lcd_init_cmd_t;

static const lcd_init_cmd_t init_cmds[] = {
    {HX8357_SWRESET, 0x80, {0}}, // Soft reset, then delay 10 ms
    {HX8357_SLPOUT, 0x80, {0}},  // Exit Sleep, then delay 50 ms
    //{HX8357_MADCTL, 1, {MADCTL_MX | MADCTL_MY | MADCTL_RGB}},
    // TODO: Not sure why this doesn't work
    {HX8357_COLMOD, 1, {0x55}}, // 16 bit
    {HX8357_DISPON, 0x80, {0}}, // Main screen turn on, delay 50 ms
    {0xff, 0, {0}},             // END OF COMMAND LIST
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
  ret = spi_device_transmit(spi, &t);
  assert(ret == ESP_OK);
}

void lcd_data(spi_device_handle_t spi, const uint8_t *data, size_t length) {
  if (length == 0) return;
  esp_err_t ret;
  spi_transaction_t t = {0};
  t.length = length * 8; // size in bits
  t.tx_buffer = data;
  ret = spi_device_transmit(spi, &t);
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
  t.length = 8*4;
  t.flags = SPI_TRANS_USE_RXDATA;
  t.user = (void *)0;
  esp_err_t ret;
  ret = spi_device_transmit(spi, &t);
  assert(ret == ESP_OK);
  t.user = (void *)1;
  ret = spi_device_transmit(spi, &t);
  assert(ret == ESP_OK);
  printf("%x: %x %x %x %x\n", cmd, t.rx_data[0], t.rx_data[1], t.rx_data[2], t.rx_data[3]);
}

// Queue a list of transactions
void lcd_write(spi_device_handle_t spi, spi_transaction_t *trans, size_t length) {
  for (int x = 0; x < length; x++) {
    esp_err_t ret = spi_device_queue_trans(spi, &trans[x], portMAX_DELAY);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
  }
  spi_transaction_t *ret_trans;
  for (int x = 0; x < length; x++) {
    esp_err_t ret = spi_device_get_trans_result(spi, &ret_trans, portMAX_DELAY);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
  }
}

// Set the window
void set_window(spi_device_handle_t spi, uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
  if(x > 320) return;
  if(y > 480) return;

  uint16_t x1 = fmin(320, x + w) - 1;
  uint16_t y1 = fmin(480, y + h) - 1;

  spi_transaction_t trans[4];
  for (int i = 0; i < 4; i++) {
    memset(&trans[i], 0, sizeof(spi_transaction_t));
    if ((i & 1) == 0) {
      trans[i].user = (void *)0; // set command bit
      trans[i].length = 8;       // 8 bit command
    } else {
      trans[i].user = (void *)1; // set command bit to data
      trans[i].length = 8 * 4;   // 8 bit command
    }
    trans[i].flags = SPI_TRANS_USE_TXDATA;
  }

  trans[0].tx_data[0] = HX8357_CASET; // Column Address Set
  trans[1].tx_data[0] = x >> 8;       // Start Col High
  trans[1].tx_data[1] = x & 0xff;     // Start Col Low
  trans[1].tx_data[2] = x1 >> 8;      // End Col High
  trans[1].tx_data[3] = x1 & 0xff;    // End Col Low
  trans[2].tx_data[0] = HX8357_PASET; // Page address set
  trans[3].tx_data[0] = y >> 8;       // Start page high
  trans[3].tx_data[1] = y & 0xff;     // start page low
  trans[3].tx_data[2] = y1 >> 8;      // end page high
  trans[3].tx_data[3] = y1 & 0xff;    // end page low
  lcd_write(spi, trans, 4);
}

void write_bitmap(spi_device_handle_t spi, uint8_t *data, uint16_t x,
                  uint16_t y, uint16_t w, uint16_t h) {
  set_window(spi, x, y, w, h);
  spi_transaction_t trans[2] = {
    {
     .user = (void *)0,
     .length = 8,
     .tx_data = {HX8357_RAMWR},
     .flags = SPI_TRANS_USE_TXDATA
    },
    {
      .user = (void*) 1,
      .length = w * h * 3 * 8,
      .tx_buffer = data
    }
  };
  lcd_write(spi, trans, 2);
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
  stat("/assets/font.ttf", &st);
  printf("font size: %li\n", st.st_size);
  uint8_t *font = malloc(st.st_size);
  int64_t start = esp_timer_get_time();
  FILE *fd = fopen("/assets/font.ttf", "rb");
  fread(font, 1, st.st_size, fd);
  fclose(fd);
  stbtt_fontinfo info;
  stbtt_InitFont(&info, font, stbtt_GetFontOffsetForIndex(font, 0));
  int64_t end = esp_timer_get_time() - start;
  printf("Time to load fonts %lld (%lld millis)\n", end, end / 1000L);
  heap_caps_print_heap_info(MALLOC_CAP_DMA);
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
    .max_transfer_sz = 320 * 3 * 480
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

  size_t lines = 40;
  uint8_t *line = heap_caps_malloc(320 * lines * 3, MALLOC_CAP_DMA);
  for (int i = 0; i < 320 * lines * 3; i += 3) {
    line[i] = 0x00;
    line[i + 1] = 0x00;
    line[i + 2] = 0x00;
  }

  start = esp_timer_get_time();
  for (int y = 0; y < 480; y += lines) {
    write_bitmap(lcd, line, 0, y, 320, lines);
  }
  free(line);
  end = esp_timer_get_time() - start;
  printf("Time to blank %lld (%lld millis)\n", end, end / 1000L);

  int ascent = 0;
  float scale = stbtt_ScaleForPixelHeight(&info, 50);
  stbtt_GetFontVMetrics(&info, &ascent, 0, 0);
  int baseline = (int) (ascent * scale);
  const char *hello = "Phone";
  size_t w = 0, h = 0;
  start = esp_timer_get_time();
  // TODO: this repeats work let's fix that.
  for (int i = 0; i < 5; i++) {
    int advance, lsb, x0, y0, x1, y1;
    stbtt_GetCodepointHMetrics(&info, hello[i], &advance, &lsb);
    float shift = w - (float) floor(w);
    stbtt_GetCodepointBitmapBoxSubpixel(&info, hello[i], scale, scale, shift, 0, &x0, &y0, &x1, &y1);
    h = fmax(h, baseline + (y1 - y0));
    w += (advance * scale);
    if (hello[i + 1])
      w += scale*stbtt_GetCodepointKernAdvance(&info, hello[i], hello[i+1]);
  }

  uint8_t *pixels = heap_caps_calloc(w * h * 3, sizeof(uint8_t), MALLOC_CAP_DMA);
  int xoff = 0;
  for (int j = 0; j < 5; j++) {
    int advance, lsb, x0, y0, x1, y1;
    stbtt_GetCodepointHMetrics(&info, hello[j], &advance, &lsb);
    float shift = xoff - (float) floor(xoff);
    stbtt_GetCodepointBitmapBoxSubpixel(&info, hello[j], scale, scale, shift, 0, &x0, &y0, &x1, &y1);

    int wb, hb, xb, yb;
    start = esp_timer_get_time();
    uint8_t *screen = stbtt_GetCodepointBitmapSubpixel(
      &info, scale, scale, 0, 0, hello[j], &wb, &hb, &xb, &yb
    );

    for (int y = 0; y < hb / 2; y++) {
      for (int x = 0; x < wb; x++) {
        float alpha = (float) screen[y * wb + x] / 255.0;
        float blend =  alpha * (float) screen[y * wb + x];
        for (int c = 0; c < 3; c++) {
          size_t off = (y + baseline + y0) * w * 3 + (xoff + x0 + x) * 3 + c;
          pixels[off] = (uint8_t) fmin(255, blend + (1 - alpha) * (float) pixels[off]);
        }
      }
    }
    free(screen);
    xoff += (advance * scale);
    if (hello[j + 1])
      xoff += scale*stbtt_GetCodepointKernAdvance(&info, hello[j], hello[j+1]);
  }

  end = esp_timer_get_time() - start;
  printf("layout in %lld (%lld millis)\n", end, end / 1000);
  uint8_t *mask = heap_caps_calloc(w * 3, sizeof(uint8_t), MALLOC_CAP_DMA);
  for (int y_int = 0; y_int < 480 - h; y_int += 1) {
    start = esp_timer_get_time();
    if (y_int > 1) write_bitmap(lcd, mask, 320 / 2 - w / 2, y_int - 1, w, 1);
    write_bitmap(lcd, pixels, 320 / 2 - w / 2, y_int, w, h);
    end = esp_timer_get_time() - start;
    if(y_int == 10) printf("render in %lld (%lld millis)\n", end, end / 1000);
  }
  free(mask);
  free(pixels);
  free(font);
  while(1);
  printf("Restarting now.\n");
  fflush(stdout);
  esp_vfs_spiffs_unregister(NULL);
  esp_restart();
}
