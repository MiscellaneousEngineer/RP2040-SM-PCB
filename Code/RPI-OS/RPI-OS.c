//This entire superloop is being refactored into a proper OS. Currently broken

#include <stdio.h>
#include "pico/stdlib.h"
#include "ST7735_TFT.h"
#include "hardware/spi.h"
#include "hw.h"
#include "hardware/gpio.h"
#include "hardware/xosc.h"
#include "hardware/structs/scb.h"
#include "hardware/watchdog.h"
#include "hardware/rtc.h"
#include "pico/util/datetime.h"
#include <math.h>
#include "ui/widgets.h"
#include "minmea.h"
#include "hardware/uart.h"
#include "hal/board.h"

// ── Timekeeping ───────────────────────────────────────────────────────────────
static bool g_time_valid = false;

void timekeeping_init(void)
{
    rtc_init();
    datetime_t t = {.year = 2025, .month = 1, .day = 1, .dotw = 3, .hour = 0, .min = 0, .sec = 0};
    rtc_set_datetime(&t);
    sleep_us(64);
}

void timekeeping_sync_from_gps(int hours, int minutes, int seconds)
{
    // Subtract 1s to compensate for the SDK's known RTC false-tick
    if (--seconds < 0)
    {
        seconds = 59;
        if (--minutes < 0)
        {
            minutes = 59;
            if (--hours < 0)
                hours = 23;
        }
    }
    datetime_t t = {.year = 2025, .month = 1, .day = 1, .dotw = 3, .hour = hours, .min = minutes, .sec = seconds};
    rtc_set_datetime(&t);
    sleep_us(64);
    g_time_valid = true;
}

bool timekeeping_get(int *hours, int *minutes, int *seconds)
{
    datetime_t t;
    if (!rtc_get_datetime(&t))
        return false;
    *hours = t.hour;
    *minutes = t.min;
    *seconds = t.sec;
    return true;
}

void init_hw()
{
    stdio_init_all();
    spi_init(SPI_PORT, 10000000);
    gpio_set_function(SPI_RX, GPIO_FUNC_SPI);
    gpio_set_function(SPI_SCK, GPIO_FUNC_SPI);
    gpio_set_function(SPI_TX, GPIO_FUNC_SPI);
    tft_spi_init();
}

void dormant(uint16_t WAKEPIN)
{
    gpio_put(10, 0);

    sleep_ms(1000);

    gpio_acknowledge_irq(WAKEPIN, GPIO_IRQ_EDGE_RISE);

    gpio_set_dormant_irq_enabled(
        WAKEPIN,
        GPIO_IRQ_EDGE_RISE,
        true
    );

    xosc_dormant();

    // execution resumes here after wake

    //gpio_acknowledge_irq(WAKEPIN, GPIO_IRQ_EDGE_RISE);

  /*  gpio_set_dormant_irq_enabled(
        WAKEPIN,
        GPIO_IRQ_EDGE_RISE,
        false
    );*/
}

bool read_nmea_line(char *buf, size_t max_len)
{
    static size_t pos = 0;
    while (uart_is_readable(GPS_UART))
    {
        char c = uart_getc(GPS_UART);
        if (c == '\n')
        {
            buf[pos] = '\0';
            pos = 0;
            return true;
        }
        else if (c != '\r')
        {
            if (pos < max_len - 1)
                buf[pos++] = c;
            else
                pos = 0;
        }
    }
    return false;
}

bool parse_gps_time(const char *line, int *hours, int *minutes, int *seconds)
{
    if (minmea_sentence_id(line, false) != MINMEA_SENTENCE_RMC)
        return false;

    struct minmea_sentence_rmc frame;
    if (!minmea_parse_rmc(&frame, line) || !frame.valid)
        return false;

    *hours = frame.time.hours;
    *minutes = frame.time.minutes;
    *seconds = frame.time.seconds;
    return true;
}

bool parse_gps_satellites(const char *line, int *satellites)
{
    if (minmea_sentence_id(line, false) != MINMEA_SENTENCE_GSV)
        return false;
    struct minmea_sentence_gsv frame;
    if (!minmea_parse_gsv(&frame, line))
        return false;
    *satellites = frame.total_sats;
    return true;
}

int main()
{
    gpio_init(S2);
    gpio_set_dir(S2, GPIO_IN);
    gpio_pull_down(S2);
    gpio_init(S4);
    gpio_set_dir(S4, GPIO_IN);

    stdio_init_all();
    init_hw();
    timekeeping_init(); //RTC keeps time while watch is awake to compensate for when gps signal may drop out

    TFT_GreenTab_Initialize();
    gpio_init(10);
    gpio_set_dir(10, 1);
    gpio_pull_down(10);
    gpio_put(10, 1);
    fillScreen(ST7735_BLACK);

    uart_init(GPS_UART, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    while (true)
    {
        gpio_put(10, 1);
        //DrawThickRect(1, 1, 126, 158, 4, ST7735_BLUE);

        char line[160];

        if (read_nmea_line(line, sizeof(line)))
        {
            //drawText(10, 10, line, ST7735_WHITE, ST7735_BLACK, 1);
            //printf(line);
            int h, m, s;

            if (parse_gps_time(line, &h, &m, &s))
            {
                char buffer[32];
                snprintf(buffer, sizeof(buffer), "Time: %02d:%02d:%02d", h, m, s);

                //drawText(5, 120, buffer, ST7735_WHITE, ST7735_BLACK, 1);

                timekeeping_sync_from_gps(h, m, s);
            }

            int sats;
            if (parse_gps_satellites(line, &sats))
            {
                char buffer[32];
                snprintf(buffer, sizeof(buffer), "Sats: %02d", sats);
                drawText(35, 90, buffer, ST7735_WHITE, ST7735_BLACK, 1);
                
            }
        }

        // Always display time from RTC 
        {
            int h, m, s;
            if (timekeeping_get(&h, &m, &s))
            {
                char buffer[32];
                // '~' = free-running, not yet GPS-locked
                snprintf(buffer, sizeof(buffer), "%s%02d:%02d",
                         g_time_valid ? "" : "~", h+2, m);
                drawText(35, 73, buffer, ST7735_WHITE, ST7735_BLACK, 2);
                float cpercent = (float)s / 60.0f;
                if(s == 0){
                    DrawCirclePartial(64,80,60,4,1,ST7735_BLUE);
                }
                
                DrawCirclePartial(64,80,60,4,cpercent,ST7735_BLUE);
                //DrawCirclePartial(64,80,54,4,0.8,ST7735_GREEN);
            }
        }

        if (gpio_get(S2))
        {
            fillScreen(ST7735_BLACK);
            fillRect(0, 0, 5, 10, ST7735_RED);
            for (uint16_t i = 100; i != 0; i--)
            {
                char buffer[16];
                snprintf(buffer, sizeof(buffer), "%d", i);
                drawText(5, 0, buffer, ST7735_WHITE, ST7735_BLACK, 1);
                if (!gpio_get(S2))
                    break;
            }
            if (gpio_get(S2))
            {
                fillScreen(ST7735_BLACK);
                dormant(S2); 
            }
        }

        if (gpio_get(S4))
        {
            fillScreen(ST7735_BLACK);
            fillRect(123, 0, 5, 10, ST7735_RED);
            for (uint16_t i = 100; i != 0; i--)
            {
                char buffer[16];
                snprintf(buffer, sizeof(buffer), "%d", i);
                drawText(100, 0, buffer, ST7735_WHITE, ST7735_BLACK, 1);
                if (!gpio_get(S4))
                    break;
            }
            if (gpio_get(S4))
            {
                fillScreen(ST7735_BLACK);
                gpio_put(10, 0);
                watchdog_reboot(0, 0, 1);
            }
        }
    }
}