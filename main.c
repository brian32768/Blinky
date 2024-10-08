#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/watchdog.h"
#include "hardware/uart.h"

#include "minmea.h"

// SPI Defines
// We are going to use SPI 0, and allocate it to the following GPIO pins
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define SPI_PORT spi0
#define PIN_MISO 16
#define PIN_CS   17
#define PIN_SCK  18
#define PIN_MOSI 19

// UART 0 is the GPS receiver
// UART 1 is the Nextion LCD 
// The USB port requires no special set up here;
// "console on USB" is specified in the CmakeLists.txt file

#define GPS_UART_ID uart0
#define GPS_BPS 9600
#define GPS_UART_TX_PIN 0 // GP0
#define GPS_UART_RX_PIN 1 // GP1

#define NEXTION_UART_ID uart1
#define NEXTION_BPS 9600
#define NEXTION_UART_TX_PIN 8 // GP8
#define NEXTION_UART_RX_PIN 9 // GP9


#define LED_PIN PICO_DEFAULT_LED_PIN
unsigned int sentences_rxed = 0;

void parse(char *line) {
    
    char talker[3];
    minmea_talker_id(talker, line);
    
    int id = (int)minmea_sentence_id(line, false);
    switch (id) {
        default:
            printf("?? %s\n", line);
            break;

        case MINMEA_SENTENCE_GLL: {
            struct minmea_sentence_gll frame;
            if (minmea_parse_gll(&frame, line)) {
                printf("$GLL: raw coordinates, time, status, mode: (%d/%d,%d/%d) %d:%d:%d.%d %c %c\n",
                        frame.latitude.value, frame.latitude.scale, frame.longitude.value, frame.longitude.scale,
                        frame.time.hours, frame.time.minutes, frame.time.seconds, frame.time.microseconds,
                        frame.status, frame.mode);
                printf("$GLL fixed-point coordinates: (%d,%d)\n",
                        minmea_rescale(&frame.latitude, 1000),
                        minmea_rescale(&frame.longitude, 1000));
                printf("$GLL floating point degree coordinates: (%f,%f)\n",
                        minmea_tocoord(&frame.latitude),
                        minmea_tocoord(&frame.longitude));
            }
        } break;
            
        case MINMEA_SENTENCE_GSA: {
            // $BDGSA,A,3,11,12,14,21,34,42,43,44,,,,,2.0,1.1,1.7*24
            // $GPGSA,A,3,08,10,23,27,32,,,,,,,,2.0,1.1,1.7*3A
            struct minmea_sentence_gsa frame;
            if (minmea_parse_gsa(&frame, line)) {
                printf("$%sGSA mode %c, fix %d, (pdop: %d/%d, hdop: %d/%d, vdop: %d/%d)", 
                    talker, 
                    frame.mode, frame.fix_type, 
                    frame.pdop.value, frame.pdop.scale, 
                    frame.hdop.value, frame.hdop.scale, 
                    frame.vdop.value, frame.vdop.scale
                );
                for (int i = 0; i < 12; i++) {
                    int sat = frame.sats[i];
                    if (sat) printf(" %d", sat);
                }
                printf("\n");
            }
        } break;
            
        case MINMEA_SENTENCE_RMC: {
            static float current_lat = 0;
            static float current_lon = 0;
            struct minmea_sentence_rmc frame;
            if (minmea_parse_rmc(&frame, line)) {
                float lat = minmea_tocoord(&frame.latitude);
                float lon = minmea_tocoord(&frame.longitude);
                if (lat != current_lat || lon != current_lon) {
    /*                printf("$RMC raw: (%d/%d,%d/%d) %d/%d\n",
                            frame.latitude.value, frame.latitude.scale,
                            frame.longitude.value, frame.longitude.scale,
                            frame.speed.value, frame.speed.scale);
                    printf("     fixed-point, 3 places: (%d,%d) %d\n",
                            minmea_rescale(&frame.latitude, 1000),
                            minmea_rescale(&frame.longitude, 1000),
                            minmea_rescale(&frame.speed, 1000));
    */
                    float knots = minmea_tofloat(&frame.speed);
                    float mph = knots * 1.150779;
                    float course = minmea_tofloat(&frame.course);
                    printf("$RMC: (%f,%f) course: %f %f speed: %f mph\n", lat,lon, course, mph);
                    current_lat = lat;
                    current_lon = lon;
                }
            }
        } break;

        case MINMEA_SENTENCE_GGA: {
            struct minmea_sentence_gga frame;
            char *s;
            if (minmea_parse_gga(&frame, line)) {
                switch(frame.fix_quality) { 
                case 0:
                    s = "NO FIX";
                    break;
                case 1:
                    s = "FIX";
                    break;
                case 2:
                    s = "DGPS FIX";
                    break;
                default:
                    s = "OTHER";
                    break;
                }
                printf("$%sGGA: fix %d: %s, sats tracked: %d\n", 
                    talker, frame.fix_quality, s, frame.satellites_tracked);
            }
        } break;

        case MINMEA_SENTENCE_GSV: {
            struct minmea_sentence_gsv frame;
            if (minmea_parse_gsv(&frame, line)) {
                printf("$%sGSV: message %d / %d in view %d\n", 
                    talker, 
                    frame.msg_nr, frame.total_msgs, frame.total_sats);
                for (int i = 0; i < 4; i++)
                    if (frame.sats[i].nr) 
                        printf(" sat nr %d, ele: %d, az: %d, snr: %d dbm\n",
                            frame.sats[i].nr,
                            frame.sats[i].elevation,
                            frame.sats[i].azimuth,
                            frame.sats[i].snr);
            }
        } break;
    }
}

static char sentence[MINMEA_MAX_SENTENCE_LENGTH];
static bool nmea_sentence_ready = false;

// RX interrupt handler collects sentence from GPS
void on_gps_rx() {
    static int inx = 0;
    while (uart_is_readable(GPS_UART_ID)) {
        uint8_t ch = uart_getc(GPS_UART_ID);

        if (inx >= sizeof sentence) {
            printf("Buffer overflow.\n");
            inx = 0;
        }

        if (ch == '\r') continue; // ignore line terminators

        if (ch == '\n') {
            gpio_put (LED_PIN, 0); // turn off led
            inx = 0;
            parse(sentence);
            nmea_sentence_ready = true;
        } else {
            gpio_put (LED_PIN, 1); // turn on led
            sentence[inx++] = ch;
            sentence[inx] = 0; // string is always terminated
            sentences_rxed++;
        }
    }
}

int main()
{
    stdio_init_all();

    // Enable the watchdog, requiring the watchdog to be updated periodically or the chip will reboot
    // second arg is pause on debug which means the watchdog will pause when stepping through code
    watchdog_enable(1000, 1);

    // SPI initialisation. This example will use SPI at 1MHz.
    spi_init(SPI_PORT, 1000*1000);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS,   GPIO_FUNC_SIO);
    gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);
    // For more examples of SPI use see https://github.com/raspberrypi/pico-examples/tree/master/spi

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    // ---- UART set up ----

    // For more examples of UART use see https://github.com/raspberrypi/pico-examples/tree/master/uart

    // Set up UART for the GPS receiver
    uart_init(GPS_UART_ID, GPS_BPS);
    gpio_set_function(GPS_UART_TX_PIN, UART_FUNCSEL_NUM(GPS_UART_ID, GPS_UART_TX_PIN));
    gpio_set_function(GPS_UART_RX_PIN, UART_FUNCSEL_NUM(GPS_UART_ID, GPS_UART_RX_PIN));
    uart_set_hw_flow(GPS_UART_ID, false, false); // No CTS or RTS
    uart_set_format(GPS_UART_ID, 8,1,UART_PARITY_NONE);
    uart_set_fifo_enabled(GPS_UART_ID, false);

    // Set up UART for the Nextion LCD receiver
    // For more examples of UART use see https://github.com/raspberrypi/pico-examples/tree/master/uart
    uart_init(NEXTION_UART_ID, NEXTION_BPS);
    gpio_set_function(NEXTION_UART_TX_PIN, UART_FUNCSEL_NUM(NEXTION_UART_ID, NEXTION_UART_TX_PIN));
    gpio_set_function(NEXTION_UART_RX_PIN, UART_FUNCSEL_NUM(NEXTION_UART_ID, NEXTION_UART_RX_PIN));
    uart_set_hw_flow(NEXTION_UART_ID, false, false); // No CTS or RTS
    uart_set_format(NEXTION_UART_ID, 8,1,UART_PARITY_NONE);
    uart_set_fifo_enabled(NEXTION_UART_ID, true);

    // Set up interrupt for GPS receiver
    int UART_IRQ = (GPS_UART_ID == uart0) ? UART0_IRQ : UART1_IRQ;
    irq_set_exclusive_handler(UART_IRQ, on_gps_rx);
    irq_set_enabled(UART_IRQ, true);

    // Now enable the UART to send interrupts - RX only
    uart_set_irq_enables(GPS_UART_ID, true, false);

    gpio_put (LED_PIN, 1); // turn on led

    if (watchdog_caused_reboot()) {
        printf("Rebooted by Watchdog!\n");
    }

    while (true) {
        static int rxed = 0;
        watchdog_update();
//        if (nmea_sentence_ready) {
//            parse(sentence);
//            nmea_sentence_ready = false;
//        }
        if (sentences_rxed != rxed) {
            rxed = sentences_rxed;
            if (uart_is_writable(NEXTION_UART_ID)) {
                uart_puts(NEXTION_UART_ID, sentence);
                uart_puts(NEXTION_UART_ID, "\r\n");
            }
            //printf("%d\n", sentences_rxed);
        }
        sleep_ms(200);
    }
}
