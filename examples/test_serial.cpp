#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pico/stdlib.h"
#include "../include/USBProtocol.h"

const uint LED_BLUE = 20;
const uint LED_GREEN = 21;

// Receive target data from USB
int usb_receive_target_data(USBTargetData* data) {
    int bytes_read = 0;
    uint8_t* data_ptr = (uint8_t*)data;
    
    // Read sizeof(USBTargetData) bytes
    while (bytes_read < (int)sizeof(USBTargetData)) {
        int c = getchar_timeout_us(100000);  // 100ms timeout per byte
        if (c == PICO_ERROR_TIMEOUT) {
            if (bytes_read == 0) {
                return -1;  // No data available
            }
            return -2;  // Incomplete data
        }
        
        data_ptr[bytes_read] = (uint8_t)c;
        bytes_read++;
    }
    
    return bytes_read;
}

// Send ACK back over USB
void usb_send_ack() {
    uint8_t ack = USB_MSG_ACK;
    putchar(ack);
}

// Send error back over USB
void usb_send_error(uint8_t error_code) {
    uint8_t response[2] = {USB_MSG_ERROR, error_code};
    putchar(response[0]);
    putchar(response[1]);
}

int main()
{
    // Initialize USB stdio
    stdio_init_all();
    sleep_ms(2000);  // Wait for USB to be ready
    
    // Setup LEDs
    gpio_init(LED_BLUE);
    gpio_set_dir(LED_BLUE, GPIO_OUT);
    gpio_init(LED_GREEN);
    gpio_set_dir(LED_GREEN, GPIO_OUT);
    
    // Ready indicator
    printf("\n=== USB Target Data Receiver ===\n");
    printf("Waiting for target data...\n");
    
    USBTargetData target;
    
    while (true) {
        // Try to receive target data
        int result = usb_receive_target_data(&target);
        
        if (result == -1) {
            // No data available, just wait
            sleep_ms(100);
            continue;
        } else if (result == -2) {
            // Incomplete data
            printf("ERROR: Incomplete data received\n");
            usb_send_error(0x01);
            gpio_put(LED_GREEN, 1);
            sleep_ms(100);
            gpio_put(LED_GREEN, 0);
            continue;
        }
        
        // Validate checksum
        if (!usb_validate_target_data(&target)) {
            printf("ERROR: Checksum validation failed\n");
            usb_send_error(0x02);
            gpio_put(LED_GREEN, 1);
            sleep_ms(200);
            gpio_put(LED_GREEN, 0);
            continue;
        }
        
        // Check message type
        if (target.msg_type != USB_MSG_TARGET_DATA) {
            printf("ERROR: Invalid message type: 0x%02x\n", target.msg_type);
            usb_send_error(0x03);
            continue;
        }
        
        // Success! Print received data
        printf("✓ TARGET DATA RECEIVED:\n");
        printf("  Latitude:  %.6f°\n", target.latitude);
        printf("  Longitude: %.6f°\n", target.longitude);
        printf("  Altitude:  %.2f m\n", target.altitude);
        printf("\n");
        
        // Send ACK
        usb_send_ack();
        
        // Blue LED blink = success
        gpio_put(LED_BLUE, 1);
        sleep_ms(50);
        gpio_put(LED_BLUE, 0);
    }

    return 0;
}
