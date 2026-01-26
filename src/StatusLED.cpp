#include "../include/StatusLED.h"
#include "pico/stdlib.h"
#include <cmath>

StatusLED::StatusLED(uint32_t pin) 
    : pin_(pin), 
      state_(LEDState::OFF), 
      current_level_(false),
      last_update_time_(0) {
}

StatusLED::~StatusLED() {
    off();
}

bool StatusLED::init() {
    gpio_init(pin_);
    gpio_set_dir(pin_, GPIO_OUT);
    off();
    return true;
}

void StatusLED::setState(LEDState state) {
    state_ = state;
    last_update_time_ = time_us_64();
    
    if (state == LEDState::OFF) {
        off();
    } else if (state == LEDState::ON) {
        on();
    }
}

void StatusLED::update() {
    uint64_t current_time = time_us_64();
    
    switch (state_) {
        case LEDState::OFF:
            if (current_level_) {
                off();
            }
            break;
            
        case LEDState::ON:
            if (!current_level_) {
                on();
            }
            break;
            
        case LEDState::SLOW:
        case LEDState::FAST: {
            uint32_t period = getBlinkPeriod();
            if (current_time - last_update_time_ >= period) {
                toggle();
                last_update_time_ = current_time;
            }
            break;
        }
        
        case LEDState::PULSE: {
            // Pulsing effect using sine wave
            uint32_t period = 2000000; // 2 second period
            double phase = (double)((current_time / 1000) % period) / (double)period;
            double brightness = (std::sin(phase * 2.0 * 3.14159265) + 1.0) / 2.0;
            
            // Simple threshold for on/off (could use PWM for smooth pulsing)
            if (brightness > 0.5 && !current_level_) {
                on();
            } else if (brightness <= 0.5 && current_level_) {
                off();
            }
            break;
        }
    }
}

void StatusLED::on() {
    setLevel(true);
}

void StatusLED::off() {
    setLevel(false);
}

void StatusLED::toggle() {
    setLevel(!current_level_);
}

uint32_t StatusLED::getBlinkPeriod() {
    switch (state_) {
        case LEDState::SLOW:
            return 500000; // 500ms (1 Hz)
        case LEDState::FAST:
            return 100000; // 100ms (5 Hz)
        default:
            return 500000;
    }
}

void StatusLED::setLevel(bool level) {
    current_level_ = level;
    gpio_put(pin_, level ? 1 : 0);
}
