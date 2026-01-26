#ifndef STATUS_LED_H
#define STATUS_LED_H

#include <cstdint>

/**
 * @enum LEDState
 * @brief LED state enumeration
 */
enum class LEDState {
    OFF,      // LED off
    ON,       // LED solid on
    SLOW,     // Slow blink (1 Hz)
    FAST,     // Fast blink (5 Hz)
    PULSE     // Pulsing effect
};

/**
 * @class StatusLED
 * @brief Status LED controller for visual feedback
 * 
 * Controls LEDs to indicate system status:
 * - GPS Fix LED: Shows GPS fix quality
 * - ROS Data LED: Shows ROS2 message reception
 */
class StatusLED {
public:
    /**
     * @brief Constructor
     * @param pin GPIO pin number for LED
     */
    StatusLED(uint32_t pin);
    
    /**
     * @brief Destructor
     */
    ~StatusLED();

    /**
     * @brief Initialize LED
     * @return true if successful
     */
    bool init();

    /**
     * @brief Set LED state
     * @param state Desired LED state
     */
    void setState(LEDState state);

    /**
     * @brief Update LED (call regularly for blinking effects)
     * Should be called in main loop
     */
    void update();

    /**
     * @brief Turn LED on
     */
    void on();

    /**
     * @brief Turn LED off
     */
    void off();

    /**
     * @brief Toggle LED state
     */
    void toggle();

private:
    uint32_t pin_;
    LEDState state_;
    bool current_level_;
    uint64_t last_update_time_;
    
    /**
     * @brief Get blink period for current state (microseconds)
     * @return Blink period in microseconds
     */
    uint32_t getBlinkPeriod();
    
    /**
     * @brief Set physical LED level
     * @param level true=on, false=off
     */
    void setLevel(bool level);
};

#endif // STATUS_LED_H
