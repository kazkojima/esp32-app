// RGB led

#define GPIO_LED_RED   (CONFIG_GPIO_LED_RED)
#define GPIO_LED_GREEN (CONFIG_GPIO_LED_GREEN)
#define GPIO_LED_BLUE  (CONFIG_GPIO_LED_BLUE)

#define RGBLED_ON      (CONFIG_RGBLED_ON)
#define RGBLED_OFF     (1-CONFIG_RGBLED_ON)

extern volatile uint8_t rgb_led_red;
extern volatile uint8_t rgb_led_green;
extern volatile uint8_t rgb_led_blue;
