# All data is binary such that: GPIO 0A pin #X = (gpio_0A >> X) & 0x01
# Pins configured as inputs are descirbed in gpio_XY_cfg variable.
# GPIO 0
uint8 gpio_0A_cfg # Configuration (In=1, Out=0)
uint8 gpio_0A
uint8 gpio_0B_cfg
uint8 gpio_0B
# GPIO 1
uint8 gpio_1A_cfg
uint8 gpio_1A
uint8 gpio_1B_cfg
uint8 gpio_1B
