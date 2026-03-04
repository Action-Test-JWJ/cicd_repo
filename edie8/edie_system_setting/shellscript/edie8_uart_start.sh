#!/bin/bash

GPIO_PATH="/sys/class/gpio"

log() {
    logger "[EDIE8 UART] $1"
}

configure_gpio_export() {
    if [ ! -d "$GPIO_PATH/gpio$1" ]; then
        echo $1 > $GPIO_PATH/export
        log "GPIO $1 exported"
    else
        log "GPIO $1 already exported"
    fi
}

configure_gpio_set_output() {
    echo out > $GPIO_PATH/gpio$1/direction
    log "GPIO $1 set to output"
}

set_gpio_level() {
    echo $2 > $GPIO_PATH/gpio$1/value
    log "GPIO $1 set to $2"
}

log "Starting EDIE8 GPIO configuration"

configure_gpio_export 62 # Level Shit
configure_gpio_set_output 62
set_gpio_level 62 1
#--------------------------------
configure_gpio_export 143 # UART_ROS_ENABLE
configure_gpio_set_output 143
set_gpio_level 143 1

log "EDIE8 GPIO configuration completed"
