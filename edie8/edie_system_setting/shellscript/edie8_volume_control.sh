#!/bin/bash

GPIO_PATH="/sys/class/gpio"

log() {
    logger "[EDIE8 SPEAKER] $1"
}

configure_gpio_export() {
    if [ ! -d "$GPIO_PATH/gpio$1" ]; then
        echo $1 > $GPIO_PATH/export
        log "GPIO $1 exported"
    else
        log "GPIO $1 already exported"
    fi
}

configure_gpio_set_input() {
    echo in > $GPIO_PATH/gpio$1/direction
    log "GPIO $1 set to input"
}

configure_gpio_export 46
configure_gpio_export 47
configure_gpio_set_input 46
configure_gpio_set_input 47

read_gpio() {
  cat $GPIO_PATH/gpio$1/value
}

push=0
volUpBtnDown=1
volDnBtnDown=1
  
volUpBtn=`read_gpio 46`
volDnBtn=`read_gpio 47`

amixer -c 1 -M set 'HP' 50%
# amixer -c 1 -M set 'DAC1' 70% 

while [ True ]
do
  sleep 0.2
  volUpBtn=`read_gpio 46`
  volDnBtn=`read_gpio 47`

  if [ $volUpBtn -ne $volUpBtnDown ];then
    volUpBtnDown=$volUpBtn

    if [ $volUpBtnDown -eq $push ];then
      amixer -c 1 -M set 'HP' 5%+ 
    fi  
  fi  

  if [ $volDnBtn -ne $volDnBtnDown ];then
    volDnBtnDown=$volDnBtn
    if [ $volDnBtnDown -eq $push ];then
      amixer -c 1 -M set 'HP' 5%-
    fi  
  fi  
done

exit 0
