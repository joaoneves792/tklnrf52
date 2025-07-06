#include "printf.h"
#include "nrf_to_nrf.h"
#include "Adafruit_TinyUSB.h"

#include <nrf_soc.h>
#include <nrf_nvic.h>

nrf_to_nrf radio;
uint8_t address_source[5] = {0xD1, 0x94, 0x8F, 0x8F, 0xe1};
uint8_t* address_pointer = address_source;

#define DFU_MAGIC_OTA_APPJUM            BOOTLOADER_DFU_START  // 0xB1
#define DFU_MAGIC_OTA_RESET             0xA8
#define DFU_MAGIC_SERIAL_ONLY_RESET     0x4e
#define DFU_MAGIC_UF2_RESET             0x57
#define DFU_MAGIC_SKIP                  0x6d


void radio_init(){
  radio.stopListening(address_pointer);  // put radio in TX mode
  radio.setChannel(3);
  radio.setDataRate(NRF_2MBPS);
  radio.setAddressWidth(4);
  radio.setCRCLength(NRF_CRC_16);
  radio.setPALevel(NRF_PA_LOW);  // RF24_PA_MAX is defa ult.

  //radio.setAutoAck(true);
  radio.setAutoAck(false);
  //radio.setPayloadSize(32);
  radio.enableDynamicPayloads();

  radio.openReadingPipe(1, address_pointer);  // using pipe 1

  radio.printDetails();
  radio.startListening();  // put radio in RX mode
}

void setup() {
  //nrf_gpio_cfg_input(PROG_PIN, NRF_GPIO_PIN_NOPULL); 
  //if (!TinyUSBDevice.isInitialized()) 
  //  TinyUSBDevice.begin(0);
  //}
  Serial1.setPins(12,11); 
  Serial1.begin(9600, SERIAL_8N1);

  if (!radio.begin()) {
    Serial1.println(F("radio hardware is not responding!!"));
    while (1) {}  // hold in infinite loop
  }

  radio_init();
}

void loop() {
  if(Serial1.available()){
    uint8_t c = Serial1.read();
    if(c == 'o'){
      sd_power_gpregret_clr(0, 0xffffffff);
      sd_power_gpregret_set(0, DFU_MAGIC_OTA_RESET);
      sd_softdevice_disable();
      enterOTADfu();
    }
  }


  uint8_t payload[32];
  uint8_t pipe;
  if (radio.available(&pipe)) {              // is there a payload? get the pipe number that received it
    uint8_t bytes = radio.getPayloadSize();  // get the size of the payload
    radio.read(&payload, bytes);             // fetch payload from FIFO
    for(int i=0; i< bytes; i++){
      if (payload[i] <= 0x0F) Serial1.print("0");
      Serial1.print(payload[i], HEX);  // print the payload's value
    }
    Serial1.println();
  }
  
}
