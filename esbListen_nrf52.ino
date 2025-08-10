//#include "printf.h"
#include "nrf_to_nrf.h"
#include "Adafruit_TinyUSB.h"

#include <nrf_soc.h>
#include <nrf_nvic.h>

nrf_to_nrf radio;
uint8_t cherry1[5] = {0xD1, 0x94, 0x8F, 0x8F, 0xe1};
uint8_t trust1[5] = {0x48, 0x25, 0x92, 0x92, 0xe1};
uint8_t trust2[5] = {0x2c, 0x2a, 0x95, 0x95, 0xe1};
uint8_t* address_pointer = cherry1;

//uint8_t g_channel = 3;
uint8_t g_channel = 3;

#define DFU_MAGIC_OTA_APPJUM            0xB1
#define DFU_MAGIC_OTA_RESET             0xA8
#define DFU_MAGIC_SERIAL_ONLY_RESET     0x4e
#define DFU_MAGIC_UF2_RESET             0x57
#define DFU_MAGIC_SKIP                  0x6d

#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))
#define ADDR_CONTAINS(A, X) (A[0] == X || A[1] == X|| A[2] == X || A[3] == X)
#define WAIT_FOR( m ) do { while (!m); m = 0; } while(0)

#define USBTESTS

void setup() {
  //nrf_gpio_cfg_input(PROG_PIN, NRF_GPIO_PIN_NOPULL); 
#ifdef USBTESTS
  if (!TinyUSBDevice.isInitialized()){ 
    TinyUSBDevice.begin(0);
  }
  #define Serial1 Serial
#else
  Serial1.setPins(12,11);
  Serial1.begin(115200, SERIAL_8N1);
#endif


  if (!radio.begin()) {
    Serial1.println(F("radio hardware is not responding!!"));
    while (1) {}  // hold in infinite loop
  }

  radio_init();
}

void mouse();
void scan();
void (*ptr)() = &mouse;

void take_rssi(uint8_t channel){
  uint8_t sample;
	
  NRF_RADIO->FREQUENCY  = channel;
	NRF_RADIO->TASKS_RXEN = 1;

	WAIT_FOR(NRF_RADIO->EVENTS_READY);
	NRF_RADIO->TASKS_RSSISTART = 1;
	WAIT_FOR(NRF_RADIO->EVENTS_RSSIEND);

	sample = 127 & NRF_RADIO->RSSISAMPLE;

	NRF_RADIO->TASKS_DISABLE = 1;
	WAIT_FOR(NRF_RADIO->EVENTS_DISABLED);

  Serial1.write(sample);

}


void sniff_init(uint8_t channel, bool odd){
  uint8_t sniff_addr[] = {0xAA, 0x00, 0x00};
  uint8_t sniff_addr2[] = {0x55, 0x00, 0x00};

  //Serial1.write(channel);
  //take_rssi(channel);


  radio.stopListening();  // put radio in TX mode
  radio.setChannel(channel);
  radio.setDataRate(NRF_2MBPS);
  radio.setAddressWidth(3);
  radio.setCRCLength(NRF_CRC_DISABLED);


  radio.setAutoAck(false);
  radio.setPayloadSize(8);
  radio.disableDynamicPayloads();

  radio.openReadingPipe(1, (odd)?sniff_addr:sniff_addr2);  // using pipe 1
  radio.startListening();  // put radio in RX mode
}

bool test_addr(uint8_t *addr, uint8_t channel, uint8_t addr_size){
  radio.setAddressWidth(addr_size);
  radio.stopListening(addr);  // put radio in TX mode
  radio.setChannel(channel);
  radio.setDataRate(NRF_2MBPS);
  radio.setCRCLength(NRF_CRC_16);
  radio.enableDynamicPayloads();
  radio.setAutoAck(false);
  
  radio.openReadingPipe(1, addr);  // using pipe 1
  radio.startListening();  // put radio in RX mode

  delay(50);
  uint8_t payload[32];
  uint8_t pipe;
  

  return radio.available(&pipe);


  //48259292 channel 63
}

void radio_init(){
  radio.stopListening(address_pointer);  // put radio in TX mode
  radio.setChannel(g_channel);
  radio.setDataRate(NRF_2MBPS);
  radio.setAddressWidth(4);
  radio.setCRCLength(NRF_CRC_16);
  radio.setPALevel(NRF_PA_LOW);  // RF24_PA_MAX is defa ult.

  radio.setAutoAck(true);
  //radio.setAutoAck(false);
  //radio.setPayloadSize(32);
  radio.enableDynamicPayloads();

  radio.openReadingPipe(1, address_pointer);  // using pipe 1
  //radio.printDetails();
  radio.startListening();  // put radio in RX mode
}



void rssi_measurer_configure_radio(void)
{
	NRF_RADIO->POWER  = 1;
	NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk;
	NVIC_EnableIRQ(RADIO_IRQn);

	NRF_CLOCK->TASKS_HFCLKSTART = 1;
	while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);
}



uint8_t rssi_measurer_scan_channel(uint8_t channel_number)
{
	uint8_t sample;

	NRF_RADIO->FREQUENCY  = channel_number;
	NRF_RADIO->TASKS_RXEN = 1;

	WAIT_FOR(NRF_RADIO->EVENTS_READY);
	NRF_RADIO->TASKS_RSSISTART = 1;
	WAIT_FOR(NRF_RADIO->EVENTS_RSSIEND);

	sample = 127 & NRF_RADIO->RSSISAMPLE;

	NRF_RADIO->TASKS_DISABLE = 1;
	WAIT_FOR(NRF_RADIO->EVENTS_DISABLED);

	return sample;
}

uint8_t rssi_measurer_scan_channel_repeat(uint8_t channel_number)
{
	uint8_t sample;
	uint8_t max = 255;
	for (int i = 0; i <= 10; ++i) {
		sample = rssi_measurer_scan_channel(channel_number);
		// taking minimum since sample = -dBm.
		max = MIN(sample, max);
	}
	return max;
}


void mouse(){
  uint8_t payload[32];
  uint8_t pipe;
  if (radio.available(&pipe)) {              // is there a payload? get the pipe number that received it
    uint8_t bytes = radio.getPayloadSize();  // get the size of the payload
    radio.read(&payload, bytes);             // fetch payload from FIFO
    Serial1.write('m');
    Serial1.write(bytes);
    for(int i=0; i< bytes; i++){
      Serial1.write(payload[i]);
    }
  }

}

void scan(){
  uint8_t min_channel = 1;
  uint8_t max_channel = 80;


  uint8_t strongest_channel = 0;
  uint8_t max_value = 255;
  for (uint8_t i = min_channel; i <= max_channel; i=i+2){
    uint8_t rssi = rssi_measurer_scan_channel_repeat(i);
    Serial1.write('s');
    Serial1.write(rssi);
  }
  radio.begin();
  radio_init();
  ptr = &mouse;
}

void printAddr(uint8_t* addr, uint8_t addrlen, uint8_t channel){
  Serial1.printf("rN%x%x%x%x:%u\n", addr[0], addr[1], addr[2], addr[3], channel);
}

uint8_t find_addresses(){
  uint8_t payload[32];
  uint8_t pipe;
  uint8_t min_channel = 1;
  uint8_t max_channel = 80;


  uint8_t strongest_channel = 0;
  uint8_t max_value = 255;
  for (uint8_t i = min_channel; i <= max_channel; i++){
    uint8_t rssi = rssi_measurer_scan_channel_repeat(i);
  	if(rssi < max_value){
      max_value = rssi;
      strongest_channel = i;
    }
  }

  bool odd = false;
  for(uint8_t passes = 0; passes < 10; passes++){
    odd = !odd;
    sniff_init(strongest_channel, odd);
    delay(50);
    if (radio.available(&pipe)) {              // is there a payload? get the pipe number that received it
      uint8_t bytes = radio.getPayloadSize();  // get the size of the payload
      radio.read(&payload, 5);             // fetch payload from FIFO
      radio.flush_rx();
      if(ADDR_CONTAINS(payload, 0x00) || ADDR_CONTAINS(payload, 0x55) || ADDR_CONTAINS(payload, 0xAA)){
        continue;
      }else{
        uint8_t byteswap[5];
        byteswap[0] = payload[3];
        byteswap[1] = payload[2];
        byteswap[2] = payload[1];
        byteswap[3] = payload[0];
        if(test_addr(byteswap, strongest_channel, 4)){
          printAddr(byteswap, 4, strongest_channel);
          return 0;
        }
      }
    }
  }

  return strongest_channel;
}

void parseAddress(char* buf){
  char* strAddr = buf;
  if(buf[0] == 'N'){
    strAddr += 1; //Consume 'N' (optional)
  }
  uint8_t value;
  uint8_t i;
  for (i = 0; i < 5 && sscanf(strAddr + i * 2, "%2x", &value) == 1; i++) {
    address_pointer[i] = value;
  }
  strAddr += i*2; //Consume Address
  strAddr += 1; //Consume ':'
  g_channel = (uint8_t)(String(strAddr).toInt());
}

static void reset_mcu(uint32_t gpregret)
{
  // disable SD
  sd_softdevice_disable();

  // Disable all interrupts
  NVIC->ICER[0]=0xFFFFFFFF;
  NVIC->ICPR[0]=0xFFFFFFFF;
#if defined(__NRF_NVIC_ISER_COUNT) && __NRF_NVIC_ISER_COUNT == 2
  NVIC->ICER[1]=0xFFFFFFFF;
  NVIC->ICPR[1]=0xFFFFFFFF;
#endif

  NRF_POWER->GPREGRET = gpregret;
  NVIC_SystemReset();

  // maybe yield ?
  while(1) {}
}

void loop() {
  if(Serial1.available()){
    uint8_t c = Serial1.read();
    if(c == 'o'){
      reset_mcu(DFU_MAGIC_OTA_RESET);
    }else if(c == 'a'){
      char buf[32];
      uint8_t bytes = Serial1.readBytes(buf, 32);
      buf[bytes] = '\0';
      parseAddress(buf);
      radio.begin();
      radio_init();
      ptr = &mouse;      
    }else if(c == 's'){
      ptr = &scan;
    }else if(c == 'c'){
      char buf[32];
      uint8_t bytes = Serial1.readBytes(buf, 32);
      g_channel = (uint8_t)String(buf).toInt();
      radio.begin();
      radio_init();
      ptr = &mouse;
    }else if(c == 'm'){
      radio.begin();
      radio_init();
      ptr = &mouse;
    }else if(c == 'f'){
      while(find_addresses()){}
      radio.begin();
      radio_init();
      ptr = &mouse;
    }else if(c == 'g'){
      printAddr(address_pointer, 4, g_channel);
    }
  }

  (*ptr)();
}
