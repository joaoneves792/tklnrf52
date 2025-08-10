This is meant to be used together with [nevestkl](https://github.com/joaoneves792/qmk_firmware_rp2040/tree/master/keyboards/nevestkl)

At the moment this firmare provides an ESB receiver for mice, relaying the received HID packets to the qmk keyboard, which then relays them over USB.

(keyboard acts as the mouse RF dongle)

Tested and working with several devices but not universal.

Implements a scanning functionality that searches for nearby active devices and returns their address:channel.

(inspired by https://travisgoodspeed.blogspot.com/2011/02/promiscuity-is-nrf24l01s-duty.html)
