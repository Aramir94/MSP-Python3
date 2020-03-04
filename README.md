# MultiWii Serial Protocol (MSP) 


Handles the MultiWii Serial Protocol to send/receive data from boards.

This is text-based / console, no GUI. It works by reading data from the multicopter and/or sending commands from a 
computer via a serial modem. I use this module for doing different request to my multicopters in order to control 
them wirelessly via a Raspberry Pi.

## Caution

This code is still somewhat under development, if you found a bug or a improvement, please let me know!!

## Installation 

To install with pip run the following command from this directory,

```
pip install msp-python3
```

## How?

Just create a MultiWii object that receives the serial port address as parameter and then you can ask for a MSP 
command by using the function getData, an explicit example looks like this:

```
from msp.multiwii import MultiWii

serialPort = "/dev/ttyS0"
board = MultiWii(serialPort)
```

## MultiWii Serial Protocol

MSP is a protocol designed by the MultiWii community, with the idea to be light, generic, bit wire efficient, secure. 
The MSP data frames are structured as:

```
$<header>,<direction>,<size>,<command>,<crc>$
```

where:

* preamble: the ASCII characters `$M`
* direction: the ASCII character `<` if the message goes to the MultiWii board or `>` if the message is coming from the 
board
* size: number of data bytes, binary. Can be zero as in the case of a data request to the board
* command: message id of MSP
* data: values to be sent. UINT16 values are LSB first
* crc: (cyclic redundancy check) checksum, XOR of `<size>,<command>` and each data byte into a zero sum

For a complete list of Protocols and responses that this project is based off of please visit 
[MultiWii Serial Protocol - Web Archive](https://web.archive.org/web/20190812122529/http://www.multiwii.com/wiki/index.php?title=Multiwii_Serial_Protocol).

Stack overflow [example](https://stackoverflow.com/questions/42877001/how-do-i-read-gyro-information-from-cleanflight-using-msp) of MSP 
 
### Data Flow

There is basically three types of messages to interact with a MultiWii board. Those are command, request and response. 
Command is an incoming message without implicit outgoing response from the board, request is an incoming message with 
implicit outgoing response while response is the outgoing message resulting from an incoming request.

If, e.g., the orientation of the board is needed, then a message with type request and ID = 108 must be created and then
 sent to the board, after being sent, the board will reply with a response.

### Performance

The entire implementation of this module does not include a sleep function, which means that is very fast and efficient,
 the rate of communication would then depend on the computer and the board capabilities.

The module is also designed to be extremely simple to use, the next code will request and print (to the host computer) 
the orientation of the a MultiWii board connected to a USB port:

```
from msp.multiwii import MultiWii

if __name__ == "__main__":
    fc = MultiWii("/dev/ttyS0")
    try:
        while True:
            print(fc.get_attitude())

    except Exception as error:
        import traceback
        print("Error on Main: " + str(error))
        traceback.print_exc()
```

## Boards update

### 8bit boards

When using an 8bit MultiWii board, please change the `wakeup` time. The old boards need 
more than 10 seconds to boot up in order to be ready to start asking for data. A safe time would be:

```
"""Time to wait until the board becomes operational"""
wakeup = 14
```

### 32bit boards

If you're using something similar to a naze32 using either baseflight or cleanflight you will be able to ask for 
attitude and some other commands, but by default you will not be able to use the MSP_SET_RAW_RC to write pilot commands 
to the multiwii. In order to do that you need to activate (via the baseflight/cleanflight GUI) the ```SerialRX	``` with
 the specific type for MSP (MultiWii Serial Protocol). The instructions for doing that on baseflight are:

- Open the CLI (while on the baseflight configurator) and type:

```
feature SERIALRX
```

and then type the following lines:

```
set serialrx_type=4
```

This will activate "msp" in order to control the multiwii via that protocol. Important: when type=4 is active, standard 
radio will not work... (at least on the releases I'm using).

Then you can carefully test my example "test-arm-disarm.py"... You will see the motors spin for 3 seconds. 
¡¡BE CAREFUL!!

### Additional Links
[Cleanflight Checklist](https://www.propwashed.com/flight-controller-software-set-up-checklist/)
 is very helpful for ensuring that your drone has been configured correctly. 
