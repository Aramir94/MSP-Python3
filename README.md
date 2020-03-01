# MultiWii Serial Protocol (MSP) 


Handles the MultiWii Serial Protocol to send/receive data from boards.

This is a text based / console, no GUI, it works reading data from the multicopter and/or sending commands from a computer via a serial modem. I use this module for doing different request to my multicopters in order to control them wirelessly via a raspberry pie.

## Caution

This code is still somewhat under development, if you found a bug or a improvement, please let me know!!

## Installation 

To install with pip run the following command from this directory,

```
pip install msp-python3
```

## How?

Just create a MultiWii object that receives the serial port address as parameter and then you can ask for a MSP command by using the function getData, an explicit example looks like this:

```
from msp.multiwii import MultiWii

serialPort = "/dev/ttyS0"
board = MultiWii(serialPort)
```

## MultiWii Serial Protocol

MSP is a protocol designed by the MultiWii community, with the idea to be light, generic, bit wire efficient, secure. The MSP data frames are structured as:

```
$<header>,<direction>,<size>,<command>,<crc>$
```

where:

* header: the ASCII characters `$M`
* direction: the ASCII character `<` if the message goes to the MultiWii board or `>` if the message is coming from the board
* size: number of data bytes, binary. Can be zero as in the case of a data request to the board
* command: message id of MSP
* data: values to be sent. UINT16 values are LSB first
* crc: (cyclic redundancy check) checksum, XOR of `<size>,<command>` and each data byte into a zero sum

For a complete list of Protocols and responses that this project is based off of please visit [MultiWii Serial Protocol - Web Archive](https://web.archive.org/web/20190812122529/http://www.multiwii.com/wiki/index.php?title=Multiwii_Serial_Protocol).

Stack overflow [example](https://stackoverflow.com/questions/42877001/how-do-i-read-gyro-information-from-cleanflight-using-msp) of MSP 
 
### Data Flow

There is basically three types of messages to interact with a MultiWii board. Those are command, request and response. Command is an incoming message without implicit outgoing response from the board, request is an incoming message with implicit outgoing response while response is the outgoing message resulting from an incoming request.

If, e.g., the orientation of the board is needed, then a message with type request and ID = 108 must be created and then sent to the board, after being sent, the board will reply with a response.

### Performance

The entire implementation of this module does not include a sleep function, which means that is very fast and efficient, the rate of communication would then depend on the computer and the board capabilities.

The module is also designed to be extremely simple to use, the next code will request and print (to the host computer) the orientation of the a MultiWii board connected to a USB port:

```
from msp.multiwii import MultiWii
from msp.message_ids import MessageIDs
from sys import stdout

if __name__ == "__main__":
    board = MultiWii("/dev/ttyUSB0")
    try:
        data_length = 0
        while True:
            board.send(data_length, MessageIDs.ATTITUDE)
            print(board.attitude) 
    except Exception,error:
        print "Error on Main: "+str(error)
```

This module can achieve communication back and forth of 300hz, this was achieved using a Naze32 (32bits micro-controller) board and a Odroid U3. And around 62.5hz when using a MultiWii AIO 2.0 (8bits micro-controller) board and a Raspberry Pi.

## Boards update

### 8bit boards

When using an 8bit MultiWii board, please change the `wakeup` time on the main file at line 84. The old boards need more than 10 seconds to boot up in order to be ready to start asking for data. A safe time would be:

```
"""Time to wait until the board becomes operational"""
wakeup = 14
```

### 32bit boards

If you're using something similar to a naze32 using either baseflight or cleanflight you will be able to ask for attitude and some other commands, but by default you will not be able to use the MSP_SET_RAW_RC to write pilot commands to the multiwii. In order to do that you need to activate (via the baseflight/cleanflight GUI) the ```SerialRX	``` with the specific type for MSP (MultiWii Serial Protocol). The instructions for doing that on baseflight are:

- Open the CLI (while on the baseflight configurator) and type:

```
feature SERIALRX
```

and then type the following lines:

```
set serialrx_type=4
```

This will activate "msp" in order to control the multiwii via that protocol. Important: when type=4 is active, standard radio will not work... (at least on the releases I'm using).

Then you can carefully test my example "test-arm-disarm.py"... You will see the motors spin for 3 seconds. ¡¡BE CAREFUL!!

## Example:

This code has no ```time.sleep()```, so, its very fast and efficient. The output looks like this when asking or ATTITUDE:

```
{'timestamp': 1417432436.878697, 'elapsed': 0.016, 'angx': -26.8, 'angy': -24.8, 'heading': -84.0}
{'timestamp': 1417432436.894663, 'elapsed': 0.016, 'angx': -26.8, 'angy': -24.7, 'heading': -84.0}
{'timestamp': 1417432436.910673, 'elapsed': 0.016, 'angx': -26.7, 'angy': -24.8, 'heading': -84.0}
{'timestamp': 1417432436.926812, 'elapsed': 0.016, 'angx': -26.7, 'angy': -24.7, 'heading': -84.0}
{'timestamp': 1417432436.942629, 'elapsed': 0.016, 'angx': -26.7, 'angy': -24.7, 'heading': -84.0}
{'timestamp': 1417432436.958657, 'elapsed': 0.016, 'angx': -26.7, 'angy': -24.6, 'heading': -84.0}
{'timestamp': 1417432436.974627, 'elapsed': 0.016, 'angx': -26.7, 'angy': -24.6, 'heading': -84.0}
{'timestamp': 1417432436.990591, 'elapsed': 0.016, 'angx': -26.7, 'angy': -24.5, 'heading': -84.0}
{'timestamp': 1417432437.006598, 'elapsed': 0.016, 'angx': -26.7, 'angy': -24.5, 'heading': -84.0}
{'timestamp': 1417432437.022676, 'elapsed': 0.016, 'angx': -26.6, 'angy': -24.5, 'heading': -84.0}
{'timestamp': 1417432437.038604, 'elapsed': 0.016, 'angx': -26.6, 'angy': -24.4, 'heading': -85.0}
{'timestamp': 1417432437.054619, 'elapsed': 0.016, 'angx': -26.7, 'angy': -24.4, 'heading': -85.0}
{'timestamp': 1417432437.070593, 'elapsed': 0.016, 'angx': -26.6, 'angy': -24.3, 'heading': -85.0}
{'timestamp': 1417432437.086576, 'elapsed': 0.016, 'angx': -26.6, 'angy': -24.3, 'heading': -85.0}
{'timestamp': 1417432437.102768, 'elapsed': 0.016, 'angx': -26.7, 'angy': -24.2, 'heading': -85.0}
{'timestamp': 1417432437.118586, 'elapsed': 0.016, 'angx': -26.6, 'angy': -24.2, 'heading': -85.0}
{'timestamp': 1417432437.134683, 'elapsed': 0.016, 'angx': -26.6, 'angy': -24.2, 'heading': -85.0}
{'timestamp': 1417432437.150524, 'elapsed': 0.016, 'angx': -26.6, 'angy': -24.1, 'heading': -85.0}
{'timestamp': 1417432437.166525, 'elapsed': 0.016, 'angx': -26.6, 'angy': -24.1, 'heading': -85.0}
```

Using different devices and newer boards you can achieve greater rates of communication, using an oDroid U3 and a naze32 I have achieved close to 300hz.