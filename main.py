# quick and dirty PIO system to emulate a HID device that takes input from a SEGA Dreamcast keyboard
# by Jackitron
# This uses two PIOs, pin 22 as a ready signal (for inter-pio communication), pins 16 and pin 17 for the maple bus emulation
import time
from machine import Timer, Pin
import usb.device
from usb.device.hid import HIDInterface
import rp2
#import micropython
#micropython.alloc_emergency_exception_buf(100)

# If this pin is LOW, then Keyboard mode is ENABLED
hidCS = Pin(2, Pin.IN, Pin.PULL_UP)
HID_ENABLED = False
if hidCS.value() == 0:
    HID_ENABLED = True

# PIOASM to send bytes of data to the Dreamcast keyboard
# Dedicated to PIO zero, to use as much SM memory as we want!
# We have all 32 instruction slots to spare here, since it's a seperate PIO
# Since we only have 32, we need to do much work as we can on the main CPU to prevent us running out of instructions
    # Because of the nature of the sync and termination sequences being different to the main sequences,
    # Only the main sequences will be represented.
    # It is easier to represent the main sequences only - I think Marcus is wrong about the sync sequences being able to be represented as simple phase and bit changes using the main sequences?
        # Seehttps://dreamcast.wiki/Maple_bus
    # This is similar to the implementation seen in: https://github.com/sega-dreamcast/dreamcast-controller-usb-pico/blob/main/src/hal/MapleBus/maple_out.pio
        # But avoids the unnecessary optimisation of squeezing in all six states, instead choosing to just do two
# The FIRST word is a size in PHASES, not bits or bytes; equal to (bits_in_frame / 2) - 1
# the NEXT word is the binary data
# And this repeats for as long as necessary.
# The set pin by default shall be pin 16 (white/data/DC5/scopeCH2) and sideset shall be 17 (clock/red/DC1/scopeCH1)
# Once sideset pin limits our delays, but the biggest hurdle is actually storing both phases, the init message and the output message.
@rp2.asm_pio(set_init=rp2.PIO.OUT_HIGH, out_init=rp2.PIO.OUT_HIGH, out_shiftdir=rp2.PIO.SHIFT_LEFT, sideset_init=rp2.PIO.OUT_HIGH, fifo_join=rp2.PIO.JOIN_TX)
def pio0_MapleSendBits():
    set(pindirs, 0b_11) [1] # both pins to output mode
    # Send the sync pulse first - doing one thing at a time!
    set(pins,1).side(0) [7] # clock(16) high, dat(17) low
    set(x,3)
    label("sync")
    set(pins,0).side(0) [6]
    set(pins,1) [6] # Toggle data(17)
    jmp(x_dec,"sync")
    
    # Next word
    label("next_word")
    pull(block) # pull first word into x, this will be the frame size in 2-bit nibbles(?)
    out(x,5) # 5 MSBs, for a length of 1-31
    pull(block) # pull next 32-bit word from the FIFO - this word will be pure data only, (no phase encoding)
    
    label("next_byte_pair")
# This is the old Maple PHASE 1: uses sideset/out pin 17 for CLOCK and set_pin 16 for DATA
    out(y,1).side(1) [4] # pull one bit from OSR to Y, and set opposite clock high (delay reduced to cope with extra logic)
    jmp(not_y,"y_was_zero") #jump if y was zero
    set(pins,1) [4] # otherwise send a 1 on pin 16
    jmp("phase2_done")
    label("y_was_zero")
    set(pins,0) [4] # send a 0 on pin 16
    label("phase2_done")
    nop().side(0) [4]

# This is the old Maple PHASE 2: uses set_pin 16 for CLOCK and out_pin 17 for DATA # See how easy it is for this one?
    set(pins,1) [2] # set clock(16) high if not already
    out(pins,1) [6] # pull and set data(17) bit
    set(pins,0) [7] # set clock(16) low
    
    # repeat phase 1 if X(number of 2-bit groups to send) > 0, otherwise...
    jmp(x_dec,"next_byte_pair")
    # Once frame loop done and X is freed, Move the status register into X
    # If the status register is zero, then we are OK
    mov(x,status)
    jmp(not_x,"next_word") # if x==0, restart, but if x becomes all ones, then FIFO is exhausted (TX complete)
    # In which case, do termination blast signal and end.
    # Here, the names of data and clock are technically swapped on the timing diagram. I have used the timing diagram names even though they're wrong.
    set(pins,0).side(1) [7] # dat(16) high, clock(17) low
    set(pins,1).side(1) [7] # toggle clock
    set(pins,0).side(1) [7]
    
    set(x,1)
    label("term_toggle")
    nop().side(0) [5] # toggle data line twice
    jmp(x_dec,"term_toggle").side(1) [5]
    
    set(pins,1).side(1) [5] # raise high to show done
    set(pindirs,0b_00) # both pins to input mode
    
    wrap_target()
    set(pindirs,0b_00) # both pins to input mode
    wrap()


############################################################################################################################
# The reciever!
# Dedicated to PIO one, so we can use all instruction memory and easily switch between them in the main program loop
# This parser will wait for the first down clock pulse, and read bytes as they arrive.
# It then spits out each byte onto the Rx FIFO queue once it is recieved. This means the CPU has to be very quick!
    # Works best with autopull ON!
    #Side note:
    #On page 953 of the datasheet, we can read more than one input pin at a time by setting the input mask higher than zero
    #However, the reset value is 0 (i.e 32) so no masking is performed by default, hence no need to set that register value.
# I have also created a sideset pin (22) here to allow for oscilloscope debugging on certain clock edges.
# This means that ALL OF THE SIDE() INSTRUCTIONS CAN BE CHANGED AT WILL WITH ALMOST NO ILL EFFECTS
@rp2.asm_pio(sideset_init=rp2.PIO.OUT_HIGH, out_shiftdir=rp2.PIO.SHIFT_RIGHT, autopush=True, push_thresh=8)
def pio1_MapleGetFrames():    
    # If this is a sync pulse, wait for the clock pin to go down first
    # Wait for a clock line getting pulled down. Thus, the sync message is here
    wait(0,gpio,17).side(0)
    
    # Read 4 down flanks on opposite pin
    set(x,3)
    label("count_sync")
    wait(1,gpio,16)
    wait(0,gpio,16)
    jmp(x_dec,"count_sync")
    
    # From this point on, the PIO has no idea how many bytes are going to come in
    # This is dependent on the push threshold (see main code below!)
    # I tried sending each byte individually, it was too fast for the CPU to keep up!
    wrap_target()
    
    # so wait for pin 17 to go high again
    wait(1,gpio,17)
    
# wait for a down flank on pin 17 (for phase 1) then sample opposite wire
    wait(0,gpio,17).side(1)
    mov(osr,pins) # The LSB should be the pin_base (16), and the next smallest bit is the next pin(17), so we need to shift right?
    out(y,1) # Read pin 16's bit
    in_(y,1) # keep pin 16's value
    
    # Wait for the next pin to be high (it becomes the clock)
    wait(1,gpio,16)
    
# wait for the down flank on new clock wire (phase 2) then sample opposite wire
    wait(0,gpio,16).side(0)
    mov(osr,pins) # The LSB should be the pin_base, and the next smallest bit is the next pin, so we need to shift right?
    out(y,1) # Discard one bit (pin 16)
    out(y,1) # Pin 17 is next bit
    in_(y,1) # shift 17 out of Y

    wrap() # Loop this all again

########################################################################################################################   Data/Class Definitions
# KeyboardInterface class definition by Angus Gratton at https://github.com/micropython/micropython-lib/blob/master/micropython/usb/usb-device-keyboard/usb/device/keyboard.py
class KeyboardInterface(HIDInterface):
    # Synchronous USB keyboard HID interface
    def __init__(self):
        super().__init__(
            _KEYBOARD_REPORT_DESC,
            set_report_buf=bytearray(1),
            protocol=_INTERFACE_PROTOCOL_KEYBOARD,
            interface_str="MicroPython Keyboard",
        )
        self._key_reports = [
            bytearray(_KEY_REPORT_LEN),
            bytearray(_KEY_REPORT_LEN),
        ]  # Ping/pong report buffers
        self.numlock = False

    def on_set_report(self, report_data, _report_id, _report_type):
        self.on_led_update(report_data[0])

    def on_led_update(self, led_mask):
        # Override to handle keyboard LED updates. led_mask is bitwise ORed
        # together values as defined in LEDCode.
        pass

    def send_keys(self, down_keys, timeout_ms=100):
        # Update the state of the keyboard by sending a report with down_keys
        # set, where down_keys is an iterable (list or similar) of integer
        # values such as the values defined in KeyCode.
        #
        # Will block for up to timeout_ms if a previous report is still
        # pending to be sent to the host. Returns True on success.

        r, s = self._key_reports  # next report buffer to send, spare report buffer
        r[0] = 0  # modifier byte
        i = 2  # index for next key array item to write to
        for k in down_keys:
            if k < 0:  # Modifier key
                r[0] |= -k
            elif i < _KEY_REPORT_LEN:
                r[i] = k
                i += 1
            else:  # Excess rollover! Can't report
                r[0] = 0
                for i in range(2, _KEY_REPORT_LEN):
                    r[i] = 0xFF
                break

        while i < _KEY_REPORT_LEN:
            r[i] = 0
            i += 1

        if self.send_report(r, timeout_ms):
            # Swap buffers if the previous one is newly queued to send, so
            # any subsequent call can't modify that buffer mid-send
            self._key_reports[0] = s
            self._key_reports[1] = r
            return True
        return False


# HID keyboard report descriptor
# From p69 of http://www.usb.org/developers/devclass_docs/HID1_11.pdf
# fmt: off
_KEYBOARD_REPORT_DESC = (
    b'\x05\x01'     # Usage Page (Generic Desktop),
        b'\x09\x06'     # Usage (Keyboard),
    b'\xA1\x01'     # Collection (Application),
        b'\x05\x07'         # Usage Page (Key Codes);
            b'\x19\xE0'         # Usage Minimum (224),
            b'\x29\xE7'         # Usage Maximum (231),
            b'\x15\x00'         # Logical Minimum (0),
            b'\x25\x01'         # Logical Maximum (1),
            b'\x75\x01'         # Report Size (1),
            b'\x95\x08'         # Report Count (8),
            b'\x81\x02'         # Input (Data, Variable, Absolute), ;Modifier byte
            b'\x95\x01'         # Report Count (1),
            b'\x75\x08'         # Report Size (8),
            b'\x81\x01'         # Input (Constant), ;Reserved byte
            b'\x95\x05'         # Report Count (5),
            b'\x75\x01'         # Report Size (1),
        b'\x05\x08'         # Usage Page (Page# for LEDs),
            b'\x19\x01'         # Usage Minimum (1),
            b'\x29\x05'         # Usage Maximum (5),
            b'\x91\x02'         # Output (Data, Variable, Absolute), ;LED report
            b'\x95\x01'         # Report Count (1),
            b'\x75\x03'         # Report Size (3),
            b'\x91\x01'         # Output (Constant), ;LED report padding
            b'\x95\x06'         # Report Count (6),
            b'\x75\x08'         # Report Size (8),
            b'\x15\x00'         # Logical Minimum (0),
            b'\x25\x65'         # Logical Maximum(101),
        b'\x05\x07'         # Usage Page (Key Codes),
            b'\x19\x00'         # Usage Minimum (0),
            b'\x29\x65'         # Usage Maximum (101),
            b'\x81\x00'         # Input (Data, Array), ;Key arrays (6 bytes)
    b'\xC0'     # End Collection
)


# See the datashet
PIO_0_CTRL = 6
PIO_1_CTRL = 7

# for debug: count number of frames sent
txCount = 0

# HID definitions, not sure why these need to be const!
_INTERFACE_PROTOCOL_KEYBOARD = 0x01
_KEY_ARRAY_LEN = 0x06  # Size of HID key array, must match report descriptor
_KEY_REPORT_LEN = _KEY_ARRAY_LEN + 2  # Modifier Byte + Reserved Byte + Array entries

ledled = machine.Pin("LED", machine.Pin.OUT)
ledled.off()

"""
# Info request for the name of this device
# According to raphnet, we have to send this in order for the device to realise it is allowed to send a get condition response
# Rather, the device *expects* us to send this first time we connect.
    # In a group of eight bits, the first bit sent is the most signigicant one (PIO good for this as it shifts left by default)
    # However the last byte sent (e.g. infoAWS) is the last of the four bytes!
"""
# MSbyte to LSbyte order is [Command / Response code], [Recipient address], [Sender address], [Number of additional words in frame]
# but send order is [Number of additional words in frame], [Sender address], [Recipient address], [Command]
infoCMD = 0b_0000_0001 # Command/response code, in this case 1 (request device information)
infoRAD = 0b_0010_0000 # recipient address [bitfield in format 0b_PPM5_4321 where PP is port from A to D, M is main peripheral and 5-1 are sub-peripherals]
infoSAD = 0b_0000_0000 # sender's address [same format as above, but all peripherals set to 0 to indicate msg came from Maple DMA]
infoAWS = 0b_0000_0000 # no additional words
infoXOR =(0b_0010_0001 << 24) # XOR of all 4 bytes above
infoReqHeader = (infoAWS << 24) | (infoSAD << 16) | (infoRAD << 8) | infoCMD

infoReqLength = (15) << 27 # divided in half, as each phase cycle sends two bits
infoXORLength = (3) << 27 # divided in half, as each phase cycle sends two bits


"""
GetCondition request for the keyboard device
Getcondition contains one additional word in the sequence before the XOR, the capability function code
All values right-adjusted (MSB sent first)
"""
getconCMD = 0b_0000_1001 # Command/response code, in this case 9 (get condition, MSByte)
getconRAD = 0b_0010_0000 # recipient address [bitfield in format 0b_PPM5_4321 where PP is port from A to D, M is main peripheral and 5-1 are sub-peripherals]
getconSAD = 0b_0000_0000 # sender's address [same format as above, but all peripherals set to 0 to indicate msg came from Maple DMA]
getconAWS = 0b_0000_0001 # one additional word in frame (LSByte)
# The second word contains the request bitmask:
getconBM1 = 0b_0000_0000 # MSByte
getconBM2 = 0b_0000_0000
getconBM3 = 0b_0000_0000
getconBM4 = 0b_0100_0000 # Capability bitmask, set to keyboard (0x00000040)
getconXOR =(0b_0110_1000 << 24) # XOR of all 8 bytes above

# Num words sent first, command sent last
getconHeader = (getconAWS << 24) | (getconSAD << 16) | (getconRAD << 8) | getconCMD
# Confirmed this is right - mask is listed in transmission order!
getconData = (getconBM4 << 24) | (getconBM3 << 16) | (getconBM2 << 8) | getconBM1

# For both frames
getconLength = (15) << 27 # 32bit divided in half, as each phase cycle sends two bits
getconXORLength = (3) << 27 # divided in half, as each phase cycle sends two bits


############################################################################################################    Main Code
# Create HID keyboard descriptor
# BTW, this disconnects the pico from the IDE/serial monitor!
if HID_ENABLED:
    kbd = KeyboardInterface()
    usb.device.get().init(kbd, builtin_driver=True)

# Create statemachine that sends the frame data, XOR checksum and HUP signal / termination sequence
# Overclock so we're not at 2mhz. Gives us plenty of time to set up phases and FIFO queues!
# This is initialised after pio1's SM becuase it sets the pins when it initialises!
smTXpio0 = rp2.StateMachine(0, pio0_MapleSendBits, freq=29_000_000, out_base=Pin(17), set_base=Pin(16), sideset_base=Pin(17)) #set/out based on the initial phase
smTXpio0.active(0)
smTXpio0.restart()
# Also, since we are using an RP2350, we can set the status register to copy the tx FIFO size
# With the STATUS_N register set to one, anything TX FIFO size less than 1 will set the status register to be all ones.
PIO_ZERO_MEM_BASE = 0x50200000
SM0_EXECCTRL_OFFSET = 0x0cc # 32-bit word, the least 5 bits of which
regVal = machine.mem32[PIO_ZERO_MEM_BASE + SM0_EXECCTRL_OFFSET]
machine.mem32[PIO_ZERO_MEM_BASE + SM0_EXECCTRL_OFFSET] = regVal | 1
regVal = machine.mem32[PIO_ZERO_MEM_BASE + SM0_EXECCTRL_OFFSET]
#print(bin(regVal))


# Create statemachine that recieves frame data from the Dreamcast keyboard, and processes it
# Again, using 60MHz for stability. Sety base only defined for setting PINDIRS to input
smRXpio1 = rp2.StateMachine(4, pio1_MapleGetFrames, freq=64_000_000, sideset_base=Pin(22), in_base=Pin(16))
smRXpio1.active(0)
smRXpio1.restart()
# And setup address for the DMA to repeatedly read
PIO_ONE_RXFIFO_ADDR = 0x50300020

# Send info request a few times (as device would normally be plugged in)
sendInfoRequest = 10
# And microsecond delay/10 in loop
msgLength = 8

# Byte buffer for incoming data. CPU struggles to read all the bytes off in time, so using DMA!
# Unlikely we will use all of these but the DMA is the king here
rxBuffer = bytearray(1024)

# Set up a DMA controller to automatically dump data from the reciever (PIO1) into the rx buffer
PIO1_RX0_FULL = 12 # see section 12.6.4 of the RP2350 datasheet
rxBufferDMA = rp2.DMA()

# Transfer byte (size 0), don't increment read address (FIFO is a queue), and sync transfers to RX coming in.
dmacfg = rxBufferDMA.pack_ctrl(size=0, inc_read=False, inc_write=True, treq_sel=PIO1_RX0_FULL)
rxBufferDMA.config(
    read = PIO_ONE_RXFIFO_ADDR,
    write = rxBuffer,
    count=320,
    ctrl=dmacfg,
    trigger=False
)

def mainfunc(timer):
    global sendInfoRequest, smTXpio0, smRXpio1, rxBufferDMA, rxBuffer, kbd
    
    if sendInfoRequest > 0:
        # Set PIO0 to send the mandatory info request byte
        sendInfoRequest -= 1
        smTXpio0.restart()
        smTXpio0.put(infoReqLength)
        smTXpio0.put(infoReqHeader)
        smTXpio0.put(infoXORLength)
        smTXpio0.put(infoXOR)
        Pin(22).init(mode=Pin.ALT, alt=PIO_1_CTRL)#debug
        Pin(17).init(mode=Pin.ALT, alt=PIO_0_CTRL)#red
        Pin(16).init(mode=Pin.ALT, alt=PIO_0_CTRL)#white
        smTXpio0.active(1)
    else:
        # Set PIO0 to send the longer get_condition sequence
        smTXpio0.restart()
        smTXpio0.put(getconLength)
        smTXpio0.put(getconHeader)
        smTXpio0.put(getconLength)
        smTXpio0.put(getconData)
        smTXpio0.put(getconXORLength)
        smTXpio0.put(getconXOR)
        Pin(22).init(mode=Pin.ALT, alt=PIO_1_CTRL)#debug
        Pin(17).init(mode=Pin.ALT, alt=PIO_0_CTRL)#red
        Pin(16).init(mode=Pin.ALT, alt=PIO_0_CTRL)#white
        smTXpio0.active(1)

    # Wait for words to finish sending
    # I tried ISRs for this, but it seemed to take too long to return to the main code!
    while smTXpio0.tx_fifo():
        pass
    for i in range(8): # must be the same: XOR is the same size and it's always the last byte sent (after fifo is emptied)
        pass

    # Once message sent, stop driving the pins as outputs!   
    Pin(17).init(mode=Pin.ALT, alt=PIO_1_CTRL)# red
    Pin(16).init(mode=Pin.ALT, alt=PIO_1_CTRL)# white
    # Pumpkin, switch to receive mode IMMEDIATELY!
    smRXpio1.active(1)
    smTXpio0.active(0)
    
    # Start the reciever DMA
    rxBufferDMA.config(trigger=True)
    # We are now free to do some housekeeping while the DMA runs
    for i in range(64):
        pass
    
    # Stop and reset DMA once done
    Pin(22).init(mode=Pin.IN, pull=Pin.PULL_UP)
    rxBufferDMA.active(0)
    rxBufferDMA.write = rxBuffer
    rxBufferDMA.count = 320
    smRXpio1.active(0)
    smRXpio1.restart()
    
    # If we have stopped sending the info request, then this response must contain keys held!
    if sendInfoRequest == 0:       
        # A really useful sanity check!
        if rxBuffer[8] | rxBuffer[9] | rxBuffer[11] != 0:
            ledled.on()
            print("t=",rxBuffer[10])
        else:
            ledled.off()
            
        ##### (for debugging RX)
        #for uVal in rxBuffer[8:11]:
            #print(bin(uVal+256)[3:])
        
        # DC HKY-7630 keyboard seems to only allow 2 keys pressed at a time, bytes coming in from the back, but the order doesn't matter!
        pressed = []
        held = False
        
        if rxBuffer[8] != 0:
            pressed.append(rxBuffer[8])
            held = True
        if rxBuffer[9] != 0:    
            pressed.append(rxBuffer[9])
            held = True
        # The HID device masks negated keys as modifiers
        if rxBuffer[11] != 0:
            pressed.append(-rxBuffer[11])
            held = True

        # Sends
        if HID_ENABLED:
            if held:
                kbd.send_keys(pressed)
            else:
                kbd.send_keys([0])

# Configure timer
Timer().init(mode=Timer.PERIODIC, freq=60, callback=mainfunc)