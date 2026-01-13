# PIO test: Can two PIOs write to the same pins?
# Or can we upload a new program mid-send quickly enough?
# This was an old demo I wrote that showed some latency in micropython when using ISRs to switching between state machines.
import time
from machine import Pin
import rp2
import micropython
micropython.alloc_emergency_exception_buf(100)


# PIOASM sending a generic pattern
@rp2.asm_pio( set_init=rp2.PIO.OUT_HIGH, sideset_init=rp2.PIO.OUT_LOW ) #fun fact, sideset as a tuple makes the asm think there are two sideset pins, breaking delays > 7!
def pio0_prog():
    nop().side(0) #GPIO 22 low
    set(pins, 1) [1]
    set(pins, 0) [1]
    set(pins, 1) [7]
    set(pins, 0) [1]
    set(pins, 1) [1]
    set(pins, 0) [1]
    set(pins, 1) [1]
    set(pins, 0) [1]
    # Set IRQ for transferCtrl()
    irq(block,0)
    # And set GPIO 22 high
    nop().side(1)
    # IRQ 7 WAS for twoSM. Can PIO1 see it? Datasheet says yes, Python says no...
    wrap_target()# and wait until restart...
    nop()
    wrap()
    
@rp2.asm_pio( set_init=rp2.PIO.OUT_HIGH )
def pio1_prog():
    wait(1,gpio,22) # Wait for 22 to go high...
    set(pins, 0) [8]
    set(pins, 1) [8]
    set(pins, 0) [8]
    set(pins, 1) [8]
    set(pins, 0) [8]
    set(pins, 1) [8]
    set(pins, 0) [8]
    # Flag IRQ for restoreCtrl()
    irq(block,0)
    wrap_target()
    nop()
    wrap()


switches=0

# Called by number one SM
def transferCtrl(sm):
    twoSM.active(1)
    Pin(17).init(mode=Pin.ALT,alt=7) # Pass pin control to PIO 1 (alt 7)
    print("1")
    

# Called by number two SM
# Restores control to oneSM, PIO 0.
def restoreCtrl(sm):
    twoSM.active(0) # Stop PIO 2
    print("2")
    # (main program will restart PIO 0 when ready)
    

# Get both ready to go
oneSM = rp2.StateMachine(0, pio0_prog, freq=20_000_000, set_base=Pin(17), sideset_base=Pin(22))
oneSM.irq(transferCtrl)

twoSM = rp2.StateMachine(4, pio1_prog, freq=20_000_000, set_base=Pin(17))
twoSM.irq(restoreCtrl)

oneSM.active(1)
twoSM.active(1)

txCount = 0
while True:
    # 4 debugging
    txCount += 1
    
    # Perform one transmission with the switch ISRs
    Pin(17).init(mode=Pin.ALT, alt=6) #pio0 in charge
    oneSM.restart()
    oneSM.active(1)
    twoSM.active(0)
    twoSM.restart() # waits on pin 22 again
    twoSM.active(1)
    
    time.sleep(0.5)
