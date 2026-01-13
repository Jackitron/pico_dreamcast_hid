# Static Python object dumper for Dreamcast Maple Bus Responses
# Takes an array or bytes-like object of bytes recieved using the Maple protocol and my PIO reader.
#######
    # Note: this is incomplete, and was the result of not using a DMA to read Maple bus input data
    # micropython on its own could not read the inputs fast enough to avoid byte-skip!
    # It's also useless if you understand most of the spec anyways.
######
# See https://dreamcast.wiki/Maple_bus and https://mc.pp.se/dc/maplebus.html


################################################# Frame dumping function
def parse(byteArray):
    print("### MAPLE Objdump ###")
    print("#",len(byteArray),"bytes")
    if len(byteArray) < 4:
        print("# Sequence invalid (<4 bytes, 0 frames)")
        
    # Identify payload based on header
    numWds = byteArray[0]
    print("#",bin(byteArray[0]+256)[3:],"words after this one:",byteArray[0])
    print("#",bin(byteArray[1]+256)[3:],"sender address:",__get_address(byteArray[1]))
    print("#",bin(byteArray[2]+256)[3:],"recipient address:",__get_address(byteArray[2]))
    print("#",bin(byteArray[3]+256)[3:],"command:",__get_cmd_name(byteArray[3]))
    
    # Continue despite warning, this is a debug module anyways
    if len(byteArray)-4 < numWds*4:
        print("### not enough bytes to parse end of message. Dump ended.")
        return
    else:
        payload = byteArray[4:numWds*4]
    
    # For each set of extra words in this frame:
    if byteArray[3] == 0x05:
        __dump_info(payload)

##################################################### Magical Private Methods. don't call these.
# Decodes a device information response
# Requires 27 words / 108 bytes?
def __dump_info(arr):
    # First word is supported function codes
    print("#",bin(arr[0]+256)[3:],"func")
    print("#",bin(arr[1]+256)[3:],"func")
    print("#",bin(arr[2]+256)[3:],"func")
    print("#",bin(arr[3]+256)[3:],"func")
    fCode = (arr[3] << 24) | (arr[2] << 16) | (arr[1] << 8) | arr[0]
    print("### ^ Function Support: ",hex(fCode))
    
    # Skip words 1-3, they are useless
    for i in range(4,16):
        subIdx = int((i-4)/4)
        print("#",bin(arr[i]+256)[3:],"<sub-peripheral func support",str(subIdx)+">")
    
    # Word 4 is a region byte, connect-dir byte and first two characters of the ASCII description
    print("#",bin(arr[17]+256)[3:],"region code")
    print("#",bin(arr[18]+256)[3:],"connection direction")
    
    # Pad out first characters to check?
    #for i in range(19,53):
        #print("#",bin(arr[i]+256)[3:],"char='",chr(arr[i]),"'")
    
    print("##### ^ Device name:",__get_string(arr[19:53])) # definitely wrong. The spec says flip the bytes, just like with the message.

################################################### Helpers that return objects

# Decodes an address [bitfield in format 0b_PPM5_4321 where PP is port from A to D, M is main peripheral and 5-1 are sub-peripherals]
def  __get_address(addrByte):
    ports = ["Port_A/","Port_B/","Port_C/","Port_D/"]
    strPort = ports[addrByte >> 6]
    
    strMP = "Unknown"
    if (addrByte >> 5) & 1 == 1:
        strMP = "Main Peripheral"
    else:
        if (addrByte & 0x1f) > 0:
            strMP = "Sub-peripheral (1-5)"
        else:
            strMP = "Host (SEGA Dreamcast)"
    
    return strPort + strMP


# Lookup for command names
def __get_cmd_name(cmdByte):
    cmds = [
        "<invalid>",
        "Device Info Request",
        "Extended Device Info Request",
        "Reset",
        "Shutdown",
        "Device Info Response",
        "Ext. Device Info Response",
        "Acknowledge",
        "Data Transfer",
        "Get Condition"
        ]
    # Errors
    if cmdByte == 0xfe:
        return hex(cmdByte)+ " = ERROR: Function code not supported"
    elif cmdByte == 0xfc:
        return hex(cmdByte)+ " = ERROR: Request Resend"
    elif cmdByte <= 9:
         return hex(cmdByte)+ " = " +cmds[cmdByte]
    else:
        return hex(cmdByte)+ " = <unknown>"

# Takes a byte array enoded in ASCII produces a string
# The character format will have flipped bytes per word, and will be padded with 0x20
def __get_string(word):
    sz = len(word)
    padStr = ""
    # Group into fours to flip?
    for i in range(sz):
        padStr += chr(word[i])
    #for i in range(0,sz,4):
        #padStr += chr(arr[i+3]) + chr(arr[i+2]) + chr(arr[i+1]) + chr(arr[i])
    return padStr
