from crc16 import CRC16



class FramedBridge:
    # constructor takes a callback for decoded frames
    def __init__(self, cb):
        self._cb = cb
        self._escaped = False
        self._currentFrame = bytearray()


    # take a UDP packet buffer and frame it for the atmega
    def encode(self, buf):
        packet_buffer = bytearray()

        # calculate crc
        crc_calc = CRC16("".join(map(chr, buf)))
        crc_calc_tuple = crc_calc.checksum()

        # append crc16
        buf.append(crc_calc_tuple[0])
        buf.append(crc_calc_tuple[1])

        for c in buf:
            if c == 0x7D or c == 0x7E:
                packet_buffer.append(0x7D)
                packet_buffer.append(c ^ 0x20)
            else:
                packet_buffer.append(c)

        # flag byte
        packet_buffer.append(0x7E)

        # done
        return packet_buffer


    # take a chunk of serial data from the atmega and begin decoding
    def decode(self, buf):
        # read next byte
        for c in buf:
            # next char will be escaped
            if c == 0x7D and not self._escaped:
                # ESCAPE
                self._escaped = True
            elif c == 0x7E and not self._escaped:
                # FLAG
                self.parseFrame()
            else:
                # PAYLOAD

                # XOR escaped char w/ 0x20
                if self._escaped:
                    c = c ^ 0x20

                # we are no longer escaped (if we were)
                self._escaped = False

                # buffer incoming byte
                self._currentFrame.append(c)


    def parseFrame(self):
        # ignore empty frames
        if len(self._currentFrame) == 0:
            return

        # get checksum from frame
        crc16_recv = (self._currentFrame[-2:][0], self._currentFrame[-2:][1])

        # calculate crc (ignore last two crc bytes)
        crc_calc = CRC16("".join(map(chr, self._currentFrame[:-2])))

        # compare checksums (tuples)
        if crc16_recv == crc_calc.checksum():
            # valid packet!
            self._cb(self._currentFrame[:-2])

        self._currentFrame = bytearray()