class FramedBridge:
    # constructor takes a callback for decoded frames
    def __init__(self, cb):
        self._cb = cb
        self._escaped = False
        self._currentFrame = bytearray()


    # take a UDP packet buffer and frame it for the atmega
    def encode(self, buf):
        packet_buffer = bytearray()

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

        # fire callback with frame
        self._cb(self._currentFrame)

        # clear our frame holder
        self._currentFrame = bytearray()