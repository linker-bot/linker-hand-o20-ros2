import struct

POLY = 0x8005
INIT = 0x0000

def crc16(data: bytes) -> int:
    crc = INIT
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ POLY if crc & 1 else crc >> 1
    return crc & 0xFFFF

def build_read_force(dev_id: int = 0x01) -> str:
    header = b'\xFF\xFF\xFD\x00'
    dev    = bytes([dev_id])
    cmd    = b'\x02'
    param  = b'\x90\x00\x02\x00'      # 手册原参数 4 字节
    length = (len(param) + 3).to_bytes(2, 'little')  # 4+3=7 → 07 00

    crc_body = dev + length + cmd + param
    crc_val  = crc16(crc_body).to_bytes(2, 'little')

    frame = header + crc_body + crc_val
    return frame.hex().upper()

# -------------------------
if __name__ == '__main__':
    print(build_read_force(0x01))