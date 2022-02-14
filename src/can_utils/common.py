def make_can_frame(node, index, sub_index=0, data=0, write=True):
    data = int(data)
    frame = bytearray(2)
    protocol_payload = bytearray([6])
    node_payload = bytearray([node])

    if write:
        mode_payload = bytearray([0x22])
    else:
        mode_payload = bytearray([0x40])

    index_payload = bytearray([index & 0xFF, (index >> 8) & 0xFF])
    sub_index_payload = bytearray([sub_index])
    data_payload = bytearray([data & 0xFF, (data >> 8) & 0xFF, (data >> 16) & 0xFF, (data >> 24) & 0xFF])
    payload = frame + protocol_payload + node_payload + mode_payload + index_payload + sub_index_payload + data_payload
    payload.append(0x08)

    return payload
