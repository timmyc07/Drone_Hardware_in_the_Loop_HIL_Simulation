import {
  common,
  ardupilotmega,
  MavLinkProtocolV1,
  x25crc,
  minimal,
  standard,
  MavLinkPacketSplitter,
  MavLinkPacketParser,
  MavLinkProtocolV2,
} from "node-mavlink";
export const REGISTRY = {
  ...minimal.REGISTRY,
  ...standard.REGISTRY,
  ...common.REGISTRY,
  ...ardupilotmega.REGISTRY,
};
export function packetReader(socket, onPacket, onCrcError) {
  const magicNumbers = Object.fromEntries(
    Object.values(REGISTRY).map((Type) => [Type.MSG_ID, Type.MAGIC_NUMBER]),
  );
  const splitter = new MavLinkPacketSplitter({}, { magicNumbers, onCrcError });
  const parser = new MavLinkPacketParser();
  socket
    .pipe(splitter)
    .pipe(parser)
    .on("data", (packet) => {
      packet.header.magic = packet.buffer[0];
      const Type = REGISTRY[packet.header.msgid];
      if (Type)
        onPacket(
          packet.header,
          packet.protocol.data(packet.payload, Type),
          packet,
        );
    });
  return [splitter, parser];
}
export function mavlinkEncoder() {
  const protocol = new MavLinkProtocolV2(
    INTERNAL_SYSTEM_ID,
    INTERNAL_COMPONENT_ID,
  );
  let sequence = 0;
  return (message) => protocol.serialize(message, sequence++ % 256);
}
export function commandMessage(command, params = []) {
  if (
    !Number.isInteger(command) ||
    command < 0 ||
    command > 65535 ||
    !Array.isArray(params) ||
    params.length > 7 ||
    !Array.from(params).every(Number.isFinite)
  )
    throw new Error("Invalid MAVLink command");
  const message = Object.assign(new common.CommandLong(), {
    targetSystem: 1,
    targetComponent: 1,
    command,
    confirmation: 0,
  });
  for (let i = 1; i <= 7; i++) message[`_param${i}`] = params[i - 1] ?? 0;
  return message;
}

export const INTERNAL_SYSTEM_ID = 255;
export const INTERNAL_COMPONENT_ID = 191;
export const EXTERNAL_COMPONENT_ID = 190;
// UDP datagrams are complete bounded containers, never a continuation of a previous datagram.
export function decodeDatagram(data) {
  if (!Buffer.isBuffer(data) || data.length === 0 || data.length > 8192)
    throw new Error("Invalid MAVLink datagram size");
  const frames = [];
  let offset = 0;
  while (offset < data.length) {
    if (frames.length >= 32)
      throw new Error("Too many MAVLink frames per datagram");
    const v2 = data[offset] === 253,
      headerSize = v2 ? 10 : 6;
    if (!v2 && data[offset] !== 254) throw new Error("Invalid MAVLink framing");
    if (data.length - offset < headerSize)
      throw new Error("Incomplete MAVLink header");
    if (v2 && data[offset + 2] !== 0)
      throw new Error("Signed or unsupported MAVLink packet rejected");
    const length = headerSize + data[offset + 1] + 2;
    if (offset + length > data.length)
      throw new Error("Incomplete MAVLink datagram frame");
    const raw = Buffer.from(data.subarray(offset, offset + length)),
      protocol = v2 ? new MavLinkProtocolV2() : new MavLinkProtocolV1(),
      header = protocol.header(raw),
      Type = REGISTRY[header.msgid];
    header.magic = raw[0];
    if (!Type) throw new Error("Unknown MAVLink dialect message");
    if (protocol.crc(raw) !== x25crc(raw, 1, 2, Type.MAGIC_NUMBER))
      throw new Error("MAVLink CRC mismatch");
    frames.push({
      header,
      message: protocol.data(protocol.payload(raw), Type),
      buffer: raw,
    });
    offset += length;
  }
  return frames;
}
