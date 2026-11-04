import {
  common,
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
      const Type = REGISTRY[packet.header.msgid];
      if (Type)
        onPacket(packet.header, packet.protocol.data(packet.payload, Type));
    });
  return [splitter, parser];
}
export function mavlinkEncoder() {
  const protocol = new MavLinkProtocolV2(255, 190);
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
