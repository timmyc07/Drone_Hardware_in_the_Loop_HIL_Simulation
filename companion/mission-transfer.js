import { common } from "node-mavlink";
const family = (frame) =>
  [0, 5].includes(frame)
    ? "global"
    : [3, 6].includes(frame)
      ? "relative"
      : null;
export class MissionTransfer {
  constructor({ send, verifyItem, diagnostic, onPhase }) {
    Object.assign(this, { sendMessage: send, verifyItem, diagnostic, onPhase });
    this.active = false;
  }
  send(message) {
    this.lastMessage = message;
    this.sendMessage(message);
  }
  requestItem(seq) {
    this.send(
      Object.assign(new common.MissionRequestInt(), {
        targetSystem: 1,
        targetComponent: 1,
        seq,
        missionType: 0,
      }),
    );
  }
  abort(reason) {
    if (!this.active) return;
    this.active = false;
    this.phase = "error";
    clearTimeout(this.deadline);
    clearInterval(this.retry);
    try {
      this.onPhase("error", reason);
    } catch {}
    try {
      this.diagnostic("error", { reason });
    } catch {}
    this.reject(new Error(reason));
  }
  upload(items) {
    if (this.active)
      return Promise.reject(new Error("Mission transfer already active"));
    this.items = items;
    this.requested = new Set();
    this.nextRequest = 0;
    this.downloaded = [];
    this.phase = "uploading";
    this.active = true;
    return new Promise((resolve, reject) => {
      this.resolve = resolve;
      this.reject = reject;
      this.deadline = setTimeout(
        () => this.abort("Mission upload/download wall deadline exceeded"),
        15000,
      );
      this.retry = setInterval(() => {
        if (this.active && this.lastMessage)
          try {
            this.sendMessage(this.lastMessage);
          } catch (e) {
            this.abort(e.message);
          }
      }, 1000);
      try {
        if (!Array.isArray(items) || items.length < 1 || items.length > 53 ||
            Array.from(items).some((item, index) => !item || item.seq !== index))
          throw new Error("Mission items must be a dense sequence of 1..53 items");
        this.onPhase("uploading");
        this.send(
          Object.assign(new common.MissionCount(), {
            targetSystem: 1,
            targetComponent: 1,
            count: items.length,
            missionType: 0,
          }),
        );
      } catch (e) {
        this.abort(e.message);
      }
    });
  }
  handle(id, message) {
    if (!this.active || ![39, 40, 44, 47, 51, 73].includes(id)) return false;
    try {
      if (
        message.missionType !== 0 ||
        message.targetSystem !== 255 ||
        ![0, 190].includes(message.targetComponent)
      )
        throw Error("Unexpected mission transfer target/type");
      if (id === 39) throw Error("Float MISSION_ITEM download is not accepted");
      if (this.phase === "uploading") {
        if (id === 40 || id === 51) {
          const seq = message.seq;
          if (
            !Number.isInteger(seq) ||
            seq < 0 ||
            seq >= this.items.length ||
            seq > this.nextRequest ||
            seq < this.nextRequest - 1
          )
            throw Error("Unexpected mission request sequence");
          this.diagnostic("uploadRequest", { messageId: id, seq });
          this.requested.add(seq);
          this.nextRequest = Math.max(this.nextRequest, seq + 1);
          this.send(
            Object.assign(new common.MissionItemInt(), {
              ...this.items[seq],
              targetSystem: 1,
              targetComponent: 1,
            }),
          );
        } else if (id === 47) {
          this.diagnostic("uploadAck", { result: message.type });
          if (message.type !== 0 || this.requested.size !== this.items.length)
            throw Error("Mission upload ACK failed or incomplete");
          this.phase = "verifying";
          this.onPhase("verifying");
          this.send(
            Object.assign(new common.MissionRequestList(), {
              targetSystem: 1,
              targetComponent: 1,
              missionType: 0,
            }),
          );
        }
      } else if (this.phase === "verifying") {
        if (id === 44) {
          if (message.count !== this.items.length)
            throw Error("Downloaded mission count differs");
          if (!this.downloaded.length) this.requestItem(0);
        } else if (id === 73) {
          if (
            Number.isInteger(message.seq) &&
            message.seq >= 0 &&
            message.seq < this.downloaded.length
          ) {
            const prior = this.downloaded[message.seq];
            if (
              family(message.frame) !== family(prior.frame) ||
              [
                "command",
                "autocontinue",
                "x",
                "y",
                "z",
                "param1",
                "param2",
                "param3",
                "param4",
              ].some((key) => message[key] !== prior[key])
            )
              throw Error("Changed duplicate mission item");
            this.requestItem(this.downloaded.length);
            return true;
          }
          if (message.seq !== this.downloaded.length)
            throw Error("Unexpected downloaded mission sequence");
          this.diagnostic("downloadItem", {
            item: { ...message },
            expected: this.items[message.seq],
          });
          this.verifyItem(message, this.items[message.seq]);
          this.downloaded.push({ ...message });
          if (this.downloaded.length === this.items.length) {
            this.sendMessage(
              Object.assign(new common.MissionAck(), {
                targetSystem: 1,
                targetComponent: 1,
                type: 0,
                missionType: 0,
              }),
            );
            this.diagnostic("verified", { items: this.downloaded });
            this.onPhase("ready");
            this.phase = "ready";
            this.active = false;
            clearTimeout(this.deadline);
            clearInterval(this.retry);
            this.resolve(this.downloaded);
          } else this.requestItem(this.downloaded.length);
        }
      }
    } catch (e) {
      this.abort(e.message);
    }
    return true;
  }
}
