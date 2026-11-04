const PREARM = 0x10000000;
const MAX_AGE_MS = 5000;
const uint = (value, max) =>
  Number.isInteger(value) && value >= 0 && value <= max;
const utf8 = new TextDecoder("utf-8", { fatal: true });

export class ControllerHealth {
  #kind;
  #now;
  #heartbeat = null;
  #sysStatus = null;
  #messages = [];

  constructor(kind, now = () => performance.now()) {
    if (!["px4", "ardupilot"].includes(kind) || typeof now !== "function")
      throw new Error("Invalid controller health configuration");
    this.#kind = kind;
    this.#now = now;
  }

  update(id, message) {
    if (!message || typeof message !== "object" || Array.isArray(message))
      return false;
    try {
      const receivedAt = this.#now();
      if (!Number.isFinite(receivedAt)) return false;
      if (id === 0) {
        if (
          !uint(message.baseMode, 255) ||
          message.autopilot !== (this.#kind === "px4" ? 12 : 3)
        )
          return false;
        this.#heartbeat = { receivedAt, armed: (message.baseMode & 128) !== 0 };
      } else if (id === 1) {
        const present = message.onboardControlSensorsPresent;
        const enabled = message.onboardControlSensorsEnabled;
        const health = message.onboardControlSensorsHealth;
        if (
          ![present, enabled, health].every((value) => uint(value, 0xffffffff))
        )
          return false;
        this.#sysStatus = {
          receivedAt,
          present,
          enabled,
          health,
          unhealthyMask: (enabled & ~health) >>> 0,
        };
      } else if (id === 253) {
        const { severity, text: latin1, id: textId, chunkSeq } = message;
        if (
          !uint(severity, 7) ||
          !uint(textId, 65535) ||
          !uint(chunkSeq, 255) ||
          typeof latin1 !== "string" ||
          latin1.length > 50 ||
          Array.from(latin1).some((char) => char.charCodeAt(0) > 255)
        )
          return false;
        // node-mavlink 2.3.0 maps each wire byte to a Latin1 codepoint, not UTF-8.
        const bytes = Buffer.from(latin1.split("\0", 1)[0], "latin1");
        let text = null;
        try {
          text = utf8
            .decode(bytes)
            .replace(/[\u0000-\u001f\u007f-\u009f]/g, " ");
        } catch {
          /* A split UTF-8 codepoint stays a raw fragment; never invent replacement text. */
        }
        this.#messages.push({
          receivedAt,
          severity,
          text,
          rawBytesHex: bytes.toString("hex"),
          id: textId,
          chunkSeq,
          chunked: textId !== 0,
        });
        if (this.#messages.length > 20) this.#messages.shift();
      } else return false;
      return true;
    } catch {
      return false;
    }
  }

  snapshot({ verified, live } = {}) {
    const now = this.#now();
    const age = (entry) =>
      entry === null ? null : Math.max(0, now - entry.receivedAt);
    const heartbeatAgeMs = age(this.#heartbeat);
    const sysStatus =
      this.#sysStatus === null
        ? null
        : {
            present: this.#sysStatus.present,
            enabled: this.#sysStatus.enabled,
            health: this.#sysStatus.health,
            ageMs: age(this.#sysStatus),
            unhealthyMask: this.#sysStatus.unhealthyMask,
          };
    let prearm = "unknown";
    if (
      verified === true &&
      live === true &&
      heartbeatAgeMs !== null &&
      heartbeatAgeMs <= MAX_AGE_MS
    ) {
      if (this.#heartbeat.armed) prearm = "armed";
      else if (sysStatus !== null && sysStatus.ageMs <= MAX_AGE_MS) {
        // Pinned PX4 reports only the health bit; AP advertises whether arming checks are enabled.
        prearm =
          this.#kind === "ardupilot" && (sysStatus.enabled & PREARM) === 0
            ? "checks-disabled"
            : (sysStatus.health & PREARM) !== 0
              ? "reported-ok"
              : "failed";
      }
    }
    return {
      sysStatus,
      heartbeatAgeMs,
      prearm,
      messages: this.#messages.map(({ receivedAt, ...message }) => ({
        ...message,
        ageMs: Math.max(0, now - receivedAt),
      })),
    };
  }
}
