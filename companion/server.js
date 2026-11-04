import { createServer } from "node:http";
import { randomBytes, timingSafeEqual } from "node:crypto";
import { realpath, stat, readFile } from "node:fs/promises";
import { resolve, sep, extname } from "node:path";
import { fileURLToPath } from "node:url";
import { WebSocketServer, WebSocket } from "ws";

const DIST = resolve(fileURLToPath(new URL("../dist", import.meta.url)));
const LIMIT = 1024 * 1024;
const types = {
  ".html": "text/html; charset=utf-8",
  ".js": "text/javascript; charset=utf-8",
  ".css": "text/css; charset=utf-8",
  ".json": "application/json; charset=utf-8",
  ".svg": "image/svg+xml",
  ".png": "image/png",
  ".ico": "image/x-icon",
  ".wasm": "application/wasm",
};
const plain = (v) =>
  v !== null &&
  typeof v === "object" &&
  !Array.isArray(v) &&
  Object.getPrototypeOf(v) === Object.prototype;
const keys = (v, expected) =>
  plain(v) &&
  Object.keys(v).length === expected.length &&
  expected.every((k) => Object.hasOwn(v, k));

export async function startCompanion({ port = 8787, sessionFactory } = {}) {
  const factory =
    sessionFactory ??
    (async (controller) => {
      const Session =
        controller === "ardupilot"
          ? (await import("./ardupilot-session.js")).ArduPilotSession
          : (await import("./px4-session.js")).Px4Session;
      return new Session();
    });
  let controller = "px4";
  let session = await factory(controller);
  const token = randomBytes(32).toString("base64url");
  let client,
    closing = false;
  const wss = new WebSocketServer({
    noServer: true,
    maxPayload: 64 * 1024,
    perMessageDeflate: false,
  });
  const validToken = (candidate) => {
    const actual = Buffer.from(typeof candidate === "string" ? candidate : "");
    const expected = Buffer.from(token);
    return (
      actual.length === expected.length && timingSafeEqual(actual, expected)
    );
  };
  function allowed(req, originRequired = false) {
    const address = server.address();
    if (!address || typeof address === "string") return false;
    const hosts = [`127.0.0.1:${address.port}`, `localhost:${address.port}`];
    if (!hosts.includes(req.headers.host)) return false;
    const origin = req.headers.origin;
    if (originRequired && !origin) return false;
    if (origin && !hosts.some((host) => origin === `http://${host}`))
      return false;
    return req.headers["sec-fetch-site"] !== "cross-site";
  }
  const send = (ws, value) => {
    if (ws.readyState !== WebSocket.OPEN) return;
    const payload = JSON.stringify(value);
    if (ws.bufferedAmount + Buffer.byteLength(payload) > LIMIT) {
      ws.terminate();
      return;
    }
    ws.send(payload);
  };
  const forwardState = (value) => {
    if (client) send(client, { type: "state", value });
  };
  session.on("state", forwardState);
  const server = createServer(async (req, res) => {
    res.setHeader("X-Content-Type-Options", "nosniff");
    res.setHeader("Cache-Control", "no-store");
    res.setHeader("Content-Security-Policy", "frame-ancestors 'none'");
    res.setHeader("X-Frame-Options", "DENY");
    const fail = (code) => {
      res.writeHead(code);
      res.end();
    };
    if (!allowed(req)) return fail(403);
    if (!["GET", "HEAD"].includes(req.method)) return fail(405);
    try {
      const path = new URL(req.url, "http://localhost").pathname;
      if (path === "/api/bootstrap") {
        res.writeHead(200, {
          "Content-Type": "application/json; charset=utf-8",
        });
        return res.end(
          req.method === "HEAD"
            ? undefined
            : JSON.stringify({
                token,
                wsPath: "/bridge",
                state: session.state(),
              }),
        );
      }
      if (path === "/api/recording") {
        if (!validToken(req.headers["x-flight-token"])) return fail(403);
        const record = await session.exportRecording();
        res.writeHead(200, {
          "Content-Type": "application/json; charset=utf-8",
        });
        return res.end(
          req.method === "HEAD" ? undefined : JSON.stringify(record),
        );
      }
      let decoded = decodeURIComponent(path);
      if (
        decoded.includes("\0") ||
        decoded.includes("\\") ||
        decoded.split("/").includes("..")
      )
        return fail(400);
      if (decoded === "/") decoded = "/index.html";
      const root = await realpath(DIST);
      const file = await realpath(resolve(root, `.${decoded}`));
      if (!file.startsWith(root + sep) || !(await stat(file)).isFile())
        return fail(404);
      const body = await readFile(file);
      res.writeHead(200, {
        "Content-Type": types[extname(file)] || "application/octet-stream",
      });
      res.end(req.method === "HEAD" ? undefined : body);
    } catch {
      if (!res.headersSent) fail(404);
      else res.destroy();
    }
  });
  server.on("upgrade", (req, socket, head) => {
    const reject = (code) => {
      socket.end(
        `HTTP/1.1 ${code} ${code === 409 ? "Conflict" : "Forbidden"}\r\nConnection: close\r\n\r\n`,
      );
    };
    try {
      const url = new URL(req.url, "http://localhost");
      if (
        url.pathname !== "/bridge" ||
        !allowed(req, true) ||
        !validToken(url.searchParams.get("token")) ||
        [...url.searchParams.keys()].some((k) => k !== "token") ||
        url.searchParams.getAll("token").length !== 1
      )
        return reject(403);
      if (client) return reject(409);
      wss.handleUpgrade(req, socket, head, (ws) =>
        wss.emit("connection", ws, req),
      );
    } catch {
      reject(403);
    }
  });
  wss.on("connection", (ws) => {
    client = ws;
    let lastPong = Date.now();
    let chain = Promise.resolve();
    let pending = 0;
    ws.on("error", () => {});
    ws.on("pong", () => {
      lastPong = Date.now();
    });
    const heartbeat = setInterval(() => {
      if (Date.now() - lastPong >= 10000 || ws.bufferedAmount > LIMIT)
        return ws.terminate();
      ws.ping();
    }, 5000);
    heartbeat.unref();
    ws.on("message", (data, isBinary) => {
      if (pending >= 32) {
        ws.close(1008, "Command queue full");
        return;
      }
      pending++;
      chain = chain
        .then(async () => {
          if (ws.readyState !== WebSocket.OPEN) return;
          let command;
          try {
            if (isBinary) throw Error("Invalid command");
            command = JSON.parse(data.toString());
            if (
              !plain(command) ||
              !Number.isSafeInteger(command.id) ||
              command.id <= 0
            )
              throw Error("Invalid command");
            let value;
            if (command.type === "inspect" && keys(command, ["id", "type"]))
              value = session.state();
            else if (
              command.type === "start" &&
              keys(command, ["id", "type", "value"]) &&
              keys(command.value, ["controller", "binaryPath", "config"]) &&
              ["px4", "ardupilot"].includes(command.value.controller) &&
              typeof command.value.binaryPath === "string" &&
              plain(command.value.config)
            ) {
              if (
                session.state().phase !== "stopped" &&
                command.value.controller === controller
              )
                value = await session.start(command.value);
              else {
                if (!["idle", "stopped"].includes(session.state().phase))
                  throw Error("Stop the current controller before switching");
                await session.close();
                const candidate = await factory(command.value.controller);
                try {
                  value = await candidate.start(command.value);
                  if (closing) throw Error("Companion is closing");
                } catch (error) {
                  await Promise.resolve(candidate.close()).catch(() => {});
                  throw error;
                }
                session.off("state", forwardState);
                session = candidate;
                controller = command.value.controller;
                session.on("state", forwardState);
                forwardState(session.state());
              }
            } else if (
              command.type === "groundStation" &&
              keys(command, ["id", "type", "value"]) &&
              keys(command.value, ["enabled", "targetPort", "owner"]) &&
              typeof command.value.enabled === "boolean" &&
              Number.isInteger(command.value.targetPort) &&
              command.value.targetPort >= 1024 &&
              command.value.targetPort <= 65535 &&
              ["web", "gcs"].includes(command.value.owner) &&
              (command.value.enabled || command.value.owner === "web") &&
              typeof session.configureGroundStation === "function"
            ) {
              value = await session.configureGroundStation(command.value);
            } else if (
              command.type === "mission" &&
              keys(command, ["id", "type", "value"]) &&
              typeof session.uploadMission === "function" &&
              Array.isArray(command.value) &&
              command.value.length >= 1 &&
              command.value.length <= 50 &&
              Array.from(command.value).every(
                (point) =>
                  keys(point, ["north", "east", "altitude", "hold"]) &&
                  Object.values(point).every(Number.isFinite),
              )
            ) {
              if (session.state().groundStation?.owner === "gcs")
                throw Error(
                  "The external ground station owns flight and mission control",
                );
              value = await session.uploadMission(command.value);
            } else if (
              command.type === "command" &&
              keys(command, ["id", "type", "value"]) &&
              [
                "arm",
                "takeoff",
                "land",
                "disarm",
                "pause",
                "resume",
                "stop",
                "mission",
              ].includes(command.value)
            ) {
              if (
                session.state().groundStation?.owner === "gcs" &&
                !["pause", "resume", "stop"].includes(command.value)
              )
                throw Error(
                  "The external ground station owns flight and mission control",
                );
              value = await session.command(command.value);
            } else throw Error("Invalid command");
            send(ws, {
              type: "reply",
              id: command.id,
              ok: true,
              value: value ?? null,
            });
          } catch (error) {
            send(ws, {
              type: "reply",
              id:
                Number.isSafeInteger(command?.id) && command.id > 0
                  ? command.id
                  : null,
              ok: false,
              error: error.message,
            });
          }
        })
        .finally(() => {
          pending--;
        });
    });
    ws.on("close", () => {
      clearInterval(heartbeat);
      chain.finally(async () => {
        if (!closing && session.state().groundStation?.owner !== "gcs") {
          try {
            await session.command("pause");
          } catch {}
        }
        if (client === ws) client = undefined;
      });
    });
    send(ws, { type: "state", value: session.state() });
  });
  try {
    await new Promise((resolveListen, reject) => {
      server.once("error", reject);
      server.listen(port, "127.0.0.1", resolveListen);
    });
  } catch (error) {
    await Promise.resolve(session.close()).catch(() => {});
    wss.close();
    throw error;
  }
  const address = server.address();
  async function close() {
    closing = true;
    for (const ws of wss.clients) ws.terminate();
    try {
      await session.close();
    } finally {
      await new Promise((resolveClose) => server.close(resolveClose));
      wss.close();
    }
  }
  return { server, address, close };
}

export async function runCompanionCli(
  start = startCompanion,
  signals = process,
) {
  const { address, close } = await start();
  console.log(`http://127.0.0.1:${address.port}/`);
  let stopping = false;
  const stop = async () => {
    if (stopping) return;
    stopping = true;
    try {
      await close();
    } catch (error) {
      console.error(error.message);
      signals.exitCode = 1;
    }
  };
  signals.on("SIGINT", stop);
  signals.on("SIGTERM", stop);
}

if (
  process.argv[1] &&
  resolve(process.argv[1]) === fileURLToPath(import.meta.url)
) {
  runCompanionCli().catch((error) => {
    console.error(error.message);
    process.exitCode = 1;
  });
}
