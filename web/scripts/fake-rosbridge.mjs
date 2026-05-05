// Dev-only stand-in for rosbridge_server. Speaks just enough of the rosbridge
// v2.0 protocol (https://github.com/RobotWebTools/rosbridge_suite/blob/ros2/ROSBRIDGE_PROTOCOL.md)
// to let the webapp connect, advertise /tour_config, and subscribe to /tour_status.
//
// Usage:
//   node scripts/fake-rosbridge.mjs
//
// Then push specific /tour_status payloads by POSTing JSON to the HTTP control
// port:
//   curl -X POST http://localhost:9091/status -H 'Content-Type: application/json' \
//     -d '{"state":"NAVIGATING","current_target":"stop_3","remaining":["stop_5"],"visited":["stop_1"],"last_event":"navigating to stop_3","timestamp":0}'
//
// /tour_config publishes from the page are logged to stdout.

import http from "node:http";
import { WebSocketServer } from "ws";

const WS_PORT = 9090;
const HTTP_PORT = 9091;

const subscribers = new Set();

const wss = new WebSocketServer({ port: WS_PORT });
wss.on("connection", (socket) => {
  console.log(`[ws] client connected (${wss.clients.size} total)`);
  socket.on("message", (raw) => {
    let msg;
    try {
      msg = JSON.parse(raw.toString());
    } catch {
      console.warn("[ws] non-JSON frame:", raw.toString());
      return;
    }
    switch (msg.op) {
      case "advertise":
        console.log(`[ws] advertise ${msg.topic} (${msg.type})`);
        break;
      case "unadvertise":
        console.log(`[ws] unadvertise ${msg.topic}`);
        break;
      case "subscribe":
        console.log(`[ws] subscribe ${msg.topic} (${msg.type})`);
        if (msg.topic === "/tour_status" || msg.topic === "tour_status") {
          subscribers.add(socket);
        }
        break;
      case "unsubscribe":
        console.log(`[ws] unsubscribe ${msg.topic}`);
        subscribers.delete(socket);
        break;
      case "publish":
        console.log(`[ws] publish ${msg.topic}: ${JSON.stringify(msg.msg)}`);
        break;
      default:
        console.log("[ws] unhandled op:", msg.op);
    }
  });
  socket.on("close", () => {
    subscribers.delete(socket);
    console.log(`[ws] client disconnected (${wss.clients.size} remaining)`);
  });
});
console.log(`[ws] fake rosbridge listening on ws://localhost:${WS_PORT}`);

http
  .createServer((req, res) => {
    if (req.method !== "POST" || req.url !== "/status") {
      res.statusCode = 404;
      res.end("not found");
      return;
    }
    let body = "";
    req.on("data", (chunk) => (body += chunk));
    req.on("end", () => {
      let parsed;
      try {
        parsed = JSON.parse(body);
      } catch (e) {
        res.statusCode = 400;
        res.end("bad json: " + e.message);
        return;
      }
      const frame = JSON.stringify({
        op: "publish",
        topic: "/tour_status",
        msg: { data: JSON.stringify(parsed) },
      });
      let sent = 0;
      for (const sub of subscribers) {
        if (sub.readyState === sub.OPEN) {
          sub.send(frame);
          sent++;
        }
      }
      console.log(`[http] pushed /tour_status to ${sent} subscriber(s)`);
      res.statusCode = 200;
      res.end(`pushed to ${sent}\n`);
    });
  })
  .listen(HTTP_PORT, () => {
    console.log(`[http] control endpoint at http://localhost:${HTTP_PORT}/status`);
  });
