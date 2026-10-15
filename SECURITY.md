# Security and operational boundaries

Report vulnerabilities privately through the contact form at https://emplif.ai/contact
with a minimal reproduction and affected commit. Do not attach credentials or
private robot footage. Do not open a public issue containing an exploitable secret.

The web console binds to loopback only. It checks Host, Origin, JSON content type
and a per-process request token; it is a single-user local tool, not a multi-user
hosted service. Anyone with access to the same OS account can access its session.
Do not tunnel, reverse-proxy or expose it to the Internet. The legacy robot bridge
uses an operator-IP allowlist on an isolated robot network, not encrypted user
authentication. Do not expose its port to untrusted clients.

Keys are entered in the local UI, sent to the local process, and held in memory.
The Gemini adapter sends them only in HTTPS headers to Google's API. The console
never writes keys to browser storage, files or access logs. Disconnect clears the
stored reference; an in-flight HTTP request can retain a copy until it finishes.
Python does not guarantee cryptographic erasure of process memory.
Images and prompts are transmitted to Google only for explicit observations or
an explicitly started practice session. `store=false` is requested; the provider's
service and data terms still apply. No analytics or third-party UI scripts load.

The default web mode cannot start robot motion. Explicit `--enable-motion`,
hardware setup, fresh telemetry, camera health and per-run operator checks are
required for bounded arm practice. Stop cancels delayed responses and requests a
robot-local hold; it is not a certified emergency stop. If acknowledgement fails,
use the physical remote. Closing the browser does not stop a running controller;
use Stop / Escape, or Ctrl+C in the console terminal. The gateway's heartbeat
watchdog remains essential.

The retired one-click panel is retained as historical experimental source and
refuses startup. Its full pick-and-place macro is not exposed by the new console.
