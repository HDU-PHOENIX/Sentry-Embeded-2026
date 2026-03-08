# Copilot Coding Agent Instructions

## Request Limit

The cloud agent (Copilot coding agent) must not exceed **70 requests** (turns) per session. Stop and summarize progress once the limit is reached.

## Project Context

This repository contains the 2026-season Sentry robot embedded firmware (STM32/Keil MDK).

A parallel task involving the RTTView Python tool (located in the developer's local `RTT_Viewer_sourecode/RTTView/` directory; the exact path varies by machine) is being worked on separately. That task aims to integrate Daplink v2 with a GDB Server so that Ozone can be used for debugging, while keeping the core RTT_View functionality intact.

### RTTView Task Goals
1. Enable Daplink v2 to bridge through a GDB Server for Ozone debugging.
2. Preserve existing RTT channel read/display functionality.
3. Implement a USB disconnection/reconnection mechanism.
4. Fall back to OpenOCD if direct GDB integration proves too complex.

### Known State (last session)
- GDB Server startup was failing; a `ConnectionRefusedError` was encountered when testing with Python's `socket` module.
- Files modified: `RTTView.py`, `xlink.py`, `RTTView.ui`.
- Next step: resolve GDB Server startup, then implement the reconnect loop.
