## [Explore the web-based readme](http://zerocm.github.io/zcm/)



# ZCM: Zero Communications and Marshalling

[![Build Status](https://github.com/ZeroCM/zcm/workflows/Build/badge.svg)](https://github.com/ZeroCM/zcm/actions)

ZCM is a micro-framework for message-passing and data-marshalling, designed originally
for robotics systems where high-bandwidth and low-latency are critical and the variance in
compute platforms is large.

ZCM is a publish/subscribe message-passing system with automatic message type-checking and
serialization. It provides bindings for a variety of programming languages, and generates
language-specific message serialization routines. Using a ZCM message feels natural
in each language.

ZCM is transport-agnostic. There is no required built-in transport. Every transport is
first-class. This is achieved by defining strict blocking and non-blocking transport APIs. As
long as a transport implementation conforms to this API, it should work flawlessly with ZCM.
This design allows ZCM to work well on anything from a high-end posix-based compute cluster
with thousands of nodes to a low-end real-time embedded-system with no operating system.

ZCM is a derivation of the [LCM project](http://lcm-proj.github.io/) created in 2006 by
the MIT DARPA Urban Challenge team. The core message-type system, publish/subscribe APIs,
and basic tools are ported directly from LCM and remain about 95% compatible. While there
are a handful of subtle differences between the two, the core distinguishing feature is
ZCM's transport agnosticism. LCM is designed completely around UDP Multicast. This transport
makes a lot of sense for LAN connected compute clusters (such the original 2006 MIT DGC
Vechicle).  However, there are many other applications that are interesting targets for
ZCM messaging.  These include: local system messaging (IPC), multi-threaded messaging
(in-process), embedded-system peripherals (UART, I2C, etc), and web applications
(Web Sockets).  By refusing to make hard assumptions about the transport layer, ZCM opens
the door to a wide set of use-cases that were neither possible nor practical with LCM.

To learn more about what ZCM tries to be, and its guiding principles, check out the
[Project Philosphy](docs/philosophy.md).

To dive, in and see some examples, check out the [Tutorial](docs/tutorial.md).

If you have previously used LCM, check out [From LCM to ZCM](docs/lcm_to_zcm.md).

To learn how you can contribute to this project, check out [Contributing](docs/contributing.md)

## Quick Links
 - [Project Philosphy](docs/philosophy.md)
 - [Tutorial](docs/tutorial.md)
 - [From LCM to ZCM](docs/lcm_to_zcm.md)
 - [ZCM Type System](docs/zcmtypesys.md)
 - [Transport Layer](docs/transports.md)
 - [Embedded Applications](docs/embedded.md)
 - Web Applications (coming soon)
 - [Dependencies & Building](docs/building.md)
 - [Tools](docs/tools.md)
 - [Frequently Asked Questions](docs/FAQs.md)
 - [Continuous Integration](https://github.com/ZeroCM/zcm/actions/workflows/build.yml)
 - [Contributing](docs/contributing.md)

## Features
 - Type-safe and version-safe message serialization
 - A useful suite of tools for logging, log-playback, and real-time message inspection (spy)
 - A wide set of optionally built-in transports including UDP Multicast, IPC, In-Process, Serial, and ZeroMQ
 - A well-defined interface for building custom transports
 - Strong support for embedded applications. The core embedded code is restricted to C89.
 - Only one true dependency: A modern C++11 compiler for the non-embedded code.

## Supported platforms and languages
 - Platforms
   - GNU/Linux
   - Web browsers supporting the Websocket API
   - Any C89 capable embedded system
 - Languages
   - C89 and greater
   - C++
   - Java
   - MATLAB (using Java)
   - NodeJS and Client-side Javascript
   - Python
   - Julia (both v1.6.0 and v0.6.4)

## Roadmap
 - Platform Support
   - Windows
   - OS X
   - Any POSIX-1.2001 system (e.g., Cygwin, Solaris, BSD, etc.)
 - Consider porting the rest of the LCM languages
   - C#
   - Lua
 - Explore alternative messaging paradigms using ZCM Types (e.g. those found in ZeroMQ)
 - Break from the original LCM APIs to improve API consistency
   - Goal for v2.0
   - v1.0 will **always** strive for API compatibility

## Subtle differences to LCM

ZCM is approximately 95% API compatible with LCM. Porting existing Unix-based LCM
programs to ZCM is very easy in many cases. A quick `sed -i 's/lcm/zcm/g'` works for
most applications. ZCM uses the same binary-compatible formats for UDP Multicast, Logging,
and ZCMType encodings. Thus LCM and ZCM applications can communicate flawlessly. This
allows LCM users to gradually migrate to ZCM.

### Known incompatibilities:
 - `zcm_get_fileno()` is not supported
 - `zcm_handle_timeout()` is not supported
 - Any applications using GLib via LCM may have build errors
   - ZCM does *not* depend on GLib
 - ZCMType drops support for the LCMType-style enums
