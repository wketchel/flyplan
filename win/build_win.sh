#!/bin/bash
# Builds the Windows app from Linux. Toolchain notes (this sandbox): mingw-w64, apt rustc-1.89 + rust-1.89-src
# (Debian strips std's windows-targets dependency: re-add it to library/std/Cargo.toml), rsbegin.o/rsend.o compiled
# into lib/rustlib/x86_64-pc-windows-gnu/lib/self-contained, and /usr/local/bin/mingw-rust-link to pass them to gcc.
set -e
cd "$(dirname "$0")"
RUSTC=/usr/lib/rust-1.89/bin/rustc RUSTC_BOOTSTRAP=1 cargo build --release
