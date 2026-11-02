#!/bin/bash
# Builds the PC version: the same app pages plus pc.js, a launcher and a short read-me.
set -e
cd "$(dirname "$0")"
V=$(grep -o 'versionName="[^"]*"' AndroidManifest.xml | cut -d'"' -f2)
OUT="build/FlyPlan PC"
rm -rf "$OUT" && mkdir -p "$OUT"
cp assets/app.js assets/app.css assets/leaflet.js assets/leaflet.css pc/pc.js "$OUT/"
sed 's#<script src="app.js"></script>#<script src="pc.js"></script>\n<script src="app.js"></script>#' assets/index.html > "$OUT/index.html"
printf '@echo off\r\nrem Opens FlyPlan in its own Microsoft Edge window.\r\nstart "" msedge --app="file:///%%~dp0index.html"\r\n' > "$OUT/FlyPlan.cmd"
sed "s/@VERSION@/$V/" pc/README.txt | sed 's/$/\r/' > "$OUT/Read me.txt"
(cd build && rm -f "FlyPlan PC.zip" && zip -qr "FlyPlan PC.zip" "FlyPlan PC")
echo "built build/FlyPlan PC.zip ($V)"
