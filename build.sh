#!/bin/bash
set -e
cd "$(dirname "$0")"
SDK=/usr/lib/android-sdk; JAR=$SDK/platforms/android-23/android.jar; BT=$SDK/build-tools/29.0.3
rm -rf build && mkdir -p build/gen build/classes
$BT/aapt package -f -0 arsc -M AndroidManifest.xml -S res -A assets -I $JAR -F build/unsigned.apk -J build/gen
javac -nowarn -Xlint:-options -source 8 -target 8 -bootclasspath $JAR -d build/classes $(find src build/gen -name '*.java')
$BT/dx --dex --output=build/classes.dex build/classes
(cd build && $BT/aapt add unsigned.apk classes.dex >/dev/null)
$BT/zipalign -f -p 4 build/unsigned.apk build/aligned.apk
[ -f release.keystore ] || keytool -genkeypair -keystore release.keystore -storepass waypoint -keypass waypoint -alias planner -keyalg RSA -keysize 2048 -validity 10000 -dname "CN=Waypoint Planner" >/dev/null 2>&1
$BT/apksigner sign --ks release.keystore --ks-pass pass:waypoint --key-pass pass:waypoint --out build/WaypointPlanner.apk build/aligned.apk
$BT/apksigner verify --verbose build/WaypointPlanner.apk | head -5
