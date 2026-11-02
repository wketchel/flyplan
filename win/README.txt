FlyPlan: Mapping & Waypoints for Windows  (version 1.34)

Plans DJI waypoint and mapping missions and sends them to the controller over USB.
Not made by or affiliated with DJI. No warranty: check every route in DJI Fly and fly at your own risk.

SETUP
1. Unzip this folder somewhere permanent, such as Documents\FlyPlan.
   Keep "WebView2Loader.dll" next to "FlyPlan.exe".
2. Double-click "FlyPlan.exe". The first time, Windows SmartScreen may say it
   protected your PC: click "More info", then "Run anyway" (the app isn't code-signed).
3. If it says the map view couldn't start, install the free "Microsoft Edge
   WebView2 Runtime" from microsoft.com. Windows 10 and 11 usually have it.

SENDING A MISSION TO THE CONTROLLER
1. One time only: in DJI Fly, open Waypoints, drop two points and save. That's
   your placeholder.
2. Close DJI Fly on the controller.
3. Plug the controller into the PC with a USB-C data cable, unlock it, and set
   USB to "File transfer" if it asks.
4. Close any File Explorer window showing the controller.
5. In FlyPlan tap Send to controller > Find controller, pick the placeholder,
   and tap Replace.
6. In DJI Fly, open the mission, check the route, and save it.
Each placeholder is backed up first to Documents\FlyPlan\Backups.

SYNC WITH A TABLET (Google Drive)
- Install Google Drive for desktop. Sync > Create sync file (first device) or
  Use existing file (second device), and pick the file in G:\My Drive.
- Markers, boundaries, saved missions and photos sync. Map files don't.

YOUR OWN ORTHOMOSAIC
- Map menu > My maps > Import MBTiles. The file is read where it is, so leave
  it in place.

The app's own data lives in %LOCALAPPDATA%\Waypoint Planner.
