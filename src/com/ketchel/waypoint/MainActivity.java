package com.ketchel.waypoint;

import android.Manifest;
import android.app.Activity;
import android.app.PendingIntent;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.pm.PackageManager;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbManager;
import android.location.Location;
import android.location.LocationListener;
import android.location.LocationManager;
import android.net.Uri;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.provider.DocumentsContract;
import android.webkit.JavascriptInterface;
import android.webkit.WebResourceRequest;
import android.webkit.WebResourceResponse;
import android.webkit.WebChromeClient;
import android.webkit.WebSettings;
import android.webkit.WebView;
import android.webkit.WebViewClient;

import org.json.JSONArray;
import org.json.JSONObject;

import java.io.OutputStream;

public class MainActivity extends Activity {
    static final String ACTION_USB_PERMISSION = "com.ketchel.waypoint.USB_PERMISSION";
    static final int REQ_SAVE = 41;
    static final int REQ_LOC = 42;
    static final int REQ_TREE = 43;
    static final int REQ_OPEN = 44;
    static final int REQ_MB = 45;
    static final int REQ_SYNC = 46;
    static final int REQ_PHOTO = 47;
    static final int REQ_MEDIALOC = 48;

    WebView web;
    Handler main;
    UsbManager usb;
    Runnable pendingUsbAction;
    byte[] pendingSave;
    volatile String importPurpose = "mission";
    boolean pageReady;
    boolean usbAttachedAtLaunch;
    TileCache tiles;
    LocalStore local;
    MbTiles mb;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        main = new Handler(Looper.getMainLooper());
        usb = (UsbManager) getSystemService(Context.USB_SERVICE);
        tiles = new TileCache(this);
        TileCache.usgsMax = getSharedPreferences("maps", MODE_PRIVATE).getInt("usgsMax", 16);
        local = new LocalStore(this);
        mb = new MbTiles(this);

        web = new WebView(this);
        WebSettings ws = web.getSettings();
        ws.setJavaScriptEnabled(true);
        ws.setDomStorageEnabled(true);
        ws.setAllowFileAccess(true);
        ws.setCacheMode(WebSettings.LOAD_DEFAULT);
        ws.setBuiltInZoomControls(false);
        web.setWebChromeClient(new WebChromeClient() {
            @Override
            public boolean onShowFileChooser(WebView v, android.webkit.ValueCallback<Uri[]> cb, FileChooserParams p) {
                if (fileCb != null) fileCb.onReceiveValue(null);
                fileCb = cb;
                fileMulti = p.getMode() == FileChooserParams.MODE_OPEN_MULTIPLE;
                // Photo locations are hidden from apps unless they hold this permission (Android 10+).
                if (Build.VERSION.SDK_INT >= 29 && checkSelfPermission("android.permission.ACCESS_MEDIA_LOCATION") != PackageManager.PERMISSION_GRANTED
                        && !getSharedPreferences("maps", MODE_PRIVATE).getBoolean("askedMediaLoc", false)) {
                    getSharedPreferences("maps", MODE_PRIVATE).edit().putBoolean("askedMediaLoc", true).apply();
                    requestPermissions(new String[]{"android.permission.ACCESS_MEDIA_LOCATION"}, REQ_MEDIALOC);
                    return true;
                }
                openPhotoChooser();
                return true;
            }
        });
        web.setWebViewClient(new WebViewClient() {
            @Override
            public WebResourceResponse shouldInterceptRequest(WebView view, WebResourceRequest req) {
                String u = req.getUrl().toString();
                WebResourceResponse m = mb.intercept(u);
                if (m != null) return m;
                WebResourceResponse r = tiles.intercept(u);
                return r != null ? r : super.shouldInterceptRequest(view, req);
            }

            @Override
            public void onPageFinished(WebView view, String url) {
                pageReady = true;
                if (usbAttachedAtLaunch) {
                    usbAttachedAtLaunch = false;
                    emit(obj("type", "usbAttached"));
                }
            }
        });
        web.addJavascriptInterface(new Bridge(), "Android");
        web.setBackgroundColor(0xFF0B3A4A);
        web.loadUrl("file:///android_asset/index.html");
        setContentView(web);

        IntentFilter f = new IntentFilter(ACTION_USB_PERMISSION);
        try {
            if (Build.VERSION.SDK_INT >= 33) {
                Context.class.getMethod("registerReceiver", BroadcastReceiver.class, IntentFilter.class, int.class)
                        .invoke(this, usbReceiver, f, 2 /* RECEIVER_EXPORTED */);
            } else {
                registerReceiver(usbReceiver, f);
            }
        } catch (Exception e) {
            registerReceiver(usbReceiver, f);
        }

        if (checkSelfPermission(Manifest.permission.ACCESS_FINE_LOCATION) != PackageManager.PERMISSION_GRANTED) {
            requestPermissions(new String[]{Manifest.permission.ACCESS_FINE_LOCATION, Manifest.permission.ACCESS_COARSE_LOCATION}, REQ_LOC);
        }
        handleIntent(getIntent());
    }

    @Override
    protected void onNewIntent(Intent intent) {
        super.onNewIntent(intent);
        handleIntent(intent);
    }

    void handleIntent(Intent i) {
        if (i != null && UsbManager.ACTION_USB_DEVICE_ATTACHED.equals(i.getAction())) {
            if (pageReady) emit(obj("type", "usbAttached"));
            else usbAttachedAtLaunch = true;
        }
    }

    @Override
    protected void onDestroy() {
        try { unregisterReceiver(usbReceiver); } catch (Exception ignored) { }
        super.onDestroy();
    }

    @Override
    public void onBackPressed() {
        moveTaskToBack(true);
    }

    final BroadcastReceiver usbReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            if (!ACTION_USB_PERMISSION.equals(intent.getAction())) return;
            boolean granted = intent.getBooleanExtra(UsbManager.EXTRA_PERMISSION_GRANTED, false);
            final Runnable r = pendingUsbAction;
            pendingUsbAction = null;
            if (granted && r != null) new Thread(r).start();
            else if (!granted) emit(err("USB access was not allowed. Tap Find controller and choose Allow."));
        }
    };

    /* ---------- helpers ---------- */
    static JSONObject obj(String... kv) {
        JSONObject o = new JSONObject();
        try { for (int i = 0; i + 1 < kv.length; i += 2) o.put(kv[i], kv[i + 1]); } catch (Exception ignored) { }
        return o;
    }
    static JSONObject err(String msg) { return obj("type", "error", "message", msg); }

    void emit(final JSONObject o) {
        main.post(new Runnable() {
            @Override
            public void run() {
                web.evaluateJavascript("window.onNative && window.onNative(" + JSONObject.quote(o.toString()) + ")", null);
            }
        });
    }

    /** Ask for USB permission if needed, then run the action on a background thread. */
    void withUsb(final UsbDevice dev, final Runnable action) {
        if (usb.hasPermission(dev)) { new Thread(action).start(); return; }
        pendingUsbAction = action;
        Intent i = new Intent(ACTION_USB_PERMISSION);
        i.setPackage(getPackageName());
        int flags = PendingIntent.FLAG_UPDATE_CURRENT;
        if (Build.VERSION.SDK_INT >= 31) flags |= 0x02000000; /* FLAG_MUTABLE: system adds the grant extra */
        PendingIntent pi = PendingIntent.getBroadcast(this, 0, i, flags);
        usb.requestPermission(dev, pi);
    }

    /* ---------- JavaScript bridge ---------- */
    /* ---------- photos: gallery or camera for <input type=file> ---------- */
    android.webkit.ValueCallback<Uri[]> fileCb;
    boolean fileMulti;
    Uri cameraUri;

    void openPhotoChooser() {
        Intent pick = new Intent(Intent.ACTION_GET_CONTENT);
        pick.addCategory(Intent.CATEGORY_OPENABLE);
        pick.setType("image/*");
        if (fileMulti) pick.putExtra(Intent.EXTRA_ALLOW_MULTIPLE, true);
        Intent chooser = Intent.createChooser(pick, "Add photos");
        cameraUri = null;
        try {
            android.content.ContentValues cv = new android.content.ContentValues();
            cv.put(android.provider.MediaStore.Images.Media.DISPLAY_NAME, "Waypoint_" + System.currentTimeMillis() + ".jpg");
            cv.put(android.provider.MediaStore.Images.Media.MIME_TYPE, "image/jpeg");
            cameraUri = getContentResolver().insert(android.provider.MediaStore.Images.Media.EXTERNAL_CONTENT_URI, cv);
            if (cameraUri != null) {
                Intent cam = new Intent(android.provider.MediaStore.ACTION_IMAGE_CAPTURE);
                cam.putExtra(android.provider.MediaStore.EXTRA_OUTPUT, cameraUri);
                cam.addFlags(Intent.FLAG_GRANT_WRITE_URI_PERMISSION);
                chooser.putExtra(Intent.EXTRA_INITIAL_INTENTS, new Intent[]{cam});
            }
        } catch (Exception e) { cameraUri = null; }
        try { startActivityForResult(chooser, REQ_PHOTO); }
        catch (Exception e) { finishPhotos(null); }
    }

    /** Asks Android for the original file (with GPS) when the app may see photo locations. */
    Uri original(Uri u) {
        if (Build.VERSION.SDK_INT < 29 || checkSelfPermission("android.permission.ACCESS_MEDIA_LOCATION") != PackageManager.PERMISSION_GRANTED) return u;
        try {
            Uri m = u;
            if (DocumentsContract.isDocumentUri(this, u)) {
                Object r = android.provider.MediaStore.class.getMethod("getMediaUri", Context.class, Uri.class).invoke(null, this, u);
                if (r != null) m = (Uri) r;
            }
            if (!"media".equals(m.getAuthority())) return u;
            return (Uri) android.provider.MediaStore.class.getMethod("setRequireOriginal", Uri.class).invoke(null, m);
        } catch (Throwable t) { return u; }
    }

    void finishPhotos(Uri[] uris) {
        if (fileCb != null) fileCb.onReceiveValue(uris);
        fileCb = null;
    }

    @Override
    public void onRequestPermissionsResult(int requestCode, String[] permissions, int[] results) {
        super.onRequestPermissionsResult(requestCode, permissions, results);
        if (requestCode == REQ_MEDIALOC) openPhotoChooser();
    }

    /* ---------- sync file (Google Drive through the system file picker) ---------- */
    android.content.SharedPreferences syncPrefs() { return getSharedPreferences("sync", MODE_PRIVATE); }
    Uri syncUri() { String u = syncPrefs().getString("uri", null); return u == null ? null : Uri.parse(u); }

    void syncError(String code, String msg) { emit(obj("type", "syncError", "code", code, "message", msg)); }

    void syncReadNow() {
        Uri u = syncUri();
        if (u == null) { syncError("none", "No sync file is set."); return; }
        try {
            java.io.InputStream in = getContentResolver().openInputStream(u);
            if (in == null) throw new java.io.FileNotFoundException();
            java.io.ByteArrayOutputStream bo = new java.io.ByteArrayOutputStream();
            byte[] buf = new byte[16384];
            int n;
            while ((n = in.read(buf)) > 0) {
                bo.write(buf, 0, n);
                if (bo.size() > 200 * 1024 * 1024) { in.close(); throw new Exception("the sync file is too large"); }
            }
            in.close();
            JSONObject o = new JSONObject();
            o.put("type", "syncData");
            o.put("text", new String(bo.toByteArray(), "UTF-8"));
            emit(o);
        } catch (SecurityException e) {
            syncError("perm", "The app lost access to the sync file. Tap Choose a different file and pick it again.");
        } catch (java.io.FileNotFoundException e) {
            syncError("missing", "Couldn't open the sync file. It may have been moved or deleted, or Drive is offline. Tap Choose a different file if it moved.");
        } catch (Exception e) {
            syncError("io", "Couldn't read the sync file: " + (e.getMessage() == null ? e.toString() : e.getMessage()));
        }
    }

    void syncWriteNow(String text) {
        Uri u = syncUri();
        if (u == null) { syncError("none", "No sync file is set."); return; }
        try {
            byte[] bytes = text.getBytes("UTF-8");
            OutputStream os;
            try { os = getContentResolver().openOutputStream(u, "wt"); }
            catch (Exception e) { os = getContentResolver().openOutputStream(u, "w"); }
            if (os == null) throw new java.io.FileNotFoundException();
            os.write(bytes);
            os.flush();
            os.close();
            emit(obj("type", "syncWritten"));
        } catch (SecurityException e) {
            syncError("perm", "The app lost access to the sync file. Tap Choose a different file and pick it again.");
        } catch (Exception e) {
            syncError("io", "Couldn't save the sync file: " + (e.getMessage() == null ? e.toString() : e.getMessage()));
        }
    }

    @Override
    protected void onResume() {
        super.onResume();
        if (pageReady) emit(obj("type", "resume"));
    }

    class Bridge {
        @JavascriptInterface
        public String syncInfo() {
            try {
                JSONObject o = new JSONObject();
                o.put("name", syncUri() == null ? "" : syncPrefs().getString("name", "sync file"));
                o.put("device", Build.MODEL == null ? "Tablet" : Build.MODEL);
                return o.toString();
            } catch (Exception e) { return "{}"; }
        }

        @JavascriptInterface
        public void syncPick(final boolean create) {
            main.post(new Runnable() {
                public void run() {
                    Intent i = new Intent(create ? Intent.ACTION_CREATE_DOCUMENT : Intent.ACTION_OPEN_DOCUMENT);
                    i.addCategory(Intent.CATEGORY_OPENABLE);
                    if (create) { i.setType("application/json"); i.putExtra(Intent.EXTRA_TITLE, "FlyPlan sync.json"); }
                    else i.setType("*/*");
                    i.addFlags(Intent.FLAG_GRANT_READ_URI_PERMISSION | Intent.FLAG_GRANT_WRITE_URI_PERMISSION | Intent.FLAG_GRANT_PERSISTABLE_URI_PERMISSION);
                    try { startActivityForResult(i, REQ_SYNC); }
                    catch (Exception e) { syncError("io", "No file picker is available."); }
                }
            });
        }

        @JavascriptInterface
        public void syncRead() { new Thread(new Runnable() { public void run() { syncReadNow(); } }).start(); }

        @JavascriptInterface
        public void syncWrite(final String text) { new Thread(new Runnable() { public void run() { syncWriteNow(text); } }).start(); }

        @JavascriptInterface
        public void syncForget() {
            Uri u = syncUri();
            if (u != null) {
                try { getContentResolver().releasePersistableUriPermission(u, Intent.FLAG_GRANT_READ_URI_PERMISSION | Intent.FLAG_GRANT_WRITE_URI_PERMISSION); }
                catch (Exception ignored) { }
            }
            syncPrefs().edit().clear().apply();
        }

        @JavascriptInterface
        public void saveKmz(final String name, final String template, final String waylines) {
            try {
                pendingSave = Kmz.build(template, waylines, null);
            } catch (Exception e) {
                emit(obj("type", "toast", "message", "Could not build KMZ: " + e.getMessage()));
                return;
            }
            main.post(new Runnable() {
                @Override
                public void run() {
                    Intent i = new Intent(Intent.ACTION_CREATE_DOCUMENT);
                    i.addCategory(Intent.CATEGORY_OPENABLE);
                    i.setType("application/vnd.google-earth.kmz");
                    i.putExtra(Intent.EXTRA_TITLE, name + ".kmz");
                    startActivityForResult(i, REQ_SAVE);
                }
            });
        }

        @JavascriptInterface
        public void saveText(final String name, final String mime, final String content) {
            try { pendingSave = content.getBytes("UTF-8"); } catch (Exception e) { return; }
            main.post(new Runnable() {
                public void run() {
                    Intent i = new Intent(Intent.ACTION_CREATE_DOCUMENT);
                    i.addCategory(Intent.CATEGORY_OPENABLE);
                    i.setType(mime);
                    i.putExtra(Intent.EXTRA_TITLE, name);
                    startActivityForResult(i, REQ_SAVE);
                }
            });
        }

        @JavascriptInterface
        public void searchPlace(final String query, final double lat, final double lng) {
            new Thread(new Runnable() {
                public void run() { doSearch(query, lat, lng); }
            }).start();
        }

        @JavascriptInterface
        public void probeUsgsZoom(final double lat, final double lng) {
            new Thread(new Runnable() {
                public void run() {
                    int z = TileCache.probe("usgs:USGSImageryOnly", lat, lng, 16, 20);
                    if (z > TileCache.usgsMax) {
                        TileCache.usgsMax = z;
                        getSharedPreferences("maps", MODE_PRIVATE).edit().putInt("usgsMax", z).apply();
                    }
                    try {
                        JSONObject o = new JSONObject();
                        o.put("type", "usgsZoom");
                        o.put("zoom", z);
                        emit(o);
                    } catch (Exception ignored) { }
                }
            }).start();
        }

        @JavascriptInterface
        public void importMbtiles() {
            main.post(new Runnable() {
                public void run() {
                    Intent i = new Intent(Intent.ACTION_OPEN_DOCUMENT);
                    i.addCategory(Intent.CATEGORY_OPENABLE);
                    i.setType("*/*");
                    try { startActivityForResult(i, REQ_MB); }
                    catch (Exception e) { emit(obj("type", "toast", "message", "No file picker is available.")); }
                }
            });
        }

        @JavascriptInterface
        public void listMaps() {
            new Thread(new Runnable() { public void run() { emitMaps(null); } }).start();
        }

        @JavascriptInterface
        public void deleteMap(final String id) {
            new Thread(new Runnable() { public void run() { mb.delete(id); emitMaps(null); } }).start();
        }

        @JavascriptInterface
        public void importKmz() { importFor("mission"); }

        @JavascriptInterface
        public void importFor(final String purpose) {
            importPurpose = purpose == null ? "mission" : purpose;
            main.post(new Runnable() {
                public void run() {
                    Intent i = new Intent(Intent.ACTION_OPEN_DOCUMENT);
                    i.addCategory(Intent.CATEGORY_OPENABLE);
                    i.setType("*/*");
                    try { startActivityForResult(i, REQ_OPEN); }
                    catch (Exception e) { emit(obj("type", "toast", "message", "No file picker is available.")); }
                }
            });
        }

        @JavascriptInterface
        public boolean isOnController() { return local.onController(); }

        @JavascriptInterface
        public void requestFolderAccess() {
            main.post(new Runnable() {
                public void run() {
                    Intent i = new Intent(Intent.ACTION_OPEN_DOCUMENT_TREE);
                    i.putExtra("android.provider.extra.INITIAL_URI", LocalStore.initialUri());
                    i.addFlags(Intent.FLAG_GRANT_READ_URI_PERMISSION | Intent.FLAG_GRANT_WRITE_URI_PERMISSION
                            | Intent.FLAG_GRANT_PERSISTABLE_URI_PERMISSION | Intent.FLAG_GRANT_PREFIX_URI_PERMISSION);
                    try { startActivityForResult(i, REQ_TREE); }
                    catch (Exception e) { emit(err("This device has no folder picker, so the app can't reach DJI Fly's folder here.")); }
                }
            });
        }

        @JavascriptInterface
        public void findController() {
            if (local.onController()) { listLocal(); return; }
            final UsbDevice dev = MtpTransfer.pickDevice(usb);
            if (dev == null) {
                emit(err("No controller found. Check the cable is a data cable, then on the controller pull down the notification shade and set USB to File transfer."));
                return;
            }
            withUsb(dev, new Runnable() {
                @Override
                public void run() {
                    try {
                        MtpTransfer t = new MtpTransfer(usb, dev);
                        try {
                            JSONArray arr = t.listMissions();
                            JSONObject o = new JSONObject();
                            o.put("type", "missions");
                            o.put("device", t.deviceLabel());
                            o.put("missions", arr);
                            emit(o);
                        } finally {
                            t.close();
                        }
                    } catch (Exception e) {
                        emit(err(e.getMessage() == null ? e.toString() : e.getMessage()));
                    }
                }
            });
        }

        @JavascriptInterface
        public void sendMission(final String folder, final String name, final String template, final String waylines, final String preview) {
            if (local.onController()) {
                new Thread(new Runnable() {
                    public void run() {
                        try {
                            Uri tree = local.treeUri();
                            if (tree == null) { emit(obj("type", "needAccess")); return; }
                            JSONObject res = local.replaceMission(tree, folder, template, waylines, preview);
                            res.put("type", "sent");
                            emit(res);
                        } catch (Exception e) {
                            emit(err(e.getMessage() == null ? e.toString() : e.getMessage()));
                        }
                    }
                }).start();
                return;
            }
            final UsbDevice dev = MtpTransfer.pickDevice(usb);
            if (dev == null) { emit(err("The controller is no longer connected.")); return; }
            withUsb(dev, new Runnable() {
                @Override
                public void run() {
                    try {
                        MtpTransfer t = new MtpTransfer(usb, dev);
                        try {
                            JSONObject res = t.replaceMission(MainActivity.this, folder, template, waylines, preview);
                            res.put("type", "sent");
                            emit(res);
                        } finally {
                            t.close();
                        }
                    } catch (Exception e) {
                        emit(err(e.getMessage() == null ? e.toString() : e.getMessage()));
                    }
                }
            });
        }

        @JavascriptInterface
        public void listOfflineAreas() {
            new Thread(new Runnable() {
                public void run() { emitAreas(); }
            }).start();
        }

        @JavascriptInterface
        public void downloadOfflineArea(final String json) {
            if (tiles.running) { emit(obj("type", "toast", "message", "A download is already running.")); return; }
            final TileCache.Area a;
            try {
                JSONObject o = new JSONObject(json);
                o.put("id", String.valueOf(System.currentTimeMillis()));
                o.put("date", new java.text.SimpleDateFormat("MMM d, yyyy", java.util.Locale.US).format(new java.util.Date()));
                a = TileCache.Area.from(o);
            } catch (Exception e) {
                emit(obj("type", "offDone", "message", "Could not start the download: " + e.getMessage(), "cls", "bad"));
                return;
            }
            new Thread(new Runnable() {
                public void run() {
                    tiles.download(a, new TileCache.Listener() {
                        public void progress(int done, int total, int failed, long bytes) {
                            try {
                                JSONObject o = new JSONObject();
                                o.put("type", "offProgress"); o.put("done", done); o.put("total", total);
                                o.put("failed", failed); o.put("bytes", bytes);
                                emit(o);
                            } catch (Exception ignored) { }
                        }
                        public void finished(boolean cancelled, int done, int total, int failed, long bytes) {
                            String msg = cancelled ? "Download stopped. Tiles already saved are kept."
                                    : failed == 0 ? "Saved " + a.name + " (" + total + " tiles). It works without internet now."
                                    : "Saved " + a.name + ", but " + failed + " of " + total + " tiles failed. Download again with internet to fill the gaps.";
                            emit(obj("type", "offDone", "message", msg, "cls", cancelled || failed > 0 ? "bad" : "ok"));
                            emitAreas();
                        }
                    });
                }
            }).start();
        }

        @JavascriptInterface
        public void cancelOfflineDownload() { tiles.cancel = true; }

        @JavascriptInterface
        public void deleteOfflineArea(final String id) {
            new Thread(new Runnable() {
                public void run() {
                    try { tiles.deleteArea(id); } catch (Exception ignored) { }
                    emitAreas();
                }
            }).start();
        }

        @JavascriptInterface
        public void deleteAllOffline() {
            new Thread(new Runnable() {
                public void run() {
                    try { tiles.deleteAll(); } catch (Exception ignored) { }
                    emitAreas();
                }
            }).start();
        }

        @JavascriptInterface
        public void getLocation() {
            main.post(new Runnable() {
                @Override
                public void run() { fetchLocation(); }
            });
        }
    }

    void listLocal() {
        new Thread(new Runnable() {
            public void run() {
                try {
                    Uri tree = local.treeUri();
                    if (tree == null) { emit(obj("type", "needAccess")); return; }
                    JSONObject o = new JSONObject();
                    o.put("type", "missions");
                    o.put("device", "this controller");
                    o.put("missions", local.listMissions(tree));
                    emit(o);
                } catch (Exception e) {
                    emit(err(e.getMessage() == null ? e.toString() : e.getMessage()));
                }
            }
        }).start();
    }

    /** Esri's geocoder first (good with place names, gives a map extent); Android's built-in geocoder as a fallback. */
    void doSearch(String query, double lat, double lng) {
        JSONArray results = new JSONArray();
        boolean networkFailed = false;
        try {
            String u = "https://geocode.arcgis.com/arcgis/rest/services/World/GeocodeServer/findAddressCandidates?f=json&maxLocations=8"
                    + "&outFields=Match_addr,Type,Place_addr&SingleLine=" + java.net.URLEncoder.encode(query, "UTF-8")
                    + "&location=" + lng + "," + lat;
            java.net.HttpURLConnection c = (java.net.HttpURLConnection) new java.net.URL(u).openConnection();
            c.setConnectTimeout(8000);
            c.setReadTimeout(10000);
            c.setRequestProperty("User-Agent", TileCache.UA);
            java.io.InputStream in = c.getInputStream();
            java.io.ByteArrayOutputStream bo = new java.io.ByteArrayOutputStream();
            byte[] buf = new byte[8192];
            int n;
            while ((n = in.read(buf)) > 0) bo.write(buf, 0, n);
            in.close();
            c.disconnect();
            JSONArray cands = new JSONObject(bo.toString("UTF-8")).optJSONArray("candidates");
            if (cands != null) for (int i = 0; i < cands.length(); i++) {
                JSONObject cd = cands.getJSONObject(i), loc = cd.getJSONObject("location"), at = cd.optJSONObject("attributes");
                JSONObject r = new JSONObject();
                r.put("name", cd.optString("address"));
                String type = at == null ? "" : at.optString("Type", ""), place = at == null ? "" : at.optString("Place_addr", "");
                r.put("detail", (type + (type.length() > 0 && place.length() > 0 ? " · " : "") + place).trim());
                r.put("lat", loc.getDouble("y"));
                r.put("lng", loc.getDouble("x"));
                JSONObject ex = cd.optJSONObject("extent");
                if (ex != null) {
                    JSONArray e = new JSONArray();
                    e.put(ex.getDouble("ymin")); e.put(ex.getDouble("xmin")); e.put(ex.getDouble("ymax")); e.put(ex.getDouble("xmax"));
                    r.put("ext", e);
                }
                results.put(r);
            }
        } catch (java.io.IOException e) {
            networkFailed = true;
        } catch (Exception ignored) { }
        if (results.length() == 0 && android.location.Geocoder.isPresent()) {
            try {
                java.util.List<android.location.Address> list = new android.location.Geocoder(this, java.util.Locale.getDefault())
                        .getFromLocationName(query, 8);
                if (list != null) for (android.location.Address a : list) {
                    JSONObject r = new JSONObject();
                    String line = a.getMaxAddressLineIndex() >= 0 ? a.getAddressLine(0) : "";
                    r.put("name", a.getFeatureName() != null && !a.getFeatureName().matches("\\d+") ? a.getFeatureName() : line);
                    r.put("detail", line);
                    r.put("lat", a.getLatitude());
                    r.put("lng", a.getLongitude());
                    results.put(r);
                }
                networkFailed = false;
            } catch (Exception ignored) { }
        }
        try {
            JSONObject o = new JSONObject();
            o.put("type", "places");
            o.put("query", query);
            o.put("results", results);
            if (results.length() == 0 && networkFailed) o.put("error", "offline");
            emit(o);
        } catch (Exception ignored) { }
    }

    void readImport(Uri uri) {
        try {
            String name = "Imported";
            android.database.Cursor c = getContentResolver().query(uri, new String[]{android.provider.OpenableColumns.DISPLAY_NAME}, null, null, null);
            if (c != null) { if (c.moveToFirst() && c.getString(0) != null) name = c.getString(0); c.close(); }
            java.io.InputStream in = getContentResolver().openInputStream(uri);
            java.io.ByteArrayOutputStream bo = new java.io.ByteArrayOutputStream();
            byte[] buf = new byte[16384];
            int n;
            while ((n = in.read(buf)) > 0) {
                bo.write(buf, 0, n);
                if (bo.size() > 25 * 1024 * 1024) { in.close(); throw new Exception("file is too large"); }
            }
            in.close();
            byte[] bytes = bo.toByteArray();
            JSONObject o = new JSONObject();
            o.put("type", "imported");
            o.put("purpose", importPurpose);
            o.put("name", name);
            if (bytes.length > 1 && bytes[0] == 'P' && bytes[1] == 'K') {
                java.util.Map<String, byte[]> files = Kmz.unzip(bytes);
                String firstKml = null;
                for (java.util.Map.Entry<String, byte[]> e : files.entrySet()) {
                    String k = e.getKey().toLowerCase(java.util.Locale.US);
                    String v = new String(e.getValue(), "UTF-8");
                    if (k.endsWith("waylines.wpml")) o.put("waylines", v);
                    else if (k.endsWith("template.kml")) o.put("template", v);
                    else if (k.endsWith(".kml") && firstKml == null) firstKml = v;
                }
                if (!o.has("waylines") && !o.has("template")) {
                    if (firstKml == null) throw new Exception("no mission or KML inside that KMZ");
                    o.put("kml", firstKml);
                } else if (o.has("template") && !o.has("waylines") && !o.getString("template").contains("Placemark")) {
                    throw new Exception("that KMZ has no waypoints");
                }
            } else {
                String text = new String(bytes, "UTF-8");
                if (text.contains("wpml:")) o.put("waylines", text); else o.put("kml", text);
            }
            emit(o);
        } catch (Exception e) {
            emit(obj("type", "toast", "message", "Could not import: " + e.getMessage()));
        }
    }

    void emitMaps(String added) {
        try {
            JSONObject o = new JSONObject();
            o.put("type", "maps");
            o.put("maps", mb.list());
            if (added != null) o.put("added", added);
            emit(o);
        } catch (Exception ignored) { }
    }

    void emitAreas() {
        try {
            JSONObject o = tiles.summary();
            o.put("type", "offAreas");
            o.put("running", tiles.running);
            emit(o);
        } catch (Exception ignored) { }
    }

    /* ---------- location ---------- */
    void fetchLocation() {
        if (checkSelfPermission(Manifest.permission.ACCESS_FINE_LOCATION) != PackageManager.PERMISSION_GRANTED
                && checkSelfPermission(Manifest.permission.ACCESS_COARSE_LOCATION) != PackageManager.PERMISSION_GRANTED) {
            requestPermissions(new String[]{Manifest.permission.ACCESS_FINE_LOCATION, Manifest.permission.ACCESS_COARSE_LOCATION}, REQ_LOC);
            emit(obj("type", "toast", "message", "Allow location access, then tap My location again."));
            return;
        }
        final LocationManager lm = (LocationManager) getSystemService(Context.LOCATION_SERVICE);
        Location best = null;
        for (String p : lm.getProviders(true)) {
            try {
                Location l = lm.getLastKnownLocation(p);
                if (l != null && (best == null || l.getTime() > best.getTime())) best = l;
            } catch (SecurityException ignored) { }
        }
        if (best != null && System.currentTimeMillis() - best.getTime() < 5 * 60 * 1000) { sendLocation(best); return; }
        String provider = lm.isProviderEnabled(LocationManager.GPS_PROVIDER) ? LocationManager.GPS_PROVIDER
                : lm.isProviderEnabled(LocationManager.NETWORK_PROVIDER) ? LocationManager.NETWORK_PROVIDER : null;
        if (provider == null) {
            if (best != null) sendLocation(best);
            else emit(obj("type", "toast", "message", "Turn on Location in the tablet's settings."));
            return;
        }
        if (best != null) sendLocation(best);
        try {
            lm.requestSingleUpdate(provider, new LocationListener() {
                public void onLocationChanged(Location l) { sendLocation(l); }
                public void onStatusChanged(String p, int s, Bundle b) { }
                public void onProviderEnabled(String p) { }
                public void onProviderDisabled(String p) { }
            }, Looper.getMainLooper());
        } catch (SecurityException ignored) { }
    }

    void sendLocation(Location l) {
        try {
            JSONObject o = new JSONObject();
            o.put("type", "location");
            o.put("lat", l.getLatitude());
            o.put("lng", l.getLongitude());
            emit(o);
        } catch (Exception ignored) { }
    }

    /* ---------- save-as result ---------- */
    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        super.onActivityResult(requestCode, resultCode, data);
        if (requestCode == REQ_MB) {
            if (resultCode != RESULT_OK || data == null || data.getData() == null) return;
            final Uri uri = data.getData();
            new Thread(new Runnable() {
                public void run() {
                    try {
                        String name = null;
                        android.database.Cursor c = getContentResolver().query(uri, new String[]{android.provider.OpenableColumns.DISPLAY_NAME}, null, null, null);
                        if (c != null) { if (c.moveToFirst()) name = c.getString(0); c.close(); }
                        String id = mb.importFrom(MainActivity.this, uri, name, new MbTiles.Progress() {
                            public void update(long done, long total) {
                                try {
                                    JSONObject o = new JSONObject();
                                    o.put("type", "mbProgress"); o.put("done", done); o.put("total", total);
                                    emit(o);
                                } catch (Exception ignored) { }
                            }
                        });
                        emitMaps(id);
                    } catch (Exception e) {
                        emit(obj("type", "mbError", "message", e.getMessage() == null ? e.toString() : e.getMessage()));
                    }
                }
            }).start();
            return;
        }
        if (requestCode == REQ_PHOTO) {
            Uri[] out = null;
            if (resultCode == RESULT_OK) {
                java.util.ArrayList<Uri> list = new java.util.ArrayList<Uri>();
                if (data != null && data.getClipData() != null) {
                    for (int i = 0; i < data.getClipData().getItemCount(); i++) list.add(original(data.getClipData().getItemAt(i).getUri()));
                } else if (data != null && data.getData() != null) {
                    list.add(original(data.getData()));
                } else if (cameraUri != null) {
                    list.add(cameraUri);
                    cameraUri = null;
                }
                if (!list.isEmpty()) out = list.toArray(new Uri[0]);
            }
            if (cameraUri != null) { // camera wasn't used: remove the empty gallery entry
                try { getContentResolver().delete(cameraUri, null, null); } catch (Exception ignored) { }
                cameraUri = null;
            }
            finishPhotos(out);
            return;
        }
        if (requestCode == REQ_SYNC) {
            if (resultCode != RESULT_OK || data == null || data.getData() == null) return;
            Uri uri = data.getData();
            try {
                getContentResolver().takePersistableUriPermission(uri, Intent.FLAG_GRANT_READ_URI_PERMISSION | Intent.FLAG_GRANT_WRITE_URI_PERMISSION);
            } catch (Exception e) {
                syncError("perm", "Android didn't allow keeping access to that file, so it can't be used for sync. Try saving it in Google Drive.");
                return;
            }
            String name = "sync file";
            try {
                android.database.Cursor c = getContentResolver().query(uri, new String[]{android.provider.OpenableColumns.DISPLAY_NAME}, null, null, null);
                if (c != null) { if (c.moveToFirst() && c.getString(0) != null) name = c.getString(0); c.close(); }
            } catch (Exception ignored) { }
            syncPrefs().edit().putString("uri", uri.toString()).putString("name", name).apply();
            emit(obj("type", "syncPicked", "name", name));
            return;
        }
        if (requestCode == REQ_OPEN) {
            if (resultCode != RESULT_OK || data == null || data.getData() == null) return;
            final Uri uri = data.getData();
            new Thread(new Runnable() {
                public void run() { readImport(uri); }
            }).start();
            return;
        }
        if (requestCode == REQ_TREE) {
            if (resultCode != RESULT_OK || data == null || data.getData() == null) {
                emit(err("Folder access wasn't granted."));
                return;
            }
            Uri tree = data.getData();
            try {
                getContentResolver().takePersistableUriPermission(tree, Intent.FLAG_GRANT_READ_URI_PERMISSION | Intent.FLAG_GRANT_WRITE_URI_PERMISSION);
            } catch (Exception e) {
                emit(err("Android didn't allow keeping access to that folder: " + e.getMessage()));
                return;
            }
            local.saveTree(tree);
            listLocal();
            return;
        }
        if (requestCode != REQ_SAVE) return;
        if (resultCode != RESULT_OK || data == null || data.getData() == null || pendingSave == null) return;
        Uri uri = data.getData();
        try {
            OutputStream os = getContentResolver().openOutputStream(uri);
            os.write(pendingSave);
            os.close();
            emit(obj("type", "toast", "message", "File saved"));
        } catch (Exception e) {
            emit(obj("type", "toast", "message", "Could not save: " + e.getMessage()));
        }
        pendingSave = null;
    }
}
