package com.ketchel.waypoint;

import android.content.Context;
import android.webkit.WebResourceResponse;

import org.json.JSONArray;
import org.json.JSONObject;

import java.io.ByteArrayInputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.InputStream;
import java.net.HttpURLConnection;
import java.net.URL;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicLong;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/** Disk cache for Esri map tiles: serves the map offline and downloads whole areas ahead of time. */
final class TileCache {
    static final Pattern TILE = Pattern.compile("^https://(server\\.arcgisonline\\.com|basemap\\.nationalmap\\.gov)/arcgis/rest/services/(.+?)/MapServer/tile/(\\d+)/(\\d+)/(\\d+)", Pattern.CASE_INSENSITIVE);
    static final String USGS_PREFIX = "usgs:";
    static final Pattern TERRAIN = Pattern.compile("^https://s3\\.amazonaws\\.com/elevation-tiles-prod/terrarium/(\\d+)/(\\d+)/(\\d+)\\.png");
    static final String TERRAIN_SVC = "terrain:terrarium";
    static final String UA = "WaypointPlanner/1.1 (Android)";
    static final int THREADS = 4;

    interface Listener {
        void progress(int done, int total, int failed, long bytes);
        void finished(boolean cancelled, int done, int total, int failed, long bytes);
    }

    final File root, areasFile;
    volatile boolean cancel, running;

    TileCache(Context c) {
        File base = c.getExternalFilesDir(null);
        if (base == null) base = c.getFilesDir();
        root = new File(base, "tiles");
        root.mkdirs();
        areasFile = new File(base, "offline_areas.json");
    }

    File file(String service, int z, int x, int y) {
        return new File(root, service.replace('/', '_').replace(':', '_') + "/" + z + "/" + x + "/" + y);
    }

    /** Services are named like "World_Imagery" (Esri) or "usgs:USGSImageryOnly" (USGS The National Map). */
    static String url(String service, int z, int x, int y) {
        if (service.startsWith("terrain:"))
            return "https://s3.amazonaws.com/elevation-tiles-prod/terrarium/" + z + "/" + x + "/" + y + ".png";
        if (service.startsWith(USGS_PREFIX))
            return "https://basemap.nationalmap.gov/arcgis/rest/services/" + service.substring(USGS_PREFIX.length()) + "/MapServer/tile/" + z + "/" + y + "/" + x;
        return "https://server.arcgisonline.com/ArcGIS/rest/services/" + service + "/MapServer/tile/" + z + "/" + y + "/" + x;
    }

    /** Deepest zoom each service has real tiles for. */
    static volatile int usgsMax = 16;
    static int maxZoom(String service) { return service.startsWith("terrain:") ? 14 : service.startsWith(USGS_PREFIX) ? usgsMax : 19; }

    /** Finds the deepest zoom where a service returns real imagery at this spot (not an error or a "no data" tile). */
    static int probe(String service, double lat, double lng, int floor, int ceiling) {
        for (int z = ceiling; z > floor; z--) {
            try {
                byte[] t = get(url(service, z, lonX(lng, z), latY(lat, z)));
                if (t == null || t.length < 1500) continue;
                byte[] blank = get(url(service, z, lonX(-150, z), latY(0, z)));   // open Pacific: no imagery there
                if (blank != null && java.util.Arrays.equals(t, blank)) continue;
                return z;
            } catch (Exception ignored) { }
        }
        return floor;
    }

    static byte[] get(String u) throws Exception {
        HttpURLConnection c = (HttpURLConnection) new URL(u).openConnection();
        c.setConnectTimeout(8000);
        c.setReadTimeout(10000);
        c.setRequestProperty("User-Agent", UA);
        try {
            if (c.getResponseCode() != 200) return null;
            String ct = c.getContentType();
            if (ct != null && !ct.startsWith("image")) return null;
            InputStream in = c.getInputStream();
            java.io.ByteArrayOutputStream bo = new java.io.ByteArrayOutputStream();
            byte[] buf = new byte[8192];
            int n;
            while ((n = in.read(buf)) > 0) bo.write(buf, 0, n);
            in.close();
            return bo.toByteArray();
        } finally {
            c.disconnect();
        }
    }

    /* ---------- serving tiles to the WebView ---------- */
    WebResourceResponse intercept(String u) {
        Matcher tm = TERRAIN.matcher(u);
        if (tm.find()) {
            int z = Integer.parseInt(tm.group(1)), x = Integer.parseInt(tm.group(2)), y = Integer.parseInt(tm.group(3));
            File f = file(TERRAIN_SVC, z, x, y);
            if (!f.exists()) {
                try { fetch(url(TERRAIN_SVC, z, x, y), f); } catch (Exception e) { return notFound(); }
            }
            try { return ok(f); } catch (Exception e) { return notFound(); }
        }
        Matcher m = TILE.matcher(u);
        if (!m.find()) return null;
        String svc = (m.group(1).toLowerCase(java.util.Locale.US).contains("nationalmap") ? USGS_PREFIX : "") + m.group(2);
        int z = Integer.parseInt(m.group(3)), y = Integer.parseInt(m.group(4)), x = Integer.parseInt(m.group(5));
        File f = file(svc, z, x, y);
        if (!f.exists()) {
            try { fetch(url(svc, z, x, y), f); } catch (Exception e) { return notFound(); }
        }
        try { return ok(f); } catch (Exception e) { return notFound(); }
    }

    static WebResourceResponse ok(File f) throws Exception {
        InputStream in = new FileInputStream(f);
        byte[] sig = new byte[4];
        FileInputStream peek = new FileInputStream(f);
        int n = peek.read(sig);
        peek.close();
        String mime = (n == 4 && (sig[0] & 0xFF) == 0x89 && sig[1] == 'P') ? "image/png" : "image/jpeg";
        Map<String, String> h = new HashMap<String, String>();
        h.put("Cache-Control", "max-age=86400");
        h.put("Access-Control-Allow-Origin", "*");
        return new WebResourceResponse(mime, null, 200, "OK", h, in);
    }

    static WebResourceResponse notFound() {
        return new WebResourceResponse("text/plain", "utf-8", 404, "Not Found", null, new ByteArrayInputStream(new byte[0]));
    }

    static long fetch(String u, File dest) throws Exception {
        HttpURLConnection c = (HttpURLConnection) new URL(u).openConnection();
        c.setConnectTimeout(10000);
        c.setReadTimeout(15000);
        c.setRequestProperty("User-Agent", UA);
        try {
            if (c.getResponseCode() != 200) throw new Exception("HTTP " + c.getResponseCode());
            dest.getParentFile().mkdirs();
            File tmp = new File(dest.getPath() + ".part" + Thread.currentThread().getId());
            InputStream in = c.getInputStream();
            FileOutputStream out = new FileOutputStream(tmp);
            byte[] buf = new byte[16384];
            long total = 0;
            int r;
            while ((r = in.read(buf)) > 0) { out.write(buf, 0, r); total += r; }
            out.close();
            in.close();
            if (total == 0 || !tmp.renameTo(dest)) { tmp.delete(); if (!dest.exists()) throw new Exception("empty tile"); }
            return total;
        } finally {
            c.disconnect();
        }
    }

    /* ---------- tile math ---------- */
    static int lonX(double lon, int z) { int n = 1 << z; return clamp((int) Math.floor((lon + 180) / 360 * n), n); }
    static int latY(double lat, int z) {
        int n = 1 << z;
        double r = Math.toRadians(Math.max(-85.0511, Math.min(85.0511, lat)));
        return clamp((int) Math.floor((1 - Math.log(Math.tan(r) + 1 / Math.cos(r)) / Math.PI) / 2 * n), n);
    }
    static int clamp(int v, int n) { return Math.max(0, Math.min(n - 1, v)); }

    static final class Area {
        String id, name, date; double s, w, n, e; int minZ, maxZ, tiles; String[] layers;
        int x1(int z) { return lonX(w, z); }
        int x2(int z) { return lonX(e, z); }
        int y1(int z) { return latY(n, z); }
        int y2(int z) { return latY(s, z); }
        boolean covers(int z, int x, int y) { return z >= minZ && z <= maxZ && x >= x1(z) && x <= x2(z) && y >= y1(z) && y <= y2(z); }
        int top(String svc) { return Math.min(maxZ, maxZoom(svc)); }
        long count(String svc) { long c = 0; for (int z = minZ; z <= top(svc); z++) c += (long) (x2(z) - x1(z) + 1) * (y2(z) - y1(z) + 1); return c; }
        long total() { long c = 0; for (String l : layers) c += count(l); return c; }

        static Area from(JSONObject o) throws Exception {
            Area a = new Area();
            a.id = o.optString("id", String.valueOf(System.currentTimeMillis()));
            a.name = o.optString("name", "Area");
            a.date = o.optString("date", "");
            a.s = o.getDouble("s"); a.w = o.getDouble("w"); a.n = o.getDouble("n"); a.e = o.getDouble("e");
            a.minZ = o.getInt("minZ"); a.maxZ = o.getInt("maxZ"); a.tiles = o.optInt("tiles", 0);
            JSONArray l = o.getJSONArray("layers");
            a.layers = new String[l.length()];
            for (int i = 0; i < l.length(); i++) a.layers[i] = l.getString(i);
            return a;
        }

        JSONObject json() throws Exception {
            JSONObject o = new JSONObject();
            o.put("id", id); o.put("name", name); o.put("date", date);
            o.put("s", s); o.put("w", w); o.put("n", n); o.put("e", e);
            o.put("minZ", minZ); o.put("maxZ", maxZ); o.put("tiles", tiles);
            JSONArray l = new JSONArray();
            for (String x : layers) l.put(x);
            o.put("layers", l);
            return o;
        }
    }

    /* ---------- downloading ---------- */
    /** Walks every tile of an area: layer, then zoom, then x, then y. */
    final class Walker {
        final Area a;
        int li = 0, z, x, y;
        Walker(Area a) { this.a = a; z = a.minZ; x = a.x1(z); y = a.y1(z); }
        synchronized Object[] next() {
            while (li < a.layers.length) {
                if (z > a.top(a.layers[li])) { li++; z = a.minZ; x = a.x1(z); y = a.y1(z); continue; }
                if (x > a.x2(z)) { z++; if (z <= a.top(a.layers[li])) { x = a.x1(z); y = a.y1(z); } continue; }
                if (y > a.y2(z)) { x++; y = a.y1(z); continue; }
                Object[] t = {a.layers[li], z, x, y};
                y++;
                return t;
            }
            return null;
        }
    }

    void download(final Area a, final Listener l) {
        running = true;
        cancel = false;
        final int total = (int) a.total();
        final AtomicInteger done = new AtomicInteger(), failed = new AtomicInteger();
        final AtomicLong bytes = new AtomicLong();
        final Walker walker = new Walker(a);
        Thread[] workers = new Thread[THREADS];
        for (int i = 0; i < THREADS; i++) {
            workers[i] = new Thread(new Runnable() {
                public void run() {
                    Object[] t;
                    while (!cancel && (t = walker.next()) != null) {
                        String svc = (String) t[0];
                        int z = (Integer) t[1], x = (Integer) t[2], y = (Integer) t[3];
                        File f = file(svc, z, x, y);
                        if (!f.exists()) {
                            try { bytes.addAndGet(fetch(url(svc, z, x, y), f)); }
                            catch (Exception e) { failed.incrementAndGet(); }
                        }
                        done.incrementAndGet();
                    }
                }
            });
            workers[i].start();
        }
        Thread reporter = new Thread(new Runnable() {
            public void run() {
                while (running) {
                    l.progress(done.get(), total, failed.get(), bytes.get());
                    try { Thread.sleep(600); } catch (InterruptedException e) { return; }
                }
            }
        });
        reporter.start();
        for (Thread w : workers) { try { w.join(); } catch (InterruptedException ignored) { } }
        running = false;
        reporter.interrupt();
        boolean wasCancelled = cancel;
        if (!wasCancelled) {
            a.tiles = total;
            try {
                JSONArray arr = readAreas();
                arr.put(a.json());
                writeAreas(arr);
            } catch (Exception ignored) { }
        }
        l.finished(wasCancelled, done.get(), total, failed.get(), bytes.get());
    }

    /* ---------- saved areas ---------- */
    JSONArray readAreas() {
        try {
            if (!areasFile.exists()) return new JSONArray();
            return new JSONArray(new String(MtpTransfer.readAll(areasFile), "UTF-8"));
        } catch (Exception e) {
            return new JSONArray();
        }
    }

    void writeAreas(JSONArray arr) throws Exception {
        FileOutputStream o = new FileOutputStream(areasFile);
        o.write(arr.toString().getBytes("UTF-8"));
        o.close();
    }

    static long size(File f) {
        if (f.isFile()) return f.length();
        long s = 0;
        File[] kids = f.listFiles();
        if (kids != null) for (File k : kids) s += size(k);
        return s;
    }

    static void deleteTree(File f) {
        File[] kids = f.listFiles();
        if (kids != null) for (File k : kids) deleteTree(k);
        f.delete();
    }

    JSONObject summary() throws Exception {
        JSONObject o = new JSONObject();
        o.put("areas", readAreas());
        o.put("bytes", size(root));
        return o;
    }

    void deleteArea(String id) throws Exception {
        JSONArray arr = readAreas(), keep = new JSONArray();
        Area target = null;
        java.util.List<Area> others = new java.util.ArrayList<Area>();
        for (int i = 0; i < arr.length(); i++) {
            Area a = Area.from(arr.getJSONObject(i));
            if (a.id.equals(id)) target = a; else { keep.put(arr.getJSONObject(i)); others.add(a); }
        }
        writeAreas(keep);
        if (target == null) return;
        for (String svc : target.layers) {
            for (int z = target.minZ; z <= target.top(svc); z++) {
                for (int x = target.x1(z); x <= target.x2(z); x++) {
                    for (int y = target.y1(z); y <= target.y2(z); y++) {
                        boolean shared = false;
                        for (Area o : others) if (o.covers(z, x, y)) { shared = true; break; }
                        if (!shared) file(svc, z, x, y).delete();
                    }
                }
            }
        }
    }

    void deleteAll() throws Exception {
        deleteTree(root);
        root.mkdirs();
        writeAreas(new JSONArray());
    }
}
