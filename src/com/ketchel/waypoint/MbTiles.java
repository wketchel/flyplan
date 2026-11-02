package com.ketchel.waypoint;

import android.content.Context;
import android.database.Cursor;
import android.database.sqlite.SQLiteDatabase;
import android.net.Uri;
import android.webkit.WebResourceResponse;

import org.json.JSONArray;
import org.json.JSONObject;

import java.io.ByteArrayInputStream;
import java.io.File;
import java.io.FileOutputStream;
import java.io.InputStream;
import java.util.HashMap;
import java.util.Map;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * Your own basemaps from MBTiles files (for example a WebODM orthophoto export). Files are copied into app storage
 * and served to the map from https://mbtiles.invalid/{id}/{z}/{x}/{y}, so they work fully offline.
 */
final class MbTiles {
    static final Pattern URL = Pattern.compile("^https://mbtiles\\.invalid/([A-Za-z0-9_-]+)/(\\d+)/(\\d+)/(\\d+)");

    interface Progress { void update(long done, long total); }

    final File dir;
    final Map<String, SQLiteDatabase> open = new HashMap<String, SQLiteDatabase>();
    final Map<String, Boolean> xyz = new HashMap<String, Boolean>();
    final Map<String, String> mime = new HashMap<String, String>();

    MbTiles(Context c) {
        File base = c.getExternalFilesDir(null);
        if (base == null) base = c.getFilesDir();
        dir = new File(base, "maps");
        dir.mkdirs();
    }

    File file(String id) { return new File(dir, id + ".mbtiles"); }

    synchronized SQLiteDatabase db(String id) {
        SQLiteDatabase d = open.get(id);
        if (d != null && d.isOpen()) return d;
        File f = file(id);
        if (!f.exists()) return null;
        d = SQLiteDatabase.openDatabase(f.getPath(), null, SQLiteDatabase.OPEN_READONLY | SQLiteDatabase.NO_LOCALIZED_COLLATORS);
        open.put(id, d);
        Map<String, String> meta = metadata(d);
        xyz.put(id, "xyz".equalsIgnoreCase(meta.get("scheme")));
        String fmt = meta.get("format");
        mime.put(id, fmt == null ? null : fmt.toLowerCase().contains("png") ? "image/png" : fmt.toLowerCase().contains("webp") ? "image/webp" : "image/jpeg");
        return d;
    }

    static Map<String, String> metadata(SQLiteDatabase d) {
        Map<String, String> m = new HashMap<String, String>();
        Cursor c = null;
        try {
            c = d.rawQuery("SELECT name, value FROM metadata", null);
            while (c.moveToNext()) m.put(c.getString(0), c.getString(1));
        } catch (Exception ignored) {
        } finally {
            if (c != null) c.close();
        }
        return m;
    }

    static double lon(double x, int z) { return x / (1 << z) * 360.0 - 180.0; }
    static double lat(double y, int z) { double n = Math.PI - 2.0 * Math.PI * y / (1 << z); return Math.toDegrees(Math.atan(Math.sinh(n))); }

    JSONObject info(String id) throws Exception {
        SQLiteDatabase d = db(id);
        if (d == null) return null;
        Map<String, String> meta = metadata(d);
        JSONObject o = new JSONObject();
        o.put("id", id);
        String name = meta.get("name");
        File side = new File(dir, id + ".name");
        if (side.exists()) name = new String(MtpTransfer.readAll(side), "UTF-8");
        o.put("name", name == null || name.length() == 0 ? "My map" : name);
        int minZ = -1, maxZ = -1;
        try { minZ = Integer.parseInt(meta.get("minzoom")); maxZ = Integer.parseInt(meta.get("maxzoom")); } catch (Exception ignored) { }
        if (minZ < 0 || maxZ < 0) {
            Cursor c = d.rawQuery("SELECT MIN(zoom_level), MAX(zoom_level) FROM tiles", null);
            if (c.moveToFirst()) { minZ = c.getInt(0); maxZ = c.getInt(1); }
            c.close();
        }
        o.put("minzoom", minZ);
        o.put("maxzoom", maxZ);
        double[] b = null;
        String bs = meta.get("bounds");
        if (bs != null) {
            String[] p = bs.split(",");
            if (p.length == 4) b = new double[]{Double.parseDouble(p[0].trim()), Double.parseDouble(p[1].trim()), Double.parseDouble(p[2].trim()), Double.parseDouble(p[3].trim())};
        }
        if (b == null && maxZ >= 0) {
            Cursor c = d.rawQuery("SELECT MIN(tile_column), MAX(tile_column), MIN(tile_row), MAX(tile_row) FROM tiles WHERE zoom_level = ?", new String[]{String.valueOf(maxZ)});
            if (c.moveToFirst()) {
                int x1 = c.getInt(0), x2 = c.getInt(1), r1 = c.getInt(2), r2 = c.getInt(3);
                boolean isXyz = Boolean.TRUE.equals(xyz.get(id));
                int yTop = isXyz ? r1 : (1 << maxZ) - 1 - r2, yBot = isXyz ? r2 : (1 << maxZ) - 1 - r1;
                b = new double[]{lon(x1, maxZ), lat(yBot + 1, maxZ), lon(x2 + 1, maxZ), lat(yTop, maxZ)};
            }
            c.close();
        }
        if (b != null) { JSONArray a = new JSONArray(); for (double v : b) a.put(v); o.put("bounds", a); }
        o.put("size", file(id).length());
        return o;
    }

    JSONArray list() {
        JSONArray arr = new JSONArray();
        File[] fs = dir.listFiles();
        if (fs == null) return arr;
        for (File f : fs) {
            String n = f.getName();
            if (!n.endsWith(".mbtiles")) continue;
            try { JSONObject o = info(n.substring(0, n.length() - 8)); if (o != null) arr.put(o); } catch (Exception ignored) { }
        }
        return arr;
    }

    WebResourceResponse intercept(String u) {
        Matcher m = URL.matcher(u);
        if (!m.find()) return null;
        String id = m.group(1);
        int z = Integer.parseInt(m.group(2)), x = Integer.parseInt(m.group(3)), y = Integer.parseInt(m.group(4));
        try {
            SQLiteDatabase d = db(id);
            if (d != null) {
                int row = Boolean.TRUE.equals(xyz.get(id)) ? y : (1 << z) - 1 - y;
                Cursor c = d.rawQuery("SELECT tile_data FROM tiles WHERE zoom_level = ? AND tile_column = ? AND tile_row = ?",
                        new String[]{String.valueOf(z), String.valueOf(x), String.valueOf(row)});
                try {
                    if (c.moveToFirst()) {
                        byte[] data = c.getBlob(0);
                        String type = mime.get(id);
                        if (type == null) type = data.length > 3 && (data[0] & 0xFF) == 0x89 && data[1] == 'P' ? "image/png" : "image/jpeg";
                        Map<String, String> h = new HashMap<String, String>();
                        h.put("Cache-Control", "max-age=3600");
                        h.put("Access-Control-Allow-Origin", "*");
                        return new WebResourceResponse(type, null, 200, "OK", h, new ByteArrayInputStream(data));
                    }
                } finally {
                    c.close();
                }
            }
        } catch (Exception ignored) { }
        return TileCache.notFound();
    }

    /** Copies a picked file into app storage, checks it is a real MBTiles file, and returns its new id. */
    String importFrom(Context ctx, Uri uri, String displayName, Progress p) throws Exception {
        long total = -1;
        android.database.Cursor q = ctx.getContentResolver().query(uri, new String[]{android.provider.OpenableColumns.SIZE}, null, null, null);
        if (q != null) { if (q.moveToFirst() && !q.isNull(0)) total = q.getLong(0); q.close(); }
        String id = "m" + System.currentTimeMillis();
        File tmp = new File(dir, id + ".part");
        InputStream in = ctx.getContentResolver().openInputStream(uri);
        FileOutputStream out = new FileOutputStream(tmp);
        byte[] buf = new byte[1 << 16];
        long done = 0, last = 0;
        int n;
        try {
            while ((n = in.read(buf)) > 0) {
                out.write(buf, 0, n);
                done += n;
                if (done - last > (4 << 20)) { last = done; p.update(done, total); }
            }
        } finally {
            in.close();
            out.close();
        }
        p.update(done, total);
        // validate
        SQLiteDatabase d = null;
        try {
            d = SQLiteDatabase.openDatabase(tmp.getPath(), null, SQLiteDatabase.OPEN_READONLY | SQLiteDatabase.NO_LOCALIZED_COLLATORS);
            Cursor c = d.rawQuery("SELECT COUNT(*) FROM (SELECT 1 FROM tiles LIMIT 1)", null);
            c.moveToFirst();
            int has = c.getInt(0);
            c.close();
            if (has == 0) throw new Exception("the file has no map tiles in it");
        } catch (Exception e) {
            if (d != null) d.close();
            tmp.delete();
            throw new Exception("That doesn't look like an MBTiles map (" + e.getMessage() + ").");
        }
        d.close();
        if (!tmp.renameTo(file(id))) { tmp.delete(); throw new Exception("Could not save the map."); }
        String nm = displayName == null ? "My map" : displayName.replaceAll("(?i)\\.mbtiles$", "");
        FileOutputStream side = new FileOutputStream(new File(dir, id + ".name"));
        side.write(nm.getBytes("UTF-8"));
        side.close();
        return id;
    }

    synchronized boolean delete(String id) {
        SQLiteDatabase d = open.remove(id);
        if (d != null) d.close();
        new File(dir, id + ".name").delete();
        return file(id).delete();
    }
}
