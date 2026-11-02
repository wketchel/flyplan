package com.ketchel.waypoint;

import android.content.ContentResolver;
import android.content.Context;
import android.content.SharedPreferences;
import android.content.UriPermission;
import android.database.Cursor;
import android.net.Uri;
import android.os.Build;
import android.provider.DocumentsContract;
import android.provider.DocumentsContract.Document;

import org.json.JSONArray;
import org.json.JSONObject;

import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.FileOutputStream;
import java.io.InputStream;
import java.io.OutputStream;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.Date;
import java.util.List;
import java.util.Locale;

/**
 * Used when the app runs on the controller itself: reaches DJI Fly's waypoint folder through
 * Android's folder picker (the same way file managers such as CX File Explorer do on Android 10–12)
 * and overwrites the placeholder mission in place.
 */
final class LocalStore {
    static final String TARGET_DOC = "primary:Android/data/dji.go.v5/files/waypoint";
    static final String AUTH = "com.android.externalstorage.documents";

    final Context ctx;
    final ContentResolver cr;
    final SharedPreferences prefs;

    LocalStore(Context c) {
        ctx = c;
        cr = c.getContentResolver();
        prefs = c.getSharedPreferences("local", Context.MODE_PRIVATE);
    }

    /** True when this device looks like a DJI controller (or the folder was already granted). */
    boolean onController() {
        String id = (Build.MANUFACTURER + " " + Build.BRAND + " " + Build.MODEL + " " + Build.PRODUCT).toUpperCase(Locale.US);
        return id.contains("DJI") || treeUri() != null;
    }

    static Uri initialUri() {
        return DocumentsContract.buildDocumentUri(AUTH, TARGET_DOC);
    }

    void saveTree(Uri tree) { prefs.edit().putString("tree", tree.toString()).apply(); }

    Uri treeUri() {
        String s = prefs.getString("tree", null);
        if (s == null) return null;
        Uri u = Uri.parse(s);
        for (UriPermission p : cr.getPersistedUriPermissions()) {
            if (p.getUri().equals(u) && p.isWritePermission()) return u;
        }
        return null;
    }

    static final class Entry { String id, name, mime; long modified; }

    List<Entry> children(Uri tree, String docId) {
        List<Entry> out = new ArrayList<Entry>();
        Uri q = DocumentsContract.buildChildDocumentsUriUsingTree(tree, docId);
        Cursor c = cr.query(q, new String[]{Document.COLUMN_DOCUMENT_ID, Document.COLUMN_DISPLAY_NAME,
                Document.COLUMN_MIME_TYPE, Document.COLUMN_LAST_MODIFIED}, null, null, null);
        if (c == null) return out;
        try {
            while (c.moveToNext()) {
                Entry e = new Entry();
                e.id = c.getString(0); e.name = c.getString(1); e.mime = c.getString(2); e.modified = c.getLong(3);
                out.add(e);
            }
        } finally {
            c.close();
        }
        return out;
    }

    /** Document id of DJI Fly's waypoint folder inside the granted tree. */
    String waypointDoc(Uri tree) throws Exception {
        String root = DocumentsContract.getTreeDocumentId(tree);
        if (root.equals(TARGET_DOC)) return root;
        if (!TARGET_DOC.startsWith(root.endsWith("/") ? root : root + "/") && !(root.endsWith(":") && TARGET_DOC.startsWith(root)))
            throw new Exception("The folder you picked doesn't contain DJI Fly's missions. Tap Grant folder access and choose Android › data › dji.go.v5 › files › waypoint.");
        String rest = TARGET_DOC.substring(root.length()).replaceAll("^/", "");
        String cur = root;
        for (String part : rest.split("/")) {
            if (part.isEmpty()) continue;
            String next = null;
            for (Entry e : children(tree, cur)) if (part.equalsIgnoreCase(e.name)) { next = e.id; break; }
            if (next == null) throw new Exception("Could not find DJI Fly's waypoint folder. Save a waypoint mission in DJI Fly first.");
            cur = next;
        }
        return cur;
    }

    static final class Mission { String folder; Entry kmz; }

    List<Mission> missions(Uri tree) throws Exception {
        List<Mission> out = new ArrayList<Mission>();
        for (Entry f : children(tree, waypointDoc(tree))) {
            if (!Document.MIME_TYPE_DIR.equals(f.mime)) continue;
            for (Entry k : children(tree, f.id)) {
                if (k.name != null && k.name.toLowerCase(Locale.US).endsWith(".kmz")) {
                    Mission m = new Mission(); m.folder = f.name; m.kmz = k; out.add(m); break;
                }
            }
        }
        Collections.sort(out, new Comparator<Mission>() {
            public int compare(Mission a, Mission b) { return Long.compare(b.kmz.modified, a.kmz.modified); }
        });
        return out;
    }

    JSONArray listMissions(Uri tree) throws Exception {
        JSONArray arr = new JSONArray();
        SimpleDateFormat fmt = new SimpleDateFormat("MMM d, yyyy h:mm a", Locale.US);
        for (Mission m : missions(tree)) {
            JSONObject o = new JSONObject();
            o.put("folder", m.folder);
            o.put("date", m.kmz.modified > 0 ? "Saved " + fmt.format(new Date(m.kmz.modified)) : "Saved mission");
            arr.put(o);
        }
        return arr;
    }

    String childId(Uri tree, String parent, String name) {
        for (Entry e : children(tree, parent)) if (name.equalsIgnoreCase(e.name)) return e.id;
        return null;
    }

    String makeDir(Uri tree, String parent, String name) throws Exception {
        Uri u = DocumentsContract.createDocument(cr, DocumentsContract.buildDocumentUriUsingTree(tree, parent), Document.MIME_TYPE_DIR, name);
        if (u == null) throw new Exception("couldn't create the picture folder");
        return DocumentsContract.getDocumentId(u);
    }

    /** Replaces DJI Fly's map picture for the mission (waypoint/map_preview/<id>/<id>.jpg). */
    void writePreview(Uri tree, String folder, byte[] jpg) throws Exception {
        String wp = waypointDoc(tree);
        String mp = childId(tree, wp, "map_preview");
        if (mp == null) mp = makeDir(tree, wp, "map_preview");
        String dir = childId(tree, mp, folder);
        if (dir == null) dir = makeDir(tree, mp, folder);
        String name = folder + ".jpg";
        String f = childId(tree, dir, name);
        Uri fu = f != null ? DocumentsContract.buildDocumentUriUsingTree(tree, f)
                : DocumentsContract.createDocument(cr, DocumentsContract.buildDocumentUriUsingTree(tree, dir), "image/jpeg", name);
        if (fu == null) throw new Exception("couldn't create the picture");
        OutputStream o = cr.openOutputStream(fu, "wt");
        try { o.write(jpg); } finally { o.close(); }
    }

    JSONObject replaceMission(Uri tree, String folder, String template, String waylines, String previewB64) throws Exception {
        Mission target = null;
        for (Mission m : missions(tree)) if (m.folder.equals(folder)) target = m;
        if (target == null) throw new Exception("That mission is no longer in DJI Fly. Tap Find missions again.");
        Uri file = DocumentsContract.buildDocumentUriUsingTree(tree, target.kmz.id);

        // Back up the placeholder.
        byte[] old;
        InputStream in = cr.openInputStream(file);
        try {
            ByteArrayOutputStream bo = new ByteArrayOutputStream();
            byte[] buf = new byte[16384];
            int n;
            while ((n = in.read(buf)) > 0) bo.write(buf, 0, n);
            old = bo.toByteArray();
        } finally {
            in.close();
        }
        File dir = new File(ctx.getExternalFilesDir(null) != null ? ctx.getExternalFilesDir(null) : ctx.getFilesDir(), "backups");
        dir.mkdirs();
        File backup = new File(dir, folder + "_" + new SimpleDateFormat("yyyyMMdd-HHmmss", Locale.US).format(new Date()) + ".kmz");
        FileOutputStream fo = new FileOutputStream(backup);
        fo.write(old);
        fo.close();

        // Overwrite in place with the aircraft codes copied from the placeholder.
        Kmz.Codes codes = Kmz.readCodes(old);
        byte[] data = Kmz.build(template, waylines, codes);
        OutputStream out = cr.openOutputStream(file, "wt");
        try {
            out.write(data);
        } catch (Exception e) {
            try { out.close(); } catch (Exception ignored) { }
            OutputStream restore = cr.openOutputStream(file, "wt");
            restore.write(old);
            restore.close();
            throw new Exception("Writing the new mission failed (" + e.getMessage() + "). The placeholder was put back.");
        }
        out.close();

        String pv = "";
        if (previewB64 != null && previewB64.length() > 0) {
            try { writePreview(tree, folder, android.util.Base64.decode(previewB64, android.util.Base64.DEFAULT)); pv = " Its picture in DJI Fly now shows the name."; }
            catch (Exception e) { pv = " (The picture in DJI Fly wasn't updated: " + e.getMessage() + ".)"; }
        }
        JSONObject res = new JSONObject();
        res.put("message", "Replaced mission " + folder.substring(0, Math.min(8, folder.length())) + "… in DJI Fly. Backup saved as " + backup.getName() + "." + pv);
        if (codes.droneEnum != null) { res.put("droneEnum", codes.droneEnum); res.put("droneSub", codes.droneSub == null ? 0 : codes.droneSub); }
        return res;
    }
}
