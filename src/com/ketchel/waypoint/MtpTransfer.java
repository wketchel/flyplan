package com.ketchel.waypoint;

import android.content.Context;
import android.hardware.usb.UsbConstants;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbDeviceConnection;
import android.hardware.usb.UsbInterface;
import android.hardware.usb.UsbManager;
import android.mtp.MtpDevice;
import android.mtp.MtpObjectInfo;
import android.os.ParcelFileDescriptor;

import org.json.JSONArray;
import org.json.JSONObject;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.lang.reflect.Method;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.Date;
import java.util.List;
import java.util.Locale;

/** Talks to the DJI controller over USB (MTP) and swaps a DJI Fly waypoint mission file. */
final class MtpTransfer {
    static final int DJI_VENDOR = 0x2CA3;
    static final int ROOT = 0xFFFFFFFF;
    static final int FORMAT_FOLDER = 0x3001;
    static final int FORMAT_UNDEFINED = 0x3000;
    static final String[] PATH = {"Android", "data", "dji.go.v5", "files", "waypoint"};

    final UsbDevice device;
    final UsbDeviceConnection conn;
    final MtpDevice mtp;

    static UsbDevice pickDevice(UsbManager usb) {
        UsbDevice fallback = null;
        for (UsbDevice d : usb.getDeviceList().values()) {
            if (d.getVendorId() == DJI_VENDOR) return d;
            for (int i = 0; i < d.getInterfaceCount(); i++) {
                UsbInterface itf = d.getInterface(i);
                String n = itf.getName();
                if (itf.getInterfaceClass() == UsbConstants.USB_CLASS_STILL_IMAGE || (n != null && n.toUpperCase(Locale.US).contains("MTP"))) {
                    if (fallback == null) fallback = d;
                }
            }
        }
        return fallback;
    }

    MtpTransfer(UsbManager usb, UsbDevice dev) throws Exception {
        device = dev;
        conn = usb.openDevice(dev);
        if (conn == null) throw new Exception("Could not open the USB connection to the controller.");
        mtp = new MtpDevice(dev);
        if (!mtp.open(conn)) {
            conn.close();
            throw new Exception("The controller is connected but not sharing files. On the controller, set USB to File transfer, then try again. If the tablet's My Files app opened the controller, close it.");
        }
    }

    String deviceLabel() {
        String p = device.getProductName();
        return p != null ? p : "the controller";
    }

    void close() {
        try { mtp.close(); } catch (Exception ignored) { }
        try { conn.close(); } catch (Exception ignored) { }
    }

    Integer child(int storage, int parent, String name) {
        int[] hs = mtp.getObjectHandles(storage, 0, parent);
        if (hs == null) return null;
        for (int h : hs) {
            MtpObjectInfo i = mtp.getObjectInfo(h);
            if (i != null && name.equalsIgnoreCase(i.getName())) return h;
        }
        return null;
    }

    /** Returns {storageId, handle} of DJI Fly's waypoint folder. */
    int[] findWaypointFolder() throws Exception {
        int[] storages = mtp.getStorageIds();
        if (storages == null || storages.length == 0) throw new Exception("The controller did not share any storage. Unlock it and set USB to File transfer.");
        for (int s : storages) {
            int parent = ROOT;
            boolean ok = true;
            for (String part : PATH) {
                Integer h = child(s, parent, part);
                if (h == null) { ok = false; break; }
                parent = h;
            }
            if (ok) return new int[]{s, parent};
        }
        throw new Exception("Could not find DJI Fly's waypoint folder on the controller. Save a waypoint mission in DJI Fly first. If you already have, the controller firmware may be hiding that folder over USB.");
    }

    static final class Mission {
        String folder; int kmz; long modified; long size;
    }

    List<Mission> missions(int storage, int wpFolder) {
        List<Mission> out = new ArrayList<Mission>();
        int[] folders = mtp.getObjectHandles(storage, 0, wpFolder);
        if (folders == null) return out;
        for (int fh : folders) {
            MtpObjectInfo fi = mtp.getObjectInfo(fh);
            if (fi == null || fi.getFormat() != FORMAT_FOLDER) continue;
            int[] files = mtp.getObjectHandles(storage, 0, fh);
            if (files == null) continue;
            for (int h : files) {
                MtpObjectInfo i = mtp.getObjectInfo(h);
                if (i != null && i.getName() != null && i.getName().toLowerCase(Locale.US).endsWith(".kmz")) {
                    Mission m = new Mission();
                    m.folder = fi.getName();
                    m.kmz = h;
                    m.modified = i.getDateModified();
                    m.size = i.getCompressedSize();
                    out.add(m);
                    break;
                }
            }
        }
        Collections.sort(out, new Comparator<Mission>() {
            public int compare(Mission a, Mission b) { return Long.compare(b.modified, a.modified); }
        });
        return out;
    }

    JSONArray listMissions() throws Exception {
        int[] wp = findWaypointFolder();
        JSONArray arr = new JSONArray();
        SimpleDateFormat fmt = new SimpleDateFormat("MMM d, yyyy h:mm a", Locale.US);
        for (Mission m : missions(wp[0], wp[1])) {
            JSONObject o = new JSONObject();
            o.put("folder", m.folder);
            o.put("date", m.modified > 0 ? "Saved " + fmt.format(new Date(m.modified)) : "Saved mission");
            o.put("size", m.size);
            arr.put(o);
        }
        return arr;
    }

    JSONObject replaceMission(Context ctx, String folderName, String template, String waylines, String previewB64) throws Exception {
        int[] wp = findWaypointFolder();
        int storage = wp[0];
        Integer folder = child(storage, wp[1], folderName);
        if (folder == null) throw new Exception("That mission is no longer on the controller. Tap Find controller again.");
        Mission target = null;
        for (Mission m : missions(storage, wp[1])) if (m.folder.equals(folderName)) target = m;
        if (target == null) throw new Exception("No KMZ file inside that mission folder.");
        MtpObjectInfo oldInfo = mtp.getObjectInfo(target.kmz);
        String fileName = oldInfo.getName();

        // 1. Back up the placeholder to the tablet.
        File dir = new File(ctx.getExternalFilesDir(null) != null ? ctx.getExternalFilesDir(null) : ctx.getFilesDir(), "backups");
        dir.mkdirs();
        String stamp = new SimpleDateFormat("yyyyMMdd-HHmmss", Locale.US).format(new Date());
        File backup = new File(dir, folderName + "_" + stamp + ".kmz");
        if (!mtp.importFile(target.kmz, backup.getPath())) throw new Exception("Could not read the placeholder mission from the controller.");
        byte[] old = readAll(backup);

        // 2. Build the new KMZ with the aircraft codes DJI Fly wrote into the placeholder.
        Kmz.Codes codes = Kmz.readCodes(old);
        byte[] data = Kmz.build(template, waylines, codes);
        File tmp = new File(ctx.getCacheDir(), "upload.kmz");
        FileOutputStream fo = new FileOutputStream(tmp);
        fo.write(data);
        fo.close();

        // 3. Replace the file: MTP has no overwrite, so delete then create with the same name.
        if (!mtp.deleteObject(target.kmz)) throw new Exception("The controller would not let the old mission be replaced. Nothing was changed.");
        try {
            upload(storage, folder, fileName, tmp);
        } catch (Exception e) {
            try { upload(storage, folder, fileName, backup); } catch (Exception ignored) { }
            throw new Exception("Writing the new mission failed (" + e.getMessage() + "). The placeholder was put back.");
        }

        String pv = "";
        if (previewB64 != null && previewB64.length() > 0) {
            try { writePreview(ctx, storage, wp[1], folderName, android.util.Base64.decode(previewB64, android.util.Base64.DEFAULT)); pv = " Its picture in DJI Fly now shows the name."; }
            catch (Exception e) { pv = " (The picture in DJI Fly wasn't updated: " + e.getMessage() + ".)"; }
        }
        JSONObject res = new JSONObject();
        res.put("message", "Replaced mission " + folderName.substring(0, Math.min(8, folderName.length())) + "… on " + deviceLabel()
                + ". Backup saved on the tablet as " + backup.getName() + "." + pv);
        if (codes.droneEnum != null) { res.put("droneEnum", codes.droneEnum); res.put("droneSub", codes.droneSub == null ? 0 : codes.droneSub); }
        return res;
    }

    /** Replaces DJI Fly's map picture for the mission (waypoint/map_preview/<id>/<id>.jpg). */
    void writePreview(Context ctx, int storage, int wpFolder, String folderName, byte[] jpg) throws Exception {
        Integer mp = child(storage, wpFolder, "map_preview");
        if (mp == null) mp = makeFolder(storage, wpFolder, "map_preview");
        Integer dir = child(storage, mp, folderName);
        if (dir == null) dir = makeFolder(storage, mp, folderName);
        String name = folderName + ".jpg";
        Integer old = child(storage, dir, name);
        if (old != null && !mtp.deleteObject(old)) throw new Exception("couldn't replace the old picture");
        File tmp = new File(ctx.getCacheDir(), "preview.jpg");
        FileOutputStream fo = new FileOutputStream(tmp);
        fo.write(jpg);
        fo.close();
        upload(storage, dir, name, tmp);
    }

    Integer makeFolder(int storage, int parent, String name) throws Exception {
        Class<?> bc = Class.forName("android.mtp.MtpObjectInfo$Builder");
        Object b = bc.getConstructor().newInstance();
        bc.getMethod("setStorageId", int.class).invoke(b, storage);
        bc.getMethod("setParent", int.class).invoke(b, parent);
        bc.getMethod("setFormat", int.class).invoke(b, FORMAT_FOLDER);
        bc.getMethod("setAssociationType", int.class).invoke(b, 1);
        bc.getMethod("setName", String.class).invoke(b, name);
        MtpObjectInfo info = (MtpObjectInfo) bc.getMethod("build").invoke(b);
        MtpObjectInfo created = (MtpObjectInfo) MtpDevice.class.getMethod("sendObjectInfo", MtpObjectInfo.class).invoke(mtp, info);
        if (created == null) throw new Exception("couldn't create the picture folder");
        return created.getObjectHandle();
    }

    /** sendObjectInfo/sendObject are API 24; called reflectively because this app compiles against API 23. */
    void upload(int storage, int parent, String name, File f) throws Exception {
        Class<?> bc = Class.forName("android.mtp.MtpObjectInfo$Builder");
        Object b = bc.getConstructor().newInstance();
        bc.getMethod("setStorageId", int.class).invoke(b, storage);
        bc.getMethod("setParent", int.class).invoke(b, parent);
        bc.getMethod("setFormat", int.class).invoke(b, FORMAT_UNDEFINED);
        bc.getMethod("setName", String.class).invoke(b, name);
        bc.getMethod("setCompressedSize", long.class).invoke(b, f.length());
        bc.getMethod("setDateModified", long.class).invoke(b, System.currentTimeMillis());
        MtpObjectInfo info = (MtpObjectInfo) bc.getMethod("build").invoke(b);
        Method soi = MtpDevice.class.getMethod("sendObjectInfo", MtpObjectInfo.class);
        MtpObjectInfo created = (MtpObjectInfo) soi.invoke(mtp, info);
        if (created == null) throw new Exception("controller refused the new file");
        ParcelFileDescriptor pfd = ParcelFileDescriptor.open(f, ParcelFileDescriptor.MODE_READ_ONLY);
        try {
            Method so = MtpDevice.class.getMethod("sendObject", int.class, long.class, ParcelFileDescriptor.class);
            Boolean ok = (Boolean) so.invoke(mtp, created.getObjectHandle(), f.length(), pfd);
            if (ok == null || !ok) throw new Exception("file transfer did not complete");
        } finally {
            pfd.close();
        }
    }

    static byte[] readAll(File f) throws Exception {
        byte[] b = new byte[(int) f.length()];
        FileInputStream in = new FileInputStream(f);
        int off = 0, n;
        while (off < b.length && (n = in.read(b, off, b.length - off)) > 0) off += n;
        in.close();
        return b;
    }
}
