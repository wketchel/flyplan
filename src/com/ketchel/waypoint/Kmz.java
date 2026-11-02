package com.ketchel.waypoint;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.util.zip.ZipEntry;
import java.util.zip.ZipInputStream;
import java.util.zip.ZipOutputStream;

/** Builds DJI WPML KMZ files, optionally copying aircraft codes from a mission DJI Fly created. */
final class Kmz {
    static final String TEMPLATE = "wpmz/template.kml";
    static final String WAYLINES = "wpmz/waylines.wpml";
    static final Pattern DRONE = Pattern.compile("<wpml:droneInfo>.*?</wpml:droneInfo>", Pattern.DOTALL);
    static final Pattern PAYLOAD = Pattern.compile("<wpml:payloadInfo>.*?</wpml:payloadInfo>", Pattern.DOTALL);
    static final Pattern NS = Pattern.compile("xmlns:wpml=\"([^\"]+)\"");
    static final Pattern ENUM = Pattern.compile("<wpml:droneEnumValue>\\s*(\\d+)\\s*</wpml:droneEnumValue>");
    static final Pattern SUB = Pattern.compile("<wpml:droneSubEnumValue>\\s*(\\d+)\\s*</wpml:droneSubEnumValue>");

    /** Info pulled from the placeholder mission. */
    static final class Codes {
        String droneInfo, payloadInfo, ns;
        Integer droneEnum, droneSub;
        Map<String, byte[]> extras = new LinkedHashMap<String, byte[]>();
    }

    static Map<String, byte[]> unzip(byte[] data) throws Exception {
        Map<String, byte[]> out = new LinkedHashMap<String, byte[]>();
        ZipInputStream zin = new ZipInputStream(new ByteArrayInputStream(data));
        ZipEntry e;
        byte[] buf = new byte[16384];
        while ((e = zin.getNextEntry()) != null) {
            if (e.isDirectory()) continue;
            ByteArrayOutputStream bo = new ByteArrayOutputStream();
            int n;
            while ((n = zin.read(buf)) > 0) bo.write(buf, 0, n);
            out.put(e.getName(), bo.toByteArray());
        }
        zin.close();
        return out;
    }

    static Codes readCodes(byte[] placeholderKmz) throws Exception {
        Codes c = new Codes();
        Map<String, byte[]> files = unzip(placeholderKmz);
        byte[] t = files.get(TEMPLATE);
        if (t == null) t = files.get(WAYLINES);
        if (t != null) {
            String s = new String(t, "UTF-8");
            Matcher m = DRONE.matcher(s);
            if (m.find()) c.droneInfo = m.group();
            m = PAYLOAD.matcher(s);
            if (m.find()) c.payloadInfo = m.group();
            m = NS.matcher(s);
            if (m.find()) c.ns = m.group(1);
            if (c.droneInfo != null) {
                m = ENUM.matcher(c.droneInfo);
                if (m.find()) c.droneEnum = Integer.valueOf(m.group(1));
                m = SUB.matcher(c.droneInfo);
                if (m.find()) c.droneSub = Integer.valueOf(m.group(1));
            }
        }
        for (Map.Entry<String, byte[]> e : files.entrySet()) {
            if (!e.getKey().equals(TEMPLATE) && !e.getKey().equals(WAYLINES)) c.extras.put(e.getKey(), e.getValue());
        }
        return c;
    }

    static String inject(String xml, Codes c) {
        if (c == null) return xml;
        if (c.ns != null) xml = NS.matcher(xml).replaceFirst(Matcher.quoteReplacement("xmlns:wpml=\"" + c.ns + "\""));
        if (c.droneInfo != null) {
            xml = DRONE.matcher(xml).replaceFirst(Matcher.quoteReplacement(c.droneInfo));
            if (c.payloadInfo != null) {
                if (PAYLOAD.matcher(xml).find()) xml = PAYLOAD.matcher(xml).replaceFirst(Matcher.quoteReplacement(c.payloadInfo));
                else xml = xml.replaceFirst(Pattern.quote("</wpml:droneInfo>"), Matcher.quoteReplacement("</wpml:droneInfo>" + c.payloadInfo));
            }
        }
        return xml;
    }

    static byte[] build(String template, String waylines, Codes c) throws Exception {
        ByteArrayOutputStream bo = new ByteArrayOutputStream();
        ZipOutputStream zo = new ZipOutputStream(bo);
        put(zo, TEMPLATE, inject(template, c).getBytes("UTF-8"));
        put(zo, WAYLINES, inject(waylines, c).getBytes("UTF-8"));
        if (c != null) for (Map.Entry<String, byte[]> e : c.extras.entrySet()) put(zo, e.getKey(), e.getValue());
        zo.close();
        return bo.toByteArray();
    }

    static void put(ZipOutputStream zo, String name, byte[] data) throws Exception {
        zo.putNextEntry(new ZipEntry(name));
        zo.write(data);
        zo.closeEntry();
    }
}
