//! Platform-independent parts of the Windows app: KMZ building, file import parsing, MBTiles reading and settings.
//! Kept free of Windows APIs so it can be tested on any machine.

use regex::Regex;
use rusqlite::{Connection, OpenFlags, OptionalExtension};
use serde_json::{json, Value};
use std::collections::BTreeMap;
use std::io::{Cursor, Read, Write};
use std::path::{Path, PathBuf};

pub const TEMPLATE: &str = "wpmz/template.kml";
pub const WAYLINES: &str = "wpmz/waylines.wpml";

/* ---------- KMZ ---------- */

/// Aircraft details copied from a placeholder mission DJI Fly saved (same idea as the tablet's Kmz.java).
#[derive(Default, Debug)]
pub struct Codes {
    pub drone_info: Option<String>,
    pub payload_info: Option<String>,
    pub ns: Option<String>,
    pub drone_enum: Option<i64>,
    pub drone_sub: Option<i64>,
    pub extras: Vec<(String, Vec<u8>)>,
}

fn re(p: &str) -> Regex { Regex::new(p).unwrap() }

pub fn unzip(data: &[u8]) -> Result<Vec<(String, Vec<u8>)>, String> {
    let mut z = zip::ZipArchive::new(Cursor::new(data)).map_err(|e| format!("not a valid KMZ ({e})"))?;
    let mut out = Vec::new();
    for i in 0..z.len() {
        let mut f = z.by_index(i).map_err(|e| e.to_string())?;
        if f.is_dir() { continue; }
        let mut b = Vec::new();
        f.read_to_end(&mut b).map_err(|e| e.to_string())?;
        out.push((f.name().to_string(), b));
    }
    Ok(out)
}

pub fn read_codes(placeholder: &[u8]) -> Result<Codes, String> {
    let files = unzip(placeholder)?;
    let mut c = Codes::default();
    let src = files.iter().find(|(n, _)| n == TEMPLATE).or_else(|| files.iter().find(|(n, _)| n == WAYLINES));
    if let Some((_, b)) = src {
        let s = String::from_utf8_lossy(b);
        c.drone_info = re(r"(?s)<wpml:droneInfo>.*?</wpml:droneInfo>").find(&s).map(|m| m.as_str().to_string());
        c.payload_info = re(r"(?s)<wpml:payloadInfo>.*?</wpml:payloadInfo>").find(&s).map(|m| m.as_str().to_string());
        c.ns = re(r#"xmlns:wpml="([^"]+)""#).captures(&s).map(|m| m[1].to_string());
        if let Some(d) = &c.drone_info {
            c.drone_enum = re(r"<wpml:droneEnumValue>\s*(\d+)\s*</wpml:droneEnumValue>").captures(d).and_then(|m| m[1].parse().ok());
            c.drone_sub = re(r"<wpml:droneSubEnumValue>\s*(\d+)\s*</wpml:droneSubEnumValue>").captures(d).and_then(|m| m[1].parse().ok());
        }
    }
    c.extras = files.into_iter().filter(|(n, _)| n != TEMPLATE && n != WAYLINES).collect();
    Ok(c)
}

fn inject(xml: &str, c: Option<&Codes>) -> String {
    let Some(c) = c else { return xml.to_string() };
    let mut x = xml.to_string();
    if let Some(ns) = &c.ns {
        x = re(r#"xmlns:wpml="[^"]+""#).replacen(&x, 1, regex::NoExpand(&format!("xmlns:wpml=\"{ns}\""))).into_owned();
    }
    if let Some(d) = &c.drone_info {
        x = re(r"(?s)<wpml:droneInfo>.*?</wpml:droneInfo>").replacen(&x, 1, regex::NoExpand(d)).into_owned();
        if let Some(p) = &c.payload_info {
            let pre = re(r"(?s)<wpml:payloadInfo>.*?</wpml:payloadInfo>");
            x = if pre.is_match(&x) { pre.replacen(&x, 1, regex::NoExpand(p)).into_owned() }
                else { x.replacen("</wpml:droneInfo>", &format!("</wpml:droneInfo>{p}"), 1) };
        }
    }
    x
}

pub fn build_kmz(template: &str, waylines: &str, c: Option<&Codes>) -> Result<Vec<u8>, String> {
    let mut buf = Cursor::new(Vec::new());
    {
        let mut z = zip::ZipWriter::new(&mut buf);
        let opt = zip::write::SimpleFileOptions::default().compression_method(zip::CompressionMethod::Deflated);
        let mut put = |name: &str, data: &[u8]| -> Result<(), String> {
            z.start_file(name, opt).map_err(|e| e.to_string())?;
            z.write_all(data).map_err(|e| e.to_string())
        };
        put(TEMPLATE, inject(template, c).as_bytes())?;
        put(WAYLINES, inject(waylines, c).as_bytes())?;
        if let Some(c) = c { for (n, d) in &c.extras { put(n, d)?; } }
        z.finish().map_err(|e| e.to_string())?;
    }
    Ok(buf.into_inner())
}

/* ---------- importing missions and features (same rules as the tablet) ---------- */

pub fn parse_import(name: &str, bytes: &[u8], purpose: &str) -> Result<Value, String> {
    if bytes.len() > 25 * 1024 * 1024 { return Err("file is too large".into()); }
    let mut o = json!({ "type": "imported", "purpose": purpose, "name": name });
    if bytes.len() > 1 && bytes[0] == b'P' && bytes[1] == b'K' {
        let mut first_kml = None;
        for (n, d) in unzip(bytes)? {
            let k = n.to_lowercase();
            let v = String::from_utf8_lossy(&d).into_owned();
            if k.ends_with("waylines.wpml") { o["waylines"] = json!(v); }
            else if k.ends_with("template.kml") { o["template"] = json!(v); }
            else if k.ends_with(".kml") && first_kml.is_none() { first_kml = Some(v); }
        }
        if o.get("waylines").is_none() && o.get("template").is_none() {
            o["kml"] = json!(first_kml.ok_or("no mission or KML inside that KMZ")?);
        } else if o.get("waylines").is_none() && !o["template"].as_str().unwrap_or("").contains("Placemark") {
            return Err("that KMZ has no waypoints".into());
        }
    } else {
        let t = String::from_utf8_lossy(bytes).into_owned();
        if t.contains("wpml:") { o["waylines"] = json!(t); } else { o["kml"] = json!(t); }
    }
    Ok(o)
}

/* ---------- MBTiles ---------- */

pub struct MbInfo { pub name: String, pub minzoom: i64, pub maxzoom: i64, pub bounds: Option<[f64; 4]>, pub format: String }

pub fn open_mb(path: &Path) -> Result<Connection, String> {
    let c = Connection::open_with_flags(path, OpenFlags::SQLITE_OPEN_READ_ONLY | OpenFlags::SQLITE_OPEN_NO_MUTEX)
        .map_err(|e| format!("couldn't open the file ({e})"))?;
    c.query_row("SELECT count(*) FROM sqlite_master WHERE name='tiles'", [], |r| r.get::<_, i64>(0))
        .map_err(|_| "that file is not an MBTiles map".to_string())
        .and_then(|n| if n > 0 { Ok(c) } else { Err("no map tiles were found in that file".to_string()) })
}

pub fn mb_info(c: &Connection, file_name: &str) -> Result<MbInfo, String> {
    let mut meta = BTreeMap::new();
    if let Ok(mut st) = c.prepare("SELECT name, value FROM metadata") {
        let rows = st.query_map([], |r| Ok((r.get::<_, String>(0)?, r.get::<_, Option<String>>(1)?))).map_err(|e| e.to_string())?;
        for r in rows.flatten() { if let Some(v) = r.1 { meta.insert(r.0.to_lowercase(), v); } }
    }
    let (mut minz, mut maxz) = (meta.get("minzoom").and_then(|v| v.trim().parse().ok()), meta.get("maxzoom").and_then(|v| v.trim().parse().ok()));
    if minz.is_none() || maxz.is_none() {
        let (a, b): (Option<i64>, Option<i64>) = c.query_row("SELECT min(zoom_level), max(zoom_level) FROM tiles", [], |r| Ok((r.get(0)?, r.get(1)?)))
            .map_err(|e| e.to_string())?;
        minz = minz.or(a); maxz = maxz.or(b);
    }
    let minzoom = minz.ok_or("the map has no tiles")?;
    let maxzoom = maxz.unwrap_or(minzoom);
    let mut bounds = meta.get("bounds").and_then(|b| {
        let v: Vec<f64> = b.split(',').filter_map(|x| x.trim().parse().ok()).collect();
        (v.len() == 4 && v[2] > v[0] && v[3] > v[1]).then(|| [v[0], v[1], v[2], v[3]])
    });
    if bounds.is_none() {
        let r: Option<(i64, i64, i64, i64)> = c.query_row(
            "SELECT min(tile_column), max(tile_column), min(tile_row), max(tile_row) FROM tiles WHERE zoom_level=?1",
            [minzoom], |r| Ok((r.get(0)?, r.get(1)?, r.get(2)?, r.get(3)?))).optional().ok().flatten();
        if let Some((x1, x2, y1, y2)) = r {
            let n = 2f64.powi(minzoom as i32);
            let lng = |x: f64| x / n * 360.0 - 180.0;
            let lat = |y: f64| { let t = std::f64::consts::PI * (1.0 - 2.0 * (n - y) / n); t.sinh().atan().to_degrees() };
            bounds = Some([lng(x1 as f64), lat(y1 as f64), lng(x2 as f64 + 1.0), lat(y2 as f64 + 1.0)]);
        }
    }
    let stem = Path::new(file_name).file_stem().map(|s| s.to_string_lossy().into_owned()).unwrap_or_else(|| "My map".into());
    Ok(MbInfo { name: meta.get("name").cloned().filter(|s| !s.trim().is_empty()).unwrap_or(stem), minzoom, maxzoom, bounds, format: meta.get("format").cloned().unwrap_or_default() })
}

/// Which tiles exist at zoom z, as a flat [x, y, ...] list in XYZ numbering (for drawing the map's outline).
pub fn mb_coverage(c: &Connection, z: i64) -> Vec<i64> {
    let mut out = Vec::new();
    if let Ok(mut st) = c.prepare("SELECT tile_column, tile_row FROM tiles WHERE zoom_level=?1 LIMIT 200000") {
        if let Ok(rows) = st.query_map([z], |r| Ok((r.get::<_, i64>(0)?, r.get::<_, i64>(1)?))) {
            for (x, row) in rows.flatten() { out.push(x); out.push((1i64 << z) - 1 - row); }
        }
    }
    out
}

/// One tile by XYZ address (MBTiles stores rows from the bottom).
pub fn mb_tile(c: &Connection, z: i64, x: i64, y: i64) -> Option<Vec<u8>> {
    let row = (1i64 << z) - 1 - y;
    c.query_row("SELECT tile_data FROM tiles WHERE zoom_level=?1 AND tile_column=?2 AND tile_row=?3", [z, x, row], |r| r.get::<_, Vec<u8>>(0)).ok()
}

pub fn mime_of(d: &[u8]) -> &'static str {
    if d.starts_with(&[0x89, b'P']) { "image/png" } else if d.starts_with(&[0xff, 0xd8]) { "image/jpeg" } else if d.starts_with(b"RIFF") { "image/webp" } else { "application/octet-stream" }
}

/* ---------- settings ---------- */

pub struct Settings { pub path: PathBuf, pub v: Value }

impl Settings {
    pub fn load(dir: &Path) -> Settings {
        let path = dir.join("settings.json");
        let v = std::fs::read(&path).ok().and_then(|b| serde_json::from_slice(&b).ok()).unwrap_or_else(|| json!({}));
        Settings { path, v }
    }
    pub fn save(&self) {
        let _ = std::fs::create_dir_all(self.path.parent().unwrap());
        let tmp = self.path.with_extension("json.tmp");
        if std::fs::write(&tmp, serde_json::to_vec_pretty(&self.v).unwrap()).is_ok() { let _ = std::fs::rename(&tmp, &self.path); }
    }
    pub fn sync_path(&self) -> Option<PathBuf> { self.v["syncPath"].as_str().filter(|s| !s.is_empty()).map(PathBuf::from) }
    pub fn maps(&self) -> Vec<Value> { self.v["maps"].as_array().cloned().unwrap_or_default() }
    pub fn set_maps(&mut self, m: Vec<Value>) { self.v["maps"] = Value::Array(m); }
}

/// The map list as the app expects it, flagging files that have gone missing.
pub fn maps_message(s: &Settings, added: Option<&str>) -> Value {
    let list: Vec<Value> = s.maps().into_iter().map(|mut m| {
        let missing = !Path::new(m["path"].as_str().unwrap_or("")).exists();
        if let Some(f) = Path::new(m["path"].as_str().unwrap_or("")).file_stem() { m["file"] = json!(f.to_string_lossy()); }
        if missing { m["missing"] = json!(true); }
        m.as_object_mut().map(|o| o.remove("path"));
        m
    }).collect();
    let mut o = json!({ "type": "maps", "maps": list });
    if let Some(a) = added { o["added"] = json!(a); }
    o
}

pub fn add_map(s: &mut Settings, path: &Path) -> Result<String, String> {
    let c = open_mb(path)?;
    let fname = path.file_name().map(|f| f.to_string_lossy().into_owned()).unwrap_or_default();
    let info = mb_info(&c, &fname)?;
    let p = path.to_string_lossy().into_owned();
    let mut maps = s.maps();
    if let Some(m) = maps.iter().find(|m| m["path"].as_str() == Some(&p)) { return Ok(m["id"].as_str().unwrap_or("").to_string()); }
    let id = format!("w{}", std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|d| d.as_millis()).unwrap_or(0));
    let size = std::fs::metadata(path).map(|m| m.len()).unwrap_or(0);
    maps.push(json!({ "id": id, "path": p, "name": info.name, "size": size, "minzoom": info.minzoom, "maxzoom": info.maxzoom,
        "bounds": info.bounds.map(|b| b.to_vec()), "format": info.format }));
    s.set_maps(maps);
    s.save();
    Ok(id)
}

/// Standard base64 (as the app sends the mission picture). Returns None for empty or invalid input.
pub fn b64decode(s: &str) -> Option<Vec<u8>> {
    let mut out = Vec::with_capacity(s.len() * 3 / 4);
    let (mut acc, mut bits) = (0u32, 0u32);
    for ch in s.bytes() {
        let v = match ch { b'A'..=b'Z' => ch - b'A', b'a'..=b'z' => ch - b'a' + 26, b'0'..=b'9' => ch - b'0' + 52, b'+' => 62, b'/' => 63, b'=' | b'\n' | b'\r' | b' ' => continue, _ => return None } as u32;
        acc = (acc << 6) | v; bits += 6;
        if bits >= 8 { bits -= 8; out.push((acc >> bits) as u8); acc &= (1 << bits) - 1; }
    }
    if out.is_empty() { None } else { Some(out) }
}

/* ---------- dates for the mission list ---------- */

pub fn fmt_date(unix_ms: i64) -> String {
    // Local time is applied by the caller (offset in minutes) so this stays testable.
    let secs = unix_ms.div_euclid(1000);
    let days = secs.div_euclid(86400);
    let sod = secs.rem_euclid(86400);
    // civil-from-days (Howard Hinnant)
    let z = days + 719468;
    let era = z.div_euclid(146097);
    let doe = z - era * 146097;
    let yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
    let y = yoe + era * 400;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = doy - (153 * mp + 2) / 5 + 1;
    let m = if mp < 10 { mp + 3 } else { mp - 9 };
    let y = if m <= 2 { y + 1 } else { y };
    let months = ["Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"];
    let (h, mi) = (sod / 3600, (sod % 3600) / 60);
    let (h12, ap) = (if h % 12 == 0 { 12 } else { h % 12 }, if h < 12 { "AM" } else { "PM" });
    format!("{} {}, {} {}:{:02} {}", months[(m - 1) as usize], d, y, h12, mi, ap)
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn kmz_roundtrip_with_codes() {
        let ph_t = r#"<kml xmlns:wpml="http://www.dji.com/wpmz/1.0.6"><wpml:droneInfo><wpml:droneEnumValue>68</wpml:droneEnumValue><wpml:droneSubEnumValue>0</wpml:droneSubEnumValue></wpml:droneInfo><wpml:payloadInfo><wpml:payloadEnumValue>80</wpml:payloadEnumValue></wpml:payloadInfo></kml>"#;
        let ph = build_kmz(ph_t, "<x/>", None).unwrap();
        let mut ph2 = Cursor::new(Vec::new());
        { let mut z = zip::ZipWriter::new(&mut ph2); let o = zip::write::SimpleFileOptions::default();
          for (n, d) in unzip(&ph).unwrap() { z.start_file(n, o).unwrap(); z.write_all(&d).unwrap(); }
          z.start_file("res/preview.png", o).unwrap(); z.write_all(b"png").unwrap(); z.finish().unwrap(); }
        let c = read_codes(&ph2.into_inner()).unwrap();
        assert_eq!(c.drone_enum, Some(68)); assert_eq!(c.drone_sub, Some(0)); assert_eq!(c.ns.as_deref(), Some("http://www.dji.com/wpmz/1.0.6"));
        let mine = r#"<kml xmlns:wpml="http://www.dji.com/wpmz/1.0.2"><wpml:droneInfo><wpml:droneEnumValue>1</wpml:droneEnumValue></wpml:droneInfo></kml>"#;
        let out = build_kmz(mine, mine, Some(&c)).unwrap();
        let files = unzip(&out).unwrap();
        let t = String::from_utf8(files[0].1.clone()).unwrap();
        assert!(t.contains("1.0.6") && t.contains("<wpml:droneEnumValue>68") && t.contains("payloadEnumValue>80"), "{t}");
        assert_eq!(files.len(), 3);
        assert_eq!(files[2].0, "res/preview.png");
    }
    #[test]
    fn import_rules() {
        let k = build_kmz("<Placemark/>", "<wpml:x/>", None).unwrap();
        let o = parse_import("a.kmz", &k, "mission").unwrap();
        assert!(o["waylines"].is_string() && o["template"].is_string());
        let o = parse_import("a.kml", b"<kml><Placemark/></kml>", "markers").unwrap();
        assert_eq!(o["purpose"], "markers"); assert!(o["kml"].is_string());
    }
    #[test]
    fn b64() { assert_eq!(b64decode("SGVsbG8gd29ybGQ=").unwrap(), b"Hello world"); assert!(b64decode("").is_none()); }
    #[test]
    fn dates() { assert_eq!(fmt_date(1761955200000), "Nov 1, 2025 12:00 AM"); assert_eq!(fmt_date(1761998700000), "Nov 1, 2025 12:05 PM"); }
    #[test]
    fn mbtiles() {
        let dir = std::env::var("MB_TEST_DIR").unwrap_or_default();
        if dir.is_empty() { return; }
        for f in ["qgis", "mbutil", "noindex", "pk", "deep"] {
            let p = Path::new(&dir).join(format!("{f}.mbtiles"));
            let c = open_mb(&p).unwrap();
            let i = mb_info(&c, &format!("{f}.mbtiles")).unwrap();
            assert!(i.bounds.is_some(), "{f}");
            let (z, x, row): (i64, i64, i64) = c.query_row("SELECT zoom_level,tile_column,tile_row FROM tiles LIMIT 1", [], |r| Ok((r.get(0)?, r.get(1)?, r.get(2)?))).unwrap();
            let y = (1 << z) - 1 - row;
            assert!(mb_tile(&c, z, x, y).is_some(), "{f}");
            println!("{f}: {} z{}-{} {:?}", i.name, i.minzoom, i.maxzoom, i.bounds);
        }
    }
}
