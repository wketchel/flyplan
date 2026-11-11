//! Convert a GeoTIFF (for example a WebODM orthophoto) to an MBTiles basemap
//! using an external GDAL install. No Windows APIs, so this can be tested on Linux.

use serde_json::Value;
use std::env;
use std::ffi::OsStr;
use std::fs;
use std::io::Read;
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{SystemTime, UNIX_EPOCH};

/// Shown in the app when GDAL cannot be found. Does not claim that FlyPlan ships GDAL.
pub const GDAL_MISSING: &str = "FlyPlan needs GDAL to turn a GeoTIFF into a map. Install QGIS or OSGeo4W, or put a GDAL folder next to FlyPlan.exe (for example gdal\\bin\\gdal_translate.exe). FlyPlan does not include GDAL.";

const NOT_GEOREFERENCED: &str = "That file isn’t georeferenced, so it can’t be placed on the map. WebODM’s odm_orthophoto.tif should work; a plain picture without coordinates will not.";

/// A discovered GDAL command-line install.
#[derive(Debug, Clone)]
pub struct Gdal {
    pub bin_dir: PathBuf,
    pub info: PathBuf,
    pub warp: PathBuf,
    pub translate: PathBuf,
    pub addo: PathBuf,
    /// GDAL_DATA, PROJ_LIB, PROJ_DATA, and similar, inferred from the install layout.
    pub extra_env: Vec<(String, String)>,
}

/// Look for GDAL in this order: a portable copy next to the app, PATH, then
/// common QGIS / OSGeo4W locations. `extra_roots` is for tests (and is checked
/// after the portable app folder, before PATH).
pub fn find_gdal(exe_dir: Option<&Path>, extra_roots: &[PathBuf]) -> Option<Gdal> {
    let mut seen = Vec::new();
    if let Some(dir) = exe_dir {
        for cand in portable_bins(dir) {
            if let Some(g) = tools_in(&cand) {
                return Some(g);
            }
            seen.push(cand);
        }
    }
    for root in extra_roots {
        for cand in portable_bins(root).into_iter().chain(std::iter::once(root.clone())) {
            if seen.iter().any(|p| p == &cand) { continue; }
            if let Some(g) = tools_in(&cand) {
                return Some(g);
            }
            seen.push(cand);
        }
    }
    for dir in path_dirs() {
        if seen.iter().any(|p| p == &dir) { continue; }
        if let Some(g) = tools_in(&dir) {
            return Some(g);
        }
        seen.push(dir);
    }
    for dir in common_windows_bins() {
        if seen.iter().any(|p| p == &dir) { continue; }
        if let Some(g) = tools_in(&dir) {
            return Some(g);
        }
    }
    None
}

fn portable_bins(root: &Path) -> Vec<PathBuf> {
    vec![
        root.to_path_buf(),
        root.join("gdal"),
        root.join("gdal").join("bin"),
        root.join("bin"),
    ]
}

fn path_dirs() -> Vec<PathBuf> {
    env::var_os("PATH")
        .map(|p| env::split_paths(&p).collect())
        .unwrap_or_default()
}

fn common_windows_bins() -> Vec<PathBuf> {
    let mut dirs = Vec::new();
    if let Some(root) = env::var_os("OSGEO4W_ROOT") {
        dirs.push(PathBuf::from(&root).join("bin"));
        dirs.push(PathBuf::from(root).join("apps").join("gdal").join("bin"));
    }
    for p in [
        r"C:\OSGeo4W\bin",
        r"C:\OSGeo4W64\bin",
        r"C:\OSGeo4W\apps\gdal\bin",
        r"C:\Program Files\GDAL",
        r"C:\Program Files\GDAL\bin",
        r"C:\Program Files (x86)\GDAL",
        r"C:\Program Files (x86)\GDAL\bin",
    ] {
        dirs.push(PathBuf::from(p));
    }
    for pf in [r"C:\Program Files", r"C:\Program Files (x86)"] {
        if let Ok(rd) = fs::read_dir(pf) {
            let mut qgis: Vec<PathBuf> = rd
                .flatten()
                .map(|e| e.path())
                .filter(|p| {
                    p.file_name()
                        .map(|n| n.to_string_lossy().to_ascii_uppercase().starts_with("QGIS"))
                        .unwrap_or(false)
                })
                .collect();
            qgis.sort();
            qgis.reverse(); // prefer a newer "QGIS 3.40" over "QGIS 3.28"
            for p in qgis {
                dirs.push(p.join("bin"));
                dirs.push(p.join("apps").join("gdal").join("bin"));
            }
        }
    }
    dirs
}

fn tool(dir: &Path, name: &str) -> Option<PathBuf> {
    for cand in [dir.join(name), dir.join(format!("{name}.exe"))] {
        if cand.is_file() {
            return Some(cand);
        }
    }
    None
}

fn tools_in(dir: &Path) -> Option<Gdal> {
    let info = tool(dir, "gdalinfo")?;
    let warp = tool(dir, "gdalwarp")?;
    let translate = tool(dir, "gdal_translate")?;
    let addo = tool(dir, "gdaladdo")?;
    Some(Gdal {
        extra_env: implied_env(dir),
        bin_dir: dir.to_path_buf(),
        info,
        warp,
        translate,
        addo,
    })
}

/// Infer GDAL_DATA / PROJ_* from a bin directory's install layout (OSGeo4W, QGIS, conda, portable).
pub fn implied_env(bin_dir: &Path) -> Vec<(String, String)> {
    let prefix = bin_dir.parent().unwrap_or(bin_dir);
    let mut gdal_cands = vec![
        bin_dir.join("gdal-data"),
        bin_dir.join("data"),
        bin_dir.join("share").join("gdal"),
        prefix.join("share").join("gdal"),
        prefix.join("apps").join("gdal").join("share").join("gdal"),
        prefix.join("gdal-data"),
    ];
    let mut proj_cands = vec![
        bin_dir.join("proj"),
        bin_dir.join("share").join("proj"),
        prefix.join("share").join("proj"),
        prefix.join("share").join("proj4"),
        prefix.join("apps").join("proj").join("share").join("proj"),
        prefix.join("apps").join("proj4").join("share").join("proj"),
    ];
    if let Ok(rd) = fs::read_dir(prefix.join("apps")) {
        for e in rd.flatten() {
            let n = e.file_name().to_string_lossy().to_ascii_lowercase();
            if n.starts_with("gdal") {
                gdal_cands.push(e.path().join("share").join("gdal"));
            }
            if n.starts_with("proj") {
                proj_cands.push(e.path().join("share").join("proj"));
            }
        }
    }
    let mut env = Vec::new();
    if let Some(p) = first_dir(&gdal_cands) {
        env.push(("GDAL_DATA".into(), p.to_string_lossy().into_owned()));
    }
    if let Some(p) = first_dir(&proj_cands) {
        let s = p.to_string_lossy().into_owned();
        env.push(("PROJ_DATA".into(), s.clone()));
        env.push(("PROJ_LIB".into(), s));
    }
    env
}

fn first_dir(cands: &[PathBuf]) -> Option<PathBuf> {
    cands.iter().find(|p| p.is_dir()).cloned()
}

impl Gdal {
    fn command(&self, tool: &Path) -> Command {
        let mut c = Command::new(tool);
        let mut path = self.bin_dir.as_os_str().to_os_string();
        if let Some(old) = env::var_os("PATH") {
            path.push(if cfg!(windows) { ";" } else { ":" });
            path.push(old);
        }
        c.env("PATH", path);
        for (k, v) in &self.extra_env {
            c.env(k, v);
        }
        // Bound memory for large rasters; let GDAL stream tiles to the MBTiles file.
        c.env("GDAL_CACHEMAX", "256");
        c.env("GDAL_NUM_THREADS", "ALL_CPUS");
        // FlyPlan is a windowed app: without this each GDAL step would flash a console window.
        #[cfg(windows)]
        {
            use std::os::windows::process::CommandExt;
            c.creation_flags(0x0800_0000); // CREATE_NO_WINDOW
        }
        c
    }
}

/// What gdalinfo reported about a candidate GeoTIFF.
#[derive(Debug, Clone, Default)]
pub struct GeoInfo {
    pub width: i64,
    pub height: i64,
    pub has_alpha: bool,
    pub has_nodata: bool,
    pub has_crs: bool,
    pub has_geotransform: bool,
    pub has_gcps: bool,
}

pub fn inspect(gdal: &Gdal, path: &Path) -> Result<GeoInfo, String> {
    let (ok, out) = run(gdal, &gdal.info, &[OsStr::new("-json"), path.as_os_str()], None)?;
    if !ok {
        return Err(format!(
            "GDAL couldn’t read that file. {}",
            short_gdal_error(&out)
        ));
    }
    let json: Value = serde_json::from_str(json_slice(&out)).map_err(|_| {
        "GDAL didn’t return a usable description of that file.".to_string()
    })?;
    Ok(geoinfo_from_json(&json))
}

pub fn geoinfo_from_json(j: &Value) -> GeoInfo {
    let size = j.get("size").and_then(|s| s.as_array());
    let width = size.and_then(|s| s.first()).and_then(|v| v.as_i64()).unwrap_or(0);
    let height = size.and_then(|s| s.get(1)).and_then(|v| v.as_i64()).unwrap_or(0);
    let bands = j.get("bands").and_then(|b| b.as_array());
    let has_alpha = bands.is_some_and(|bs| {
        bs.iter().any(|b| {
            b.get("colorInterpretation")
                .and_then(|v| v.as_str())
                .is_some_and(|s| s.eq_ignore_ascii_case("alpha"))
        })
    });
    let has_nodata = bands.is_some_and(|bs| bs.iter().any(|b| b.get("noDataValue").and_then(|v| v.as_f64()).is_some()));
    let has_crs = crs_present(j);
    let has_geotransform = valid_geotransform(j.get("geoTransform"));
    let has_gcps = j
        .get("gcps")
        .and_then(|g| g.get("gcp").or_else(|| g.get("gcps")))
        .and_then(|g| g.as_array())
        .is_some_and(|a| !a.is_empty());
    GeoInfo { width, height, has_alpha, has_nodata, has_crs, has_geotransform, has_gcps }
}

fn crs_present(j: &Value) -> bool {
    let cs = j.get("coordinateSystem");
    if cs.and_then(|c| c.get("wkt")).and_then(|v| v.as_str()).is_some_and(|s| !s.trim().is_empty()) {
        return true;
    }
    if cs.and_then(|c| c.get("projjson")).is_some() {
        return true;
    }
    if j.pointer("/stac/proj:epsg").and_then(|v| v.as_i64()).is_some() {
        return true;
    }
    if j.pointer("/stac/proj:wkt2").and_then(|v| v.as_str()).is_some_and(|s| !s.trim().is_empty()) {
        return true;
    }
    if j.pointer("/gcps/coordinateSystem/wkt").and_then(|v| v.as_str()).is_some_and(|s| !s.trim().is_empty()) {
        return true;
    }
    false
}

fn valid_geotransform(v: Option<&Value>) -> bool {
    let Some(a) = v.and_then(|v| v.as_array()) else { return false };
    if a.len() < 6 { return false; }
    let xres = a[1].as_f64().unwrap_or(0.0);
    let yres = a[5].as_f64().unwrap_or(0.0);
    xres != 0.0 && yres != 0.0
}

pub fn validate(info: &GeoInfo) -> Result<(), String> {
    if info.width <= 0 || info.height <= 0 {
        return Err("That file doesn’t look like a GeoTIFF image.".into());
    }
    if !info.has_crs || (!info.has_geotransform && !info.has_gcps) {
        return Err(NOT_GEOREFERENCED.into());
    }
    Ok(())
}

/// `name.mbtiles`, or `name-2.mbtiles`, `name-3.mbtiles`, … if the name is taken.
pub fn unique_mbtiles_path(tif: &Path) -> Result<PathBuf, String> {
    let dir = tif.parent().filter(|p| !p.as_os_str().is_empty()).unwrap_or_else(|| Path::new("."));
    let stem = tif.file_stem().and_then(|s| s.to_str()).filter(|s| !s.is_empty()).unwrap_or("orthophoto");
    unique_in(dir, stem, "mbtiles")
}

fn unique_in(dir: &Path, stem: &str, ext: &str) -> Result<PathBuf, String> {
    let first = dir.join(format!("{stem}.{ext}"));
    if !first.exists() {
        return Ok(first);
    }
    for i in 2..10_000 {
        let p = dir.join(format!("{stem}-{i}.{ext}"));
        if !p.exists() {
            return Ok(p);
        }
    }
    Err("Couldn’t find a free name for the new map file next to the GeoTIFF.".into())
}

/// Convert `input` GeoTIFF to `output` MBTiles (PNG tiles, TMS, Web Mercator, overviews).
/// The source file is never overwritten. Progress is 0–100 with a short status message.
pub fn convert(
    gdal: &Gdal,
    input: &Path,
    output: &Path,
    mut progress: impl FnMut(u32, u32, &str),
) -> Result<(), String> {
    if output.as_os_str().is_empty() {
        return Err("No output path was given.".into());
    }
    progress(1, 100, "Checking the GeoTIFF…");
    let info = inspect(gdal, input)?;
    validate(&info)?;

    let dir = output.parent().filter(|p| !p.as_os_str().is_empty()).unwrap_or_else(|| Path::new("."));
    let stem = output.file_stem().and_then(|s| s.to_str()).unwrap_or("map");
    let tag = unique_tag();
    let mut temps = TempFiles::default();
    let vrt = temps.add(dir.join(format!("{stem}.flyplan-{tag}.vrt")));
    let part = temps.add(dir.join(format!("{stem}.flyplan-{tag}.mbtiles.part")));

    let mut warp_args: Vec<std::ffi::OsString> = vec![
        "-t_srs".into(), "EPSG:3857".into(),
        "-r".into(), "bilinear".into(),
        "-of".into(), "VRT".into(),
        "-wo".into(), "NUM_THREADS=ALL_CPUS".into(),
        "-multi".into(),
    ];
    // Preserve an existing alpha band. Only build alpha from nodata when the file has no alpha;
    // inventing nodata from black would punch holes in dark roofs and water.
    if !info.has_alpha && info.has_nodata {
        warp_args.push("-dstalpha".into());
    }
    warp_args.push(input.as_os_str().to_os_string());
    warp_args.push(vrt.as_os_str().to_os_string());
    progress(4, 100, "Preparing the image for the map…");
    run_progress(gdal, &gdal.warp, &warp_args, 4, 12, "Preparing the image for the map…", &mut progress)?;

    let name = stem.to_string();
    let translate_args: Vec<std::ffi::OsString> = vec![
        "-of".into(), "MBTILES".into(),
        "-co".into(), "TILE_FORMAT=PNG".into(),
        "-co".into(), "ZLEVEL=6".into(),
        "-co".into(), "ZOOM_LEVEL_STRATEGY=AUTO".into(),
        "-co".into(), "WRITE_BOUNDS=YES".into(),
        "-co".into(), "WRITE_MINMAXZOOM=YES".into(),
        "-co".into(), "TYPE=overlay".into(),
        "-co".into(), format!("NAME={name}").into(),
        vrt.as_os_str().to_os_string(),
        part.as_os_str().to_os_string(),
    ];
    progress(12, 100, "Building map tiles…");
    run_progress(gdal, &gdal.translate, &translate_args, 12, 80, "Building map tiles…", &mut progress)?;

    // Lower-zoom overviews so the map is visible when zoomed out. Extra factors are harmless
    // if the image is already only one or two tiles across.
    let addo_args: Vec<std::ffi::OsString> = vec![
        "-r".into(), "average".into(),
        part.as_os_str().to_os_string(),
        "2".into(), "4".into(), "8".into(), "16".into(), "32".into(),
        "64".into(), "128".into(), "256".into(), "512".into(), "1024".into(),
    ];
    progress(80, 100, "Building overview zooms…");
    run_progress(gdal, &gdal.addo, &addo_args, 80, 98, "Building overview zooms…", &mut progress)?;

    if let Some(parent) = output.parent() {
        if !parent.as_os_str().is_empty() {
            fs::create_dir_all(parent).map_err(|e| format!("Couldn't create the folder for the map file: {e}"))?;
        }
    }
    if output.exists() {
        return Err("The map file name was taken before the conversion finished. Try again.".into());
    }
    fs::rename(&part, output).map_err(|e| {
        format!("The map was built, but it couldn't be saved as {}: {e}", output.display())
    })?;
    temps.forget(&part);
    progress(100, 100, "Done");
    Ok(())
}

#[derive(Default)]
struct TempFiles { paths: Vec<PathBuf> }

impl TempFiles {
    fn add(&mut self, p: PathBuf) -> PathBuf {
        self.paths.push(p.clone());
        p
    }
    fn forget(&mut self, p: &Path) {
        self.paths.retain(|x| x != p);
    }
}

impl Drop for TempFiles {
    fn drop(&mut self) {
        for p in &self.paths {
            remove_sidecar(p);
        }
    }
}

fn remove_sidecar(p: &Path) {
    let _ = fs::remove_file(p);
    let display = p.display().to_string();
    for suf in [".aux.xml", "-journal", "-wal", "-shm"] {
        let _ = fs::remove_file(PathBuf::from(format!("{display}{suf}")));
    }
}

fn unique_tag() -> String {
    let t = SystemTime::now().duration_since(UNIX_EPOCH).map(|d| d.as_millis()).unwrap_or(0);
    format!("{}-{}", std::process::id(), t)
}

fn run(
    gdal: &Gdal,
    tool: &Path,
    args: &[impl AsRef<OsStr>],
    mut on_out: Option<&mut dyn FnMut(&[u8])>,
) -> Result<(bool, String), String> {
    let mut cmd = gdal.command(tool);
    cmd.args(args);
    cmd.stdin(Stdio::null());
    cmd.stdout(Stdio::piped());
    cmd.stderr(Stdio::piped());
    let mut child = cmd.spawn().map_err(|e| {
        format!("Couldn't start GDAL ({}): {e}", tool.display())
    })?;
    let mut stdout = child.stdout.take().ok_or_else(|| "GDAL produced no output.".to_string())?;
    let mut stderr = child.stderr.take().ok_or_else(|| "GDAL produced no output.".to_string())?;
    let err_buf = Arc::new(Mutex::new(Vec::new()));
    let err_c = err_buf.clone();
    let t = thread::spawn(move || {
        let mut b = Vec::new();
        let _ = stderr.read_to_end(&mut b);
        *err_c.lock().unwrap() = b;
    });
    let mut out = Vec::new();
    let mut tmp = [0u8; 1024];
    loop {
        match stdout.read(&mut tmp) {
            Ok(0) => break,
            Ok(n) => {
                out.extend_from_slice(&tmp[..n]);
                if let Some(cb) = on_out.as_mut() {
                    cb(&out);
                }
            }
            Err(e) if e.kind() == std::io::ErrorKind::Interrupted => continue,
            Err(e) => return Err(format!("Couldn't read GDAL’s output: {e}")),
        }
    }
    let _ = t.join();
    let err = err_buf.lock().unwrap().clone();
    if let Some(cb) = on_out.as_mut() {
        let mut both = out.clone();
        both.extend_from_slice(&err);
        cb(&both);
    }
    let status = child.wait().map_err(|e| format!("GDAL did not finish: {e}"))?;
    let mut text = String::new();
    text.push_str(&String::from_utf8_lossy(&out));
    if !err.is_empty() {
        if !text.is_empty() { text.push('\n'); }
        text.push_str(&String::from_utf8_lossy(&err));
    }
    Ok((status.success(), text))
}

fn run_progress(
    gdal: &Gdal,
    tool: &Path,
    args: &[impl AsRef<OsStr>],
    lo: u32,
    hi: u32,
    message: &str,
    progress: &mut impl FnMut(u32, u32, &str),
) -> Result<(), String> {
    let mut last = lo;
    let (ok, out) = run(gdal, tool, args, Some(&mut |buf| {
        if let Some(p) = last_progress(buf) {
            let mapped = lo + (hi.saturating_sub(lo)) * p.min(100) / 100;
            if mapped != last {
                last = mapped;
                progress(mapped, 100, message);
            }
        }
    }))?;
    if !ok {
        return Err(format!("GDAL couldn’t finish this step ({message}). {}", short_gdal_error(&out)));
    }
    progress(hi, 100, message);
    Ok(())
}

fn last_progress(buf: &[u8]) -> Option<u32> {
    let s = String::from_utf8_lossy(buf);
    let b = s.as_bytes();
    let mut i = 0;
    let mut last = None;
    while i < b.len() {
        if b[i].is_ascii_digit() {
            let start = i;
            while i < b.len() && b[i].is_ascii_digit() { i += 1; }
            if let Ok(n) = s[start..i].parse::<u32>() {
                if n <= 100 {
                    let rest = &s[i..];
                    let marked = rest.starts_with("...") || rest.starts_with('%') || rest.starts_with(" - done");
                    // Mid-stream GDAL may have written "0...10...20...30" with no trailing dots yet.
                    let trailing = rest.is_empty() && s[..start].ends_with("...");
                    if marked || trailing {
                        last = Some(n);
                    }
                }
            }
        } else {
            i += 1;
        }
    }
    last
}

fn json_slice(out: &str) -> &str {
    let start = out.find('{').unwrap_or(0);
    let end = out.rfind('}').map(|i| i + 1).unwrap_or(out.len());
    if start < end { &out[start..end] } else { out }
}

fn short_gdal_error(out: &str) -> String {
    let lines: Vec<&str> = out
        .lines()
        .map(|l| l.trim())
        .filter(|l| {
            !l.is_empty()
                && !l.starts_with("0...")
                && !l.contains("...10...")
                && !l.ends_with(" - done.")
                && !l.starts_with("Processing ")
                && !l.starts_with("Input file size")
        })
        .collect();
    let useful: Vec<&str> = if let Some(i) = lines.iter().rposition(|l| l.starts_with("ERROR") || l.starts_with("ERROR ")) {
        lines[i..].to_vec()
    } else {
        lines
    };
    let s = useful.iter().rev().take(8).rev().cloned().collect::<Vec<_>>().join(" ");
    let s = s.trim();
    if s.is_empty() {
        "No further detail was given.".into()
    } else if s.len() > 500 {
        format!("{}…", &s[..500])
    } else {
        s.to_string()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs::File;
    use std::io::Write;
    use std::sync::atomic::{AtomicU64, Ordering};
    use std::sync::Mutex;

    // find_gdal reads PATH; one test temporarily changes it. Keep those tests off each other's toes.
    static PATH_LOCK: Mutex<()> = Mutex::new(());

    fn tmp() -> PathBuf {
        static N: AtomicU64 = AtomicU64::new(0);
        let p = env::temp_dir().join(format!(
            "flyplan-geotiff-{}-{}",
            std::process::id(),
            N.fetch_add(1, Ordering::Relaxed)
        ));
        let _ = fs::remove_dir_all(&p);
        fs::create_dir_all(&p).unwrap();
        p
    }

    #[test]
    fn unique_names_avoid_collisions() {
        let d = tmp();
        let tif = d.join("South Pasture 11-8-2026.tif");
        File::create(&tif).unwrap();
        let a = unique_mbtiles_path(&tif).unwrap();
        assert_eq!(a.file_name().unwrap(), "South Pasture 11-8-2026.mbtiles");
        File::create(&a).unwrap();
        let b = unique_mbtiles_path(&tif).unwrap();
        assert_eq!(b.file_name().unwrap(), "South Pasture 11-8-2026-2.mbtiles");
        File::create(&b).unwrap();
        let c = unique_mbtiles_path(&tif).unwrap();
        assert_eq!(c.file_name().unwrap(), "South Pasture 11-8-2026-3.mbtiles");
        let _ = fs::remove_dir_all(&d);
    }

    #[test]
    fn geoinfo_detects_alpha_crs_and_transform() {
        let j: Value = serde_json::from_str(r#"{
            "size":[64,64],
            "coordinateSystem":{"wkt":"GEOGCRS[\"WGS 84\"]"},
            "geoTransform":[-97.5,0.0001,0,32.8,0,-0.0001],
            "stac":{"proj:epsg":4326},
            "bands":[
                {"band":1,"colorInterpretation":"Red"},
                {"band":4,"colorInterpretation":"Alpha"}
            ]
        }"#).unwrap();
        let i = geoinfo_from_json(&j);
        assert!(i.has_alpha && i.has_crs && i.has_geotransform && !i.has_nodata);
        assert_eq!((i.width, i.height), (64, 64));
        assert!(validate(&i).is_ok());
    }

    #[test]
    fn rejects_ungeoreferenced_and_identity_without_crs() {
        let plain: Value = serde_json::from_str(r#"{"size":[10,10],"bands":[{"colorInterpretation":"Red"}]}"#).unwrap();
        let i = geoinfo_from_json(&plain);
        assert!(!i.has_crs && !i.has_geotransform);
        assert!(validate(&i).unwrap_err().contains("isn’t georeferenced"));

        let ident: Value = serde_json::from_str(r#"{
            "size":[10,10],
            "geoTransform":[0,1,0,0,0,1],
            "bands":[{"colorInterpretation":"Gray"}]
        }"#).unwrap();
        let i = geoinfo_from_json(&ident);
        assert!(i.has_geotransform && !i.has_crs);
        assert!(validate(&i).is_err());
    }

    #[test]
    fn implied_env_finds_share_layout() {
        let d = tmp();
        let bin = d.join("bin");
        fs::create_dir_all(bin.join("dummy")).unwrap();
        fs::create_dir_all(d.join("share").join("gdal")).unwrap();
        fs::create_dir_all(d.join("share").join("proj")).unwrap();
        let env = implied_env(&bin);
        let map: std::collections::HashMap<_, _> = env.into_iter().collect();
        assert_eq!(Path::new(&map["GDAL_DATA"]), d.join("share").join("gdal"));
        assert_eq!(Path::new(&map["PROJ_DATA"]), d.join("share").join("proj"));
        assert_eq!(map["PROJ_LIB"], map["PROJ_DATA"]);
        let _ = fs::remove_dir_all(&d);
    }

    #[test]
    fn implied_env_finds_versioned_qgis_apps_layout() {
        let d = tmp();
        let bin = d.join("bin");
        fs::create_dir_all(&bin).unwrap();
        fs::create_dir_all(d.join("apps").join("gdal-3.8").join("share").join("gdal")).unwrap();
        fs::create_dir_all(d.join("apps").join("proj9").join("share").join("proj")).unwrap();
        let env = implied_env(&bin);
        let map: std::collections::HashMap<_, _> = env.into_iter().collect();
        assert!(map["GDAL_DATA"].ends_with("gdal"));
        assert!(map["PROJ_DATA"].contains("proj"));
        let _ = fs::remove_dir_all(&d);
    }

    fn write_fake_tools(dir: &Path) {
        fs::create_dir_all(dir).unwrap();
        for name in ["gdalinfo", "gdalwarp", "gdal_translate", "gdaladdo"] {
            let p = dir.join(name);
            let mut f = File::create(&p).unwrap();
            writeln!(f, "#!/bin/sh\nexit 0").unwrap();
            #[cfg(unix)]
            {
                use std::os::unix::fs::PermissionsExt;
                fs::set_permissions(&p, fs::Permissions::from_mode(0o755)).unwrap();
            }
        }
    }

    #[test]
    fn find_gdal_prefers_portable_next_to_app_over_path() {
        let d = tmp();
        let app = d.join("app");
        let portable = app.join("gdal").join("bin");
        let path_dir = d.join("on-path");
        write_fake_tools(&portable);
        write_fake_tools(&path_dir);
        // Mark them so we can tell which was chosen.
        fs::write(portable.join("marker"), b"portable").unwrap();
        fs::write(path_dir.join("marker"), b"path").unwrap();
        let _guard = PATH_LOCK.lock().unwrap();
        let old = env::var_os("PATH");
        unsafe { env::set_var("PATH", &path_dir); }
        let g = find_gdal(Some(&app), &[]).expect("should find portable GDAL");
        if let Some(v) = old { unsafe { env::set_var("PATH", v); } } else { unsafe { env::remove_var("PATH"); } }
        assert_eq!(g.bin_dir, portable);
        let _ = fs::remove_dir_all(&d);
    }

    #[test]
    fn last_progress_reads_gdal_dot_output() {
        assert_eq!(last_progress(b"Input file size is 58, 69\n"), None);
        assert_eq!(last_progress(b"0...10...20...30"), Some(30));
        assert_eq!(last_progress(b"0...10...20...30...40...50...60...70...80...90...100 - done.\n"), Some(100));
        assert_eq!(last_progress(b"45%\n"), Some(45));
    }

    #[test]
    fn command_passes_paths_with_spaces_as_single_args() {
        // Regression: never join paths into a shell string.
        let args = [Path::new("/tmp/South Pasture/odm_orthophoto.tif")];
        assert_eq!(args[0].as_os_str(), OsStr::new("/tmp/South Pasture/odm_orthophoto.tif"));
    }

    fn maybe_gdal() -> Option<Gdal> {
        find_gdal(None, &[])
    }

    fn write_png_and_geotiff(gdal: &Gdal, dir: &Path, name: &str, georef: bool, with_alpha: bool) -> PathBuf {
        // A 64x64 PNG: left half opaque red, right half transparent (or black if RGB).
        let png_path = dir.join(format!("{name}.png"));
        let status = std::process::Command::new("/usr/bin/python3")
            .arg("-c")
            .arg(format!(
                "from PIL import Image\n\
                 im = Image.new('{}', (64, 64), {})\n\
                 [im.putpixel((x,y), {}) for y in range(64) for x in range(32)]\n\
                 im.save(r'{}')\n",
                if with_alpha { "RGBA" } else { "RGB" },
                if with_alpha { "(0,0,0,0)" } else { "(0,0,0)" },
                if with_alpha { "(200,40,30,255)" } else { "(200,40,30)" },
                png_path.display(),
            ))
            .status()
            .expect("python3/PIL to write a test PNG");
        assert!(status.success(), "failed to write test PNG");
        let tif = dir.join(format!("{name}.tif"));
        let mut args: Vec<std::ffi::OsString> = vec!["-of".into(), "GTiff".into()];
        if georef {
            args.extend([
                "-a_srs".into(), "EPSG:4326".into(),
                "-a_ullr".into(), "-97.5000".into(), "32.8000".into(), "-97.4990".into(), "32.7990".into(),
            ]);
        }
        if !with_alpha && georef {
            args.extend(["-a_nodata".into(), "0".into()]);
        }
        args.push(png_path.as_os_str().to_os_string());
        args.push(tif.as_os_str().to_os_string());
        let (ok, out) = run(gdal, &gdal.translate, &args, None).unwrap();
        assert!(ok, "gdal_translate fixture failed: {out}");
        tif
    }

    #[test]
    fn convert_preserves_alpha_and_source_and_opens_as_mbtiles() {
        let _guard = PATH_LOCK.lock().unwrap();
        let Some(gdal) = maybe_gdal() else {
            eprintln!("skipping GDAL integration test (no GDAL on PATH)");
            return;
        };
        let d = tmp();
        // Unicode + spaces in the folder and file name.
        let folder = d.join("Río Grande 11-8-2026");
        fs::create_dir_all(&folder).unwrap();
        let tif = write_png_and_geotiff(&gdal, &folder, "odm_orthophoto", true, true);
        // Occupied output name should bump the suffix.
        File::create(folder.join("odm_orthophoto.mbtiles")).unwrap();
        let out = unique_mbtiles_path(&tif).unwrap();
        assert_eq!(out.file_name().unwrap(), "odm_orthophoto-2.mbtiles");
        convert(&gdal, &tif, &out, |_d, _t, _m| {}).expect("convert");
        assert!(tif.is_file(), "source GeoTIFF must be left in place");
        assert!(out.is_file());
        // No leftover temps.
        let leftovers: Vec<_> = fs::read_dir(&folder).unwrap().flatten()
            .map(|e| e.file_name().to_string_lossy().into_owned())
            .filter(|n| n.contains("flyplan-") || n.ends_with(".vrt") || n.ends_with(".part"))
            .collect();
        assert!(leftovers.is_empty(), "temp files left behind: {leftovers:?}");

        let c = crate::core::open_mb(&out).expect("open mbtiles");
        let info = crate::core::mb_info(&c, out.file_name().unwrap().to_str().unwrap()).expect("mb_info");
        assert!(info.format.to_lowercase().contains("png"), "format={}", info.format);
        assert!(info.maxzoom >= info.minzoom);
        let b = info.bounds.expect("bounds");
        assert!((b[0] - -97.5).abs() < 0.01 && (b[2] + 97.499).abs() < 0.01, "bounds={b:?}");
        // At least one tile, PNG with a transparent region and an opaque region.
        let (z, x, row): (i64, i64, i64) = c
            .query_row("SELECT zoom_level, tile_column, tile_row FROM tiles ORDER BY zoom_level DESC LIMIT 1", [], |r| Ok((r.get(0)?, r.get(1)?, r.get(2)?)))
            .unwrap();
        let y = (1 << z) - 1 - row;
        let tile = crate::core::mb_tile(&c, z, x, y).expect("tile");
        assert_eq!(&tile[..8], b"\x89PNG\r\n\x1a\n");
        let tile_path = folder.join("tile.png");
        fs::write(&tile_path, &tile).unwrap();
        let py = std::process::Command::new("/usr/bin/python3")
            .arg("-c")
            .arg(format!(
                "from PIL import Image\nim=Image.open(r'{}')\nassert im.mode=='RGBA', im.mode\npx=list(im.getdata())\nassert any(p[3]==0 for p in px) and any(p[3]==255 for p in px)\n",
                tile_path.display()
            ))
            .status()
            .unwrap();
        assert!(py.success(), "tile should be RGBA with both transparent and opaque pixels");
        let _ = fs::remove_dir_all(&d);
    }

    #[test]
    fn convert_builds_alpha_from_nodata() {
        let _guard = PATH_LOCK.lock().unwrap();
        let Some(gdal) = maybe_gdal() else {
            eprintln!("skipping GDAL integration test (no GDAL on PATH)");
            return;
        };
        let d = tmp();
        let tif = write_png_and_geotiff(&gdal, &d, "nodata", true, false);
        let info = inspect(&gdal, &tif).unwrap();
        assert!(info.has_nodata && !info.has_alpha, "{info:?}");
        let out = d.join("nodata.mbtiles");
        convert(&gdal, &tif, &out, |_d, _t, _m| {}).expect("convert nodata");
        let c = crate::core::open_mb(&out).unwrap();
        let (z, x, row): (i64, i64, i64) = c
            .query_row("SELECT zoom_level, tile_column, tile_row FROM tiles ORDER BY zoom_level DESC LIMIT 1", [], |r| Ok((r.get(0)?, r.get(1)?, r.get(2)?)))
            .unwrap();
        let tile = crate::core::mb_tile(&c, z, x, (1 << z) - 1 - row).unwrap();
        fs::write(d.join("t.png"), &tile).unwrap();
        let py = std::process::Command::new("/usr/bin/python3")
            .arg("-c")
            .arg(format!(
                "from PIL import Image\nim=Image.open(r'{}')\nassert im.mode=='RGBA'\npx=list(im.getdata())\nassert any(p[3]==0 for p in px) and any(p[3]==255 for p in px)\n",
                d.join("t.png").display()
            ))
            .status()
            .unwrap();
        assert!(py.success());
        let _ = fs::remove_dir_all(&d);
    }

    #[test]
    fn convert_rejects_plain_tiff_and_leaves_no_output() {
        let _guard = PATH_LOCK.lock().unwrap();
        let Some(gdal) = maybe_gdal() else {
            eprintln!("skipping GDAL integration test (no GDAL on PATH)");
            return;
        };
        let d = tmp();
        let tif = write_png_and_geotiff(&gdal, &d, "plain", false, true);
        let out = d.join("plain.mbtiles");
        let err = convert(&gdal, &tif, &out, |_d, _t, _m| {}).unwrap_err();
        assert!(err.contains("isn’t georeferenced"), "{err}");
        assert!(!out.exists());
        let leftovers: Vec<_> = fs::read_dir(&d).unwrap().flatten()
            .map(|e| e.file_name().to_string_lossy().into_owned())
            .filter(|n| n.contains("flyplan-") || n.ends_with(".mbtiles") || n.ends_with(".vrt"))
            .collect();
        assert!(leftovers.is_empty(), "temp/output left behind: {leftovers:?}");
        let _ = fs::remove_dir_all(&d);
    }
}
