//! FlyPlan for Windows: the same planner as the tablet, in a window, with USB sending to the RC Pro 2.
#![cfg_attr(windows, windows_subsystem = "windows")]

mod core;
#[cfg(windows)]
mod wpd;

#[cfg(not(windows))]
fn main() { eprintln!("This program runs on Windows."); }

#[cfg(windows)]
fn main() { app::run(); }

#[cfg(windows)]
mod app {
    use crate::core::{self, Settings};
    use crate::wpd;
    use raw_window_handle::{DisplayHandle, HandleError, HasDisplayHandle, HasWindowHandle, RawDisplayHandle, RawWindowHandle, Win32WindowHandle, WindowHandle, WindowsDisplayHandle};
    use rusqlite::Connection;
    use serde_json::{json, Value};
    use std::borrow::Cow;
    use std::collections::HashMap;
    use std::num::NonZeroIsize;
    use std::path::{Path, PathBuf};
    use std::sync::{Arc, Mutex};
    use tao::event::{Event, WindowEvent};
    use tao::event_loop::{ControlFlow, EventLoopBuilder, EventLoopProxy};
    use tao::platform::windows::WindowExtWindows;
    use tao::window::WindowBuilder;
    use wry::http::{header::CONTENT_TYPE, Request, Response, StatusCode};
    use wry::{WebContext, WebViewBuilder, WebViewBuilderExtWindows};

    const INDEX: &str = include_str!("../../assets/index.html");
    const APP_JS: &str = include_str!("../../assets/app.js");
    const APP_CSS: &str = include_str!("../../assets/app.css");
    const LEAFLET_JS: &str = include_str!("../../assets/leaflet.js");
    const LEAFLET_CSS: &str = include_str!("../../assets/leaflet.css");
    const HOST_JS: &str = include_str!("host.js");

    enum Ev { Emit(String) }

    #[derive(Clone)]
    struct Ctx {
        proxy: EventLoopProxy<Ev>,
        settings: Arc<Mutex<Settings>>,
        maps: Arc<Mutex<HashMap<String, Connection>>>,
        hwnd: isize,
        backups: PathBuf,
    }

    impl Ctx {
        fn emit(&self, v: Value) {
            let s = serde_json::to_string(&v.to_string()).unwrap();
            let _ = self.proxy.send_event(Ev::Emit(format!("window.onNative && window.onNative({s})")));
        }
        fn js(&self, code: String) { let _ = self.proxy.send_event(Ev::Emit(code)); }
        fn toast(&self, m: &str) { self.emit(json!({ "type": "toast", "message": m })); }
        fn dialog(&self) -> rfd::FileDialog { rfd::FileDialog::new().set_parent(&Parent(self.hwnd)) }
    }

    struct Parent(isize);
    impl HasDisplayHandle for Parent {
        fn display_handle(&self) -> Result<DisplayHandle<'_>, HandleError> {
            Ok(unsafe { DisplayHandle::borrow_raw(RawDisplayHandle::Windows(WindowsDisplayHandle::new())) })
        }
    }
    impl HasWindowHandle for Parent {
        fn window_handle(&self) -> Result<WindowHandle<'_>, HandleError> {
            let h = Win32WindowHandle::new(NonZeroIsize::new(self.0).ok_or(HandleError::Unavailable)?);
            Ok(unsafe { WindowHandle::borrow_raw(RawWindowHandle::Win32(h)) })
        }
    }

    fn local_dir() -> PathBuf {
        std::env::var_os("LOCALAPPDATA").map(PathBuf::from).unwrap_or_else(std::env::temp_dir).join("Waypoint Planner") // data folder keeps its original name so existing settings carry over
    }
    fn documents() -> PathBuf {
        std::env::var_os("USERPROFILE").map(|p| PathBuf::from(p).join("Documents")).unwrap_or_else(std::env::temp_dir)
    }
    fn drive_folder() -> Option<PathBuf> {
        for d in ["G:\\My Drive", "H:\\My Drive", "I:\\My Drive"] { if Path::new(d).is_dir() { return Some(PathBuf::from(d)); } }
        None
    }
    fn file_name(p: &Path) -> String { p.file_name().map(|f| f.to_string_lossy().into_owned()).unwrap_or_default() }
    fn arg(a: &Value, i: usize) -> String { a.get(i).and_then(|v| v.as_str()).unwrap_or("").to_string() }

    pub fn run() {
        let dir = local_dir();
        let _ = std::fs::create_dir_all(&dir);
        let settings = Settings::load(&dir);
        let sync_name = settings.sync_path().map(|p| file_name(&p)).unwrap_or_default();

        let event_loop = EventLoopBuilder::<Ev>::with_user_event().build();
        let window = WindowBuilder::new()
            .with_title("FlyPlan: Mapping & Waypoints")
            .with_inner_size(tao::dpi::LogicalSize::new(1400.0, 900.0))
            .with_maximized(true)
            .with_window_icon(tao::window::Icon::from_rgba(include_bytes!("../icon64.rgba").to_vec(), 64, 64).ok())
            .build(&event_loop)
            .expect("window");

        let ctx = Ctx {
            proxy: event_loop.create_proxy(),
            settings: Arc::new(Mutex::new(settings)),
            maps: Arc::new(Mutex::new(HashMap::new())),
            hwnd: window.hwnd() as isize,
            backups: documents().join("FlyPlan").join("Backups"),
        };

        let init = format!("window.__hostState = {};\n{}", json!({ "syncName": sync_name, "device": "PC" }), HOST_JS);
        let mut web_context = WebContext::new(Some(dir.join("WebView2")));
        let proto_ctx = ctx.clone();
        let ipc_ctx = ctx.clone();
        let webview = WebViewBuilder::with_web_context(&mut web_context)
            .with_https_scheme(true)
            .with_background_color((11, 58, 74, 255))
            .with_initialization_script(&init)
            .with_custom_protocol("wp".into(), move |_id, req| serve(&proto_ctx, req))
            .with_ipc_handler(move |req: Request<String>| handle(&ipc_ctx, req.body()))
            .with_url("https://wp.localhost/")
            .build(&window);
        let webview = match webview {
            Ok(w) => w,
            Err(e) => {
                rfd::MessageDialog::new().set_title("FlyPlan")
                    .set_description(format!("Couldn't start the map view. Install the Microsoft Edge WebView2 Runtime from microsoft.com and try again.\n\n{e}"))
                    .show();
                return;
            }
        };

        event_loop.run(move |event, _, flow| {
            *flow = ControlFlow::Wait;
            match event {
                Event::UserEvent(Ev::Emit(js)) => { let _ = webview.evaluate_script(&js); }
                Event::WindowEvent { event: WindowEvent::CloseRequested, .. } => *flow = ControlFlow::Exit,
                Event::WindowEvent { event: WindowEvent::Focused(true), .. } => { let _ = webview.evaluate_script("window.onNative && window.onNative('{\"type\":\"resume\"}')"); }
                _ => {}
            }
        });
    }

    /* ---------- the app's files and map tiles ---------- */
    fn serve(ctx: &Ctx, req: Request<Vec<u8>>) -> Response<Cow<'static, [u8]>> {
        let path = req.uri().path().to_string();
        let ok = |ct: &str, body: Cow<'static, [u8]>| Response::builder().header(CONTENT_TYPE, ct).header("Cache-Control", "no-cache").body(body).unwrap();
        let file = |s: &'static str| Cow::Borrowed(s.as_bytes());
        match path.as_str() {
            "/" | "/index.html" => ok("text/html; charset=utf-8", file(INDEX)),
            "/app.js" => ok("text/javascript; charset=utf-8", file(APP_JS)),
            "/app.css" => ok("text/css; charset=utf-8", file(APP_CSS)),
            "/leaflet.js" => ok("text/javascript; charset=utf-8", file(LEAFLET_JS)),
            "/leaflet.css" => ok("text/css; charset=utf-8", file(LEAFLET_CSS)),
            p if p.starts_with("/mbtiles/") => {
                let parts: Vec<&str> = p.trim_start_matches("/mbtiles/").split('/').collect();
                let tile = (|| -> Option<Vec<u8>> {
                    if parts.len() != 4 { return None; }
                    let (z, x, y) = (parts[1].parse().ok()?, parts[2].parse().ok()?, parts[3].trim_end_matches(".png").parse().ok()?);
                    let mut maps = ctx.maps.lock().ok()?;
                    if !maps.contains_key(parts[0]) {
                        let path = ctx.settings.lock().ok()?.maps().into_iter().find(|m| m["id"].as_str() == Some(parts[0]))?["path"].as_str()?.to_string();
                        maps.insert(parts[0].to_string(), core::open_mb(Path::new(&path)).ok()?);
                    }
                    core::mb_tile(maps.get(parts[0])?, z, x, y)
                })();
                match tile {
                    Some(d) => Response::builder().header(CONTENT_TYPE, core::mime_of(&d)).header("Cache-Control", "max-age=3600").body(Cow::Owned(d)).unwrap(),
                    None => Response::builder().status(StatusCode::NOT_FOUND).body(Cow::Borrowed(&b""[..])).unwrap(),
                }
            }
            _ => Response::builder().status(StatusCode::NOT_FOUND).body(Cow::Borrowed(&b""[..])).unwrap(),
        }
    }

    /* ---------- requests from the app ---------- */
    fn handle(ctx: &Ctx, body: &str) {
        let Ok(msg) = serde_json::from_str::<Value>(body) else { return };
        let cmd = msg["cmd"].as_str().unwrap_or("").to_string();
        let a = msg["args"].clone();
        let c = ctx.clone();
        // Everything that waits on a dialog, a file or the USB device runs off the window's thread.
        std::thread::spawn(move || run_cmd(&c, &cmd, &a));
    }

    fn run_cmd(c: &Ctx, cmd: &str, a: &Value) {
        match cmd {
            "saveKmz" => {
                let name = format!("{}.kmz", arg(a, 0).trim_end_matches(".kmz"));
                let Some(p) = c.dialog().set_title("Save KMZ").set_file_name(&name).add_filter("DJI mission (KMZ)", &["kmz"]).save_file() else { return };
                match core::build_kmz(&arg(a, 1), &arg(a, 2), None).and_then(|d| std::fs::write(&p, d).map_err(|e| e.to_string())) {
                    Ok(_) => c.toast(&format!("Saved {}", file_name(&p))),
                    Err(e) => c.toast(&format!("Could not save: {e}")),
                }
            }
            "saveText" => {
                let name = arg(a, 0);
                let ext = Path::new(&name).extension().map(|e| e.to_string_lossy().into_owned()).unwrap_or_else(|| "txt".into());
                let Some(p) = c.dialog().set_title("Save").set_file_name(&name).add_filter(&ext.to_uppercase(), &[ext.as_str()]).save_file() else { return };
                match std::fs::write(&p, arg(a, 2)) { Ok(_) => c.toast(&format!("Saved {}", file_name(&p))), Err(e) => c.toast(&format!("Could not save: {e}")) }
            }
            "importFor" => {
                let purpose = arg(a, 0);
                let Some(p) = c.dialog().set_title("Import").add_filter("Maps and missions", &["kmz", "kml", "gpx", "wpml"]).add_filter("All files", &["*"]).pick_file() else { return };
                match std::fs::read(&p).map_err(|e| e.to_string()).and_then(|b| core::parse_import(&file_name(&p), &b, &purpose)) {
                    Ok(v) => c.emit(v),
                    Err(e) => c.toast(&format!("Could not import: {e}")),
                }
            }
            "syncPick" => {
                let create = a.get(0).and_then(|v| v.as_bool()).unwrap_or(false);
                let mut d = c.dialog().add_filter("FlyPlan sync file", &["json"]);
                if let Some(g) = drive_folder() { d = d.set_directory(g); }
                let picked = if create { d.set_title("Create sync file").set_file_name("FlyPlan sync.json").save_file() } else { d.set_title("Choose the sync file").pick_file() };
                let Some(p) = picked else { c.emit(json!({ "type": "syncError", "code": "cancel" })); return };
                if create && !p.exists() { if let Err(e) = std::fs::write(&p, b"") { c.emit(json!({ "type": "syncError", "code": "io", "message": format!("Couldn't create the file: {e}") })); return; } }
                let name = file_name(&p);
                { let mut s = c.settings.lock().unwrap(); s.v["syncPath"] = json!(p.to_string_lossy()); s.save(); }
                c.js(format!("window.__hostSet('syncName', {})", json!(name)));
                c.emit(json!({ "type": "syncPicked", "name": name }));
            }
            "syncRead" => {
                let Some(p) = c.settings.lock().unwrap().sync_path() else { c.emit(json!({ "type": "syncError", "code": "none", "message": "No sync file is set." })); return };
                match std::fs::read(&p) {
                    Ok(b) => c.emit(json!({ "type": "syncData", "text": String::from_utf8_lossy(&b) })),
                    Err(e) if e.kind() == std::io::ErrorKind::NotFound => c.emit(json!({ "type": "syncError", "code": "missing",
                        "message": format!("Couldn't find the sync file at {}. If Google Drive isn't running, start it; if the file moved, click Choose a different file.", p.display()) })),
                    Err(e) => c.emit(json!({ "type": "syncError", "code": "io", "message": format!("Couldn't read the sync file: {e}") })),
                }
            }
            "syncWrite" => {
                let Some(p) = c.settings.lock().unwrap().sync_path() else { return };
                match std::fs::write(&p, arg(a, 0)) {
                    Ok(_) => c.emit(json!({ "type": "syncWritten" })),
                    Err(e) => c.emit(json!({ "type": "syncError", "code": "io", "message": format!("Couldn't save the sync file: {e}") })),
                }
            }
            "openUrl" => {
                let u = arg(a, 0);
                if u.starts_with("https://") || u.starts_with("http://") {
                    use std::os::windows::process::CommandExt;
                    let _ = std::process::Command::new("rundll32").args(["url.dll,FileProtocolHandler", &u]).creation_flags(0x0800_0000).spawn();
                }
            }
            "syncForget" => { let mut s = c.settings.lock().unwrap(); s.v["syncPath"] = json!(""); s.save(); }
            "importMbtiles" => {
                let Some(p) = c.dialog().set_title("Add a map").add_filter("MBTiles map", &["mbtiles"]).pick_file() else { return };
                c.emit(json!({ "type": "mbProgress", "done": 0, "total": 0 }));
                let res = { let mut s = c.settings.lock().unwrap(); core::add_map(&mut s, &p) };
                match res {
                    Ok(id) => { let s = c.settings.lock().unwrap(); c.emit(core::maps_message(&s, Some(&id))); }
                    Err(e) => c.emit(json!({ "type": "mbError", "message": format!("Couldn't open that map: {e}") })),
                }
            }
            "listMaps" => { let s = c.settings.lock().unwrap(); c.emit(core::maps_message(&s, None)); }
            "deleteMap" => {
                let id = arg(a, 0);
                c.maps.lock().unwrap().remove(&id);
                let mut s = c.settings.lock().unwrap();
                let m: Vec<Value> = s.maps().into_iter().filter(|m| m["id"].as_str() != Some(&id)).collect();
                s.set_maps(m); s.save();
                c.emit(core::maps_message(&s, None));
            }
            "findController" => match wpd::list(0) {
                Ok(v) => c.emit(v),
                Err(e) => c.emit(json!({ "type": "error", "message": e })),
            },
            "sendMission" => match wpd::replace(&arg(a, 0), &arg(a, 2), &arg(a, 3), &c.backups, core::b64decode(&arg(a, 4))) {
                Ok(v) => c.emit(v),
                Err(e) => c.emit(json!({ "type": "error", "message": e })),
            },
            _ => {}
        }
    }
}
