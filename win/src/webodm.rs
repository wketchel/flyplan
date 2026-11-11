//! Talk to a WebODM server (usually the one running in Docker at http://localhost:8000): sign in, list finished
//! tasks that have an orthophoto, and download a task's orthophoto.tif. Uses the curl.exe that ships with Windows
//! (10 and later), so there is no HTTP or TLS library in the app. No Windows APIs, so this can be tested on Linux.

use serde_json::{json, Value};
use std::io::Write;
use std::path::Path;
use std::process::{Command, Stdio};
use std::time::Duration;

pub const DEFAULT_URL: &str = "http://localhost:8000";

/// One finished WebODM task with an orthophoto.
#[derive(Debug, Clone, PartialEq)]
pub struct Task {
    pub project: i64,
    pub project_name: String,
    pub id: String,
    pub name: String,
    pub created: String,
    pub size_mb: Option<f64>,
}

impl Task {
    pub fn to_json(&self) -> Value {
        json!({ "project": self.project, "projectName": self.project_name, "id": self.id, "name": self.name,
                "created": self.created, "sizeMb": self.size_mb })
    }
}

#[derive(Debug, PartialEq)]
pub enum OdmError {
    /// The token was refused or has expired: sign in again.
    Auth,
    Other(String),
}

impl OdmError {
    pub fn message(&self) -> String {
        match self {
            OdmError::Auth => "WebODM asked to sign in again.".into(),
            OdmError::Other(m) => m.clone(),
        }
    }
}

/// "http://localhost:8000/" or "localhost:8000" → "http://localhost:8000".
pub fn clean_url(u: &str) -> String {
    let u = u.trim().trim_end_matches('/');
    let u = if u.is_empty() { DEFAULT_URL } else { u };
    if u.starts_with("http://") || u.starts_with("https://") { u.to_string() } else { format!("http://{u}") }
}

fn curl() -> Command {
    let mut c = Command::new(if cfg!(windows) { "curl.exe" } else { "curl" });
    #[cfg(windows)]
    {
        use std::os::windows::process::CommandExt;
        c.creation_flags(0x0800_0000); // CREATE_NO_WINDOW
    }
    c.stdin(Stdio::null()).stdout(Stdio::piped()).stderr(Stdio::piped());
    c
}

fn urlencode(s: &str) -> String {
    let mut o = String::new();
    for b in s.bytes() {
        match b {
            b'A'..=b'Z' | b'a'..=b'z' | b'0'..=b'9' | b'-' | b'_' | b'.' | b'~' => o.push(b as char),
            _ => o.push_str(&format!("%{b:02X}")),
        }
    }
    o
}

/// Runs curl and returns (HTTP status, body). `body_in` is sent on stdin (so a password never appears on a
/// command line).
fn request(args: &[String], body_in: Option<&str>) -> Result<(u16, String), OdmError> {
    let mut c = curl();
    c.args(["-sS", "--max-time", "60", "-w", "\n%{http_code}"]).args(args);
    if body_in.is_some() {
        c.stdin(Stdio::piped());
    }
    let mut child = c.spawn().map_err(|e| OdmError::Other(format!("Couldn't run curl to reach WebODM ({e}). Windows 10 and later include curl.exe.")))?;
    if let (Some(b), Some(mut si)) = (body_in, child.stdin.take()) {
        let _ = si.write_all(b.as_bytes());
    }
    let out = child.wait_with_output().map_err(|e| OdmError::Other(format!("curl didn't finish: {e}")))?;
    let text = String::from_utf8_lossy(&out.stdout).to_string();
    let (body, code) = text.rsplit_once('\n').unwrap_or(("", text.as_str()));
    let code: u16 = code.trim().parse().unwrap_or(0);
    if code == 0 {
        let err = String::from_utf8_lossy(&out.stderr).trim().to_string();
        return Err(OdmError::Other(format!("Couldn't reach WebODM. Is it running? {}", err.trim_start_matches("curl: "))));
    }
    Ok((code, body.to_string()))
}

fn auth_header(token: &str) -> String { format!("Authorization: JWT {token}") }

/// Sign in and return a token (WebODM's JWT, good for several hours).
pub fn login(url: &str, user: &str, pass: &str) -> Result<String, OdmError> {
    let body = format!("username={}&password={}", urlencode(user), urlencode(pass));
    let (code, text) = request(&[format!("{url}/api/token-auth/"), "--data-binary".into(), "@-".into(),
        "-H".into(), "Content-Type: application/x-www-form-urlencoded".into()], Some(&body))?;
    let j: Value = serde_json::from_str(&text).unwrap_or(Value::Null);
    if code == 200 {
        if let Some(t) = j["token"].as_str() { return Ok(t.to_string()); }
    }
    if code == 400 || code == 401 || code == 403 {
        return Err(OdmError::Other("WebODM didn't accept that username and password.".into()));
    }
    if code == 404 {
        return Err(OdmError::Other(format!("That address doesn't look like WebODM ({url}). It's usually http://localhost:8000.")));
    }
    Err(OdmError::Other(format!("WebODM answered with an error (HTTP {code}).")))
}

fn get_json(url: &str, token: &str) -> Result<Value, OdmError> {
    let (code, text) = request(&[url.into(), "-H".into(), auth_header(token)], None)?;
    if code == 401 || code == 403 { return Err(OdmError::Auth); }
    if code != 200 { return Err(OdmError::Other(format!("WebODM answered with an error (HTTP {code}).")));
    }
    serde_json::from_str(&text).map_err(|_| OdmError::Other("WebODM sent something FlyPlan couldn't read.".into()))
}

/// WebODM lists may be plain arrays or paginated ({"results": [...]}).
fn items(v: &Value) -> Vec<Value> {
    v.as_array().cloned().or_else(|| v["results"].as_array().cloned()).unwrap_or_default()
}

/// Tasks from one project's task list that are finished and have an orthophoto, newest first.
pub fn tasks_from_json(project: i64, project_name: &str, v: &Value) -> Vec<Task> {
    let mut out: Vec<Task> = items(v).iter().filter(|t| {
        t["status"].as_i64() == Some(40)
            && t["available_assets"].as_array().is_some_and(|a| a.iter().any(|x| x.as_str() == Some("orthophoto.tif")))
    }).map(|t| Task {
        project,
        project_name: project_name.to_string(),
        id: t["id"].as_str().map(String::from).unwrap_or_else(|| t["id"].to_string()),
        name: t["name"].as_str().filter(|s| !s.trim().is_empty()).map(String::from)
            .unwrap_or_else(|| format!("Task of {}", t["created_at"].as_str().unwrap_or("").get(..10).unwrap_or(""))),
        created: t["created_at"].as_str().unwrap_or("").to_string(),
        size_mb: t["size"].as_f64().filter(|s| *s > 0.0),
    }).collect();
    out.sort_by(|a, b| b.created.cmp(&a.created));
    out
}

/// Every finished task with an orthophoto, across all projects, newest first.
pub fn list_tasks(url: &str, token: &str) -> Result<Vec<Task>, OdmError> {
    let projects = get_json(&format!("{url}/api/projects/?ordering=-created_at"), token)?;
    let mut all = Vec::new();
    for p in items(&projects) {
        let Some(pid) = p["id"].as_i64() else { continue };
        let name = p["name"].as_str().unwrap_or("Project").to_string();
        let tasks = get_json(&format!("{url}/api/projects/{pid}/tasks/?ordering=-created_at"), token)?;
        all.extend(tasks_from_json(pid, &name, &tasks));
    }
    all.sort_by(|a, b| b.created.cmp(&a.created));
    Ok(all)
}

/// A file name for a task's orthophoto: the task name with characters Windows won't accept replaced.
pub fn file_name(task_name: &str) -> String {
    let s: String = task_name.chars().map(|c| if "<>:\"/\\|?*".contains(c) || c.is_control() { '-' } else { c }).collect();
    let s = s.trim().trim_matches('.').to_string();
    let s = if s.is_empty() { "WebODM".to_string() } else { s };
    if s.to_ascii_lowercase().contains("orthophoto") { format!("{s}.tif") } else { format!("{s} orthophoto.tif") }
}

fn content_length(url: &str, token: &str) -> Option<u64> {
    let mut c = curl();
    c.args(["-sS", "-I", "-L", "--max-time", "30", url, "-H", &auth_header(token)]);
    let out = c.output().ok()?;
    let text = String::from_utf8_lossy(&out.stdout);
    text.lines().filter_map(|l| {
        let (k, v) = l.split_once(':')?;
        if k.trim().eq_ignore_ascii_case("content-length") { v.trim().parse::<u64>().ok() } else { None }
    }).last()
}

/// Download a task's orthophoto.tif to `dest`. Written to `dest.part` first and renamed when complete, so a
/// broken download never looks like a finished file. `progress(bytes so far, total bytes or 0)`.
pub fn download(url: &str, token: &str, project: i64, task: &str, dest: &Path, mut progress: impl FnMut(u64, u64)) -> Result<(), OdmError> {
    let src = format!("{url}/api/projects/{project}/tasks/{task}/download/orthophoto.tif");
    let total = content_length(&src, token).unwrap_or(0);
    let part = dest.with_extension("tif.part");
    let _ = std::fs::remove_file(&part);
    let mut c = curl();
    c.args(["-sS", "-L", "-o"]).arg(&part).args(["-w", "%{http_code}", &src, "-H", &auth_header(token)]);
    let mut child = c.spawn().map_err(|e| OdmError::Other(format!("Couldn't run curl to download from WebODM ({e}).")))?;
    loop {
        match child.try_wait() {
            Ok(Some(_)) => break,
            Ok(None) => {
                progress(std::fs::metadata(&part).map(|m| m.len()).unwrap_or(0), total);
                std::thread::sleep(Duration::from_millis(500));
            }
            Err(e) => { let _ = std::fs::remove_file(&part); return Err(OdmError::Other(format!("The download stopped: {e}"))); }
        }
    }
    let out = child.wait_with_output().map_err(|e| OdmError::Other(format!("The download stopped: {e}")))?;
    let code: u16 = String::from_utf8_lossy(&out.stdout).trim().parse().unwrap_or(0);
    if code == 401 || code == 403 {
        let _ = std::fs::remove_file(&part);
        return Err(OdmError::Auth);
    }
    if code != 200 {
        let _ = std::fs::remove_file(&part);
        let err = String::from_utf8_lossy(&out.stderr).trim().to_string();
        return Err(OdmError::Other(if code == 0 { format!("The download from WebODM failed. {err}") } else { format!("WebODM couldn't send the orthophoto (HTTP {code}).") }));
    }
    let got = std::fs::metadata(&part).map(|m| m.len()).unwrap_or(0);
    if got == 0 || (total > 0 && got != total) {
        let _ = std::fs::remove_file(&part);
        return Err(OdmError::Other("The download from WebODM was incomplete. Try again.".into()));
    }
    progress(got, total.max(got));
    std::fs::rename(&part, dest).map_err(|e| { let _ = std::fs::remove_file(&part); OdmError::Other(format!("Couldn't save the orthophoto as {}: {e}", dest.display())) })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cleans_addresses() {
        assert_eq!(clean_url(""), "http://localhost:8000");
        assert_eq!(clean_url("localhost:8000/"), "http://localhost:8000");
        assert_eq!(clean_url(" https://odm.example.com/ "), "https://odm.example.com");
    }

    #[test]
    fn picks_finished_tasks_with_orthophotos_newest_first() {
        let v = json!([
            { "id": "a1", "name": "North Side", "status": 40, "available_assets": ["orthophoto.tif", "all.zip"], "created_at": "2026-10-27T10:00:00Z", "size": 812.5 },
            { "id": "b2", "name": "Running", "status": 20, "available_assets": [], "created_at": "2026-11-10T10:00:00Z" },
            { "id": "c3", "name": "No ortho", "status": 40, "available_assets": ["georeferenced_model.laz"], "created_at": "2026-11-01T10:00:00Z" },
            { "id": "d4", "name": "", "status": 40, "available_assets": ["orthophoto.tif"], "created_at": "2026-11-09T13:15:47Z" }
        ]);
        let t = tasks_from_json(7, "Ranch", &v);
        assert_eq!(t.iter().map(|x| x.id.as_str()).collect::<Vec<_>>(), vec!["d4", "a1"]);
        assert_eq!(t[0].name, "Task of 2026-11-09");
        assert_eq!(t[1].size_mb, Some(812.5));
        assert_eq!(tasks_from_json(7, "Ranch", &json!({ "results": v })).len(), 2);
    }

    #[test]
    fn safe_file_names() {
        assert_eq!(file_name("South Pasture 11/8/2026"), "South Pasture 11-8-2026 orthophoto.tif");
        assert_eq!(file_name("Fleming-10-27-2026-orthophoto"), "Fleming-10-27-2026-orthophoto.tif");
        assert_eq!(file_name("  "), "WebODM orthophoto.tif");
    }

    #[test]
    fn urlencodes_passwords() {
        assert_eq!(urlencode("p@ss w&rd=1"), "p%40ss%20w%26rd%3D1");
    }

    /// Against a tiny fake WebODM (python3 http.server) when curl and python3 are available.
    #[test]
    fn talks_to_a_fake_webodm() {
        if Command::new("curl").arg("--version").output().is_err() || Command::new("python3").arg("--version").output().is_err() { return; }
        let dir = std::env::temp_dir().join(format!("flyplan-odm-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        let server = dir.join("fake.py");
        std::fs::write(&server, r#"
import http.server, json, sys, urllib.parse
DATA = b'II*\x00' + b'x' * 300000
class H(http.server.BaseHTTPRequestHandler):
    def log_message(self, *a): pass
    def send(self, code, body, ctype='application/json'):
        self.send_response(code); self.send_header('Content-Type', ctype); self.send_header('Content-Length', str(len(body))); self.end_headers()
        if self.command != 'HEAD': self.wfile.write(body)
    def ok(self): return self.headers.get('Authorization') == 'JWT tok123'
    def do_POST(self):
        n = int(self.headers.get('Content-Length', 0)); q = urllib.parse.parse_qs(self.rfile.read(n).decode())
        if self.path == '/api/token-auth/' and q.get('username') == ['bill'] and q.get('password') == ['p@ss w&rd']: self.send(200, b'{"token":"tok123"}')
        else: self.send(400, b'{"non_field_errors":["bad"]}')
    def do_HEAD(self): self.do_GET()
    def do_GET(self):
        if not self.ok(): return self.send(403, b'{"detail":"no"}')
        if self.path.startswith('/api/projects/?'): return self.send(200, json.dumps([{"id": 3, "name": "Ranch"}]).encode())
        if self.path.startswith('/api/projects/3/tasks/?'): return self.send(200, json.dumps([{"id": "t-9", "name": "South Pasture", "status": 40, "available_assets": ["orthophoto.tif"], "created_at": "2026-11-09T13:15:47Z"}]).encode())
        if self.path == '/api/projects/3/tasks/t-9/download/orthophoto.tif': return self.send(200, DATA, 'image/tiff')
        self.send(404, b'{}')
s = http.server.ThreadingHTTPServer(('127.0.0.1', 0), H); print(s.server_address[1], flush=True); s.serve_forever()
"#).unwrap();
        let mut child = Command::new("python3").arg(&server).stdout(Stdio::piped()).spawn().unwrap();
        let mut line = String::new();
        std::io::BufRead::read_line(&mut std::io::BufReader::new(child.stdout.take().unwrap()), &mut line).unwrap();
        let url = format!("http://127.0.0.1:{}", line.trim());
        assert!(matches!(login(&url, "bill", "wrong"), Err(OdmError::Other(_))));
        let tok = login(&url, "bill", "p@ss w&rd").unwrap();
        assert_eq!(tok, "tok123");
        assert_eq!(list_tasks(&url, "expired"), Err(OdmError::Auth));
        let t = list_tasks(&url, &tok).unwrap();
        assert_eq!((t.len(), t[0].project_name.as_str(), t[0].id.as_str()), (1, "Ranch", "t-9"));
        let dest = dir.join("South Pasture orthophoto.tif");
        let mut last = (0, 0);
        download(&url, &tok, 3, "t-9", &dest, |a, b| last = (a, b)).unwrap();
        assert_eq!(std::fs::metadata(&dest).unwrap().len(), 300004);
        assert_eq!(last, (300004, 300004));
        assert!(!dest.with_extension("tif.part").exists());
        assert!(download(&url, &tok, 3, "nope", &dir.join("x.tif"), |_, _| {}).is_err());
        assert!(!dir.join("x.tif").exists() && !dir.join("x.tif.part").exists());
        let _ = child.kill();
        let _ = std::fs::remove_dir_all(&dir);
    }
}
