//! Talks to the DJI controller over USB through Windows Portable Devices (the same system File Explorer uses for
//! phones), and swaps a DJI Fly waypoint mission file. Mirrors the tablet's MtpTransfer.java.

use crate::core;
use serde_json::{json, Value};
use std::path::Path;
use windows::core::{GUID, HSTRING, PCWSTR, PWSTR};
use windows::Win32::Devices::PortableDevices::*;
use windows::Win32::System::Com::*;
use windows::Win32::UI::Shell::PropertiesSystem::PROPERTYKEY;

const PATH: [&str; 5] = ["Android", "data", "dji.go.v5", "files", "waypoint"];

pub struct Com;
impl Com {
    pub fn init() -> Com { unsafe { let _ = CoInitializeEx(None, COINIT_MULTITHREADED); } Com }
}
impl Drop for Com { fn drop(&mut self) { unsafe { CoUninitialize() } } }

fn take(p: PWSTR) -> String {
    if p.is_null() { return String::new(); }
    let s = unsafe { p.to_string().unwrap_or_default() };
    unsafe { CoTaskMemFree(Some(p.0 as _)) };
    s
}
fn e(msg: &str, err: windows::core::Error) -> String { format!("{msg} ({})", err.message()) }

struct DeviceId { id: Vec<u16>, name: String, maker: String }

fn devices() -> Result<Vec<DeviceId>, String> {
    unsafe {
        let mgr: IPortableDeviceManager = CoCreateInstance(&PortableDeviceManager, None, CLSCTX_INPROC_SERVER)
            .map_err(|x| e("Windows Portable Devices isn't available", x))?;
        let _ = mgr.RefreshDeviceList();
        let mut n = 0u32;
        mgr.GetDevices(std::ptr::null_mut(), &mut n).map_err(|x| e("Couldn't list USB devices", x))?;
        if n == 0 { return Ok(vec![]); }
        let mut ids = vec![PWSTR::null(); n as usize];
        mgr.GetDevices(ids.as_mut_ptr(), &mut n).map_err(|x| e("Couldn't list USB devices", x))?;
        let mut out = vec![];
        for p in ids.into_iter().take(n as usize) {
            let wide: Vec<u16> = { let mut v = p.as_wide().to_vec(); v.push(0); v };
            let get = |f: &dyn Fn(PCWSTR, PWSTR, *mut u32) -> windows::core::Result<()>| -> String {
                let mut len = 0u32;
                let _ = f(PCWSTR(wide.as_ptr()), PWSTR::null(), &mut len);
                if len == 0 { return String::new(); }
                let mut buf = vec![0u16; len as usize + 1];
                if f(PCWSTR(wide.as_ptr()), PWSTR(buf.as_mut_ptr()), &mut len).is_err() { return String::new(); }
                String::from_utf16_lossy(&buf[..buf.iter().position(|&c| c == 0).unwrap_or(buf.len())])
            };
            let name = get(&|a, b, c| mgr.GetDeviceFriendlyName(a, b, c));
            let maker = get(&|a, b, c| mgr.GetDeviceManufacturer(a, b, c));
            CoTaskMemFree(Some(p.0 as _));
            out.push(DeviceId { id: wide, name, maker });
        }
        Ok(out)
    }
}

pub struct Device { dev: IPortableDevice, content: IPortableDeviceContent, props: IPortableDeviceProperties, keys: IPortableDeviceKeyCollection, pub label: String }

impl Drop for Device { fn drop(&mut self) { unsafe { let _ = self.dev.Close(); } } }

struct Obj { id: String, name: String, folder: bool, modified: Option<f64>, size: u64 }

impl Device {
    fn open(d: &DeviceId) -> Result<Device, String> {
        unsafe {
            let client: IPortableDeviceValues = CoCreateInstance(&PortableDeviceValues, None, CLSCTX_INPROC_SERVER).map_err(|x| e("WPD", x))?;
            let _ = client.SetStringValue(&WPD_CLIENT_NAME, &HSTRING::from("FlyPlan"));
            let _ = client.SetUnsignedIntegerValue(&WPD_CLIENT_MAJOR_VERSION, 1);
            let _ = client.SetUnsignedIntegerValue(&WPD_CLIENT_MINOR_VERSION, 25);
            let _ = client.SetUnsignedIntegerValue(&WPD_CLIENT_REVISION, 0);
            let _ = client.SetUnsignedIntegerValue(&WPD_CLIENT_SECURITY_QUALITY_OF_SERVICE, 0x0002_0000); // SECURITY_IMPERSONATION
            let dev: IPortableDevice = CoCreateInstance(&PortableDeviceFTM, None, CLSCTX_INPROC_SERVER).map_err(|x| e("WPD", x))?;
            dev.Open(PCWSTR(d.id.as_ptr()), &client).map_err(|x| e(&format!("Couldn't open {}", d.name), x))?;
            let content = dev.Content().map_err(|x| e("Couldn't read the controller", x))?;
            let props = content.Properties().map_err(|x| e("Couldn't read the controller", x))?;
            let keys: IPortableDeviceKeyCollection = CoCreateInstance(&PortableDeviceKeyCollection, None, CLSCTX_INPROC_SERVER).map_err(|x| e("WPD", x))?;
            for k in [WPD_OBJECT_NAME, WPD_OBJECT_ORIGINAL_FILE_NAME, WPD_OBJECT_CONTENT_TYPE, WPD_OBJECT_DATE_MODIFIED, WPD_OBJECT_SIZE] { let _ = keys.Add(&k); }
            Ok(Device { dev, content, props, keys, label: if d.name.is_empty() { "the controller".into() } else { d.name.clone() } })
        }
    }

    fn info(&self, id: &str) -> Obj {
        let w = HSTRING::from(id);
        let mut o = Obj { id: id.to_string(), name: String::new(), folder: false, modified: None, size: 0 };
        unsafe {
            if let Ok(v) = self.props.GetValues(&w, &self.keys) {
                o.name = v.GetStringValue(&WPD_OBJECT_ORIGINAL_FILE_NAME).map(take).ok().filter(|s| !s.is_empty())
                    .or_else(|| v.GetStringValue(&WPD_OBJECT_NAME).map(take).ok()).unwrap_or_default();
                if let Ok(t) = v.GetGuidValue(&WPD_OBJECT_CONTENT_TYPE) { o.folder = t == WPD_CONTENT_TYPE_FOLDER || t == WPD_CONTENT_TYPE_FUNCTIONAL_OBJECT; }
                if let Ok(p) = v.GetValue(&WPD_OBJECT_DATE_MODIFIED) { o.modified = f64::try_from(&p).ok(); }
                o.size = v.GetUnsignedLargeIntegerValue(&WPD_OBJECT_SIZE).unwrap_or(0);
            }
        }
        o
    }

    fn children(&self, parent: &str) -> Vec<Obj> {
        let mut out = vec![];
        unsafe {
            let Ok(en) = self.content.EnumObjects(0, &HSTRING::from(parent), None) else { return out };
            loop {
                let mut ids = [PWSTR::null(); 64];
                let mut got = 0u32;
                let hr = en.Next(&mut ids, &mut got);
                for p in ids.iter().take(got as usize) { let id = take(*p); out.push(self.info(&id)); }
                if hr.is_err() || got == 0 || hr.0 == 1 { break; }
            }
        }
        out
    }

    fn child(&self, parent: &str, name: &str) -> Option<Obj> {
        self.children(parent).into_iter().find(|o| o.name.eq_ignore_ascii_case(name))
    }

    /// DJI Fly's waypoint folder, searching every storage on the device.
    fn waypoint_folder(&self) -> Option<String> {
        for st in self.children("DEVICE") {
            let mut cur = st.id.clone();
            let mut ok = true;
            for part in PATH { match self.child(&cur, part) { Some(o) => cur = o.id, None => { ok = false; break; } } }
            if ok { return Some(cur); }
        }
        None
    }

    fn read(&self, id: &str) -> Result<Vec<u8>, String> {
        unsafe {
            let res = self.content.Transfer().map_err(|x| e("Couldn't read from the controller", x))?;
            let mut opt = 0u32;
            let mut st: Option<IStream> = None;
            res.GetStream(&HSTRING::from(id), &WPD_RESOURCE_DEFAULT, STGM_READ.0, &mut opt, &mut st).map_err(|x| e("Couldn't read the placeholder mission", x))?;
            let st = st.ok_or("Couldn't read the placeholder mission")?;
            let mut out = Vec::new();
            let mut buf = vec![0u8; (opt.max(65536)) as usize];
            loop {
                let mut n = 0u32;
                let hr = st.Read(buf.as_mut_ptr() as _, buf.len() as u32, Some(&mut n));
                if n > 0 { out.extend_from_slice(&buf[..n as usize]); }
                if hr.is_err() { return Err(format!("Couldn't read the placeholder mission ({})", hr.message())); }
                if n == 0 { break; }
            }
            Ok(out)
        }
    }

    fn delete(&self, id: &str) -> Result<(), String> {
        // A PROPVARIANT holding a VT_LPWSTR object ID; the collection copies it.
        #[repr(C)]
        struct PvStr { vt: u16, r1: u16, r2: u16, r3: u16, p: *const u16, pad: usize }
        let w: Vec<u16> = id.encode_utf16().chain(Some(0)).collect();
        let pv = PvStr { vt: 31, r1: 0, r2: 0, r3: 0, p: w.as_ptr(), pad: 0 };
        unsafe {
            let coll: IPortableDevicePropVariantCollection = CoCreateInstance(&PortableDevicePropVariantCollection, None, CLSCTX_INPROC_SERVER).map_err(|x| e("WPD", x))?;
            coll.Add(&pv as *const PvStr as *const windows::core::PROPVARIANT).map_err(|x| e("WPD", x))?;
            self.content.Delete(PORTABLE_DEVICE_DELETE_NO_RECURSION.0 as u32, &coll, std::ptr::null_mut())
                .map_err(|x| e("The controller would not let the old mission be replaced. Nothing was changed", x))
        }
    }

    fn upload(&self, parent: &str, name: &str, data: &[u8]) -> Result<(), String> {
        unsafe {
            let v: IPortableDeviceValues = CoCreateInstance(&PortableDeviceValues, None, CLSCTX_INPROC_SERVER).map_err(|x| e("WPD", x))?;
            let set = |k: &PROPERTYKEY, s: &str| v.SetStringValue(k, &HSTRING::from(s));
            set(&WPD_OBJECT_PARENT_ID, parent).map_err(|x| e("WPD", x))?;
            set(&WPD_OBJECT_ORIGINAL_FILE_NAME, name).map_err(|x| e("WPD", x))?;
            set(&WPD_OBJECT_NAME, name).map_err(|x| e("WPD", x))?;
            v.SetUnsignedLargeIntegerValue(&WPD_OBJECT_SIZE, data.len() as u64).map_err(|x| e("WPD", x))?;
            let g: GUID = WPD_CONTENT_TYPE_GENERIC_FILE;
            let f: GUID = WPD_OBJECT_FORMAT_UNSPECIFIED;
            let _ = v.SetGuidValue(&WPD_OBJECT_CONTENT_TYPE, &g);
            let _ = v.SetGuidValue(&WPD_OBJECT_FORMAT, &f);
            let mut st: Option<IStream> = None;
            let mut opt = 0u32;
            let mut cookie = PWSTR::null();
            self.content.CreateObjectWithPropertiesAndData(&v, &mut st, &mut opt, &mut cookie).map_err(|x| e("controller refused the new file", x))?;
            take(cookie);
            let st = st.ok_or("controller refused the new file")?;
            let chunk = opt.clamp(4096, 1 << 20) as usize;
            for part in data.chunks(chunk) {
                let mut done = 0u32;
                let hr = st.Write(part.as_ptr() as _, part.len() as u32, Some(&mut done));
                if hr.is_err() || done as usize != part.len() { return Err("file transfer did not complete".into()); }
            }
            st.Commit(STGC_DEFAULT).map_err(|x| e("file transfer did not complete", x))
        }
    }

    fn make_folder(&self, parent: &str, name: &str) -> Result<String, String> {
        unsafe {
            let v: IPortableDeviceValues = CoCreateInstance(&PortableDeviceValues, None, CLSCTX_INPROC_SERVER).map_err(|x| e("WPD", x))?;
            v.SetStringValue(&WPD_OBJECT_PARENT_ID, &HSTRING::from(parent)).map_err(|x| e("WPD", x))?;
            v.SetStringValue(&WPD_OBJECT_NAME, &HSTRING::from(name)).map_err(|x| e("WPD", x))?;
            let _ = v.SetStringValue(&WPD_OBJECT_ORIGINAL_FILE_NAME, &HSTRING::from(name));
            let g: GUID = WPD_CONTENT_TYPE_FOLDER;
            let _ = v.SetGuidValue(&WPD_OBJECT_CONTENT_TYPE, &g);
            let mut id = PWSTR::null();
            self.content.CreateObjectWithPropertiesOnly(&v, &mut id).map_err(|x| e("couldn't create the picture folder", x))?;
            Ok(take(id))
        }
    }

    /// Replaces DJI Fly's map picture for the mission (waypoint/map_preview/<id>/<id>.jpg).
    fn write_preview(&self, wp: &str, folder: &str, jpg: &[u8]) -> Result<(), String> {
        let mp = match self.child(wp, "map_preview") { Some(o) => o.id, None => self.make_folder(wp, "map_preview")? };
        let dir = match self.child(&mp, folder) { Some(o) => o.id, None => self.make_folder(&mp, folder)? };
        let name = format!("{folder}.jpg");
        if let Some(old) = self.child(&dir, &name) { self.delete(&old.id)?; }
        self.upload(&dir, &name, jpg)
    }

    fn missions(&self, wp: &str) -> Vec<(Obj, Obj)> {
        let mut out: Vec<(Obj, Obj)> = self.children(wp).into_iter().filter(|f| f.folder).filter_map(|f| {
            let kmz = self.children(&f.id).into_iter().find(|o| !o.folder && o.name.to_lowercase().ends_with(".kmz"))?;
            Some((f, kmz))
        }).collect();
        out.sort_by(|a, b| b.1.modified.unwrap_or(0.0).partial_cmp(&a.1.modified.unwrap_or(0.0)).unwrap_or(std::cmp::Ordering::Equal));
        out
    }
}

/// Opens the controller: a DJI device first, otherwise any portable device that has DJI Fly's waypoint folder.
fn open_controller() -> Result<(Device, String), String> {
    let list = devices()?;
    if list.is_empty() { return Err(NO_DEVICE.into()); }
    let mut order: Vec<&DeviceId> = list.iter().filter(|d| is_dji(d)).collect();
    order.extend(list.iter().filter(|d| !is_dji(d)));
    let mut last_err = None;
    for d in order {
        match Device::open(d) {
            Ok(dev) => { if let Some(wp) = dev.waypoint_folder() { return Ok((dev, wp)); } if is_dji(d) { last_err = Some(NO_FOLDER.to_string()); } }
            Err(x) => { if is_dji(d) { last_err = Some(x); } }
        }
    }
    Err(last_err.unwrap_or_else(|| NO_DEVICE.into()))
}
fn is_dji(d: &DeviceId) -> bool { let s = format!("{} {}", d.name, d.maker).to_lowercase(); s.contains("dji") || s.contains("rc pro") }

const NO_DEVICE: &str = "No controller found. Use a USB-C data cable, turn the controller on and unlock it, then on the controller pull down the notification shade and set USB to File transfer. If File Explorer shows the controller, close that window and try again.";
const NO_FOLDER: &str = "Found the controller but not DJI Fly's waypoint folder. Save a waypoint mission in DJI Fly first. If you already have, unlock the controller and set USB to File transfer.";

fn ole_to_unix_ms(d: f64) -> i64 { ((d - 25569.0) * 86_400_000.0) as i64 }

pub fn list(tz_offset_min: i64) -> Result<Value, String> {
    let _com = Com::init();
    let (dev, wp) = open_controller()?;
    let arr: Vec<Value> = dev.missions(&wp).into_iter().map(|(f, k)| {
        // WPD reports local time as an OLE date; show it as is.
        let date = k.modified.map(|d| format!("Saved {}", core::fmt_date(ole_to_unix_ms(d)))).unwrap_or_else(|| "Saved mission".into());
        let _ = tz_offset_min;
        json!({ "folder": f.name, "date": date, "size": k.size })
    }).collect();
    Ok(json!({ "type": "missions", "device": dev.label, "missions": arr }))
}

pub fn replace(folder: &str, template: &str, waylines: &str, backup_dir: &Path, preview: Option<Vec<u8>>) -> Result<Value, String> {
    let _com = Com::init();
    let (dev, wp) = open_controller()?;
    let f = dev.child(&wp, folder).ok_or("That mission is no longer on the controller. Tap Find controller again.")?;
    let kmz = dev.children(&f.id).into_iter().find(|o| !o.folder && o.name.to_lowercase().ends_with(".kmz")).ok_or("No KMZ file inside that mission folder.")?;

    // 1. Back up the placeholder to this PC.
    let old = dev.read(&kmz.id)?;
    std::fs::create_dir_all(backup_dir).map_err(|x| format!("Couldn't create the backup folder ({x})"))?;
    let stamp = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|d| d.as_secs()).unwrap_or(0);
    let backup = backup_dir.join(format!("{}_{}.kmz", folder, stamp));
    std::fs::write(&backup, &old).map_err(|x| format!("Couldn't save the backup ({x})"))?;

    // 2. Build the new KMZ with the aircraft codes DJI Fly wrote into the placeholder.
    let codes = core::read_codes(&old).ok();
    let data = core::build_kmz(template, waylines, codes.as_ref())?;

    // 3. Replace: delete, then create a file with the same name. Put the placeholder back if writing fails.
    dev.delete(&kmz.id)?;
    if let Err(x) = dev.upload(&f.id, &kmz.name, &data) {
        let _ = dev.upload(&f.id, &kmz.name, &old);
        return Err(format!("Writing the new mission failed ({x}). The placeholder was put back."));
    }
    let pv = match preview {
        Some(jpg) => match dev.write_preview(&wp, folder, &jpg) { Ok(_) => " Its picture in DJI Fly now shows the name.".to_string(), Err(x) => format!(" (The picture in DJI Fly wasn't updated: {x}.)") },
        None => String::new(),
    };
    let short: String = folder.chars().take(8).collect();
    let mut res = json!({ "type": "sent", "message": format!("Replaced mission {short}… on {}. Backup saved on this PC in Documents\\FlyPlan\\Backups as {}.{pv}", dev.label, backup.file_name().unwrap().to_string_lossy()) });
    if let Some(c) = codes { if let Some(d) = c.drone_enum { res["droneEnum"] = json!(d); res["droneSub"] = json!(c.drone_sub.unwrap_or(0)); } }
    Ok(res)
}
