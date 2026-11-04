/* FlyPlan for PC (Microsoft Edge or Google Chrome).
   Stands in for the Android app's native side: saving and importing files, the shared sync file and MBTiles basemaps.
   It answers through window.onNative exactly like the tablet does, so app.js runs unchanged. */
(function () {
  'use strict';
  const FSA = typeof window.showOpenFilePicker === 'function';
  const emit = o => setTimeout(() => { if (window.onNative) window.onNative(JSON.stringify(o)); }, 0);
  const toast = msg => emit({ type: 'toast', message: msg });
  const errText = e => (e && e.message) || String(e);

  /* ---------- small key-value store (IndexedDB keeps file handles between sessions) ---------- */
  let dbp = null;
  function db() {
    if (!dbp) dbp = new Promise((res, rej) => {
      const r = indexedDB.open('waypoint-planner-pc', 1);
      r.onupgradeneeded = () => r.result.createObjectStore('kv');
      r.onsuccess = () => res(r.result); r.onerror = () => rej(r.error);
    });
    return dbp;
  }
  async function kv(mode, fn) {
    const d = await db();
    return new Promise((res, rej) => {
      const tx = d.transaction('kv', mode), st = tx.objectStore('kv'), rq = fn(st);
      tx.oncomplete = () => res(rq && rq.result); tx.onerror = () => rej(tx.error);
    });
  }
  const kvGet = k => kv('readonly', s => s.get(k));
  const kvSet = (k, v) => kv('readwrite', s => s.put(v, k));
  const kvDel = k => kv('readwrite', s => s.delete(k));

  async function perm(handle, mode, ask) {
    const o = { mode };
    if ((await handle.queryPermission(o)) === 'granted') return true;
    if (!ask) return false;
    try { return (await handle.requestPermission(o)) === 'granted'; } catch (e) { return false; }
  }
  const activation = () => !navigator.userActivation || navigator.userActivation.isActive;

  /* ---------- downloads ---------- */
  function download(name, blob) {
    const a = document.createElement('a');
    a.href = URL.createObjectURL(blob); a.download = name;
    document.body.appendChild(a); a.click(); a.remove();
    setTimeout(() => URL.revokeObjectURL(a.href), 30000);
  }

  /* ---------- zip (stored, no compression; KMZ files are small) ---------- */
  const CRC = (() => { const t = new Uint32Array(256); for (let n = 0; n < 256; n++) { let c = n; for (let k = 0; k < 8; k++) c = c & 1 ? 0xedb88320 ^ (c >>> 1) : c >>> 1; t[n] = c >>> 0; } return t; })();
  function crc32(b) { let c = 0xffffffff; for (let i = 0; i < b.length; i++) c = CRC[(c ^ b[i]) & 255] ^ (c >>> 8); return (c ^ 0xffffffff) >>> 0; }
  function zip(files) {
    const enc = new TextEncoder(), parts = [], central = [];
    const now = new Date(), dosT = (now.getHours() << 11) | (now.getMinutes() << 5) | (now.getSeconds() >> 1),
      dosD = ((now.getFullYear() - 1980) << 9) | ((now.getMonth() + 1) << 5) | now.getDate();
    let off = 0;
    files.forEach(f => {
      const name = enc.encode(f.name), data = typeof f.data === 'string' ? enc.encode(f.data) : f.data, crc = crc32(data);
      const h = new DataView(new ArrayBuffer(30));
      h.setUint32(0, 0x04034b50, true); h.setUint16(4, 20, true); h.setUint16(6, 0x0800, true); h.setUint16(8, 0, true);
      h.setUint16(10, dosT, true); h.setUint16(12, dosD, true); h.setUint32(14, crc, true); h.setUint32(18, data.length, true);
      h.setUint32(22, data.length, true); h.setUint16(26, name.length, true); h.setUint16(28, 0, true);
      parts.push(new Uint8Array(h.buffer), name, data);
      const c = new DataView(new ArrayBuffer(46));
      c.setUint32(0, 0x02014b50, true); c.setUint16(4, 20, true); c.setUint16(6, 20, true); c.setUint16(8, 0x0800, true);
      c.setUint16(10, 0, true); c.setUint16(12, dosT, true); c.setUint16(14, dosD, true); c.setUint32(16, crc, true);
      c.setUint32(20, data.length, true); c.setUint32(24, data.length, true); c.setUint16(28, name.length, true);
      c.setUint32(42, off, true);
      central.push(new Uint8Array(c.buffer), name);
      off += 30 + name.length + data.length;
    });
    const csize = central.reduce((a, b) => a + b.length, 0), e = new DataView(new ArrayBuffer(22));
    e.setUint32(0, 0x06054b50, true); e.setUint16(8, files.length, true); e.setUint16(10, files.length, true);
    e.setUint32(12, csize, true); e.setUint32(16, off, true);
    return new Blob(parts.concat(central, [new Uint8Array(e.buffer)]), { type: 'application/vnd.google-earth.kmz' });
  }
  async function unzip(buf) {
    const u = new Uint8Array(buf), v = new DataView(buf), out = {}, dec = new TextDecoder();
    let e = -1;
    for (let i = u.length - 22; i >= Math.max(0, u.length - 65557); i--) if (v.getUint32(i, true) === 0x06054b50) { e = i; break; }
    if (e < 0) throw new Error('that file is not a valid KMZ');
    let p = v.getUint32(e + 16, true);
    const n = v.getUint16(e + 10, true);
    for (let k = 0; k < n; k++) {
      const method = v.getUint16(p + 10, true), csize = v.getUint32(p + 20, true), nl = v.getUint16(p + 28, true),
        xl = v.getUint16(p + 30, true), cl = v.getUint16(p + 32, true), lo = v.getUint32(p + 42, true);
      const name = dec.decode(u.subarray(p + 46, p + 46 + nl));
      const start = lo + 30 + v.getUint16(lo + 26, true) + v.getUint16(lo + 28, true), raw = u.subarray(start, start + csize);
      if (method === 0) out[name] = raw.slice();
      else if (method === 8) out[name] = new Uint8Array(await new Response(new Blob([raw]).stream().pipeThrough(new DecompressionStream('deflate-raw'))).arrayBuffer());
      p += 46 + nl + xl + cl;
    }
    return out;
  }

  /* ---------- importing missions and features ---------- */
  function pickFile(accept) {
    return new Promise(res => {
      const i = document.createElement('input');
      i.type = 'file'; i.accept = accept; i.style.display = 'none';
      i.onchange = () => { res(i.files[0] || null); i.remove(); };
      document.body.appendChild(i); i.click();
    });
  }
  async function importFor(purpose) {
    const f = await pickFile('.kmz,.kml,.gpx,.wpml,.xml');
    if (!f) return;
    try {
      if (f.size > 25 * 1024 * 1024) throw new Error('file is too large');
      const buf = await f.arrayBuffer(), u = new Uint8Array(buf), dec = new TextDecoder();
      const o = { type: 'imported', purpose: purpose || 'mission', name: f.name };
      if (u[0] === 0x50 && u[1] === 0x4b) {
        const files = await unzip(buf);
        let firstKml = null;
        Object.keys(files).forEach(k => {
          const lk = k.toLowerCase(), t = dec.decode(files[k]);
          if (lk.endsWith('waylines.wpml')) o.waylines = t;
          else if (lk.endsWith('template.kml')) o.template = t;
          else if (lk.endsWith('.kml') && firstKml == null) firstKml = t;
        });
        if (!o.waylines && !o.template) { if (firstKml == null) throw new Error('no mission or KML inside that KMZ'); o.kml = firstKml; }
        else if (o.template && !o.waylines && !o.template.includes('Placemark')) throw new Error('that KMZ has no waypoints');
      } else {
        const t = dec.decode(u);
        if (t.includes('wpml:')) o.waylines = t; else o.kml = t;
      }
      emit(o);
    } catch (e) { toast('Could not import: ' + errText(e)); }
  }

  /* ---------- sync file ---------- */
  let syncHandle = null;
  const syncReady = kvGet('syncHandle').then(h => { syncHandle = h || null; }).catch(() => {});
  function syncErr(code, message) { emit({ type: 'syncError', code, message }); }
  async function syncFile(ask) {
    await syncReady;
    if (!syncHandle) { syncErr('none', 'No sync file is set.'); return null; }
    if (!(await perm(syncHandle, 'readwrite', ask && activation()))) {
      syncErr('perm', 'Edge needs your OK to use the sync file again. Click Allow access and sync.');
      return null;
    }
    return syncHandle;
  }

  /* ---------- SQLite reader for MBTiles ----------
     Reads just the pages it needs from the file, so multi-gigabyte maps open instantly and use little memory. */
  function SqliteFile(file) { this.file = file; this.cache = new Map(); }
  SqliteFile.prototype.read = async function (off, len) { return new Uint8Array(await this.file.slice(off, off + len).arrayBuffer()); };
  SqliteFile.prototype.open = async function () {
    const h = await this.read(0, 100);
    if (new TextDecoder().decode(h.subarray(0, 15)) !== 'SQLite format 3') throw new Error('that file is not an MBTiles (SQLite) file');
    let ps = (h[16] << 8) | h[17]; if (ps === 1) ps = 65536;
    this.ps = ps; this.U = ps - h[20];
    const enc = (h[56] << 24) | (h[57] << 16) | (h[58] << 8) | h[59];
    if (enc > 1) throw new Error('unsupported text encoding in that file');
    await this.loadSchema();
  };
  SqliteFile.prototype.page = async function (n) {
    let p = this.cache.get(n);
    if (p) { this.cache.delete(n); this.cache.set(n, p); return p; }
    p = await this.read((n - 1) * this.ps, this.ps);
    this.cache.set(n, p);
    if (this.cache.size > 400) this.cache.delete(this.cache.keys().next().value);
    return p;
  };
  function varint(b, i) {
    let v = 0;
    for (let k = 0; k < 8; k++) { const c = b[i + k]; v = v * 128 + (c & 0x7f); if (!(c & 0x80)) return [v, i + k + 1]; }
    return [v * 256 + b[i + 8], i + 9];
  }
  const u32 = (b, i) => ((b[i] << 24) >>> 0) + (b[i + 1] << 16) + (b[i + 2] << 8) + b[i + 3];
  SqliteFile.prototype.header = function (pg, n) {
    const o = n === 1 ? 100 : 0, type = pg[o];
    if (![2, 5, 10, 13].includes(type)) throw new Error('damaged page ' + n);
    const leaf = type === 10 || type === 13;
    return { type, leaf, index: type === 2 || type === 10, cells: (pg[o + 3] << 8) | pg[o + 4], right: leaf ? 0 : u32(pg, o + 8), ptr: o + (leaf ? 8 : 12) };
  };
  SqliteFile.prototype.cellAt = function (pg, h, i) { const q = h.ptr + 2 * i; return (pg[q] << 8) | pg[q + 1]; };
  // Full payload of a cell that starts at byte p, following overflow pages if needed.
  SqliteFile.prototype.payload = async function (pg, p, P, isIndex) {
    const U = this.U, X = isIndex ? Math.floor((U - 12) * 64 / 255) - 23 : U - 35, M = Math.floor((U - 12) * 32 / 255) - 23;
    if (P <= X) return pg.subarray(p, p + P);
    const K = M + ((P - M) % (U - 4)), local = K <= X ? K : M;
    const out = new Uint8Array(P);
    out.set(pg.subarray(p, p + local));
    let got = local, next = u32(pg, p + local);
    while (got < P && next) {
      const ov = await this.read((next - 1) * this.ps, this.ps), take = Math.min(U - 4, P - got);
      out.set(ov.subarray(4, 4 + take), got); got += take; next = u32(ov, 0);
    }
    return out;
  };
  function record(b) {
    let [hs, i] = varint(b, 0);
    const types = [];
    while (i < hs) { let t; [t, i] = varint(b, i); types.push(t); }
    let p = hs;
    const vals = [], dec = new TextDecoder();
    types.forEach(t => {
      let v = null;
      if (t >= 1 && t <= 6) {
        const n = [0, 1, 2, 3, 4, 6, 8][t];
        v = 0;
        for (let k = 0; k < n; k++) v = v * 256 + b[p + k];
        if (b[p] & 0x80) v -= Math.pow(2, 8 * n);
        p += n;
      } else if (t === 7) { v = new DataView(b.buffer, b.byteOffset + p, 8).getFloat64(0); p += 8; }
      else if (t === 8) v = 0; else if (t === 9) v = 1;
      else if (t >= 12) {
        const n = t % 2 ? (t - 13) / 2 : (t - 12) / 2;
        v = t % 2 ? dec.decode(b.subarray(p, p + n)) : b.subarray(p, p + n);
        p += n;
      }
      vals.push(v);
    });
    return vals;
  }
  function cmpVal(a, b) {
    const rank = v => v == null ? 0 : typeof v === 'number' ? 1 : typeof v === 'string' ? 2 : 3;
    const ra = rank(a), rb = rank(b);
    if (ra !== rb) return ra - rb;
    if (ra === 0) return 0;
    if (ra === 1) return a < b ? -1 : a > b ? 1 : 0;
    const ea = ra === 2 ? new TextEncoder().encode(a) : a, eb = ra === 2 ? new TextEncoder().encode(b) : b;
    for (let i = 0; i < Math.min(ea.length, eb.length); i++) if (ea[i] !== eb[i]) return ea[i] - eb[i];
    return ea.length - eb.length;
  }
  function cmpKey(key, rec) { for (let i = 0; i < key.length; i++) { const c = cmpVal(key[i], rec[i]); if (c) return c; } return 0; }
  // Every row of a table: calls fn(rowid, values). Used for small tables, or as a fallback with no index.
  SqliteFile.prototype.scan = async function (root, fn, localOnly) {
    const stack = [root];
    while (stack.length) {
      const n = stack.pop(), pg = await this.page(n), h = this.header(pg, n);
      if (!h.leaf) { stack.push(h.right); for (let i = h.cells - 1; i >= 0; i--) stack.push(u32(pg, this.cellAt(pg, h, i))); continue; }
      for (let i = 0; i < h.cells; i++) {
        let p = this.cellAt(pg, h, i), P, rowid;
        [P, p] = varint(pg, p); [rowid, p] = varint(pg, p);
        const body = localOnly ? pg.subarray(p, p + Math.min(P, this.U - 35)) : await this.payload(pg, p, P, false);
        fn(rowid, record(body));
      }
    }
  };
  SqliteFile.prototype.row = async function (root, rowid) {
    let n = root;
    for (let depth = 0; depth < 40; depth++) {
      const pg = await this.page(n), h = this.header(pg, n);
      if (h.leaf) {
        let lo = 0, hi = h.cells - 1;
        while (lo <= hi) {
          const mid = (lo + hi) >> 1; let p = this.cellAt(pg, h, mid), P, r;
          [P, p] = varint(pg, p); [r, p] = varint(pg, p);
          if (r === rowid) return record(await this.payload(pg, p, P, false));
          if (r < rowid) lo = mid + 1; else hi = mid - 1;
        }
        return null;
      }
      let lo = 0, hi = h.cells - 1, child = h.right;
      while (lo <= hi) {
        const mid = (lo + hi) >> 1, p = this.cellAt(pg, h, mid), key = varint(pg, p + 4)[0];
        if (key >= rowid) { child = u32(pg, p); hi = mid - 1; } else lo = mid + 1;
      }
      n = child;
    }
    return null;
  };
  // Looks up an index entry whose leading columns equal key; returns the entry (columns..., rowid) or null.
  SqliteFile.prototype.seek = async function (root, key) {
    let n = root;
    for (let depth = 0; depth < 40; depth++) {
      const pg = await this.page(n), h = this.header(pg, n);
      let lo = 0, hi = h.cells - 1, child = h.right;
      while (lo <= hi) {
        const mid = (lo + hi) >> 1;
        let p = this.cellAt(pg, h, mid), left = 0, P;
        if (!h.leaf) { left = u32(pg, p); p += 4; }
        [P, p] = varint(pg, p);
        const rec = record(await this.payload(pg, p, P, true)), c = cmpKey(key, rec);
        if (c === 0) return rec;
        if (c < 0) { child = left; hi = mid - 1; } else lo = mid + 1;
      }
      if (h.leaf) return null;
      n = child;
    }
    return null;
  };
  // Index entries in order; fn returns false to stop.
  SqliteFile.prototype.walk = async function (n, fn) {
    const pg = await this.page(n), h = this.header(pg, n);
    for (let i = 0; i < h.cells; i++) {
      let p = this.cellAt(pg, h, i), P;
      if (!h.leaf) { if ((await this.walk(u32(pg, p), fn)) === false) return false; p += 4; }
      [P, p] = varint(pg, p);
      if (fn(record(await this.payload(pg, p, P, true))) === false) return false;
    }
    if (!h.leaf) return this.walk(h.right, fn);
    return true;
  };
  // First or last entry of an index (for the zoom range when metadata leaves it out).
  SqliteFile.prototype.edge = async function (root, last) {
    let n = root;
    for (let depth = 0; depth < 40; depth++) {
      const pg = await this.page(n), h = this.header(pg, n);
      if (!h.leaf) { n = last ? h.right : u32(pg, this.cellAt(pg, h, 0)); continue; }
      if (!h.cells) return null;
      let p = this.cellAt(pg, h, last ? h.cells - 1 : 0), P; [P, p] = varint(pg, p);
      return record(await this.payload(pg, p, P, true));
    }
    return null;
  };
  function colsOf(sql) {
    const a = sql.indexOf('('), b = sql.lastIndexOf(')');
    if (a < 0 || b < a) return { cols: [], pk: null, keys: [] };
    const body = sql.slice(a + 1, b), parts = []; let depth = 0, cur = '';
    for (const ch of body) { if (ch === '(') depth++; if (ch === ')') depth--; if (ch === ',' && !depth) { parts.push(cur); cur = ''; } else cur += ch; }
    parts.push(cur);
    const strip = s => s.trim().replace(/^["'`\[]|["'`\]]$/g, '').toLowerCase(), cols = [], keys = [];
    let pk = null;
    parts.forEach(p => {
      const t = p.trim(), m = /^(constraint\s+\S+\s+)?(primary\s+key|unique)\s*\(([^)]*)\)/i.exec(t);
      if (m) { keys.push(m[3].split(',').map(strip)); return; }
      if (/^(check|foreign|constraint)\b/i.test(t)) return;
      const name = strip(t.split(/\s+/)[0]);
      cols.push(name);
      if (/\binteger\s+primary\s+key\b/i.test(t)) pk = name;
      else if (/\b(primary\s+key|unique)\b/i.test(t)) keys.push([name]);
    });
    return { cols, pk, keys };
  }
  SqliteFile.prototype.loadSchema = async function () {
    const objs = [];
    await this.scan(1, (rowid, v) => objs.push({ type: v[0], name: String(v[1] || '').toLowerCase(), tbl: String(v[2] || '').toLowerCase(), root: v[3], sql: v[4] || '' }));
    this.tables = {}; this.indexes = [];
    objs.forEach(o => { if (o.type === 'table') this.tables[o.name] = Object.assign({ root: o.root, view: false, sql: o.sql }, colsOf(o.sql)); if (o.type === 'view') this.tables[o.name] = { view: true, sql: o.sql }; });
    objs.forEach(o => {
      if (o.type !== 'index') return;
      let cols;
      if (o.sql) { const m = /\(([^)]*)\)\s*$/.exec(o.sql.trim()); cols = m ? m[1].split(',').map(s => s.trim().split(/\s+/)[0].replace(/^["'`\[]|["'`\]]$/g, '').toLowerCase()) : []; }
      else { const t = this.tables[o.tbl], k = Number((/_(\d+)$/.exec(o.name) || [0, 1])[1]) - 1; cols = t && t.keys && t.keys[k] ? t.keys[k] : []; }
      this.indexes.push({ tbl: o.tbl, root: o.root, cols });
    });
    if (Object.values(this.tables).some(t => !t.view && /without\s+rowid/i.test(t.sql || ''))) throw new Error('this MBTiles layout isn’t supported');
  };
  SqliteFile.prototype.findIndex = function (tbl, cols) {
    return this.indexes.find(ix => ix.tbl === tbl && cols.every((c, i) => ix.cols[i] === c)) || null;
  };

  function MbReader(file) { this.db = new SqliteFile(file); this.missing = 0; }
  MbReader.prototype.open = async function () {
    const d = this.db; await d.open();
    const T = d.tables;
    this.meta = {};
    if (T.metadata && !T.metadata.view) {
      const ni = T.metadata.cols.indexOf('name'), vi = T.metadata.cols.indexOf('value');
      await d.scan(T.metadata.root, (r, v) => { if (v[ni] != null) this.meta[String(v[ni]).toLowerCase()] = v[vi]; });
    }
    const key = ['zoom_level', 'tile_column', 'tile_row'];
    if (T.tiles && !T.tiles.view) {
      this.mode = 'tiles'; this.t = T.tiles; this.ix = d.findIndex('tiles', key); this.dataCol = T.tiles.cols.indexOf('tile_data');
    } else if (T.map && T.images && !T.map.view && !T.images.view) {
      this.mode = 'map'; this.t = T.map; this.ix = d.findIndex('map', key); this.img = T.images;
      this.imgIx = d.findIndex('images', ['tile_id']); this.idCol = T.map.cols.indexOf('tile_id');
      this.imgIdCol = T.images.cols.indexOf('tile_id'); this.dataCol = T.images.cols.indexOf('tile_data');
    } else throw new Error('no map tiles were found in that file');
    if (this.dataCol < 0) throw new Error('no map tiles were found in that file');
    if (!this.ix) await this.buildLookup();
    if (this.mode === 'map' && !this.imgIx) {
      this.imgMap = new Map(); await d.scan(this.img.root, (r, v) => this.imgMap.set(v[this.imgIdCol], r), this.imgIdCol < this.dataCol);
    }
    let minz = parseInt(this.meta.minzoom, 10), maxz = parseInt(this.meta.maxzoom, 10);
    if (!(minz >= 0) || !(maxz >= 0)) {
      if (this.ix) { const a = await d.edge(this.ix.root, false), b = await d.edge(this.ix.root, true); minz = a ? a[0] : 0; maxz = b ? b[0] : 18; }
      else { const zs = [...this.lookup.keys()].map(k => +k.split('/')[0]); minz = Math.min(...zs); maxz = Math.max(...zs); }
    }
    this.minzoom = minz; this.maxzoom = maxz;
    const b = String(this.meta.bounds || '').split(',').map(Number);
    this.bounds = b.length === 4 && b.every(isFinite) && b[2] > b[0] && b[3] > b[1] ? b : null;
    if (!this.bounds) this.bounds = await this.guessBounds();
  };
  MbReader.prototype.buildLookup = async function () {
    const c = this.t.cols, zi = c.indexOf('zoom_level'), xi = c.indexOf('tile_column'), yi = c.indexOf('tile_row');
    this.lookup = new Map();
    const early = this.mode === 'map' || Math.max(zi, xi, yi) < this.dataCol;
    await this.db.scan(this.t.root, (r, v) => this.lookup.set(v[zi] + '/' + v[xi] + '/' + v[yi], r), early);
  };
  MbReader.prototype.guessBounds = async function () {
    // Bounds from the tiles at the lowest zoom: walk the index range for that zoom (cheap at low zoom).
    const z = this.minzoom, n = Math.pow(2, z);
    let x1 = Infinity, x2 = -1, y1 = Infinity, y2 = -1;
    const add = (x, y) => { x1 = Math.min(x1, x); x2 = Math.max(x2, x); y1 = Math.min(y1, y); y2 = Math.max(y2, y); };
    if (this.lookup) this.lookup.forEach((r, k) => { const p = k.split('/').map(Number); if (p[0] === z) add(p[1], p[2]); });
    else { let seen = 0; await this.db.walk(this.ix.root, e => { if (e[0] > z || ++seen > 200000) return false; if (e[0] === z) add(e[1], e[2]); }); }
    if (x2 < 0) return null;
    const lng = x => x / n * 360 - 180, lat = y => { const t = Math.PI * (1 - 2 * (n - y) / n); return 180 / Math.PI * Math.atan(Math.sinh(t)); };
    return [lng(x1), lat(y1), lng(x2 + 1), lat(y2 + 1)];
  };
  MbReader.prototype.coverage = async function (z) {
    const out = [], n = Math.pow(2, z); let seen = 0;
    if (this.lookup) this.lookup.forEach((r, k) => { const p = k.split('/').map(Number); if (p[0] === z) out.push(p[1], n - 1 - p[2]); });
    else await this.db.walk(this.ix.root, e => { if (e[0] > z || ++seen > 2000000) return false; if (e[0] === z) out.push(e[1], n - 1 - e[2]); });
    return out;
  };
  MbReader.prototype.tile = async function (z, x, y) {
    const d = this.db, row = Math.pow(2, z) - 1 - y; // MBTiles rows count from the bottom
    let rowid = null;
    if (this.ix) { const e = await d.seek(this.ix.root, [z, x, row]); if (e) rowid = e[e.length - 1]; }
    else rowid = this.lookup.has(z + '/' + x + '/' + row) ? this.lookup.get(z + '/' + x + '/' + row) : null;
    if (rowid == null) return null;
    let rec = await d.row(this.t.root, rowid);
    if (!rec) return null;
    if (this.mode === 'map') {
      const id = rec[this.idCol];
      let ir = null;
      if (this.imgIx) { const e = await d.seek(this.imgIx.root, [id]); if (e) ir = e[e.length - 1]; }
      else ir = this.imgMap.get(id);
      if (ir == null) return null;
      rec = await d.row(this.img.root, ir);
      if (!rec) return null;
    }
    const data = rec[this.dataCol];
    if (!data || !data.length) return null;
    const type = data[0] === 0x89 ? 'image/png' : data[0] === 0xff ? 'image/jpeg' : data[0] === 0x52 ? 'image/webp' : 'application/octet-stream';
    return new Blob([data], { type });
  };

  /* ---------- my maps ---------- */
  const maps = {}; // id -> { info, handle, reader, file, locked }
  async function loadMapList() {
    const list = (await kvGet('maps')) || [];
    for (const info of list) {
      if (maps[info.id]) continue;
      const handle = await kvGet('map:' + info.id);
      maps[info.id] = { info, handle, reader: null, locked: !handle || !(await perm(handle, 'read', false)) };
    }
    return list;
  }
  async function saveMapList() { await kvSet('maps', Object.values(maps).map(m => m.info)); }
  async function emitMaps(added) {
    try {
      await loadMapList();
      const list = Object.values(maps).map(m => Object.assign({}, m.info, { locked: m.locked || undefined, missing: m.missing || undefined }));
      emit({ type: 'maps', maps: list, added: added || undefined });
    } catch (e) { emit({ type: 'mbError', message: errText(e) }); }
  }
  async function readerFor(id) {
    const m = maps[id];
    if (!m || m.locked) return null;
    if (m.reader) return m.reader;
    if (!m.opening) m.opening = (async () => {
      const file = await m.handle.getFile(), r = new MbReader(file);
      await r.open(); m.reader = r; m.missing = false; return r;
    })().catch(e => { m.opening = null; if (!m.missing) { m.missing = true; toast('Couldn’t read the map “' + m.info.name + '”: ' + errText(e)); emitMaps(); } return null; });
    return m.opening;
  }
  async function readTile(id, z, x, y) {
    const r = await readerFor(id);
    if (!r) return null;
    try { return await r.tile(z, x, y); }
    catch (e) {
      // The file changed on disk (for example Google Drive replaced it); reopen it once.
      const m = maps[id]; m.reader = null; m.opening = null;
      const r2 = await readerFor(id);
      return r2 ? r2.tile(z, x, y) : null;
    }
  }
  async function importMbtiles() {
    if (!FSA) { emit({ type: 'mbError', message: 'Opening map files needs Microsoft Edge or Google Chrome.' }); return; }
    let handle;
    try { [handle] = await window.showOpenFilePicker({ id: 'mbtiles', types: [{ description: 'MBTiles map', accept: { 'application/octet-stream': ['.mbtiles'] } }], excludeAcceptAllOption: false }); }
    catch (e) { return; }
    emit({ type: 'mbProgress', done: 0, total: 0 });
    try {
      const file = await handle.getFile(), r = new MbReader(file);
      await r.open();
      for (const m of Object.values(maps)) if (m.handle && await m.handle.isSameEntry(handle)) { m.locked = false; m.reader = r; emitMaps(m.info.id); return; }
      const id = 'pc' + Date.now().toString(36);
      const info = { id, name: String(r.meta.name || file.name.replace(/\.mbtiles$/i, '')), file: file.name.replace(/\.mbtiles$/i, ''), size: file.size, minzoom: r.minzoom, maxzoom: r.maxzoom, bounds: r.bounds, format: r.meta.format || '' };
      maps[id] = { info, handle, reader: r, locked: false };
      try { await kvSet('map:' + id, handle); } catch (e) {}
      await saveMapList();
      emitMaps(id);
    } catch (e) { emit({ type: 'mbError', message: 'Couldn’t open that map: ' + errText(e) }); }
  }

  /* ---------- the native interface app.js talks to ---------- */
  const A = {
    isOnController: () => false,
    saveKmz(name, template, waylines) {
      if (!/\.kmz$/i.test(name)) name += '.kmz';
      download(name, zip([{ name: 'wpmz/template.kml', data: template }, { name: 'wpmz/waylines.wpml', data: waylines }]));
      toast('Saved ' + name + ' to Downloads');
    },
    saveText(name, mime, text) { download(name, new Blob([text], { type: mime || 'text/plain' })); toast('Saved ' + name + ' to Downloads'); },
    saveBinary(name, mime, b64) { const bin = atob(b64), u = new Uint8Array(bin.length); for (let i = 0; i < bin.length; i++) u[i] = bin.charCodeAt(i); download(name, new Blob([u], { type: mime })); toast('Saved ' + name + ' to Downloads'); },
    importKmz() { importFor('mission'); },
    importFor(purpose) { importFor(purpose); },
    syncInfo() { return FSA ? JSON.stringify({ name: localStorage.getItem('pc.syncName') || '', device: 'PC' }) : ''; },
    syncPick(create) {
      (async () => {
        let h;
        const opts = { id: 'sync', types: [{ description: 'FlyPlan sync file', accept: { 'application/json': ['.json'] } }] };
        try { h = create ? await window.showSaveFilePicker(Object.assign({ suggestedName: 'FlyPlan sync.json' }, opts)) : (await window.showOpenFilePicker(opts))[0]; }
        catch (e) { syncErr('cancel', ''); return; }
        if (!(await perm(h, 'readwrite', true))) { syncErr('perm', 'Edge wasn’t allowed to edit that file.'); return; }
        syncHandle = h; try { await kvSet('syncHandle', h); } catch (e) {}
        localStorage.setItem('pc.syncName', h.name);
        emit({ type: 'syncPicked', name: h.name });
      })();
    },
    syncRead() {
      const ask = activation();
      (async () => {
        const h = await syncFile(ask); if (!h) return;
        try { const f = await h.getFile(); emit({ type: 'syncData', text: await f.text() }); }
        catch (e) { syncErr(e && e.name === 'NotFoundError' ? 'missing' : 'io', e && e.name === 'NotFoundError' ? 'The sync file was moved or deleted. Click Choose a different file.' : 'Couldn’t read the sync file: ' + errText(e)); }
      })();
    },
    syncWrite(text) {
      (async () => {
        const h = await syncFile(false); if (!h) return;
        try { const w = await h.createWritable(); await w.write(text); await w.close(); emit({ type: 'syncWritten' }); }
        catch (e) { syncErr('io', 'Couldn’t save the sync file: ' + errText(e)); }
      })();
    },
    syncForget() { syncHandle = null; localStorage.removeItem('pc.syncName'); kvDel('syncHandle'); },
    importMbtiles() { importMbtiles(); },
    listMaps() { emitMaps(); },
    mapCoverage(id, z) { readerFor(id).then(r => r ? r.coverage(z) : null).then(t => { if (t) emit({ type: 'mapCov', id, z, tiles: t }); }).catch(() => {}); },
    deleteMap(id) { (async () => { delete maps[id]; await kvDel('map:' + id); await saveMapList(); emitMaps(); })(); },
    unlockMap(id) {
      const m = maps[id]; if (!m || !m.handle) return;
      (async () => { if (await perm(m.handle, 'read', true)) { m.locked = false; m.missing = false; m.reader = null; m.opening = null; emitMaps(); } })();
    },
  };
  if (!FSA) { delete A.importMbtiles; delete A.syncPick; }
  window.Android = A;

  window.PcHost = {
    _MbReader: MbReader,
    tileBlob(id, z, x, y) { return readTile(id, z, x, y); },
    mapLayer(m, opt) {
      const Layer = L.GridLayer.extend({
        createTile(c, done) {
          const img = document.createElement('img');
          img.alt = ''; img.setAttribute('role', 'presentation');
          readTile(m.id, c.z, c.x, c.y).then(blob => {
            if (!blob) { done(null, img); return; }
            const url = URL.createObjectURL(blob);
            img.onload = () => { URL.revokeObjectURL(url); done(null, img); };
            img.onerror = () => { URL.revokeObjectURL(url); done(null, img); };
            img.src = url;
          }).catch(() => done(null, img));
          return img;
        },
      });
      return new Layer(opt);
    },
  };
})();
