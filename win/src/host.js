/* Windows app bridge: gives app.js the same window.Android interface the tablet has, backed by the .exe. */
(function () {
  'use strict';
  var st = window.__hostState || {};
  function post(cmd, args) { window.ipc.postMessage(JSON.stringify({ cmd: cmd, args: args || [] })); }
  window.WinHost = true;
  window.MB_BASE = '/mbtiles/';
  window.__hostSet = function (k, v) { st[k] = v; };
  window.Android = {
    isOnController: function () { return false; },
    syncInfo: function () { return JSON.stringify({ name: st.syncName || '', device: st.device || 'PC' }); },
    saveKmz: function (n, t, w) { post('saveKmz', [n, t, w]); },
    saveText: function (n, m, t) { post('saveText', [n, m, t]); },
    saveBinary: function (n, m, b) { post('saveB64', [n, m, b]); },
    importKmz: function () { post('importFor', ['mission']); },
    importFor: function (p) { post('importFor', [p || 'mission']); },
    syncPick: function (create) { post('syncPick', [!!create]); },
    syncRead: function () { post('syncRead'); },
    syncWrite: function (text) { post('syncWrite', [text]); },
    syncForget: function () { st.syncName = ''; post('syncForget'); },
    importMbtiles: function () { post('importMbtiles'); },
    listMaps: function () { post('listMaps'); },
    deleteMap: function (id) { post('deleteMap', [id]); },
    mapCoverage: function (id, z) { post('mapCoverage', [id, z]); },
    findController: function () { post('findController'); },
    sendMission: function (folder, name, template, waylines, preview) { post('sendMission', [folder, name, template, waylines, preview || '']); },
    openUrl: function (u) { post('openUrl', [u]); },
    requestFolderAccess: function () {},
  };
})();
