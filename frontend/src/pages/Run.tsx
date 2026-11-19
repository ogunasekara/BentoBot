import { useEffect, useState } from 'preact/hooks';
import { api, fmt, MapMeta } from '../api';
import { useLive } from '../live';
import { MapView } from '../components/MapView';
import { toast } from '../components/Toasts';

function useMaps() {
  const [maps, setMaps] = useState<MapMeta[] | null>(null);
  useEffect(() => { api<MapMeta[]>('GET', '/maps').then(setMaps).catch(() => setMaps([])); }, []);
  return maps;
}

function ModeLog() {
  const [lines, setLines] = useState<string[] | null>(null);
  const load = () => api<{ lines: string[] }>('GET', '/mode/log').then((r) => setLines(r.lines)).catch(() => {});
  return (
    <details class="log" onToggle={(e) => { if ((e.target as HTMLDetailsElement).open) load(); }}>
      <summary>Launch output</summary>
      <button class="btn small" onClick={load}>Refresh</button>
      <pre>{lines ? lines.slice(-150).join('\n') || '(empty)' : 'loading…'}</pre>
    </details>
  );
}

function StartPanel() {
  const live = useLive();
  const maps = useMaps();
  const [mapId, setMapId] = useState('');
  const [busy, setBusy] = useState(false);
  const mode = live.status?.mode;
  useEffect(() => {
    const q = new URLSearchParams(location.hash.split('?')[1] || '');
    if (q.get('map')) setMapId(q.get('map')!);
    else if (maps && maps.length && !mapId) setMapId(maps[0].id);
  }, [maps]);

  const start = async (m: string, body: object = {}) => {
    setBusy(true);
    try { await api('POST', `/mode/${m}/start`, body); } catch (e) { toast((e as Error).message, 'error'); }
    setBusy(false);
  };

  return (
    <div class="main">
      {mode?.error && <div class="banner banner-error">{mode.error}</div>}
      <div class="grid">
        <section class="card">
          <div class="card-head"><h2>Mapping run</h2></div>
          <p>Start slam_toolbox and drive with the controller while the map builds. Stop and save when you're done.</p>
          <button class="btn primary" disabled={busy || !live.connected} onClick={() => start('mapping')}>Start mapping</button>
        </section>
        <section class="card">
          <div class="card-head"><h2>Localization run</h2></div>
          <p>Localize on a saved map with AMCL (uses the edited image). Drive with the controller and set the initial pose on the map.</p>
          {maps && maps.length ? (
            <>
              <label class="field">
                <span>Map</span>
                <select value={mapId} onChange={(e) => setMapId((e.target as HTMLSelectElement).value)}>
                  {maps.map((m) => <option key={m.id} value={m.id}>{m.name} · {fmt.date(m.created)}</option>)}
                </select>
              </label>
              <button class="btn primary" disabled={busy || !mapId || !live.connected}
                      onClick={() => start('localization', { map_id: mapId })}>Start localization</button>
            </>
          ) : <p class="muted">{maps ? 'No saved maps yet. Do a mapping run first.' : 'Loading maps…'}</p>}
        </section>
      </div>
      <ModeLog />
    </div>
  );
}

function SaveDialog({ onClose }: { onClose: () => void }) {
  const [name, setName] = useState('');
  const [stop, setStop] = useState(true);
  const [busy, setBusy] = useState(false);
  const save = async (e: Event) => {
    e.preventDefault();
    setBusy(true);
    try {
      const r = await api<{ map: MapMeta; warnings: string[] }>('POST', '/mode/mapping/save', { name, stop });
      toast(`Saved “${r.map.name}”`, 'ok');
      r.warnings.forEach((w) => toast(w, 'error'));
      onClose();
    } catch (err) {
      toast((err as Error).message, 'error');
    }
    setBusy(false);
  };
  return (
    <div class="dialog-back" onClick={onClose}>
      <form class="dialog" onClick={(e) => e.stopPropagation()} onSubmit={save}>
        <h2>Save map</h2>
        <p class="muted">Saves the slam_toolbox pose graph and an occupancy image (PGM + YAML) from the latest /map.</p>
        <label class="field"><span>Name</span>
          <input autoFocus required maxLength={100} value={name} placeholder="e.g. living room"
                 onInput={(e) => setName((e.target as HTMLInputElement).value)} />
        </label>
        <label class="check"><input type="checkbox" checked={stop} onChange={(e) => setStop((e.target as HTMLInputElement).checked)} /> Stop the mapping run after saving</label>
        <div class="row-end">
          <button type="button" class="btn" onClick={onClose}>Cancel</button>
          <button class="btn primary" disabled={busy || !name.trim()}>{busy ? 'Saving…' : 'Save'}</button>
        </div>
      </form>
    </div>
  );
}

function confidence(a?: { sx: number; sy: number; syaw: number }) {
  if (!a) return { label: 'no estimate', cls: 'muted' };
  const xy = Math.max(a.sx, a.sy), th = (a.syaw * 180) / Math.PI;
  if (xy < 0.15 && th < 10) return { label: 'good', cls: 'ok' };
  if (xy < 0.5 && th < 30) return { label: 'fair', cls: 'warn' };
  return { label: 'poor', cls: 'bad' };
}

export function Run() {
  const live = useLive();
  const [saving, setSaving] = useState(false);
  const [poseTool, setPoseTool] = useState(false);
  const [busy, setBusy] = useState(false);
  const mode = live.status?.mode;

  if (!live.status) return <div class="empty">Connecting…</div>;
  if (!mode || mode.mode === 'idle') return <StartPanel />;

  const stop = async (path: string, confirmText?: string) => {
    if (confirmText && !confirm(confirmText)) return;
    setBusy(true);
    try { await api('POST', path); } catch (e) { toast((e as Error).message, 'error'); }
    setBusy(false);
  };
  const setPose = async (x: number, y: number, yaw: number) => {
    setPoseTool(false);
    try {
      await api('POST', '/localization/initialpose', { x, y, yaw });
      toast(`Initial pose set: ${x.toFixed(2)}, ${y.toFixed(2)}, ${fmt.deg(yaw)}`, 'ok');
    } catch (e) { toast((e as Error).message, 'error'); }
  };

  const amcl = live.telemetry.amcl;
  const conf = confidence(amcl);
  const starting = mode.phase === 'starting';

  return (
    <div class="run">
      <MapView poseTool={poseTool} onPose={setPose} />
      {!live.map && <div class="map-hint">{starting ? `Starting ${mode.label.toLowerCase()}…` : mode.mode === 'mapping' ? 'Waiting for the first /map from slam_toolbox (drive a little)…' : 'Loading map…'}</div>}
      {poseTool && <div class="map-hint">Press where the robot is, drag towards where it faces, release.</div>}
      <div class="run-bar">
        <div class="run-info">
          <b>{mode.label}</b>
          <span class="muted">{starting ? 'starting…' : fmt.duration(mode.elapsed)}</span>
          {mode.mode === 'localization' && (
            <span class={'conf conf-' + conf.cls} title="AMCL covariance (1σ)">
              {conf.label}{amcl ? ` · ±${Math.max(amcl.sx, amcl.sy).toFixed(2)} m ±${fmt.deg(amcl.syaw)}` : ''}
            </span>
          )}
        </div>
        <div class="run-actions">
          {mode.mode === 'mapping' && <>
            <button class="btn primary" disabled={busy || starting} onClick={() => setSaving(true)}>Stop &amp; save…</button>
            {!mode.external && <button class="btn danger-outline" disabled={busy}
              onClick={() => stop('/mode/mapping/discard', 'Discard this mapping run? The map will be lost.')}>Discard</button>}
          </>}
          {mode.mode === 'localization' && <>
            <button class={'btn' + (poseTool ? ' primary' : '')} onClick={() => setPoseTool(!poseTool)}>{poseTool ? 'Cancel' : 'Set pose'}</button>
            {!mode.external && <button class="btn" disabled={busy} onClick={() => stop('/mode/stop')}>Stop run</button>}
          </>}
        </div>
      </div>
      {saving && <SaveDialog onClose={() => setSaving(false)} />}
    </div>
  );
}
