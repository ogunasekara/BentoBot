import { useEffect, useState } from 'preact/hooks';
import { api, fmt, MapMeta } from '../api';
import { useLive } from '../live';
import { toast } from '../components/Toasts';

function MapCard({ m, reload, idle, inUse }: { m: MapMeta; reload: () => void; idle: boolean; inUse: boolean }) {
  const [renaming, setRenaming] = useState(false);
  const [name, setName] = useState(m.name);
  const [busy, setBusy] = useState(false);

  const act = async (fn: () => Promise<unknown>) => {
    setBusy(true);
    try { await fn(); reload(); } catch (e) { toast((e as Error).message, 'error'); }
    setBusy(false);
  };
  const rename = (e: Event) => {
    e.preventDefault();
    act(async () => { await api('PATCH', `/maps/${m.id}`, { name }); setRenaming(false); });
  };
  const del = () => {
    if (confirm(`Delete “${m.name}”? This removes the image and the pose graph.`)) act(() => api('DELETE', `/maps/${m.id}`));
  };
  const localize = () => act(async () => {
    await api('POST', '/mode/localization/start', { map_id: m.id });
    location.hash = '#/run';
  });
  const w = m.width * m.resolution, h = m.height * m.resolution;

  return (
    <article class="mapcard">
      <a class="thumb" href={`#/edit/${encodeURIComponent(m.id)}`} title="Open in editor">
        <img src={`/api/maps/${encodeURIComponent(m.id)}/thumb.png?v=${encodeURIComponent(m.modified)}`} alt="" loading="lazy" />
      </a>
      <div class="mapcard-body">
        {renaming ? (
          <form class="rename" onSubmit={rename}>
            <input autoFocus value={name} maxLength={100} onInput={(e) => setName((e.target as HTMLInputElement).value)} />
            <button class="btn small primary" disabled={busy || !name.trim()}>Save</button>
            <button type="button" class="btn small" onClick={() => { setRenaming(false); setName(m.name); }}>Cancel</button>
          </form>
        ) : (
          <h3 class="mapname">{m.name}{inUse && <span class="tag tag-active">in use</span>}{m.edited && <span class="tag">edited</span>}</h3>
        )}
        <dl class="meta">
          <dt>Created</dt><dd>{fmt.date(m.created)}</dd>
          <dt>Size</dt><dd>{w.toFixed(1)} × {h.toFixed(1)} m <span class="muted">({m.width}×{m.height} @ {(m.resolution * 100).toFixed(0)} cm)</span></dd>
          <dt>Run</dt><dd>{fmt.duration(m.duration_s)}</dd>
          <dt>Files</dt><dd>{fmt.bytes(m.size_bytes)}{m.has_posegraph ? ' · pose graph' : ' · no pose graph'}</dd>
        </dl>
        <div class="actions">
          <button class="btn small primary" disabled={busy || !idle} title={idle ? '' : 'Stop the current run first'} onClick={localize}>Localize</button>
          <a class="btn small" href={`#/edit/${encodeURIComponent(m.id)}`}>Edit</a>
          <button class="btn small" disabled={busy} onClick={() => setRenaming(true)}>Rename</button>
          <a class="btn small" href={`/api/maps/${encodeURIComponent(m.id)}/download`} download>Download</a>
          <button class="btn small danger-outline" disabled={busy || inUse} onClick={del}>Delete</button>
        </div>
      </div>
    </article>
  );
}

export function Maps() {
  const live = useLive();
  const [maps, setMaps] = useState<MapMeta[] | null>(null);
  const [error, setError] = useState('');
  const load = () => api<MapMeta[]>('GET', '/maps').then((m) => { setMaps(m); setError(''); }).catch((e) => setError(e.message));
  useEffect(() => { load(); }, []);
  const mode = live.status?.mode;
  // refresh after a mapping run saves
  useEffect(() => { load(); }, [mode?.mode]);

  if (error) return <div class="empty error-text">{error}</div>;
  if (!maps) return <div class="empty">Loading maps…</div>;
  if (!maps.length) return <div class="empty">No saved maps yet. Start a mapping run from the <a href="#/run">Run</a> tab.</div>;
  const idle = mode?.mode === 'idle';
  return (
    <div class="maplist">
      {maps.map((m) => <MapCard key={m.id + m.modified} m={m} reload={load} idle={idle}
                                inUse={mode?.mode === 'localization' && mode.map_id === m.id} />)}
    </div>
  );
}
