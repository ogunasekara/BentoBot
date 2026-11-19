import { useEffect, useRef, useState } from 'preact/hooks';
import { api, MapMeta } from '../api';
import { paintGray } from '../live';
import { toast } from '../components/Toasts';
import { Pt, Tool, Viewport, attachGestures, fitCanvas } from '../viewport';

// map_saver trinary values
const VALUES = { occupied: 0, free: 254, unknown: 205 } as const;
type Paint = keyof typeof VALUES;
type ToolName = 'pan' | 'brush' | 'rect' | 'line';

interface Edit { idx: Int32Array; before: Uint8Array; after: Uint8Array }

function parsePgm(buf: ArrayBuffer) {
  const b = new Uint8Array(buf);
  let i = 0;
  const tokens: string[] = [];
  while (tokens.length < 4) {
    while (i < b.length && /\s/.test(String.fromCharCode(b[i]))) i++;
    if (b[i] === 35) { while (i < b.length && b[i] !== 10) i++; continue; } // # comment
    let s = '';
    while (i < b.length && !/\s/.test(String.fromCharCode(b[i]))) s += String.fromCharCode(b[i++]);
    tokens.push(s);
  }
  i++;
  const [magic, w, h, max] = [tokens[0], +tokens[1], +tokens[2], +tokens[3]];
  if (magic !== 'P5' || max > 255) throw new Error('unsupported PGM (expected 8-bit P5)');
  return { width: w, height: h, pixels: b.slice(i, i + w * h) };
}

class Doc {
  undo: Edit[] = [];
  redo: Edit[] = [];
  canvas = document.createElement('canvas');
  ctx: CanvasRenderingContext2D;
  image: ImageData;
  private cur: Map<number, number> | null = null;
  dirty: [number, number, number, number] | null = null; // x0, y0, x1, y1
  constructor(public w: number, public h: number, public px: Uint8Array) {
    this.canvas.width = w;
    this.canvas.height = h;
    this.ctx = this.canvas.getContext('2d')!;
    this.image = this.ctx.createImageData(w, h);
    paintGray(this.image.data, px, 0, px.length);
    this.ctx.putImageData(this.image, 0, 0);
  }
  begin() { this.cur = new Map(); }
  set(x: number, y: number, v: number) {
    if (x < 0 || y < 0 || x >= this.w || y >= this.h) return;
    const i = y * this.w + x;
    if (this.px[i] === v) return;
    if (this.cur && !this.cur.has(i)) this.cur.set(i, this.px[i]);
    this.px[i] = v;
    paintGray(this.image.data, this.px, i, i + 1);
    const d = this.dirty;
    this.dirty = d ? [Math.min(d[0], x), Math.min(d[1], y), Math.max(d[2], x), Math.max(d[3], y)] : [x, y, x, y];
  }
  disc(cx: number, cy: number, r: number, v: number) {
    const rr = r * r;
    for (let y = Math.floor(cy - r); y <= Math.ceil(cy + r); y++)
      for (let x = Math.floor(cx - r); x <= Math.ceil(cx + r); x++)
        if ((x - cx) ** 2 + (y - cy) ** 2 <= rr + 0.25) this.set(x, y, v);
  }
  segment(a: Pt, b: Pt, r: number, v: number) {
    const d = Math.hypot(b[0] - a[0], b[1] - a[1]);
    const n = Math.max(1, Math.ceil(d / Math.max(r * 0.5, 0.5)));
    for (let k = 0; k <= n; k++) this.disc(a[0] + ((b[0] - a[0]) * k) / n, a[1] + ((b[1] - a[1]) * k) / n, r, v);
  }
  rect(a: Pt, b: Pt, v: number) {
    const [x0, x1] = [Math.round(Math.min(a[0], b[0])), Math.round(Math.max(a[0], b[0]))];
    const [y0, y1] = [Math.round(Math.min(a[1], b[1])), Math.round(Math.max(a[1], b[1]))];
    for (let y = y0; y <= y1; y++) for (let x = x0; x <= x1; x++) this.set(x, y, v);
  }
  end() {
    const cur = this.cur;
    this.cur = null;
    if (!cur || !cur.size) return false;
    const idx = Int32Array.from(cur.keys());
    const before = Uint8Array.from(cur.values());
    const after = new Uint8Array(idx.length);
    idx.forEach((i, k) => { after[k] = this.px[i]; });
    this.undo.push({ idx, before, after });
    if (this.undo.length > 200) this.undo.shift();
    this.redo = [];
    return true;
  }
  apply(e: Edit, values: Uint8Array) {
    e.idx.forEach((i, k) => this.set(i % this.w, Math.floor(i / this.w), values[k]));
  }
  doUndo() { const e = this.undo.pop(); if (e) { this.apply(e, e.before); this.redo.push(e); } }
  doRedo() { const e = this.redo.pop(); if (e) { this.apply(e, e.after); this.undo.push(e); } }
  flush() {
    const d = this.dirty;
    if (!d) return;
    this.dirty = null;
    this.ctx.putImageData(this.image, 0, 0, d[0], d[1], d[2] - d[0] + 1, d[3] - d[1] + 1);
  }
}

export function Editor({ id }: { id: string }) {
  const [meta, setMeta] = useState<MapMeta | null>(null);
  const [error, setError] = useState('');
  const [tool, setTool] = useState<ToolName>('brush');
  const [paint, setPaint] = useState<Paint>('free');
  const [size, setSize] = useState(3);
  const [, setVersion] = useState(0);
  const [busy, setBusy] = useState(false);
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const docRef = useRef<Doc | null>(null);
  const vp = useRef(new Viewport(false, 0.05, 80)).current;
  const st = useRef({ dirty: true, preview: null as null | { kind: 'rect' | 'line'; a: Pt; b: Pt }, hover: null as Pt | null, saved: 0 });
  const opts = useRef({ tool, paint, size });
  opts.current = { tool, paint, size };
  const bump = () => { setVersion((v) => v + 1); st.current.dirty = true; };

  // load
  useEffect(() => {
    let alive = true;
    (async () => {
      try {
        const m = await api<MapMeta>('GET', `/maps/${encodeURIComponent(id)}`);
        const res = await fetch(`/api/maps/${encodeURIComponent(id)}/map.pgm`, { cache: 'no-store' });
        if (!res.ok) throw new Error('could not load map image');
        const pgm = parsePgm(await res.arrayBuffer());
        if (!alive) return;
        docRef.current = new Doc(pgm.width, pgm.height, pgm.pixels);
        setMeta(m);
        requestAnimationFrame(() => {
          fitCanvas(canvasRef.current!, vp);
          vp.fit(0, 0, pgm.width, pgm.height);
          bump();
        });
      } catch (e) { setError((e as Error).message); }
    })();
    return () => { alive = false; };
  }, [id]);

  const unsaved = () => (docRef.current ? docRef.current.undo.length !== st.current.saved : false);
  useEffect(() => {
    const h = (e: BeforeUnloadEvent) => { if (unsaved()) { e.preventDefault(); e.returnValue = ''; } };
    window.addEventListener('beforeunload', h);
    return () => window.removeEventListener('beforeunload', h);
  }, []);

  // canvas: gestures + draw loop
  useEffect(() => {
    if (!meta) return;
    const canvas = canvasRef.current!;
    const s = st.current;
    const cell = (w: Pt): Pt => [Math.floor(w[0]), Math.floor(w[1])];
    // one tool object; it reads the current tool/paint/size on every event
    let last: Pt | null = null;
    const brush: Tool = {
      down: (w) => {
        const doc = docRef.current!, o = opts.current, c = cell(w);
        if (o.tool === 'brush') { doc.begin(); doc.disc(c[0], c[1], (o.size - 1) / 2, VALUES[o.paint]); last = c; }
        else s.preview = { kind: o.tool as 'rect' | 'line', a: c, b: c };
      },
      move: (w, e) => {
        s.hover = w;
        if (!e.buttons) return;
        const doc = docRef.current!, o = opts.current, c = cell(w);
        if (last) { doc.segment(last, c, (o.size - 1) / 2, VALUES[o.paint]); last = c; }
        else if (s.preview) s.preview.b = c;
      },
      up: (w) => {
        const doc = docRef.current!, o = opts.current, c = cell(w);
        if (last) { last = null; if (doc.end()) bump(); return; }
        const p = s.preview;
        if (!p) return;
        s.preview = null;
        doc.begin();
        if (p.kind === 'rect') doc.rect(p.a, c, VALUES[o.paint]);
        else doc.segment(p.a, c, (o.size - 1) / 2, VALUES[o.paint]);
        if (doc.end()) bump();
      },
      cancel: () => {
        s.preview = null;
        if (last) {
          // a second finger arrived: revert the stroke and pinch instead
          last = null;
          const doc = docRef.current!;
          if (doc.end()) { doc.doUndo(); doc.redo.pop(); }
        }
      },
    };
    const detach = attachGestures(canvas, vp, () => { s.dirty = true; },
      () => (docRef.current && opts.current.tool !== 'pan' ? brush : null));
    const leave = () => { s.hover = null; s.dirty = true; };
    canvas.addEventListener('pointerleave', leave);

    let raf = 0;
    const draw = () => {
      raf = requestAnimationFrame(draw);
      const doc = docRef.current;
      if (!doc || (!s.dirty && !doc.dirty)) return;
      s.dirty = false;
      doc.flush();
      const dpr = fitCanvas(canvas, vp);
      const ctx = canvas.getContext('2d')!;
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      const cs = getComputedStyle(document.documentElement);
      ctx.fillStyle = cs.getPropertyValue('--map-bg');
      ctx.fillRect(0, 0, vp.w, vp.h);
      const [x0, y0] = vp.toScreen(0, 0);
      ctx.imageSmoothingEnabled = false;
      ctx.drawImage(doc.canvas, x0, y0, doc.w * vp.scale, doc.h * vp.scale);
      ctx.strokeStyle = cs.getPropertyValue('--border');
      ctx.strokeRect(x0, y0, doc.w * vp.scale, doc.h * vp.scale);

      const o = opts.current;
      const accent = cs.getPropertyValue('--accent');
      const p = s.preview;
      if (p) {
        ctx.strokeStyle = accent;
        ctx.lineWidth = 2;
        if (p.kind === 'rect') {
          const [ax, ay] = vp.toScreen(Math.min(p.a[0], p.b[0]), Math.min(p.a[1], p.b[1]));
          const [bx, by] = vp.toScreen(Math.max(p.a[0], p.b[0]) + 1, Math.max(p.a[1], p.b[1]) + 1);
          ctx.strokeRect(ax, ay, bx - ax, by - ay);
        } else {
          const [ax, ay] = vp.toScreen(p.a[0] + 0.5, p.a[1] + 0.5), [bx, by] = vp.toScreen(p.b[0] + 0.5, p.b[1] + 0.5);
          ctx.lineWidth = Math.max(2, o.size * vp.scale);
          ctx.globalAlpha = 0.6;
          ctx.lineCap = 'round';
          ctx.beginPath(); ctx.moveTo(ax, ay); ctx.lineTo(bx, by); ctx.stroke();
          ctx.globalAlpha = 1;
        }
      }
      if (s.hover && o.tool !== 'pan' && o.tool !== 'rect') {
        const [hx, hy] = vp.toScreen(Math.floor(s.hover[0]) + 0.5, Math.floor(s.hover[1]) + 0.5);
        ctx.strokeStyle = accent;
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        ctx.arc(hx, hy, Math.max(3, (o.size / 2) * vp.scale), 0, Math.PI * 2);
        ctx.stroke();
      }
    };
    raf = requestAnimationFrame(draw);
    const ro = new ResizeObserver(() => { s.dirty = true; });
    ro.observe(canvas);
    return () => {
      cancelAnimationFrame(raf); detach(); ro.disconnect();
      canvas.removeEventListener('pointerleave', leave);
    };
  }, [meta]);

  const undo = () => { docRef.current?.doUndo(); bump(); };
  const redo = () => { docRef.current?.doRedo(); bump(); };
  useEffect(() => {
    const k = (e: KeyboardEvent) => {
      if (!(e.ctrlKey || e.metaKey)) return;
      if (e.key === 'z' && !e.shiftKey) { e.preventDefault(); undo(); }
      else if (e.key === 'y' || (e.key.toLowerCase() === 'z' && e.shiftKey)) { e.preventDefault(); redo(); }
    };
    window.addEventListener('keydown', k);
    return () => window.removeEventListener('keydown', k);
  }, []);

  const save = async (overwrite: boolean) => {
    const doc = docRef.current;
    if (!doc || !meta) return;
    let name: string | null = null;
    if (overwrite) {
      if (!confirm(`Overwrite “${meta.name}”? The original image is replaced (the pose graph is kept).`)) return;
    } else {
      name = prompt('Name for the new map', `${meta.name} (edited)`);
      if (!name) return;
    }
    setBusy(true);
    try {
      const q = new URLSearchParams({ as: overwrite ? 'overwrite' : 'new', width: String(doc.w), height: String(doc.h) });
      if (name) q.set('name', name);
      const r = await api<{ map: MapMeta; warnings: string[] }>('PUT', `/maps/${encodeURIComponent(meta.id)}/image?${q}`, doc.px);
      st.current.saved = doc.undo.length;
      toast(overwrite ? 'Map saved' : `Saved as “${r.map.name}”`, 'ok');
      r.warnings.forEach((w) => toast(w, 'error'));
      if (!overwrite) location.hash = `#/edit/${encodeURIComponent(r.map.id)}`;
      else setMeta(r.map);
    } catch (e) { toast((e as Error).message, 'error'); }
    setBusy(false);
  };

  if (error) return <div class="main"><div class="empty error-text">{error}</div></div>;
  const doc = docRef.current;

  return (
    <div class="editor">
      <div class="editor-bar">
        <a class="btn small" href="#/maps" onClick={(e) => { if (unsaved() && !confirm('Discard unsaved edits?')) e.preventDefault(); }}>← Maps</a>
        <b class="editor-title">{meta ? meta.name : 'Loading…'}</b>
        <span class="spacer" />
        <button class="btn small" disabled={!doc?.undo.length} onClick={undo} title="Undo (Ctrl+Z)">Undo</button>
        <button class="btn small" disabled={!doc?.redo.length} onClick={redo} title="Redo (Ctrl+Shift+Z)">Redo</button>
        <button class="btn small" disabled={busy || !meta} onClick={() => save(false)}>Save as new</button>
        <button class="btn small primary" disabled={busy || !meta} onClick={() => save(true)}>Overwrite</button>
      </div>
      <div class="editor-tools">
        <div class="seg" role="group" aria-label="Tool">
          {(['pan', 'brush', 'line', 'rect'] as ToolName[]).map((t) => (
            <button key={t} class={tool === t ? 'on' : ''} onClick={() => setTool(t)}>
              {t === 'pan' ? 'Pan' : t === 'brush' ? 'Brush' : t === 'line' ? 'Wall' : 'Rect'}
            </button>
          ))}
        </div>
        <div class="seg" role="group" aria-label="Paint">
          {(['free', 'occupied', 'unknown'] as Paint[]).map((p) => (
            <button key={p} class={paint === p ? 'on' : ''} onClick={() => setPaint(p)}>
              <span class={'swatch swatch-' + p} />{p === 'free' ? 'Free' : p === 'occupied' ? 'Occupied' : 'Unknown'}
            </button>
          ))}
        </div>
        <label class="size">Size <input type="range" min={1} max={25} value={size} onInput={(e) => setSize(+(e.target as HTMLInputElement).value)} /> <span>{size}px{meta ? ` · ${(size * meta.resolution * 100).toFixed(0)} cm` : ''}</span></label>
      </div>
      <div class="editor-canvas">
        <canvas ref={canvasRef} class={tool === 'pan' ? 'grab' : 'crosshair'} />
        <div class="editor-hint">Two fingers (or the Pan tool) to move · pinch or wheel to zoom. Edits affect AMCL localization, not the pose graph.</div>
      </div>
    </div>
  );
}
