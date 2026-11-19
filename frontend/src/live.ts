// Live data from the backend websocket (/ws). Components subscribe with
// useLive(); the map canvas reads `live` directly in its draw loop.
import { useEffect, useState } from 'preact/hooks';

export interface Pose { frame: string; x: number; y: number; yaw: number; age: number }
export interface TopicStat { rate: number; age: number | null; stale: boolean; count: number }
export interface ModeState {
  mode: string; label: string; phase: string; map_id: string | null;
  started_at: number | null; elapsed: number | null; external: boolean; error: string | null;
  modes: { name: string; label: string; needs_map: boolean }[];
}
export interface Controller {
  present: boolean; dev: string; name?: string; uniq?: string; key?: string;
  bluetooth: { address: string; name: string }[]; profile: string | null;
}
export interface Status {
  time: number;
  mode: ModeState;
  nodes: Record<string, boolean>;
  all_nodes: string[];
  topics: Record<string, TopicStat>;
  system: {
    cpu: number; cpus: number[]; load: number[] | null;
    mem: { used: number; total: number; percent: number };
    temps: Record<string, number>;
    disk: { free: number; total: number; path: string } | null;
  };
  controller: Controller;
  teleop_enabled: boolean | null;
  map: { available: boolean; source: string | null };
}
export interface Telemetry {
  pose?: Pose | null;
  odom?: { x: number; y: number; yaw: number; v: number; w: number };
  mcu_info?: { left: number; right: number; gyro_z: number };
  mcu_cmd?: { v: number; w: number };
  joy?: { axes: number[]; buttons: number[] };
  amcl?: { x: number; y: number; yaw: number; sx: number; sy: number; syaw: number };
  teleop_enabled?: boolean | null;
}
export interface MapLayer {
  width: number; height: number; resolution: number; origin: number[];
  source: string; map_id?: string; image: HTMLCanvasElement; gray: Uint8Array; stamp: number;
}
export interface Scan { angle_min: number; angle_increment: number; pose: Pose; ranges: Float32Array; t: number }

export const live = {
  connected: false,
  status: null as Status | null,
  telemetry: {} as Telemetry,
  map: null as MapLayer | null,
  scan: null as Scan | null,
  path: [] as [number, number][],
  pathFrame: null as string | null,
  version: 0,
};

const listeners = new Set<() => void>();
function notify() {
  live.version++;
  listeners.forEach((l) => l());
}

export function useLive() {
  const [, setV] = useState(0);
  useEffect(() => {
    let pending = false;
    const l = () => {
      // coalesce bursts into one render per frame
      if (pending) return;
      pending = true;
      requestAnimationFrame(() => { pending = false; setV((v) => v + 1); });
    };
    listeners.add(l);
    return () => { listeners.delete(l); };
  }, []);
  return live;
}

// Map colours follow the theme; recoloured whenever a map arrives.
function themeColors(): [number[], number[], number[]] {
  const css = getComputedStyle(document.documentElement);
  const parse = (name: string, fb: number[]) => {
    const v = css.getPropertyValue(name).trim();
    const m = v.match(/^#([0-9a-f]{6})$/i);
    if (!m) return fb;
    const n = parseInt(m[1], 16);
    return [(n >> 16) & 255, (n >> 8) & 255, n & 255];
  };
  return [parse('--map-free', [250, 250, 247]), parse('--map-occ', [30, 34, 40]), parse('--map-unknown', [200, 202, 205])];
}

export function grayToCanvas(gray: Uint8Array | Uint8ClampedArray, w: number, h: number, canvas?: HTMLCanvasElement) {
  const c = canvas || document.createElement('canvas');
  c.width = w; c.height = h;
  const ctx = c.getContext('2d')!;
  const img = ctx.createImageData(w, h);
  paintGray(img.data, gray, 0, gray.length);
  ctx.putImageData(img, 0, 0);
  return c;
}

let colors = themeColors();
export function refreshColors() { colors = themeColors(); }

// gray (0 = occupied, 254 = free, 205 = unknown) -> RGBA
export function paintGray(out: Uint8ClampedArray, gray: ArrayLike<number>, from: number, to: number) {
  const [free, occ, unk] = colors;
  for (let i = from; i < to; i++) {
    const g = gray[i];
    const c = g >= 250 ? free : g <= 50 ? occ : unk;
    const o = i * 4;
    out[o] = c[0]; out[o + 1] = c[1]; out[o + 2] = c[2]; out[o + 3] = 255;
  }
}

async function decodeMap(header: any, png: Uint8Array) {
  const blob = new Blob([png], { type: 'image/png' });
  const url = URL.createObjectURL(blob);
  try {
    const img = new Image();
    img.src = url;
    await img.decode();
    const c = document.createElement('canvas');
    c.width = img.width; c.height = img.height;
    const ctx = c.getContext('2d')!;
    ctx.drawImage(img, 0, 0);
    const data = ctx.getImageData(0, 0, c.width, c.height);
    const gray = new Uint8Array(c.width * c.height);
    for (let i = 0; i < gray.length; i++) gray[i] = data.data[i * 4];
    grayToCanvas(gray, c.width, c.height, c);
    live.map = { ...header, image: c, gray };
    notify();
  } finally {
    URL.revokeObjectURL(url);
  }
}

let ws: WebSocket | null = null;
let retry = 500;

export function connect() {
  const proto = location.protocol === 'https:' ? 'wss' : 'ws';
  ws = new WebSocket(`${proto}://${location.host}/ws`);
  ws.binaryType = 'arraybuffer';
  ws.onopen = () => { live.connected = true; retry = 500; notify(); };
  ws.onclose = () => {
    live.connected = false;
    notify();
    setTimeout(connect, retry);
    retry = Math.min(retry * 2, 5000);
  };
  ws.onmessage = (ev) => {
    if (typeof ev.data === 'string') {
      const m = JSON.parse(ev.data);
      if (m.type === 'status') live.status = m;
      else if (m.type === 'telemetry') live.telemetry = m;
      else if (m.type === 'path') {
        if (m.reset) live.path = [];
        live.path.push(...m.points);
        live.pathFrame = m.frame;
      }
      notify();
      return;
    }
    const buf = ev.data as ArrayBuffer;
    const hlen = new DataView(buf).getUint32(0, true);
    const header = JSON.parse(new TextDecoder().decode(new Uint8Array(buf, 4, hlen)));
    const payload = buf.slice(4 + hlen);
    if (header.type === 'scan') {
      live.scan = { ...header, ranges: new Float32Array(payload), t: performance.now() };
      notify();
    } else if (header.type === 'map') {
      if (header.clear) { live.map = null; notify(); }
      else decodeMap(header, new Uint8Array(payload));
    }
  };
}

window.matchMedia?.('(prefers-color-scheme: dark)').addEventListener?.('change', () => {
  refreshColors();
  const m = live.map;
  if (m) { grayToCanvas(m.gray, m.width, m.height, m.image); notify(); }
});
