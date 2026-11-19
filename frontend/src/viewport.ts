// Pan / zoom for canvas views (mouse wheel, drag, pinch). World coordinates
// are metres with y up (live map) or pixels with y down (editor).

export type Pt = [number, number];

export class Viewport {
  cx = 0;
  cy = 0;
  scale = 40; // screen px per world unit
  w = 1;
  h = 1;
  constructor(public yUp: boolean, public minScale = 2, public maxScale = 2000) {}

  toScreen(x: number, y: number): Pt {
    return [this.w / 2 + (x - this.cx) * this.scale, this.h / 2 + (this.yUp ? -1 : 1) * (y - this.cy) * this.scale];
  }
  toWorld(px: number, py: number): Pt {
    return [this.cx + (px - this.w / 2) / this.scale, this.cy + (this.yUp ? -1 : 1) * (py - this.h / 2) / this.scale];
  }
  zoomAt(px: number, py: number, factor: number) {
    const [wx, wy] = this.toWorld(px, py);
    this.scale = Math.min(this.maxScale, Math.max(this.minScale, this.scale * factor));
    const [nx, ny] = this.toWorld(px, py);
    this.cx += wx - nx;
    this.cy += wy - ny;
  }
  panBy(dx: number, dy: number) {
    this.cx -= dx / this.scale;
    this.cy -= (this.yUp ? -1 : 1) * dy / this.scale;
  }
  fit(x0: number, y0: number, x1: number, y1: number, pad = 24) {
    const ww = Math.max(x1 - x0, 1e-6), hh = Math.max(y1 - y0, 1e-6);
    this.scale = Math.min(this.maxScale, Math.max(this.minScale,
      Math.min((this.w - 2 * pad) / ww, (this.h - 2 * pad) / hh)));
    this.cx = (x0 + x1) / 2;
    this.cy = (y0 + y1) / 2;
  }
}

export interface Tool {
  down(world: Pt, e: PointerEvent): void;
  move(world: Pt, e: PointerEvent): void;
  up(world: Pt, e: PointerEvent): void;
  cancel(): void;
}

/** Wires pointer + wheel input. `tool()` returns the active tool, or null to pan. */
export function attachGestures(canvas: HTMLCanvasElement, vp: Viewport, changed: () => void,
                               tool: () => Tool | null, onUserPan?: () => void) {
  const pts = new Map<number, Pt>();
  let active: Tool | null = null;
  let pinch: { d: number; mid: Pt } | null = null;

  const local = (e: PointerEvent | WheelEvent): Pt => {
    const r = canvas.getBoundingClientRect();
    return [e.clientX - r.left, e.clientY - r.top];
  };
  const pinchState = () => {
    const [a, b] = [...pts.values()];
    return { d: Math.hypot(a[0] - b[0], a[1] - b[1]), mid: [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2] as Pt };
  };

  const down = (e: PointerEvent) => {
    canvas.setPointerCapture(e.pointerId);
    pts.set(e.pointerId, local(e));
    if (pts.size === 1) {
      const t = e.button === 0 ? tool() : null;
      if (t) { active = t; t.down(vp.toWorld(...local(e)), e); changed(); }
    } else if (pts.size === 2) {
      if (active) { active.cancel(); active = null; }
      pinch = pinchState();
    }
    e.preventDefault();
  };
  const move = (e: PointerEvent) => {
    const prev = pts.get(e.pointerId);
    if (!prev) {
      // hover (mouse): let tools show a cursor preview
      const t = tool();
      if (t && e.pointerType === 'mouse') { t.move(vp.toWorld(...local(e)), e); changed(); }
      return;
    }
    const p = local(e);
    pts.set(e.pointerId, p);
    if (active) {
      active.move(vp.toWorld(...p), e);
    } else if (pts.size === 1) {
      vp.panBy(p[0] - prev[0], p[1] - prev[1]);
      onUserPan?.();
    } else if (pts.size === 2 && pinch) {
      const n = pinchState();
      vp.zoomAt(n.mid[0], n.mid[1], n.d / Math.max(pinch.d, 1));
      vp.panBy(n.mid[0] - pinch.mid[0], n.mid[1] - pinch.mid[1]);
      pinch = n;
      onUserPan?.();
    }
    changed();
  };
  const up = (e: PointerEvent) => {
    if (!pts.has(e.pointerId)) return;
    pts.delete(e.pointerId);
    if (active && pts.size === 0) {
      const t = active;
      active = null;
      if (e.type === 'pointercancel') t.cancel(); else t.up(vp.toWorld(...local(e)), e);
    }
    pinch = pts.size === 2 ? pinchState() : null;
    changed();
  };
  const wheel = (e: WheelEvent) => {
    e.preventDefault();
    const [x, y] = local(e);
    vp.zoomAt(x, y, Math.exp(-e.deltaY * (e.deltaMode === 1 ? 0.05 : 0.0015)));
    onUserPan?.();
    changed();
  };

  canvas.addEventListener('pointerdown', down);
  canvas.addEventListener('pointermove', move);
  canvas.addEventListener('pointerup', up);
  canvas.addEventListener('pointercancel', up);
  canvas.addEventListener('wheel', wheel, { passive: false });
  return () => {
    canvas.removeEventListener('pointerdown', down);
    canvas.removeEventListener('pointermove', move);
    canvas.removeEventListener('pointerup', up);
    canvas.removeEventListener('pointercancel', up);
    canvas.removeEventListener('wheel', wheel);
  };
}

/** Keeps a canvas' backing store matched to its CSS size and devicePixelRatio. */
export function fitCanvas(canvas: HTMLCanvasElement, vp: Viewport) {
  const r = canvas.getBoundingClientRect();
  const dpr = Math.min(window.devicePixelRatio || 1, 2);
  const w = Math.max(1, Math.round(r.width * dpr)), h = Math.max(1, Math.round(r.height * dpr));
  if (canvas.width !== w || canvas.height !== h) { canvas.width = w; canvas.height = h; }
  vp.w = r.width;
  vp.h = r.height;
  return dpr;
}

export function niceScaleBar(pxPerUnit: number, targetPx = 90): number {
  const raw = targetPx / pxPerUnit;
  const p = Math.pow(10, Math.floor(Math.log10(raw)));
  for (const m of [1, 2, 5, 10]) if (m * p >= raw) return m * p;
  return 10 * p;
}
