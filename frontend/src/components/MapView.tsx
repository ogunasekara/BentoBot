import { useEffect, useRef, useState } from 'preact/hooks';
import { live } from '../live';
import { Pt, Tool, Viewport, attachGestures, fitCanvas, niceScaleBar } from '../viewport';

interface Props {
  /** click-and-drag sets a pose (position on press, heading by dragging) */
  poseTool?: boolean;
  onPose?: (x: number, y: number, yaw: number) => void;
}

const ROBOT_RADIUS = 0.09;

function css(name: string) {
  return getComputedStyle(document.documentElement).getPropertyValue(name).trim();
}

export function MapView({ poseTool, onPose }: Props) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const [follow, setFollow] = useState(true);
  const state = useRef({ follow: true, fitted: '', drag: null as null | { from: Pt; to: Pt }, dirty: true });
  const vp = useRef(new Viewport(true, 5, 2000)).current;
  const toolRef = useRef<Tool | null>(null);

  state.current.follow = follow;

  // pose tool (initial pose)
  toolRef.current = poseTool ? {
    down: (w) => { state.current.drag = { from: w, to: w }; },
    move: (w) => { if (state.current.drag) state.current.drag.to = w; },
    up: (w) => {
      const d = state.current.drag;
      state.current.drag = null;
      if (!d) return;
      const dx = w[0] - d.from[0], dy = w[1] - d.from[1];
      const yaw = Math.hypot(dx, dy) > 0.05 ? Math.atan2(dy, dx) : (live.telemetry.pose?.yaw ?? 0);
      onPose?.(d.from[0], d.from[1], yaw);
    },
    cancel: () => { state.current.drag = null; },
  } : null;

  useEffect(() => {
    const canvas = canvasRef.current!;
    const st = state.current;
    const detach = attachGestures(canvas, vp, () => { st.dirty = true; }, () => toolRef.current,
      () => { if (st.follow) setFollow(false); });
    let raf = 0;
    let lastVersion = -1;
    const ro = new ResizeObserver(() => { st.dirty = true; });
    ro.observe(canvas);

    const draw = () => {
      raf = requestAnimationFrame(draw);
      if (!st.dirty && live.version === lastVersion) return;
      st.dirty = false;
      lastVersion = live.version;
      const dpr = fitCanvas(canvas, vp);
      const ctx = canvas.getContext('2d')!;
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      ctx.fillStyle = css('--map-bg') || '#ddd';
      ctx.fillRect(0, 0, vp.w, vp.h);

      const map = live.map;
      const pose = live.telemetry.pose;

      // fit to the map once per map source, or centre on the robot
      const key = map ? `${map.source}:${map.map_id || ''}` : 'none';
      if (map && st.fitted !== key) {
        st.fitted = key;
        const [ox, oy] = map.origin;
        vp.fit(ox, oy, ox + map.width * map.resolution, oy + map.height * map.resolution);
        if (vp.scale > 120) vp.scale = 120;
      }
      if (st.follow && pose) { vp.cx = pose.x; vp.cy = pose.y; }

      if (map) {
        const [ox, oy] = map.origin;
        const [sx, sy] = vp.toScreen(ox, oy + map.height * map.resolution);
        ctx.imageSmoothingEnabled = vp.scale * map.resolution < 2;
        ctx.drawImage(map.image, sx, sy, map.width * map.resolution * vp.scale, map.height * map.resolution * vp.scale);
      } else {
        // a 1 m grid so the robot's motion is still visible without a map
        ctx.strokeStyle = css('--grid');
        ctx.lineWidth = 1;
        const [x0, y1] = vp.toWorld(0, 0), [x1, y0] = vp.toWorld(vp.w, vp.h);
        const step = vp.scale < 15 ? 5 : 1;
        ctx.beginPath();
        for (let x = Math.floor(x0 / step) * step; x <= x1; x += step) {
          const [px] = vp.toScreen(x, 0); ctx.moveTo(px, 0); ctx.lineTo(px, vp.h);
        }
        for (let y = Math.floor(y0 / step) * step; y <= y1; y += step) {
          const [, py] = vp.toScreen(0, y); ctx.moveTo(0, py); ctx.lineTo(vp.w, py);
        }
        ctx.stroke();
      }

      // driven path
      const path = live.path;
      if (path.length > 1 && (!pose || live.pathFrame === pose.frame)) {
        ctx.strokeStyle = css('--path');
        ctx.lineWidth = 2;
        ctx.lineJoin = 'round';
        ctx.beginPath();
        path.forEach(([x, y], i) => {
          const [px, py] = vp.toScreen(x, y);
          if (i) ctx.lineTo(px, py); else ctx.moveTo(px, py);
        });
        ctx.stroke();
      }

      // latest scan
      const scan = live.scan;
      if (scan && pose && scan.pose.frame === pose.frame && performance.now() - scan.t < 2000) {
        ctx.fillStyle = css('--scan');
        const { x, y, yaw } = scan.pose;
        const sz = Math.max(2, Math.min(4, vp.scale * 0.03));
        for (let i = 0; i < scan.ranges.length; i++) {
          const r = scan.ranges[i];
          if (!r) continue;
          const a = yaw + scan.angle_min + i * scan.angle_increment;
          const [px, py] = vp.toScreen(x + r * Math.cos(a), y + r * Math.sin(a));
          ctx.fillRect(px - sz / 2, py - sz / 2, sz, sz);
        }
      }

      // localization uncertainty (2 sigma)
      const amcl = live.telemetry.amcl;
      if (amcl && pose && pose.frame === 'map') {
        const [px, py] = vp.toScreen(amcl.x, amcl.y);
        ctx.strokeStyle = css('--accent');
        ctx.fillStyle = css('--accent-soft');
        ctx.beginPath();
        ctx.ellipse(px, py, Math.max(2 * amcl.sx * vp.scale, 3), Math.max(2 * amcl.sy * vp.scale, 3), 0, 0, Math.PI * 2);
        ctx.fill();
        ctx.stroke();
      }

      if (pose) drawRobot(ctx, vp, pose.x, pose.y, pose.yaw, css('--robot'), pose.age > 2);

      const drag = st.drag;
      if (drag) {
        const [fx, fy] = drag.from, [tx, ty] = drag.to;
        const yaw = Math.hypot(tx - fx, ty - fy) > 0.05 ? Math.atan2(ty - fy, tx - fx) : (pose?.yaw ?? 0);
        drawRobot(ctx, vp, fx, fy, yaw, css('--accent'), false, true);
      }

      // scale bar
      const len = niceScaleBar(vp.scale);
      const px = len * vp.scale;
      ctx.fillStyle = css('--fg');
      ctx.fillRect(12, vp.h - 16, px, 3);
      ctx.font = '12px system-ui, sans-serif';
      ctx.fillText(len >= 1 ? `${len} m` : `${Math.round(len * 100)} cm`, 12, vp.h - 22);
    };
    raf = requestAnimationFrame(draw);
    return () => { cancelAnimationFrame(raf); detach(); ro.disconnect(); };
  }, []);

  useEffect(() => { state.current.dirty = true; }, [follow, poseTool]);

  const fit = () => {
    const map = live.map;
    setFollow(false);
    if (map) {
      const [ox, oy] = map.origin;
      vp.fit(ox, oy, ox + map.width * map.resolution, oy + map.height * map.resolution);
    }
    state.current.dirty = true;
  };
  const zoom = (f: number) => { vp.zoomAt(vp.w / 2, vp.h / 2, f); state.current.dirty = true; };

  return (
    <div class={'mapview' + (poseTool ? ' tool-active' : '')}>
      <canvas ref={canvasRef} />
      <div class="map-controls">
        <button class={'icon-btn' + (follow ? ' on' : '')} title="Follow robot" onClick={() => setFollow(!follow)}>⌖</button>
        <button class="icon-btn" title="Fit map" onClick={fit}>⤢</button>
        <button class="icon-btn" title="Zoom in" onClick={() => zoom(1.4)}>+</button>
        <button class="icon-btn" title="Zoom out" onClick={() => zoom(1 / 1.4)}>−</button>
      </div>
    </div>
  );
}

function drawRobot(ctx: CanvasRenderingContext2D, vp: Viewport, x: number, y: number, yaw: number,
                   color: string, stale: boolean, ghost = false) {
  const [px, py] = vp.toScreen(x, y);
  const r = Math.max(ROBOT_RADIUS * vp.scale, 8);
  ctx.save();
  ctx.translate(px, py);
  ctx.rotate(-yaw);
  ctx.globalAlpha = stale ? 0.4 : ghost ? 0.8 : 1;
  ctx.fillStyle = color;
  ctx.strokeStyle = '#fff';
  ctx.lineWidth = 2;
  // heading nose, then the body over its base
  ctx.beginPath();
  ctx.moveTo(r * 2, 0);
  ctx.lineTo(r * 0.2, -r * 0.8);
  ctx.lineTo(r * 0.2, r * 0.8);
  ctx.closePath();
  ctx.fill();
  ctx.stroke();
  ctx.beginPath();
  ctx.arc(0, 0, r, 0, Math.PI * 2);
  ctx.fill();
  ctx.stroke();
  ctx.beginPath();
  ctx.arc(r * 0.45, 0, r * 0.28, 0, Math.PI * 2);
  ctx.fillStyle = '#fff';
  ctx.fill();
  ctx.restore();
}
