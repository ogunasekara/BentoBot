import { ComponentChildren } from 'preact';

export function Dot({ ok, warn }: { ok: boolean | null | undefined; warn?: boolean }) {
  const cls = ok == null ? 'dot dot-unknown' : ok ? (warn ? 'dot dot-warn' : 'dot dot-ok') : 'dot dot-bad';
  return <span class={cls} aria-label={ok == null ? 'unknown' : ok ? 'ok' : 'down'} />;
}

export function Card({ title, children, right, wide, span2 }: { title: string; children: ComponentChildren; right?: ComponentChildren; wide?: boolean; span2?: boolean }) {
  return (
    <section class={'card' + (wide ? ' card-wide' : '') + (span2 ? ' card-2' : '')}>
      <div class="card-head"><h2>{title}</h2>{right}</div>
      {children}
    </section>
  );
}

/** Centred bar for a value in [-1, 1] (stick axes, velocities). */
export function CenterBar({ value, max = 1 }: { value: number; max?: number }) {
  const v = Math.max(-1, Math.min(1, value / max));
  const left = v < 0 ? 50 + v * 50 : 50;
  return (
    <div class="cbar">
      <div class="cbar-fill" style={{ left: `${left}%`, width: `${Math.abs(v) * 50}%` }} />
      <div class="cbar-mid" />
    </div>
  );
}

export function Meter({ frac, warn = 0.75, bad = 0.9 }: { frac: number; warn?: number; bad?: number }) {
  const f = Math.max(0, Math.min(1, frac));
  return (
    <div class="meter">
      <div class={'meter-fill' + (f >= bad ? ' bad' : f >= warn ? ' warn' : '')} style={{ width: `${f * 100}%` }} />
    </div>
  );
}

export function KV({ k, children }: { k: string; children: ComponentChildren }) {
  return <div class="kv"><span class="k">{k}</span><span class="v">{children}</span></div>;
}
