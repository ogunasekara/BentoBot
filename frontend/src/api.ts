export class ApiError extends Error {}

export async function api<T = any>(method: string, path: string, body?: unknown): Promise<T> {
  const init: RequestInit = { method, headers: {} };
  if (body instanceof Uint8Array) {
    init.body = body;
    (init.headers as Record<string, string>)['Content-Type'] = 'application/octet-stream';
  } else if (body !== undefined) {
    init.body = JSON.stringify(body);
    (init.headers as Record<string, string>)['Content-Type'] = 'application/json';
  }
  let res: Response;
  try {
    res = await fetch('/api' + path, init);
  } catch {
    throw new ApiError('robot unreachable');
  }
  const text = await res.text();
  let data: any = null;
  try { data = text ? JSON.parse(text) : null; } catch { data = text; }
  if (!res.ok) throw new ApiError((data && data.error) || `${res.status} ${res.statusText}`);
  return data as T;
}

export interface MapMeta {
  id: string; name: string; created: string; modified: string;
  duration_s: number | null; resolution: number; width: number; height: number;
  origin: number[]; size_bytes: number; has_posegraph: boolean;
  source: string; parent?: string; edited?: boolean; version?: number;
}

export const fmt = {
  bytes(n: number) {
    if (n < 1024) return `${n} B`;
    const u = ['KB', 'MB', 'GB', 'TB'];
    let i = -1;
    do { n /= 1024; i++; } while (n >= 1024 && i < u.length - 1);
    return `${n.toFixed(n < 10 ? 1 : 0)} ${u[i]}`;
  },
  duration(s: number | null | undefined) {
    if (s == null) return '–';
    s = Math.round(s);
    const h = Math.floor(s / 3600), m = Math.floor((s % 3600) / 60), r = s % 60;
    return h ? `${h}h ${m}m` : m ? `${m}m ${r.toString().padStart(2, '0')}s` : `${r}s`;
  },
  date(iso: string) {
    const d = new Date(iso);
    return d.toLocaleString(undefined, { year: 'numeric', month: 'short', day: 'numeric', hour: '2-digit', minute: '2-digit' });
  },
  num(v: number | undefined | null, digits = 2) {
    return v == null || !isFinite(v) ? '–' : v.toFixed(digits);
  },
  deg(rad: number | undefined | null) {
    return rad == null ? '–' : `${((rad * 180) / Math.PI).toFixed(0)}°`;
  },
};
