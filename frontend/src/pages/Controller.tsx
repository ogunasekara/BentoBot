import { useEffect, useRef, useState } from 'preact/hooks';
import { api } from '../api';
import { useLive } from '../live';
import { Card, CenterBar, Dot, KV } from '../components/Bits';
import { toast } from '../components/Toasts';

interface BtStatus { available: boolean; error?: string; address?: string; name?: string; powered?: boolean; discovering?: boolean }
interface BtDevice {
  address: string; name: string; paired: boolean; trusted: boolean; connected: boolean;
  rssi: number | null; icon: string; gamepad: boolean; busy: string | null;
}
interface TeleopParams {
  axis_linear: number; axis_angular: number; scale_linear: number; scale_angular: number;
  invert_linear: boolean; invert_angular: boolean; deadzone: number;
}

function HostSetup() {
  return (
    <details class="help">
      <summary>One-time host setup</summary>
      <ol>
        <li>Run bluetoothd on the Jetson: <code>sudo systemctl enable --now bluetooth</code></li>
        <li>Make sure the adapter isn't blocked: <code>rfkill list</code>, then <code>sudo rfkill unblock bluetooth</code></li>
        <li>The container needs <code>/var/run/dbus</code> mounted (docker-compose.yml does this).</li>
        <li>Some gamepads (e.g. DualShock 4, 8BitDo, some generic pads) won't connect while
          <code>ClassicBondedOnly=true</code>. In <code>/etc/bluetooth/input.conf</code> set
          <code>ClassicBondedOnly=false</code> (also present in JetPack 4.x's patched BlueZ 5.48), then <code>sudo systemctl restart bluetooth</code>.</li>
        <li>On BlueZ ≥ 5.56, <code>UserspaceHID=true</code> in the same file uses uhid instead of the kernel HID driver, which helps some BLE pads.</li>
        <li>Xbox One/Series pads over Bluetooth may need ERTM disabled on older kernels:
          <code>echo 1 | sudo tee /sys/module/bluetooth/parameters/disable_ertm</code></li>
      </ol>
    </details>
  );
}

function Bluetooth() {
  const [status, setStatus] = useState<BtStatus | null>(null);
  const [devices, setDevices] = useState<BtDevice[]>([]);
  const [busy, setBusy] = useState<string | null>(null);
  const [showAll, setShowAll] = useState(false);

  const load = () => api<{ status: BtStatus; devices: BtDevice[] }>('GET', '/bt/devices')
    .then((r) => { setStatus(r.status); setDevices(r.devices); })
    .catch((e) => setStatus({ available: false, error: e.message }));
  useEffect(() => {
    load();
    const t = setInterval(load, 2000);
    return () => clearInterval(t);
  }, []);

  const run = async (label: string, fn: () => Promise<any>) => {
    setBusy(label);
    try {
      const r = await fn();
      const js = r && r.joystick;
      if (js) {
        if (!js.js0) toast('Connected, but /dev/input/js0 did not appear. Check the host setup below.', 'error');
        else if (!js.joy) toast(`${js.name || 'Controller'} is js0, but joy_linux isn't publishing /joy${js.restarted ? ' (even after a restart)' : ''}.`, 'error');
        else toast(`${js.name || 'Controller'} is ready on /dev/input/js0${js.restarted ? ' (joy restarted)' : ''}`, 'ok');
      }
    } catch (e) { toast((e as Error).message, 'error'); }
    setBusy(null);
    load();
  };
  const act = (d: BtDevice, action: string) =>
    run(d.address + action, () => api('POST', `/bt/devices/${d.address}/${action}`));

  if (!status) return <Card title="Bluetooth" wide><p class="muted">Checking BlueZ…</p></Card>;
  if (!status.available) {
    return (
      <Card title="Bluetooth" wide>
        <p class="error-text">{status.error}</p>
        <HostSetup />
      </Card>
    );
  }
  const paired = devices.filter((d) => d.paired);
  const nearby = devices.filter((d) => !d.paired && (showAll || d.gamepad || d.name !== d.address));

  const row = (d: BtDevice) => (
    <li key={d.address} class="bt-dev">
      <div class="bt-name">
        <Dot ok={d.connected ? true : d.paired ? null : null} />
        <div>
          <b>{d.name}</b>{d.gamepad && <span class="tag">gamepad</span>}
          <div class="muted small">{d.address}{d.rssi != null ? ` · ${d.rssi} dBm` : ''}{d.paired ? ' · paired' : ''}{d.trusted ? ' · trusted' : ''}{d.connected ? ' · connected' : ''}</div>
        </div>
      </div>
      <div class="actions">
        {d.busy || (busy && busy.startsWith(d.address)) ? <span class="muted">{d.busy || 'working'}…</span> : <>
          {!d.paired && <button class="btn small primary" disabled={!!busy} onClick={() => act(d, 'pair')}>Pair</button>}
          {d.paired && !d.connected && <button class="btn small primary" disabled={!!busy} onClick={() => act(d, 'connect')}>Connect</button>}
          {d.connected && <button class="btn small" disabled={!!busy} onClick={() => act(d, 'disconnect')}>Disconnect</button>}
          {d.paired && !d.trusted && <button class="btn small" disabled={!!busy} onClick={() => act(d, 'trust')}>Trust</button>}
          {d.paired && <button class="btn small danger-outline" disabled={!!busy}
            onClick={() => confirm(`Forget ${d.name}?`) && run(d.address + 'remove', () => api('DELETE', `/bt/devices/${d.address}`))}>Forget</button>}
        </>}
      </div>
    </li>
  );

  return (
    <Card title="Bluetooth" wide right={
      <div class="row">
        {!status.powered && <button class="btn small" onClick={() => run('power', () => api('POST', '/bt/power', { on: true }))}>Turn on</button>}
        <button class={'btn small' + (status.discovering ? '' : ' primary')} disabled={busy === 'scan'}
                onClick={() => run('scan', () => api('POST', '/bt/scan', { on: !status.discovering }))}>
          {status.discovering ? 'Stop scan' : 'Scan'}
        </button>
      </div>}>
      <p class="muted small">Adapter {status.name} ({status.address}) · {status.powered ? 'on' : 'off'}{status.discovering ? ' · scanning (30 s)…' : ''}.
        Put the controller in pairing mode, then scan.</p>
      <h3>Paired</h3>
      {paired.length ? <ul class="bt-list">{paired.map(row)}</ul> : <p class="muted">No paired devices.</p>}
      <h3>Nearby <label class="check small"><input type="checkbox" checked={showAll} onChange={(e) => setShowAll((e.target as HTMLInputElement).checked)} /> show unnamed</label></h3>
      {nearby.length ? <ul class="bt-list">{nearby.map(row)}</ul> : <p class="muted">{status.discovering ? 'Scanning…' : 'Nothing found yet. Press Scan.'}</p>}
      <HostSetup />
    </Card>
  );
}

function Calibration() {
  const live = useLive();
  const c = live.status?.controller;
  const axes = live.telemetry.joy?.axes || [];
  const [params, setParams] = useState<TeleopParams | null>(null);
  const [error, setError] = useState('');
  const [detect, setDetect] = useState<null | 'linear' | 'angular'>(null);
  const detectRef = useRef<{ base: number[]; max: number[]; until: number } | null>(null);
  const [busy, setBusy] = useState(false);

  const load = () => api<TeleopParams>('GET', '/teleop/params').then((p) => { setParams(p); setError(''); }).catch((e) => setError(e.message));
  useEffect(() => { load(); }, [c?.key, c?.profile]);

  // "detect": the axis that moves the most in 3 s wins
  useEffect(() => {
    const d = detectRef.current;
    if (!detect || !d || !params) return;
    axes.forEach((a, i) => { d.max[i] = Math.max(d.max[i] || 0, Math.abs(a - (d.base[i] || 0))); });
    if (performance.now() > d.until) {
      const best = d.max.reduce((bi, v, i, arr) => (v > (arr[bi] || 0) ? i : bi), 0);
      if ((d.max[best] || 0) < 0.3) toast('No axis moved enough. Try again and push the stick all the way.', 'error');
      else {
        // pushing forward / left should give positive values
        const signed = axes[best] - (d.base[best] || 0);
        const invert = Math.abs(signed) > 0.2 ? signed < 0 : params[detect === 'linear' ? 'invert_linear' : 'invert_angular'];
        setParams({ ...params, ...(detect === 'linear' ? { axis_linear: best, invert_linear: invert } : { axis_angular: best, invert_angular: invert }) });
        toast(`${detect === 'linear' ? 'Linear' : 'Angular'} axis: ${best}${invert ? ' (inverted)' : ''}. Click Apply to use it.`, 'ok');
      }
      detectRef.current = null;
      setDetect(null);
    }
  });
  const startDetect = (which: 'linear' | 'angular') => {
    detectRef.current = { base: [...axes], max: [], until: performance.now() + 3000 };
    setDetect(which);
  };

  const apply = async () => {
    if (!params) return;
    setBusy(true);
    try { await api('PUT', '/teleop/params', params); toast('Applied to bentobot_teleop', 'ok'); }
    catch (e) { toast((e as Error).message, 'error'); }
    setBusy(false);
  };
  const saveProfile = async () => {
    if (!params || !c?.key) return;
    setBusy(true);
    try {
      await api('PUT', `/controllers/${encodeURIComponent(c.key)}`, { name: c.name || c.key, params });
      toast(`Saved for ${c.name || c.key}. It's applied whenever this controller connects.`, 'ok');
    } catch (e) { toast((e as Error).message, 'error'); }
    setBusy(false);
  };
  const set = <K extends keyof TeleopParams>(k: K, v: TeleopParams[K]) => params && setParams({ ...params, [k]: v });
  const num = (k: keyof TeleopParams, step: number) => (
    <input type="number" step={step} value={params ? (params[k] as number) : ''}
           onInput={(e) => { const v = parseFloat((e.target as HTMLInputElement).value); if (isFinite(v)) set(k, v as never); }} />
  );
  const axisSelect = (k: 'axis_linear' | 'axis_angular') => (
    <select value={params ? params[k] : 0} onChange={(e) => set(k, +(e.target as HTMLSelectElement).value)}>
      {Array.from({ length: Math.max(axes.length, (params?.[k] ?? 0) + 1, 8) }, (_, i) => <option key={i} value={i}>axis {i}</option>)}
    </select>
  );

  const linV = params && axes[params.axis_linear] != null ? axes[params.axis_linear] * (params.invert_linear ? -1 : 1) * params.scale_linear : null;
  const angV = params && axes[params.axis_angular] != null ? axes[params.axis_angular] * (params.invert_angular ? -1 : 1) * params.scale_angular : null;

  return (
    <Card title="Axis calibration" wide>
      {!c?.present && <p class="error-text">No controller on /dev/input/js0.</p>}
      {c?.present && <KV k="Controller">{c.name} <span class="muted">({c.key})</span>{c.profile ? <span class="tag tag-active">profile applied</span> : <span class="tag">defaults</span>}</KV>}
      <div class="calib">
        <div>
          <h3>Live /joy axes</h3>
          {axes.length ? axes.map((a, i) => (
            <div key={i} class={'axis-row' + (params && (params.axis_linear === i || params.axis_angular === i) ? ' axis-sel' : '')}>
              <span class="axis-i">{i}</span><CenterBar value={a} /><span class="axis-v">{a.toFixed(2)}</span>
              <span class="axis-tag">{params?.axis_linear === i ? 'linear' : params?.axis_angular === i ? 'angular' : ''}</span>
            </div>
          )) : <p class="muted">No /joy messages. Connect a controller.</p>}
          {live.telemetry.joy && (
            <div class="buttons">{live.telemetry.joy.buttons.map((b, i) => <span key={i} class={'jbtn' + (b ? ' on' : '')}>{i}</span>)}</div>
          )}
        </div>
        <div>
          <h3>Mapping</h3>
          {error && <p class="error-text">{error}</p>}
          {params && <>
            <div class="form-grid">
              <span>Linear (forward)</span>
              <div class="row">{axisSelect('axis_linear')}
                <button class="btn small" disabled={!!detect || !axes.length} onClick={() => startDetect('linear')}>{detect === 'linear' ? 'Push forward…' : 'Detect'}</button></div>
              <span>Linear scale (m/s)</span>{num('scale_linear', 0.1)}
              <span>Invert linear</span><input type="checkbox" checked={params.invert_linear} onChange={(e) => set('invert_linear', (e.target as HTMLInputElement).checked)} />
              <span>Angular (turn)</span>
              <div class="row">{axisSelect('axis_angular')}
                <button class="btn small" disabled={!!detect || !axes.length} onClick={() => startDetect('angular')}>{detect === 'angular' ? 'Push left…' : 'Detect'}</button></div>
              <span>Angular scale (rad/s)</span>{num('scale_angular', 0.5)}
              <span>Invert angular</span><input type="checkbox" checked={params.invert_angular} onChange={(e) => set('invert_angular', (e.target as HTMLInputElement).checked)} />
              <span>Deadzone</span>{num('deadzone', 0.01)}
            </div>
            <div class="preview">
              <KV k="Would command v">{linV == null ? '–' : `${linV.toFixed(2)} m/s`}</KV>
              <KV k="Would command ω">{angV == null ? '–' : `${angV.toFixed(2)} rad/s`}</KV>
            </div>
            <div class="row">
              <button class="btn" disabled={busy} onClick={load}>Reload</button>
              <button class="btn" disabled={busy} onClick={apply}>Apply</button>
              <button class="btn primary" disabled={busy || !c?.key} onClick={saveProfile}>Save for this controller</button>
            </div>
            <p class="muted small">Forward on the stick should give a positive v; left should give a positive ω (counter-clockwise).</p>
          </>}
        </div>
      </div>
    </Card>
  );
}

export function ControllerPage() {
  const live = useLive();
  const c = live.status?.controller;
  const [busy, setBusy] = useState(false);
  const restart = async () => {
    setBusy(true);
    try { await api('POST', '/joy/restart'); toast('joy_linux restarted', 'ok'); } catch (e) { toast((e as Error).message, 'error'); }
    setBusy(false);
  };
  return (
    <div class="grid">
      <Card title="Joystick" right={<button class="btn small" disabled={busy} onClick={restart}>Restart joy</button>}>
        <KV k="/dev/input/js0"><Dot ok={c?.present} /> {c?.present ? c.name : 'missing'}</KV>
        <KV k="/joy">{live.status ? (live.status.topics['/joy']?.stale ? <><Dot ok={false} /> not publishing</> : <><Dot ok={true} /> {live.status.topics['/joy']?.rate.toFixed(0)} Hz</>) : '–'}</KV>
        <KV k="Bluetooth">{c?.bluetooth.length ? c.bluetooth.map((b) => `${b.name} (${b.address})`).join(', ') : 'none connected'}</KV>
      </Card>
      <Calibration />
      <Bluetooth />
    </div>
  );
}
