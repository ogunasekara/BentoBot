import { useLive } from '../live';
import { fmt } from '../api';
import { Card, CenterBar, Dot, KV, Meter } from '../components/Bits';

const NODE_LABELS: Record<string, string> = {
  joy: 'joy (joy_linux)',
  bentobot_teleop: 'bentobot_teleop',
  bentobot_mcu_bridge: 'bentobot_mcu_bridge',
  LD06: 'LD06 driver',
  scan_resampler: 'scan_resampler',
  bentobot_odom: 'bentobot_odom',
  laser_transform: 'static TF (laser)',
  rosbridge_websocket: 'rosbridge',
  slam_toolbox: 'slam_toolbox',
  map_server: 'map_server',
  amcl: 'amcl',
};

const TOPICS = ['/scan', '/mcu_info', '/odom', '/joy', '/mcu_cmd', '/map', '/amcl_pose'];

export function Dashboard() {
  const live = useLive();
  const s = live.status;
  const t = live.telemetry;
  if (!s) {
    return <div class="empty">{live.connected ? 'Waiting for status…' : 'Connecting to the robot…'}</div>;
  }
  const m = s.mode;
  const sys = s.system;
  const c = s.controller;

  return (
    <div class="grid">
      <Card title="Mode" right={<a class="link" href="#/run">Open run view →</a>}>
        <div class="mode-big">{m.label}{m.phase !== 'running' && m.phase !== 'idle' ? <span class="muted"> · {m.phase}</span> : null}</div>
        {m.map_id && <KV k="Map">{m.map_id}</KV>}
        {m.elapsed != null && <KV k="Running for">{fmt.duration(m.elapsed)}</KV>}
        {m.external && <p class="note">Started outside the web UI (teleop.launch.py). It can be saved here but not stopped.</p>}
        {m.error && <p class="error-text">{m.error}</p>}
      </Card>

      <Card title="Nodes" span2>
        <ul class="list">
          {Object.entries(s.nodes).map(([n, ok]) => (
            <li key={n}><Dot ok={ok} /> <span>{NODE_LABELS[n] || n}</span></li>
          ))}
        </ul>
      </Card>

      <Card title="Topics">
        <table class="table">
          <thead><tr><th>Topic</th><th class="num">Rate</th><th class="num">Last</th></tr></thead>
          <tbody>
            {TOPICS.map((name) => {
              const st = s.topics[name];
              if (!st) return null;
              return (
                <tr key={name}>
                  <td><Dot ok={st.stale ? false : st.count > 0 ? true : null} /> <code>{name}</code></td>
                  <td class="num">{st.rate ? `${st.rate.toFixed(1)} Hz` : '–'}</td>
                  <td class="num">{st.age == null ? 'never' : st.age < 1 ? 'now' : `${st.age.toFixed(1)} s ago`}</td>
                </tr>
              );
            })}
          </tbody>
        </table>
      </Card>

      <Card title="Motion" span2>
        <div class="cols2">
          <div>
            <h3>/mcu_info</h3>
            <KV k="Left wheel">{fmt.num(t.mcu_info?.left)} m/s</KV>
            <KV k="Right wheel">{fmt.num(t.mcu_info?.right)} m/s</KV>
            <KV k="Gyro z">{fmt.num(t.mcu_info?.gyro_z)} rad/s</KV>
          </div>
          <div>
            <h3>/odom</h3>
            <KV k="v">{fmt.num(t.odom?.v)} m/s</KV>
            <KV k="ω">{fmt.num(t.odom?.w)} rad/s</KV>
            <KV k="Pose">{t.odom ? `${fmt.num(t.odom.x)}, ${fmt.num(t.odom.y)}, ${fmt.deg(t.odom.yaw)}` : '–'}</KV>
          </div>
          <div>
            <h3>/mcu_cmd</h3>
            <KV k="v">{fmt.num(t.mcu_cmd?.v)} m/s</KV>
            <KV k="ω">{fmt.num(t.mcu_cmd?.w)} rad/s</KV>
          </div>
          <div>
            <h3>Pose ({t.pose?.frame || '–'})</h3>
            <KV k="x, y">{t.pose ? `${fmt.num(t.pose.x)}, ${fmt.num(t.pose.y)}` : '–'}</KV>
            <KV k="θ">{fmt.deg(t.pose?.yaw)}</KV>
          </div>
        </div>
      </Card>

      <Card title="Controller" right={<a class="link" href="#/controller">Manage →</a>}>
        <KV k="/dev/input/js0"><Dot ok={c.present} /> {c.present ? 'present' : 'missing'}</KV>
        {c.present && <KV k="Device">{c.name || '–'}</KV>}
        <KV k="Bluetooth">{c.bluetooth.length ? c.bluetooth.map((b) => b.name).join(', ') : 'none connected'}</KV>
        <KV k="Axis profile">{c.profile ? 'saved profile applied' : 'defaults'}</KV>
        {t.joy ? (
          <div class="axes">
            {t.joy.axes.map((a, i) => (
              <div class="axis-row" key={i}><span class="axis-i">{i}</span><CenterBar value={a} /><span class="axis-v">{a.toFixed(2)}</span></div>
            ))}
          </div>
        ) : <p class="muted">No /joy messages.</p>}
      </Card>

      <Card title="Jetson">
        <KV k="CPU">{sys.cpu.toFixed(0)}%</KV>
        <div class="cpus">{sys.cpus.map((p, i) => <Meter key={i} frac={p / 100} />)}</div>
        <KV k="RAM">{fmt.bytes(sys.mem.used)} / {fmt.bytes(sys.mem.total)}</KV>
        <Meter frac={sys.mem.percent / 100} warn={0.8} bad={0.92} />
        {sys.disk && (
          <>
            <KV k="Map storage free">{fmt.bytes(sys.disk.free)} of {fmt.bytes(sys.disk.total)}</KV>
            <Meter frac={1 - sys.disk.free / sys.disk.total} warn={0.85} bad={0.95} />
          </>
        )}
        <h3>Temperatures</h3>
        <div class="temps">
          {Object.entries(sys.temps).map(([k, v]) => (
            <div key={k} class={'temp' + (v >= 80 ? ' bad' : v >= 65 ? ' warn' : '')}><span>{k.replace(/-therm$/i, '')}</span><b>{v.toFixed(0)}°C</b></div>
          ))}
          {!Object.keys(sys.temps).length && <span class="muted">no thermal zones</span>}
        </div>
      </Card>
    </div>
  );
}
