import { render } from 'preact';
import { useEffect, useState } from 'preact/hooks';
import { api } from './api';
import { connect, refreshColors, useLive } from './live';
import { Dashboard } from './pages/Dashboard';
import { Run } from './pages/Run';
import { Maps } from './pages/Maps';
import { Editor } from './pages/Editor';
import { ControllerPage } from './pages/Controller';
import { toast, Toasts } from './components/Toasts';
import './styles.css';

function useHash() {
  const [hash, setHash] = useState(location.hash || '#/');
  useEffect(() => {
    const on = () => setHash(location.hash || '#/');
    window.addEventListener('hashchange', on);
    return () => window.removeEventListener('hashchange', on);
  }, []);
  return hash;
}

const TABS = [
  { href: '#/', label: 'Status' },
  { href: '#/run', label: 'Run' },
  { href: '#/maps', label: 'Maps' },
  { href: '#/controller', label: 'Controller' },
];

function StopButton() {
  const live = useLive();
  const [busy, setBusy] = useState(false);
  const enabled = live.status ? (live.telemetry.teleop_enabled ?? live.status.teleop_enabled) : null;
  const set = async (on: boolean) => {
    setBusy(true);
    try {
      await api('POST', '/teleop/enabled', { enabled: on });
    } catch (e) {
      toast(String((e as Error).message), 'error');
    } finally {
      setBusy(false);
    }
  };
  if (enabled === false) {
    return <button class="btn resume" disabled={busy} onClick={() => set(true)}>Resume teleop</button>;
  }
  return <button class="btn stop" disabled={busy || !live.connected} onClick={() => set(false)}>STOP</button>;
}

function TopBar({ hash }: { hash: string }) {
  const live = useLive();
  const mode = live.status?.mode;
  const route = '#/' + (hash.split('/')[1] || '');
  return (
    <header class="topbar">
      <div class="topbar-row">
        <a class="brand" href="#/">BentoBot</a>
        <span class={'pill ' + (live.connected ? (mode && mode.mode !== 'idle' ? 'pill-active' : '') : 'pill-bad')}>
          {!live.connected ? 'offline' : mode ? `${mode.label}${mode.phase !== 'running' && mode.phase !== 'idle' ? ` · ${mode.phase}` : ''}` : '…'}
        </span>
        <span class="spacer" />
        <StopButton />
      </div>
      <nav class="tabs">
        {TABS.map((t) => (
          <a key={t.href} href={t.href} class={route === t.href || (t.href === '#/maps' && route === '#/edit') ? 'active' : ''}>{t.label}</a>
        ))}
      </nav>
    </header>
  );
}

function PausedBanner() {
  const live = useLive();
  const enabled = live.telemetry.teleop_enabled ?? live.status?.teleop_enabled;
  if (enabled !== false) return null;
  return <div class="banner banner-stop">Teleop is paused and the robot is stopped. The controller is ignored until you resume.</div>;
}

function App() {
  const hash = useHash();
  const parts = hash.replace(/^#\/?/, '').split('/');
  let page;
  switch (parts[0]) {
    case 'run': page = <Run />; break;
    case 'maps': page = <Maps />; break;
    case 'edit': page = <Editor id={decodeURIComponent(parts[1] || '')} />; break;
    case 'controller': page = <ControllerPage />; break;
    default: page = <Dashboard />;
  }
  return (
    <>
      <TopBar hash={hash} />
      <PausedBanner />
      <main class={parts[0] === 'run' || parts[0] === 'edit' ? 'main-full' : 'main'}>{page}</main>
      <Toasts />
    </>
  );
}

refreshColors();
connect();
render(<App />, document.getElementById('app')!);
