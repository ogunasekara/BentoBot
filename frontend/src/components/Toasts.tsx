import { useEffect, useState } from 'preact/hooks';

type Kind = 'info' | 'error' | 'ok';
interface Item { id: number; text: string; kind: Kind }

let items: Item[] = [];
let nextId = 1;
const subs = new Set<() => void>();

export function toast(text: string, kind: Kind = 'info', ms = kind === 'error' ? 7000 : 3500) {
  const id = nextId++;
  items = [...items, { id, text, kind }];
  subs.forEach((s) => s());
  setTimeout(() => {
    items = items.filter((i) => i.id !== id);
    subs.forEach((s) => s());
  }, ms);
}

export function Toasts() {
  const [, setV] = useState(0);
  useEffect(() => {
    const s = () => setV((v) => v + 1);
    subs.add(s);
    return () => { subs.delete(s); };
  }, []);
  return (
    <div class="toasts" role="status" aria-live="polite">
      {items.map((i) => <div key={i.id} class={'toast toast-' + i.kind}>{i.text}</div>)}
    </div>
  );
}
