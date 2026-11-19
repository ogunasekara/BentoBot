import { defineConfig } from 'vite';
import preact from '@preact/preset-vite';

// `npm run dev` proxies the API and websocket to a running backend:
//   BENTOBOT_URL=http://bentobot.local:8080 npm run dev
const target = process.env.BENTOBOT_URL || 'http://localhost:8080';

export default defineConfig({
  plugins: [preact()],
  build: { target: 'es2019', outDir: 'dist', assetsDir: 'assets', sourcemap: false },
  server: {
    proxy: {
      '/api': target,
      '/ws': { target, ws: true },
    },
  },
});
