import { defineConfig } from "vite";
import { fileURLToPath } from "node:url";

export default defineConfig({
  root: "web",
  base: "./",
  build: {
    outDir: "../dist", emptyOutDir: true,
    rolldownOptions: { input: {
      main: fileURLToPath(new URL('./web/index.html', import.meta.url)),
      operator: fileURLToPath(new URL('./web/operator.html', import.meta.url)),
    } },
  },
});
