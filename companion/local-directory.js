import { mkdir, mkdtemp } from "node:fs/promises";
import { fileURLToPath } from "node:url";
import path from "node:path";

const directory = fileURLToPath(new URL("../local/temporary/", import.meta.url));

export async function createLocalDirectory(prefix) {
  await mkdir(directory, { recursive: true });
  return mkdtemp(path.join(directory, prefix));
}
