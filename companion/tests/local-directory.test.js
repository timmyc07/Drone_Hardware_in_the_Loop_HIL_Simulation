import test from "node:test";
import assert from "node:assert/strict";
import { rm, writeFile } from "node:fs/promises";
import path from "node:path";
import { fileURLToPath } from "node:url";
import { createLocalDirectory } from "../local-directory.js";

test("creates unique writable project directories independent of cwd", async (t) => {
  const cwd = process.cwd();
  process.chdir(path.parse(cwd).root);
  t.after(() => process.chdir(cwd));
  const directories = [];
  t.after(() => Promise.all(directories.map((dir) => rm(dir, { recursive: true, force: true }))));
  directories.push(await createLocalDirectory("test-"));
  directories.push(await createLocalDirectory("test-"));
  assert.notEqual(directories[0], directories[1]);
  const expected = fileURLToPath(new URL("../../local/temporary/", import.meta.url));
  for (const dir of directories) {
    assert.equal(path.dirname(dir), path.resolve(expected));
    await writeFile(path.join(dir, "check"), "ok");
  }
});
