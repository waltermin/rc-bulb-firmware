// check_wasm.mjs — run the wasm build of the solver against the C test vectors.
//   node check_wasm.mjs <lemtone.wasm> <lemtone_vectors.h>
import { readFileSync } from "node:fs";

const [wasmPath, vecPath] = process.argv.slice(2);
const { instance } = await WebAssembly.instantiate(readFileSync(wasmPath), {});
const W = instance.exports;
W.lt_init();

// SOLVE_VECS rows: {a, b, lm, f, mode, desat, lm_out, {duty x5}, {period x5}, {ticks x5}},
const src = readFileSync(vecPath, "utf8");
const body = src.slice(src.indexOf("SOLVE_VECS"));
const rows = [...body.matchAll(/^\s*\{([^{}]+),\s*\{([^}]*)\},\s*\{[^}]*\},\s*\{[^}]*\}\},\s*$/gm)];
const num = s => parseFloat(s.replace(/f$/, ""));
let n = 0, modeDiff = 0, worst = 0;
for (const r of rows) {
  const head = r[1].split(",").map(s => s.trim());
  const [a, b, lm, f] = head.slice(0, 4).map(num), mode = parseInt(head[4], 10);
  const duty = r[2].split(",").map(s => num(s.trim()));
  const m = W.lt_solve(a, b, lm, f);
  n++;
  if (m !== mode) { modeDiff++; continue; }
  if (m === 1) for (let i = 0; i < 5; i++) worst = Math.max(worst, Math.abs(W.lt_duty(i) - duty[i]));
}
console.log(`wasm: ${n} vectors, ${modeDiff} mode mismatches, worst exact-solve duty diff ${worst.toExponential(2)}`);
if (n < 100 || modeDiff > n / 100 || worst > 2e-3) { console.error("wasm check FAILED"); process.exit(1); }
