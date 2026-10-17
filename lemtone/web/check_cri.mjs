// check_cri.mjs — compare cri.js with reference CRI values.
//   node check_cri.mjs <cri_tables.json> <cases.json>
// cases: [{name, spd (380-780/5 nm), Ra, R: [R1..R14]}], e.g. from colour-science.
import { readFileSync } from "node:fs";
import { createRequire } from "node:module";
const { makeCri } = createRequire(import.meta.url)("./cri.js");

const [tablesPath, casesPath] = process.argv.slice(2);
const cri = makeCri(JSON.parse(readFileSync(tablesPath, "utf8")));
let worstRa = 0, worstRi = 0;
for (const c of JSON.parse(readFileSync(casesPath, "utf8"))) {
  const r = cri(c.spd);
  const dRa = Math.abs(r.Ra - c.Ra), dRi = Math.max(...r.R.map((v, i) => Math.abs(v - c.R[i])));
  worstRa = Math.max(worstRa, dRa); worstRi = Math.max(worstRi, dRi);
  console.log(`${c.name.padEnd(22)} Ra ${r.Ra.toFixed(2).padStart(6)} (ref ${c.Ra.toFixed(2).padStart(6)})  ` +
    `R9 ${r.R[8].toFixed(2).padStart(7)} (ref ${c.R[8].toFixed(2).padStart(7)})  CCT ${r.cct.toFixed(0)} Duv ${r.duv.toFixed(4)}${r.valid ? "" : "  [not valid]"}`);
}
console.log(`worst |dRa| ${worstRa.toFixed(3)}, worst |dRi| ${worstRi.toFixed(3)}`);
if (worstRa > 0.5 || worstRi > 1.5) { console.error("CRI check FAILED"); process.exit(1); }
