// cri.js — CIE 13.3-1995 colour rendering index (Ra, R1..R14) of a spectrum.
//
// makeCri(tables) with tables = web/cri_tables.json (CIE 1931 2° CMF, TCS01..14, D-series
// S0..S2, all 380-780 nm at 5 nm). Returns cri(spd) for an spd on the same grid:
//   { Ra, R: [R1..R14], cct, duv, edge, valid, ref }   ref = reference illuminant spd,
//   edge = nearest locus point is an end of the 1000-25000 K search (cct meaningless)
// Steps: CCT = nearest Planckian point in CIE 1960 uv; reference = Planck (< 5000 K) or
// CIE D-series (>= 5000 K); both normalised to Y = 100; von Kries in the CIE 13.3 c/d
// form; U*V*W* differences; Ri = 100 - 4.6 dE. valid is false past Duv 0.0054, where the
// CIE says CRI is not meaningful.
// Plain script (no modules): build.py inlines it into the page; node checks load it too.
function makeCri(T) {
  const WL = T.wl, N = WL.length, [XB, YB, ZB] = T.cmf;
  const C2 = 1.4387768775039337e-2;  // m K (CODATA, as colour-science)

  const tri = (s, refl) => {
    let X = 0, Y = 0, Z = 0;
    for (let i = 0; i < N; i++) { const v = s[i] * (refl ? refl[i] : 1); X += v * XB[i]; Y += v * YB[i]; Z += v * ZB[i]; }
    return [X, Y, Z];
  };
  const uv = ([X, Y, Z]) => { const d = X + 15 * Y + 3 * Z; return [4 * X / d, 6 * Y / d]; };
  const planck = Tk => WL.map(l => { const m = l * 1e-9; return 1 / (m ** 5 * (Math.exp(C2 / (m * Tk)) - 1)); });
  const planckUv = Tk => uv(tri(planck(Tk)));

  function cctDuv(u, v) {
    // nearest point on the Planckian locus: coarse mired scan, then golden-section refine
    const d2 = m => { const p = planckUv(1e6 / m); return (p[0] - u) ** 2 + (p[1] - v) ** 2; };
    const M0 = 1e6 / 25000, M1 = 1e6 / 1000;
    let best = M0, bd = Infinity;
    for (let m = M0; m <= M1; m += 5) { const d = d2(m); if (d < bd) { bd = d; best = m; } }
    let lo = Math.max(M0, best - 5), hi = Math.min(M1, best + 5);
    for (let k = 0; k < 50; k++) { const a = lo + (hi - lo) * 0.382, b = hi - (hi - lo) * 0.382; if (d2(a) < d2(b)) hi = b; else lo = a; }
    const m = (lo + hi) / 2, p = planckUv(1e6 / m);
    return { cct: 1e6 / m, duv: Math.sign(v - p[1]) * Math.hypot(u - p[0], v - p[1]), edge: m <= M0 + 1e-6 || m >= M1 - 1e-6 };
  }

  function daylight(Tk) {
    const t = Tk;
    const x = t <= 7000 ? -4.607e9 / t ** 3 + 2.9678e6 / t ** 2 + 0.09911e3 / t + 0.244063
                        : -2.0064e9 / t ** 3 + 1.9018e6 / t ** 2 + 0.24748e3 / t + 0.23704;
    const y = -3 * x * x + 2.87 * x - 0.275;
    const M = 0.0241 + 0.2562 * x - 0.7341 * y;
    const r3 = v => Math.round(v * 1000) / 1000;   // CIE 15 rounds M1, M2
    const M1 = r3((-1.3515 - 1.7703 * x + 5.9114 * y) / M), M2 = r3((0.03 - 31.4424 * x + 30.0717 * y) / M);
    return T.d[0].map((s0, i) => s0 + M1 * T.d[1][i] + M2 * T.d[2][i]);
  }

  const cd = ([u, v]) => [(4 - u - 10 * v) / v, (1.708 * v - 1.481 * u + 0.404) / v];

  return function cri(spd) {
    const Xk = tri(spd);
    if (!(Xk[1] > 0)) return null;
    const [uk, vk] = uv(Xk);
    const { cct, duv, edge } = cctDuv(uk, vk);
    const ref = cct < 5000 ? planck(cct) : daylight(cct);
    const kt = 100 / Xk[1], Xr = tri(ref), kr = 100 / Xr[1];
    const [ur, vr] = uv(Xr);
    const [ck, dk] = cd([uk, vk]), [cr, dr] = cd([ur, vr]);
    const R = T.tcs.map(rho => {
      // reference
      const Yr = tri(ref, rho).map(v => v * kr), [uri, vri] = uv(Yr);
      // test, chromatically adapted to the reference
      const Yt = tri(spd, rho).map(v => v * kt), [cki, dki] = cd(uv(Yt));
      const den = 16.518 + 1.481 * (cr / ck) * cki - (dr / dk) * dki;
      const uti = (10.872 + 0.404 * (cr / ck) * cki - 4 * (dr / dk) * dki) / den, vti = 5.52 / den;
      const W = Y => 25 * Math.cbrt(Y) - 17;
      const Wr = W(Yr[1]), Wt = W(Yt[1]);
      const dU = 13 * Wt * (uti - ur) - 13 * Wr * (uri - ur);
      const dV = 13 * Wt * (vti - vr) - 13 * Wr * (vri - vr);
      return 100 - 4.6 * Math.hypot(dU, dV, Wt - Wr);
    });
    const Ra = R.slice(0, 8).reduce((a, b) => a + b, 0) / 8;
    return { Ra, R, cct, duv, edge, valid: !edge && Math.abs(duv) <= 0.0054, ref };
  };
}
if (typeof module !== "undefined") module.exports = { makeCri };
