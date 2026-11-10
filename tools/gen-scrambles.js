/* ------------------------------------------------------------------
 * gen-scrambles.js — deterministically generates the 512-entry
 * scramble library (src scrambles.js) for the demo.
 *
 * Usage:  node gen-scrambles.js [cubeCorePath] [outPath]
 *   defaults: ./cube-core.js  ./scrambles.js  (relative to this file)
 *
 * Every entry is verified before it is admitted:
 *   - parses with the cube's own notation engine
 *   - produces a NON-solved cube state
 *   - produces a state DISTINCT from every other entry
 * => 512 library entries = 512 genuinely different scrambles.
 *
 * The generator is seeded (mulberry32, seed 20261110), so the output
 * is byte-reproducible on every machine.
 * ------------------------------------------------------------------ */
'use strict';
const fs = require('fs');
const path = require('path');

const cubeCorePath = path.resolve(process.argv[2] || path.join(__dirname, 'cube-core.js'));
const outPath = path.resolve(process.argv[3] || path.join(__dirname, 'scrambles.js'));
const CubeCore = require(cubeCorePath);

/* ---------- seeded PRNG ---------- */
const SEED = 20261110;
function mulberry32(a) {
  return function () {
    a |= 0; a = (a + 0x6D2B79F5) | 0;
    let t = Math.imul(a ^ (a >>> 15), 1 | a);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}
const rng = mulberry32(SEED);
const ri = (n) => Math.floor(rng() * n);
const pick = (arr) => arr[ri(arr.length)];

/* ---------- random-move generator (WCA-style cancellation rules) ---------- */
function randMoves(n, mods) {
  const moves = [];
  let pF = -1, ppF = -1;
  while (moves.length < n) {
    const f = ri(6);
    if (f === pF) continue;
    if (ppF === f && CubeCore.AXIS[f] === CubeCore.AXIS[pF]) continue; // no L R L
    moves.push({ face: f, q: pick(mods) });
    ppF = pF; pF = f;
  }
  return moves;
}

/* ---------- featured constructions ---------- */
function halfTurnMesh() { return randMoves(6 + ri(7), [2]); }            // double turns only
function loopWeave() {                                                   // (A B)-style loops
  const f1 = ri(6);
  let f2 = ri(6);
  while (f2 === f1) f2 = ri(6);
  const k = 3 + ri(4);                                                   // 3..6 repeats
  const moves = [];
  for (let i = 0; i < k; i++) {
    switch (ri(3)) {
      case 0: moves.push({ face: f1, q: 1 }, { face: f2, q: 1 }); break;
      case 1: moves.push({ face: f1, q: 1 }, { face: f2, q: 3 }); break;
      default: moves.push({ face: f1, q: 1 }, { face: f2, q: 1 },
                           { face: f1, q: 3 }, { face: f2, q: 3 });        // commutator
    }
  }
  return moves;
}
function mirrorWeave() {                                                 // palindrome sequences
  const half = randMoves(6 + ri(7), [1, 2, 3]);
  return half.concat(half.slice().reverse());
}

/* ---------- superflip: the classic 20-move sequence, verified ---------- */
const SUPERFLIP = "U R2 F B R B2 R U2 L B2 R U' D' R2 F R' L B2 U2 F2";
function isSuperflip(state) {
  // superflip = every edge flipped in place, corners + centers untouched
  for (let f = 0; f < 6; f++) {
    for (let r = 0; r < 3; r++) {
      for (let c = 0; c < 3; c++) {
        const id = f * 9 + r * 3 + c;
        const pos = CubeCore.faceletPos(f, r, c);
        const zeros = (pos[0] === 0) + (pos[1] === 0) + (pos[2] === 0);
        const home = (id / 9) | 0;
        if (zeros === 1) { if (state[id] === home) return false; }        // edge -> flipped
        else if (state[id] !== home) return false;                        // corner/center -> home
      }
    }
  }
  return true;
}

/* ---------- admission + quota fill ---------- */
const SOLVED = CubeCore.solvedState();
const seen = new Set();
const buckets = [];

function tryAdd(cat, moves) {
  const state = CubeCore.applyMoves(SOLVED, moves);
  if (CubeCore.isSolved(state)) return false;
  const key = CubeCore.toFaceletString(state);
  if (seen.has(key)) return false;
  seen.add(key);
  buckets.push({ cat: cat, seq: CubeCore.algToString(moves) });
  return true;
}
function fill(cat, count, gen) {
  let got = 0, guard = 0;
  while (got < count) {
    if (++guard > count * 20000) throw new Error('fill failed for ' + cat);
    if (tryAdd(cat, gen())) got++;
  }
}

/* ---------- build the library ---------- */
const hasSuperflip = (function () {
  const st = CubeCore.applyMoves(SOLVED, CubeCore.parseAlg(SUPERFLIP));
  return isSuperflip(st);
})();
if (!hasSuperflip) console.error('WARNING: superflip sequence did not verify — substituting a mesh');

// FEATURED: 48 (or 49 if the superflip verifies; the quota below trims to 48 total)
fill('HALF-TURN MESH', 24, halfTurnMesh);
fill('LOOP WEAVE', 12, loopWeave);
fill('MIRROR WEAVE', 12, mirrorWeave);

// random pools across depths / turn styles
fill('LIGHT-12', 48, () => randMoves(12, [1, 2, 3]));
fill('STANDARD-20', 96, () => randMoves(20, [1, 2, 3]));
fill('DEEP-25', 104, () => randMoves(25, [1, 2, 3]));
fill('EXTREME-35', 96, () => randMoves(35, [1, 2, 3]));
fill('QUARTER-18', 48, () => randMoves(18, [1]));
fill('HALF-24', 48, () => randMoves(24, [2]));
fill('MIXED-DEPTH', 24, () => randMoves(13 + ri(21), [1, 2, 3]));

/* ---------- order, name, number ---------- */
const CAT_ORDER = ['FEATURED', 'LIGHT-12', 'STANDARD-20', 'DEEP-25', 'EXTREME-35',
                   'QUARTER-18', 'HALF-24', 'MIXED-DEPTH'];
const CAT_COUNTS = {};
for (const b of buckets) CAT_COUNTS[b.cat] = (CAT_COUNTS[b.cat] || 0) + 1;

const ordered = [];
if (hasSuperflip) ordered.push({ cat: 'FEATURED', name: 'SUPERFLIP', seq: SUPERFLIP });
const counters = {};
for (const cat of ['HALF-TURN MESH', 'LOOP WEAVE', 'MIRROR WEAVE']) {
  let n = 0;
  for (const b of buckets) {
    if (b.cat !== cat) continue;
    n++;
    if (cat === 'MIRROR WEAVE' && hasSuperflip && n > 11) break;   // trim to 48 featured
    ordered.push({ cat: 'FEATURED', name: cat + ' ' + String(n).padStart(2, '0'), seq: b.seq });
  }
}
for (const cat of CAT_ORDER.slice(1)) {
  counters[cat] = 0;
  for (const b of buckets) {
    if (b.cat !== cat) continue;
    counters[cat]++;
    ordered.push({ cat: cat, name: cat + ' #' + String(counters[cat]).padStart(3, '0'), seq: b.seq });
  }
}

if (ordered.length !== 512) throw new Error('expected 512 entries, got ' + ordered.length);

/* ---------- final full verification (including superflip) ---------- */
{
  const finalSeen = new Set();
  for (const e of ordered) {
    const moves = CubeCore.parseAlg(e.seq);            // must parse
    const st = CubeCore.applyMoves(SOLVED, moves);
    if (CubeCore.isSolved(st)) throw new Error('entry solved the cube: ' + e.name);
    const k = CubeCore.toFaceletString(st);
    if (finalSeen.has(k)) throw new Error('duplicate state: ' + e.name);
    finalSeen.add(k);
  }
  if (finalSeen.size !== 512) throw new Error('distinct states: ' + finalSeen.size);
}

/* ---------- emit ---------- */
const lines = ordered.map((e, i) =>
  '    [' + (i + 1) + ', ' + JSON.stringify(e.cat) + ', ' + JSON.stringify(e.name) + ', ' + JSON.stringify(e.seq) + '],');
const out = `/* =============================================================
 * scrambles.js — 512-entry seeded scramble library
 *
 * AUTO-GENERATED by gen-scrambles.js (seed ${SEED}) — do not edit
 * by hand. Regenerate deterministically with:
 *     node tools/gen-scrambles.js src/cube-core.js src/scrambles.js
 *
 * Every entry is machine-verified at generation time:
 *   - valid standard notation (URFDLB with 2 / ' modifiers)
 *   - produces a NON-solved cube state
 *   - produces a state DISTINCT from all 511 other entries
 * so the library is exactly 512 different scrambles.
 *
 * Categories (entries: [id, category, name, sequence]):
 *   FEATURED     ${CAT_COUNTS['HALF-TURN MESH'] + CAT_COUNTS['LOOP WEAVE'] + CAT_COUNTS['MIRROR WEAVE'] + (hasSuperflip ? 1 : 0) | 0} — superflip + half-turn meshes, loop weaves, mirror weaves
 *   LIGHT-12     ${CAT_COUNTS['LIGHT-12']} — 12 random moves
 *   STANDARD-20  ${CAT_COUNTS['STANDARD-20']} — 20 random moves
 *   DEEP-25      ${CAT_COUNTS['DEEP-25']} — 25 random moves
 *   EXTREME-35   ${CAT_COUNTS['EXTREME-35']} — 35 random moves
 *   QUARTER-18   ${CAT_COUNTS['QUARTER-18']} — clockwise quarter turns only
 *   HALF-24      ${CAT_COUNTS['HALF-24']} — double turns only (stripe family)
 *   MIXED-DEPTH  ${CAT_COUNTS['MIXED-DEPTH']} — random depth 13-33
 * ============================================================= */
(function (root) {
  'use strict';
  root.ScrambleLib = {
    count: 512,
    cats: ${JSON.stringify(CAT_ORDER)},
    entries: [
${lines.join('\n')}
    ],
  };
})(typeof self !== 'undefined' ? self : globalThis);
`;
fs.writeFileSync(outPath, out);

/* ---------- report ---------- */
const lens = ordered.map((e) => e.seq.split(' ').length);
console.log('wrote', path.relative(process.cwd(), outPath),
  '(' + (fs.statSync(outPath).size / 1024).toFixed(1) + ' KB)');
console.log('entries:', ordered.length, '| distinct cube states:', seen.size,
  '| none solved: OK | superflip verified:', hasSuperflip);
const byCat = {};
for (const e of ordered) byCat[e.cat] = (byCat[e.cat] || 0) + 1;
console.log('per category:', JSON.stringify(byCat));
console.log('move-count range:', Math.min.apply(null, lens), '-', Math.max.apply(null, lens));
