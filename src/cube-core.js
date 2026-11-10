/* =============================================================
 * cube-core.js — Rubik's Cube logical model (facelet level)
 *
 * Design notes
 * ------------
 * Every sticker is a pair (cubie lattice position, face normal)
 * with integer coordinates in {-1,0,1}. A face turn is an exact
 * 90-degree rotation of that layer's stickers, mapped back to
 * facelet indices. The move engine is therefore correct by
 * construction — no hand-written permutation tables.
 *
 * Face order (Kociemba standard): U R F D L B = 0 1 2 3 4 5
 * Facelet id = face*9 + row*3 + col, row-major viewed from
 * outside the face, in the standard net orientation.
 *
 * Whole-cube orientation is tracked separately as an integer
 * 3x3 rotation matrix M (cube frame -> world frame). The facelet
 * state is always kept in the canonical frame.
 * ============================================================= */
(function (root, factory) {
  const api = factory();
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
  else root.CubeCore = api;
})(typeof self !== 'undefined' ? self : globalThis, function () {
  'use strict';

  const FACES = ['U', 'R', 'F', 'D', 'L', 'B'];
  // Face normals in the cube frame (x=right, y=up, z=front)
  const NORMALS = [
    [0, 1, 0],   // U
    [1, 0, 0],   // R
    [0, 0, 1],   // F
    [0, -1, 0],  // D
    [-1, 0, 0],  // L
    [0, 0, -1],  // B
  ];
  const AXIS = [1, 0, 2, 1, 0, 2];   // world axis index of each face normal
  const NSIGN = [1, 1, 1, -1, -1, -1]; // sign along that axis

  // ---------- facelet <-> (position, normal) ----------
  function faceletPos(f, r, c) {
    switch (f) {
      case 0: return [c - 1, 1, r - 1];      // U: row0 = back,  col0 = left
      case 1: return [1, 1 - r, 1 - c];      // R: row0 = top,   col0 = front
      case 2: return [c - 1, 1 - r, 1];      // F: row0 = top,   col0 = left
      case 3: return [c - 1, -1, 1 - r];     // D: row0 = front, col0 = left
      case 4: return [-1, 1 - r, c - 1];     // L: row0 = top,   col0 = back
      case 5: return [1 - c, 1 - r, -1];     // B: row0 = top,   col0 = right
    }
  }

  const PN = new Array(54);   // facelet id -> {pos, n}
  const KEY = new Map();      // "x,y,z|nx,ny,nz" -> facelet id
  for (let f = 0; f < 6; f++) {
    for (let r = 0; r < 3; r++) for (let c = 0; c < 3; c++) {
      const id = f * 9 + r * 3 + c;
      const pos = faceletPos(f, r, c);
      PN[id] = { pos: pos, n: NORMALS[f] };
      KEY.set(pos.join(',') + '|' + NORMALS[f].join(','), id);
    }
  }

  // Rotate an integer 3-vector by q quarter turns (+90 deg each) about +axis.
  function rotQ(v, axis, q) {
    let x = v[0], y = v[1], z = v[2];
    for (let i = 0; i < q; i++) {
      if (axis === 0) { const t = y; y = -z; z = t; }      // about X: y'=-z, z'=y
      else if (axis === 1) { const t = x; x = z; z = -t; } // about Y: x'=z, z'=-x
      else { const t = x; x = -y; y = t; }                 // about Z: x'=-y, y'=x
    }
    return [x, y, z];
  }

  // ---------- state helpers ----------
  function solvedState() {
    const s = new Uint8Array(54);
    for (let i = 0; i < 54; i++) s[i] = (i / 9) | 0;
    return s;
  }

  function isSolved(state) {
    for (let i = 0; i < 54; i++) if (state[i] !== ((i / 9) | 0)) return false;
    return true;
  }

  // Apply a quarter turn of face f, q times (q=1 clockwise, 2 double, 3 counter).
  // Canonical move = rotation about the face normal by -90 deg * q.
  function applyMove(state, face, q) {
    q = ((q % 4) + 4) % 4;
    if (q === 0) return state;
    const out = state.slice();
    const ax = AXIS[face], sg = NSIGN[face];
    // rotation about +axis by Q quarter turns:
    const Q = sg > 0 ? (4 - q) % 4 : q % 4;
    for (let id = 0; id < 54; id++) {
      const pn = PN[id];
      if (pn.pos[ax] !== sg) continue;
      const pos2 = rotQ(pn.pos, ax, Q);
      const n2 = rotQ(pn.n, ax, Q);
      const id2 = KEY.get(pos2.join(',') + '|' + n2.join(','));
      out[id2] = state[id];
    }
    return out;
  }

  function applyMoves(state, moves) {
    let s = state;
    for (const m of moves) s = applyMove(s, m.face, m.q);
    return s;
  }

  // ---------- notation ----------
  const FACE_CHAR = { U: 0, R: 1, F: 2, D: 3, L: 4, B: 5 };

  function parseAlg(str) {
    const out = [];
    const toks = String(str).trim().split(/\s+/).filter(Boolean);
    for (const t of toks) {
      const m = /^([URFDLB])(2|')?$/.exec(t);
      if (!m) throw new Error('Invalid move token: "' + t + '"');
      const face = FACE_CHAR[m[1]];
      const q = m[2] === '2' ? 2 : (m[2] === "'" ? 3 : 1);
      out.push({ face: face, q: q });
    }
    return out;
  }

  function moveToNotation(move) {
    return FACES[move.face] + (move.q === 2 ? '2' : move.q === 3 ? "'" : '');
  }

  function algToString(moves) {
    return moves.map(moveToNotation).join(' ');
  }

  function toFaceletString(state) {
    let s = '';
    for (let i = 0; i < 54; i++) s += FACES[state[i]];
    return s;
  }

  function fromFaceletString(str) {
    const s = new Uint8Array(54);
    for (let i = 0; i < 54; i++) {
      const f = FACES.indexOf(str[i]);
      if (f < 0) throw new Error('Bad facelet char: ' + str[i]);
      s[i] = f;
    }
    return s;
  }

  // ---------- scramble generation ----------
  function randomScramble(n, rng) {
    n = n || 25;
    rng = rng || Math.random;
    const moves = [];
    let prevF = -1, prevPrevF = -1;
    while (moves.length < n) {
      const f = (rng() * 6) | 0;
      if (f === prevF) continue;
      if (prevPrevF === f && AXIS[f] === AXIS[prevF]) continue; // no L R L sandwiches
      const q = 1 + ((rng() * 3) | 0);
      moves.push({ face: f, q: q });
      prevPrevF = prevF; prevF = f;
    }
    return moves;
  }

  // Merge/cancel adjacent moves on the same face.
  function simplify(moves) {
    const out = [];
    for (const m of moves) {
      const last = out[out.length - 1];
      if (last && last.face === m.face) {
        const q = (last.q + m.q) % 4;
        out.pop();
        if (q !== 0) out.push({ face: m.face, q: q });
      } else out.push({ face: m.face, q: m.q });
    }
    return out;
  }

  // ---------- whole-cube orientation group (24 rotations) ----------
  // Orientation M maps cube-frame vectors to world-frame vectors.
  // The robot may roll the whole cube +/-90 deg about the world X or Z axis.
  const GEN = {
    'X+': [[1, 0, 0], [0, 0, -1], [0, 1, 0]],  // RotX(+90)
    'X-': [[1, 0, 0], [0, 0, 1], [0, -1, 0]],  // RotX(-90)
    'Z+': [[0, -1, 0], [1, 0, 0], [0, 0, 1]],  // RotZ(+90)
    'Z-': [[0, 1, 0], [-1, 0, 0], [0, 0, 1]],  // RotZ(-90)
  };
  const GEN_NAMES = Object.keys(GEN);

  const IDENT = [[1, 0, 0], [0, 1, 0], [0, 0, 1]];

  function matMul(A, B) {
    const C = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
    for (let i = 0; i < 3; i++)
      for (let j = 0; j < 3; j++) {
        let s = 0;
        for (let k = 0; k < 3; k++) s += A[i][k] * B[k][j];
        C[i][j] = s;
      }
    return C;
  }
  function matVec(A, v) {
    return [A[0][0] * v[0] + A[0][1] * v[1] + A[0][2] * v[2],
            A[1][0] * v[0] + A[1][1] * v[1] + A[1][2] * v[2],
            A[2][0] * v[0] + A[2][1] * v[1] + A[2][2] * v[2]];
  }
  function matEq(A, B) {
    for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) if (A[i][j] !== B[i][j]) return false;
    return true;
  }
  function matKey(A) { return A[0].join(',') + ';' + A[1].join(',') + ';' + A[2].join(','); }

  // Apply a sequence of world rotations (applied left: M' = G * M).
  function applyGens(M, gens) {
    let out = M;
    for (const g of gens) out = matMul(GEN[g], out);
    return out;
  }

  // BFS over the 24-element orientation group.
  function bfsOrient(M0, isGoal) {
    if (isGoal(M0)) return [];
    const visited = new Map(); // key -> parent key
    const info = new Map();    // key -> {parent, gen}
    const startKey = matKey(M0);
    visited.set(startKey, true);
    let frontier = [M0];
    for (let depth = 1; depth <= 4; depth++) {
      const next = [];
      for (const M of frontier) {
        for (const g of GEN_NAMES) {
          const M2 = matMul(GEN[g], M);
          const k = matKey(M2);
          if (visited.has(k)) continue;
          visited.set(k, true);
          info.set(k, { parent: matKey(M), gen: g });
          if (isGoal(M2)) {
            // reconstruct
            const seq = [];
            let cur = k;
            while (cur !== startKey) {
              const e = info.get(cur);
              seq.push(e.gen);
              cur = e.parent;
            }
            return seq.reverse();
          }
          next.push(M2);
        }
      }
      frontier = next;
    }
    return null; // unreachable
  }

  const UP = [0, 1, 0];

  // Shortest sequence of robot whole-cube rolls bringing face `face` on top.
  function bringFaceUp(M, face) {
    const n = NORMALS[face];
    return bfsOrient(M, (X) => {
      const w = matVec(X, n);
      return w[0] === 0 && w[1] === 1 && w[2] === 0;
    });
  }

  // Shortest sequence of rolls restoring the canonical orientation.
  function orientToIdentity(M) {
    return bfsOrient(M, (X) => matEq(X, IDENT));
  }

  // Enumerate all 24 orientations (for tests / enumeration).
  function allOrientations() {
    const seen = new Map();
    let frontier = [IDENT];
    seen.set(matKey(IDENT), IDENT);
    while (frontier.length) {
      const next = [];
      for (const M of frontier) {
        for (const g of GEN_NAMES) {
          const M2 = matMul(GEN[g], M);
          const k = matKey(M2);
          if (!seen.has(k)) { seen.set(k, M2); next.push(M2); }
        }
      }
      frontier = next;
    }
    return [...seen.values()];
  }

  return {
    FACES: FACES,
    NORMALS: NORMALS,
    AXIS: AXIS,
    NSIGN: NSIGN,
    faceletPos: faceletPos,
    solvedState: solvedState,
    isSolved: isSolved,
    applyMove: applyMove,
    applyMoves: applyMoves,
    parseAlg: parseAlg,
    moveToNotation: moveToNotation,
    algToString: algToString,
    toFaceletString: toFaceletString,
    fromFaceletString: fromFaceletString,
    randomScramble: randomScramble,
    simplify: simplify,
    GEN: GEN,
    GEN_NAMES: GEN_NAMES,
    IDENT: IDENT,
    matMul: matMul,
    matVec: matVec,
    matEq: matEq,
    applyGens: applyGens,
    bringFaceUp: bringFaceUp,
    orientToIdentity: orientToIdentity,
    allOrientations: allOrientations,
    UP: UP,
  };
});
