/* =============================================================
 * kinematics.js — 6-DOF articulated arm kinematics
 *
 * Robot structure (industrial 6R, all SI units, world frame:
 * x = base->cube direction, y = up, z = front):
 *
 *   J1 base yaw      (vertical axis)
 *   J2 shoulder pitch
 *   J3 elbow pitch        (parallel to J2)
 *   J4 wrist pitch        (parallel to J2/J3, at wrist point)
 *   J5 wrist yaw          (through wrist point)
 *   J6 tool roll          (flange axis == tool z)
 *   + fixed tool transform to the palm center (TCP)
 *
 * Standard Denavit-Hartenberg convention (Paul / Fu):
 *     T_i = Rz(theta_i) . Tz(d_i) . Tx(a_i) . Rx(alpha_i)
 *
 * Inverse kinematics: numeric, damped-least-squares on the
 * geometric Jacobian (6x6), with joint-limit clamping and
 * multi-seed restarts. Trajectories: synchronized trapezoidal
 * velocity profiles per joint (coordinated joint moves).
 * ============================================================= */
(function (root, factory) {
  const api = factory();
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
  else root.RobotKin = api;
})(typeof self !== 'undefined' ? self : globalThis, function () {
  'use strict';

  const DEG = Math.PI / 180;

  // ---------------- arm geometry (metres) ----------------
  // Compact bench-top geometry: the robot is mounted on a 0.12 m riser
  // and works at chest height around the pedestal cube, so the arm never
  // has to reach down towards the table (no "kneeling" postures).
  const DH = [
    // d        a        alpha       theta offset
    [0.092, 0.030, Math.PI / 2, 0],        // 1: base yaw
    [0.000, 0.168, 0, Math.PI / 2],        // 2: shoulder pitch
    [0.000, 0.136, 0, 0],                  // 3: elbow pitch
    [0.000, 0.000, Math.PI / 2, 0],        // 4: wrist pitch
    [0.046, 0.000, -Math.PI / 2, 0],       // 5: wrist yaw
    [0.018, 0.000, 0, 0],                  // 6: tool roll (flange)
  ];
  const TOOL = 0.036;              // flange -> gripper base centre along tool z
  const TOOLP = TOOL + DH[5][0];   // base centre -> wrist point along tool z
  const D5W = DH[4][0];            // wrist point offset along the J4 axis

  // DH base frame -> world frame (DH z-up maps to world y-up)
  const BASE_R = [[1, 0, 0], [0, 0, 1], [0, -1, 0]];
  const BASE_P = [0, 0.12, 0];     // robot mounted on a 120 mm riser

  const LIMITS = [                 // [min, max] radians
    [-170 * DEG, 170 * DEG],
    [-160 * DEG, 100 * DEG],
    [-155 * DEG, 155 * DEG],
    [-170 * DEG, 170 * DEG],
    [-150 * DEG, 150 * DEG],
    [-Math.PI * 2, Math.PI * 2],
  ];
  const VMAX = [3.0, 3.0, 3.4, 4.2, 4.6, 5.2];   // rad/s (fast cobot-class joint speeds)
  const AMAX = [14.0, 14.0, 16.0, 28.0, 32.0, 38.0]; // rad/s^2
  // (HOME is computed below from a reachable guard pose, after the IK
  //  machinery is defined.)

  // ---------------- keep-out world model ----------------
  // Coarse capsule envelope of the arm (matches the visual model):
  //   upper arm  shoulder -> elbow,  r = 30 mm
  //   forearm    elbow    -> J5,     r = 26 mm
  //   wrist      J5       -> J6,     r = 17 mm
  // Obstacles are AABBs ("aabb": centre c + half extent h) or capsules
  // ("cap": segment a-b + radius r), registered by the application with
  // setWorld(). The IK posture selector penalizes any configuration whose
  // envelope penetrates a keep-out volume, and the runtime samples joint
  // paths against the same model — exactly how a real controller uses
  // keep-out zones / smart-collision checking.
  const LINK_SEG = [                 // [jointA, jointB, radius]
    [1, 2, 0.030],                   // upper arm
    [2, 4, 0.026],                   // forearm
    [4, 5, 0.017],                   // wrist yoke
  ];
  let WORLD = [];                    // active obstacle list

  function setWorld(obstacles) { WORLD = obstacles || []; }
  function getWorld() { return WORLD; }

  function ptSegDist(p, a, b) {
    const ab = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    const ap = [p[0] - a[0], p[1] - a[1], p[2] - a[2]];
    const ab2 = ab[0] * ab[0] + ab[1] * ab[1] + ab[2] * ab[2];
    let t = ab2 > 1e-12 ? (ap[0] * ab[0] + ap[1] * ab[1] + ap[2] * ab[2]) / ab2 : 0;
    t = Math.max(0, Math.min(1, t));
    return Math.hypot(p[0] - (a[0] + t * ab[0]), p[1] - (a[1] + t * ab[1]), p[2] - (a[2] + t * ab[2]));
  }
  function ptAabbDist(p, c, h) {
    const dx = Math.max(Math.abs(p[0] - c[0]) - h, 0);
    const dy = Math.max(Math.abs(p[1] - c[1]) - h, 0);
    const dz = Math.max(Math.abs(p[2] - c[2]) - h, 0);
    return Math.hypot(dx, dy, dz);
  }

  // Minimum clearance of the arm envelope at configuration q against the
  // active keep-out world. Positive = clear, negative = penetration depth.
  // f (an fk() result) may be passed to avoid recomputation.
  function linkClearance(q, f) {
    if (!WORLD.length) return 1e9;
    const fr = f || fk(q);
    let worst = 1e9;
    for (const [ja, jb, r] of LINK_SEG) {
      const a = fr.joints[ja].o, b = fr.joints[jb].o;
      for (const ob of WORLD) {
        if (ob.type === 'aabb') {
          // sample the capsule axis against the box
          for (let i = 0; i <= 4; i++) {
            const t = i / 4;
            const p = [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, a[2] + (b[2] - a[2]) * t];
            const d = ptAabbDist(p, ob.c, ob.h) - r;
            if (d < worst) worst = d;
          }
        } else {
          for (let i = 0; i <= 4; i++) {
            const t = i / 4;
            const p = [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, a[2] + (b[2] - a[2]) * t];
            const d = ptSegDist(p, ob.a, ob.b) - r - ob.r;
            if (d < worst) worst = d;
          }
        }
      }
    }
    return worst;
  }

  // ---------------- small matrix helpers ----------------
  function matMul(A, B) {
    const C = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
    for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) {
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
  function cross(a, b) {
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
  }
  function sub(a, b) { return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]; }
  function norm(a) { return Math.hypot(a[0], a[1], a[2]); }

  // ---------------- forward kinematics ----------------
  // Returns { p: [x,y,z] world TCP position, R: 3x3 world TCP rotation,
  //           joints: [{o:[..], z:[..]}] axis line per joint (world),
  //           frames: [{o,R}] DH frames 0..6 (world) }
  function fk(q) {
    // DH chain in the DH base frame
    let T = [[1, 0, 0], [0, 1, 0], [0, 0, 1]];  // rotation part
    let o = [0, 0, 0];                            // origin
    const joints = [];
    const frames = [{ o: [0, 0, 0], R: [[1, 0, 0], [0, 1, 0], [0, 0, 1]] }];
    for (let i = 0; i < 6; i++) {
      // record joint i+1: axis = current z, origin = current origin
      const zAxis = [T[0][2], T[1][2], T[2][2]];
      joints.push({ o: o.slice(), z: zAxis });
      const th = q[i] + DH[i][3];
      // T = T * Rz(th) * Tz(d) * Tx(a) * Rx(alpha)
      const Rz = [[Math.cos(th), -Math.sin(th), 0], [Math.sin(th), Math.cos(th), 0], [0, 0, 1]];
      let R = matMul(T, Rz);
      const o1 = matVec(R, [0, 0, DH[i][0]]).map((v, k) => v + o[k]); // + d along z
      const o2 = matVec(R, [DH[i][1], 0, 0]).map((v, k) => v + o1[k]); // + a along x
      const al = DH[i][2];
      const Rx = [[1, 0, 0], [0, Math.cos(al), -Math.sin(al)], [0, Math.sin(al), Math.cos(al)]];
      R = matMul(R, Rx);
      T = R; o = o2;
      frames.push({ o: o.slice(), R: T.map((r) => r.slice()) });
    }
    // flange -> gripper base centre along tool z
    const oTool = matVec(T, [0, 0, TOOL]).map((v, k) => v + o[k]);
    // DH base frame -> world
    const Rw = matMul(BASE_R, T);
    const pw = matVec(BASE_R, oTool).map((v, k) => v + BASE_P[k]);
    const jointsW = joints.map((j) => ({
      o: matVec(BASE_R, j.o).map((v, k) => v + BASE_P[k]),
      z: matVec(BASE_R, j.z),
    }));
    const framesW = frames.map((f) => ({
      o: matVec(BASE_R, f.o).map((v, k) => v + BASE_P[k]),
      R: matMul(BASE_R, f.R),
    }));
    return { p: pw, R: Rw, joints: jointsW, frames: framesW, pDH: oTool, RDH: T };
  }

  // ---------------- inverse kinematics (DLS) ----------------
  // Robust orientation error via quaternion log: no degeneracy at 180 deg
  // (the sum-of-cross-products formula silently vanishes near pi).
  function quatFromMat(R) {
    const tr = R[0][0] + R[1][1] + R[2][2];
    let x, y, z, w;
    if (tr > 0) {
      const S = Math.sqrt(tr + 1) * 2;
      w = 0.25 * S; x = (R[2][1] - R[1][2]) / S; y = (R[0][2] - R[2][0]) / S; z = (R[1][0] - R[0][1]) / S;
    } else if (R[0][0] > R[1][1] && R[0][0] > R[2][2]) {
      const S = Math.sqrt(1 + R[0][0] - R[1][1] - R[2][2]) * 2;
      w = (R[2][1] - R[1][2]) / S; x = 0.25 * S; y = (R[0][1] + R[1][0]) / S; z = (R[0][2] + R[2][0]) / S;
    } else if (R[1][1] > R[2][2]) {
      const S = Math.sqrt(1 + R[1][1] - R[0][0] - R[2][2]) * 2;
      w = (R[0][2] - R[2][0]) / S; x = (R[0][1] + R[1][0]) / S; y = 0.25 * S; z = (R[1][2] + R[2][1]) / S;
    } else {
      const S = Math.sqrt(1 + R[2][2] - R[0][0] - R[1][1]) * 2;
      w = (R[1][0] - R[0][1]) / S; x = (R[0][2] + R[2][0]) / S; y = (R[1][2] + R[2][1]) / S; z = 0.25 * S;
    }
    return [x, y, z, w];
  }
  function quatMul(a, b) {
    const ax = a[0], ay = a[1], az = a[2], aw = a[3];
    const bx = b[0], by = b[1], bz = b[2], bw = b[3];
    return [
      aw * bx + ax * bw + ay * bz - az * by,
      aw * by - ax * bz + ay * bw + az * bx,
      aw * bz + ax * by - ay * bx + az * bw,
      aw * bw - ax * bx - ay * by - az * bz,
    ];
  }
  function quatConj(q) { return [-q[0], -q[1], -q[2], q[3]]; }
  function matFromQuat(q) {
    const x = q[0], y = q[1], z = q[2], w = q[3];
    return [
      [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
      [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
      [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ];
  }
  function slerpR(R0, R1, u) {
    const q0 = quatFromMat(R0), q1 = quatFromMat(R1);
    let d = q0[0] * q1[0] + q0[1] * q1[1] + q0[2] * q1[2] + q0[3] * q1[3];
    let q1s = q1;
    if (d < 0) { q1s = [-q1[0], -q1[1], -q1[2], -q1[3]]; d = -d; }
    let q;
    if (d > 0.9995) {
      q = [q0[0] + u * (q1s[0] - q0[0]), q0[1] + u * (q1s[1] - q0[1]),
           q0[2] + u * (q1s[2] - q0[2]), q0[3] + u * (q1s[3] - q0[3])];
      const n = Math.hypot(q[0], q[1], q[2], q[3]) || 1;
      q = [q[0] / n, q[1] / n, q[2] / n, q[3] / n];
    } else {
      const th = Math.acos(Math.min(1, d));
      const s = Math.sin(th);
      const a = Math.sin((1 - u) * th) / s, b = Math.sin(u * th) / s;
      q = [q0[0] * a + q1s[0] * b, q0[1] * a + q1s[1] * b,
           q0[2] * a + q1s[2] * b, q0[3] * a + q1s[3] * b];
    }
    return matFromQuat(q);
  }
  function orientError(Rt, Rc) {
    // world-frame rotation vector taking Rc to Rt
    const qt = quatFromMat(Rt), qc = quatFromMat(Rc);
    let qe = quatMul(qt, quatConj(qc));
    if (qe[3] < 0) qe = [-qe[0], -qe[1], -qe[2], -qe[3]];
    const vn = Math.hypot(qe[0], qe[1], qe[2]);
    if (vn < 1e-12) return [0, 0, 0];
    const ang = 2 * Math.atan2(vn, qe[3]);
    return [qe[0] / vn * ang, qe[1] / vn * ang, qe[2] / vn * ang];
  }

  function poseErr(dp, dr, rotW) {
    return norm(dp) + rotW * norm(dr);
  }

  // One DLS descent from a seed, with monotone line search.
  function dlsDescend(q, targetP, targetR, rotW, iters) {
    // acceptance: 0.3 mm position / 0.11 deg orientation (absorbed by snap)
    const epsP = 3e-4, epsR = 2e-3;
    let f = fk(q);
    let dp = sub(targetP, f.p);
    let dr = orientError(targetR, f.R);
    let err = poseErr(dp, dr, rotW);
    for (let it = 0; it < iters; it++) {
      if (norm(dp) < epsP && norm(dr) < epsR) return { q: q, ok: true, err: err };
      const lam = 0.012 + 0.18 * Math.min(err, 0.2);
      // geometric jacobian
      const J = [[], [], [], [], [], []];
      for (let j = 0; j < 6; j++) {
        const z = f.joints[j].z;
        const r = sub(f.p, f.joints[j].o);
        const jv = cross(z, r);
        J[0][j] = jv[0]; J[1][j] = jv[1]; J[2][j] = jv[2];
        J[3][j] = z[0];  J[4][j] = z[1];  J[5][j] = z[2];
      }
      const e = [dp[0], dp[1], dp[2], rotW * dr[0], rotW * dr[1], rotW * dr[2]];
      // A = J J^T + lam^2 I
      const A = [[], [], [], [], [], []];
      for (let i = 0; i < 6; i++) {
        for (let k = 0; k < 6; k++) {
          let s = 0;
          for (let j = 0; j < 6; j++) s += J[i][j] * J[k][j];
          A[i][k] = s + (i === k ? lam * lam : 0);
        }
      }
      const y = solve6(A, e);
      if (!y) break;
      const dq = [0, 0, 0, 0, 0, 0];
      for (let j = 0; j < 6; j++) {
        let s = 0;
        for (let i = 0; i < 6; i++) s += J[i][j] * y[i];
        dq[j] = s;
      }
      let m = 0;
      for (let j = 0; j < 6; j++) m = Math.max(m, Math.abs(dq[j]));
      const scl = m > 0.25 ? 0.25 / m : 1;

      // monotone line search: accept the first step that reduces the error
      let accepted = false;
      for (const alpha of [1, 0.5, 0.25]) {
        const q2 = q.slice();
        for (let j = 0; j < 6; j++) {
          q2[j] += dq[j] * scl * alpha;
          if (q2[j] < LIMITS[j][0]) q2[j] = LIMITS[j][0];
          if (q2[j] > LIMITS[j][1]) q2[j] = LIMITS[j][1];
        }
        const f2 = fk(q2);
        const dp2 = sub(targetP, f2.p);
        const dr2 = orientError(targetR, f2.R);
        const err2 = poseErr(dp2, dr2, rotW);
        if (err2 < err) {
          q = q2; f = f2; dp = dp2; dr = dr2; err = err2;
          accepted = true;
          break;
        }
      }
      if (!accepted) break; // local minimum of this descent
    }
    return { q: q, ok: false, err: err };
  }

  function normalizeAngle(a) {
    while (a > Math.PI) a -= 2 * Math.PI;
    while (a < -Math.PI) a += 2 * Math.PI;
    return a;
  }

  // 3R position branches (base yaw + shoulder + elbow) for a world point.
  // Returns [{J1, q2, q3, elbow}] without wrist angles.
  // Both azimuth solutions (forward reach and flipped reach) are returned;
  // branches whose elbow would dip below the table are rejected.
  const A1 = DH[0][1], D1 = DH[0][0], A2 = DH[1][1], A3 = DH[2][1];
  function positionBranches(W) {
    // DH coords: x = world X, y = world -Z, z = world Y (base riser removed)
    const tx = W[0], ty = -W[2], tz = W[1] - BASE_P[1];
    const out = [];
    const rT = Math.hypot(tx, ty);
    for (const azFlip of [0, 1]) {
      const J1 = normalizeAngle(Math.atan2(ty, tx) + (azFlip ? Math.PI : 0));
      if (J1 < LIMITS[0][0] || J1 > LIMITS[0][1]) continue;
      // shoulder sits at radial +a1 along x1; the target sits at radial
      // +rT (forward) or -rT (flipped azimuth) along x1
      const tr = azFlip ? -rT : rT;
      let dr = tr - A1, dh = tz - D1;
      let L = Math.hypot(dr, dh);
      const Lmin = Math.abs(A2 - A3) + 0.02, Lmax = A2 + A3 - 0.01;
      if (L > Lmax) { const s = Lmax / L; dr *= s; dh *= s; L = Lmax; }
      if (L < Lmin) { const s = Lmin / Math.max(L, 1e-6); dr *= s; dh *= s; L = Lmin; }
      const gam = Math.atan2(dh, dr);
      const cb = Math.min(1, Math.max(-1, (L * L + A2 * A2 - A3 * A3) / (2 * L * A2)));
      const beta = Math.acos(cb);
      const ca = Math.min(1, Math.max(-1, (A2 * A2 + A3 * A3 - L * L) / (2 * A2 * A3)));
      const alpha = Math.acos(ca);
      for (const up of [1, -1]) {
        const th2 = up > 0 ? gam + beta : gam - beta;
        const th3 = up > 0 ? (alpha - Math.PI) : (Math.PI - alpha);
        const q2 = th2 - DH[1][3], q3 = th3 - DH[2][3];
        if (q2 < LIMITS[1][0] || q2 > LIMITS[1][1]) continue;
        if (q3 < LIMITS[2][0] || q3 > LIMITS[2][1]) continue;
        // reject postures whose elbow would collide with the table
        const elbowY = fk([J1, q2, 0, 0, 0, 0]).frames[2].o[1];
        if (elbowY < 0.045) continue;
        out.push({ J1: J1, q2: q2, q3: q3, elbow: up });
      }
    }
    return out;
  }

  function transpose3(R) { return [[R[0][0], R[1][0], R[2][0]], [R[0][1], R[1][1], R[2][1]], [R[0][2], R[1][2], R[2][2]]]; }

  // Full analytic IK. The wrist rotation relative to frame 3 is
  //   Rz(t4) Rx(+90) Rz(t5) Rx(-90) Rz(t6) = Rz(t4) Ry(-t5) Rz(t6)
  // a ZYZ Euler family, solved in closed form. The wrist point is
  //   W = palm - TOOLP*z5 - D5W*z4
  // where z5 is known (target tool z) but z4 lies anywhere on the circle
  // perpendicular to z5, so we sample that circle and run a fixed-point
  // refinement on z4 (the offset is small, contraction is fast).
  function analyticIk(targetP, targetR) {
    const cands = [];
    const z5t = [targetR[0][2], targetR[1][2], targetR[2][2]];
    const W0 = [
      targetP[0] - TOOLP * z5t[0],
      targetP[1] - TOOLP * z5t[1],
      targetP[2] - TOOLP * z5t[2],
    ];
    // orthonormal basis of the plane perpendicular to z5
    const ref = Math.abs(z5t[1]) < 0.9 ? [0, 1, 0] : [1, 0, 0];
    let u1 = cross(z5t, ref);
    const nu = norm(u1) || 1;
    u1 = [u1[0] / nu, u1[1] / nu, u1[2] / nu];
    const u2 = cross(z5t, u1);

    const seen = new Set();
    function tryCandidate(q) {
      for (let j = 0; j < 6; j++) {
        if (q[j] < LIMITS[j][0] || q[j] > LIMITS[j][1]) return null;
      }
      const key = q.map((v) => v.toFixed(3)).join(',');
      if (seen.has(key)) return null;
      seen.add(key);
      const f = fk(q);
      const dp = sub(targetP, f.p);
      const dr = orientError(targetR, f.R);
      const e = norm(dp) + norm(dr);
      cands.push({ q: q, err: e });
      return { q: q, err: e };
    }

    function solveWithZ4(z4init) {
      let z4 = z4init;
      for (let iter = 0; iter < 3; iter++) {
        const W = [
          W0[0] - D5W * z4[0],
          W0[1] - D5W * z4[1],
          W0[2] - D5W * z4[2],
        ];
        const branches = positionBranches(W);
        let bestLocal = null;
        for (const br of branches) {
          const f3 = fk([br.J1, br.q2, br.q3, 0, 0, 0]).frames[3];
          const Rrel = matMul(transpose3(f3.R), targetR);
          for (const bSign of [1, -1]) {
            const cosb = Math.min(1, Math.max(-1, Rrel[2][2]));
            const b = bSign * Math.acos(cosb);
            const sb = Math.sin(b);
            let th4, th5, th6;
            if (Math.abs(sb) < 1e-7) {
              th4 = 0;
              if (b > 0) th6 = Math.atan2(Rrel[1][0], Rrel[0][0]);
              else th6 = Math.atan2(Rrel[1][0], -Rrel[0][0]);
            } else {
              th4 = Math.atan2(Rrel[1][2] / sb, Rrel[0][2] / sb);
              th6 = Math.atan2(Rrel[2][1] / sb, -Rrel[2][0] / sb);
            }
            th5 = -b;
            const q4 = normalizeAngle(th4 - DH[3][3]);
            const q5 = normalizeAngle(th5 - DH[4][3]);
            const q6 = normalizeAngle(th6 - DH[5][3]);
            const r = tryCandidate([br.J1, br.q2, br.q3, q4, q5, q6]);
            if (r && (!bestLocal || r.err < bestLocal.err)) bestLocal = r;
          }
        }
        if (!bestLocal) return;
        if (bestLocal.err < 6e-4) return; // exact branch found
        // fixed-point refinement: use the true J5 axis of the best branch
        const z4New = fk(bestLocal.q).joints[4].z;
        if (norm(sub(z4New, z4)) < 1e-5) return;
        z4 = z4New;
      }
    }

    // sample the z4 circle (24 azimuths) then refine
    const NPHI = 24;
    for (let k = 0; k < NPHI; k++) {
      const phi = (k / NPHI) * 2 * Math.PI;
      const c = Math.cos(phi), s = Math.sin(phi);
      solveWithZ4([
        u1[0] * c + u2[0] * s,
        u1[1] * c + u2[1] * s,
        u1[2] * c + u2[2] * s,
      ]);
    }
    cands.sort((a, b) => a.err - b.err);
    return cands;
  }

  // ---------------- posture preference ----------------
  // Among equally-exact IK branches choose the one that (a) keeps the
  // elbow UP (q3 < 0 -> natural industrial posture, no drooping
  // "kneeling" arm), (b) stays close to the previous configuration
  // (continuity -> no dramatic re-branches between actions), and
  // (c) keeps J4 / base yaw in comfortable ranges.
  // Signed height of the elbow above the shoulder->wrist chord
  // (positive = elbow held up = natural industrial posture; negative =
  // arm sagging towards the table, the ugly "kneeling" look).
  function elbowAboveChord(q, f) {
    const fr = (f || fk(q)).frames;
    const sh = fr[1].o, el = fr[2].o, wr = fr[4].o;
    const dw = Math.hypot(wr[0] - sh[0], wr[2] - sh[2]);
    const t = dw > 1e-6 ? Math.min(1, Math.hypot(el[0] - sh[0], el[2] - sh[2]) / dw) : 0;
    const chordY = sh[1] + t * (wr[1] - sh[1]);
    return el[1] - chordY;
  }

  function postureCost(q, ref) {
    // continuity weights: J1/J4 flips look worst, J3 (elbow branch) is
    // deliberately cheap so the solver is willing to re-branch
    const w = [0.8, 1.0, 0.7, 0.9, 0.6, 0.3];
    let c = 0;
    for (let j = 0; j < 6; j++) {
      let d = q[j] - ref[j];
      if (j === 5) d = normalizeAngle(d); // J6 wrap-aware
      c += w[j] * d * d;
    }
    const f = fk(q);
    const eac = elbowAboveChord(q, f);
    if (eac < 0) c += 6.0;             // elbow below chord: drooping arm
    else if (eac < 0.015) c += 2.0;    // barely above: still saggy
    if (f.frames[2].o[1] < 0.14) c += 4.0; // elbow dipping toward the riser
    if (Math.abs(q[0]) > 2.1) c += 1.2; // base-yaw wrap / flipped azimuth
    if (q[1] > 1.05) c += 0.6;          // shoulder pitched far forward (sag)
    if (q[1] < -2.4) c += 0.4;          // shoulder strained backwards
    if (Math.abs(q[3]) > 2.3) c += 0.3; // wrist-pitch strain
    // keep-out zones: any link envelope penetrating a workcell obstacle
    // (cube, pedestal, riser) is disqualifying; a thin clearance buffer is
    // penalized so the solver prefers comfortably clear postures. The
    // penalties dominate the continuity terms (which live on the ~1-30
    // scale) so a colliding branch is only ever chosen when no clear
    // branch exists at all.
    if (WORLD.length) {
      const cl = linkClearance(q, f);
      if (cl < 0) c += 4000.0 + 20000.0 * (-cl);      // hard: penetration
      else if (cl < 0.004) c += 80.0;                 // grazing
      else if (cl < 0.012) c += 15.0;                 // thin margin
    }
    return c;
  }

  function basisMat(ex, ey, ez) {
    return [[ex[0], ey[0], ez[0]], [ex[1], ey[1], ez[1]], [ex[2], ey[2], ez[2]]];
  }

  // HOME: compact guard pose — gripper parked low-left of the cube,
  // base facing down, jaws toward the front. Solved once (analytic seeds +
  // damped least squares) so the home configuration is a genuinely
  // reachable, elbow-up posture.
  function computeHome() {
    // standby pose: gripper parked in front-left of the cube, jaws
    // toward the front, base facing down (same orientation family as
    // the working grasps, so homing never reconfigures the wrist)
    const p = [0.105, 0.245, 0.128];
    const R = basisMat([1, 0, 0], [0, 0, 1], [0, -1, 0]);
    const ref = [0, -0.5, 1.0, -0.3, -0.9, 0];
    const pool = [];
    const cands = analyticIk(p, R);
    for (const c of cands) if (c.err < 1e-3) pool.push(c.q);
    // DLS refinements from the strongest analytic candidates + fixed seeds
    const seeds = [];
    for (const c of cands.slice(0, 10)) seeds.push(c.q);
    seeds.push([0, -0.5, 1.1, -0.3, -0.9, 0]);
    seeds.push([0, -0.2, 1.5, -0.2, -1.3, 0]);
    seeds.push([0.4, -0.6, 1.0, -0.4, -1.0, 0]);
    seeds.push([-0.4, -0.6, 1.0, -0.4, -1.0, 0]);
    for (const q0 of seeds) {
      const rA = dlsDescend(q0.slice(), p, R, 0.05, 60);
      const rB = dlsDescend(rA.q, p, R, 1.0, 260);
      if (rB.ok || rB.err < 6e-4) pool.push(rB.q);
    }
    if (pool.length) {
      let best = pool[0], bc = Infinity;
      for (const q of pool) {
        const pc = postureCost(q, ref);
        if (pc < bc) { bc = pc; best = q; }
      }
      return best.slice();
    }
    return [0, -40 * DEG, -75 * DEG, -25 * DEG, -80 * DEG, 0];
  }
  const HOME = computeHome();

  function positionSeeds(W) {
    const seeds = [];
    for (const br of positionBranches(W)) {
      for (const w of [[0, 0], [0, Math.PI / 2], [0, -Math.PI / 2], [Math.PI / 2, 0], [-Math.PI / 2, 0]]) {
        seeds.push([br.J1, br.q2, br.q3, w[0], w[1], 0]);
      }
    }
    return seeds;
  }

  function ik(targetP, targetR, qSeed, opts) {
    opts = opts || {};
    let best = null;
    // requireClear: a solution whose arm envelope penetrates a keep-out
    // zone is not acceptable — used by reachability probes so the task
    // planner never schedules grasps that would sweep a link through the
    // workcell (exactly what a real cell checks offline)
    const needClear = opts.requireClear && WORLD.length > 0;
    const clearEnough = (q) => !needClear || linkClearance(q) > 0.0015;
    // wristSide: prefer the branch family whose J5 reading has this sign
    // (the two wrist branches reach the same tool pose; whole-cube rolls
    // about the world Z axis stay collision-free only in one of them)
    const side = opts.wristSide || 0;

    // J6 (tool roll) is 2*pi-periodic: represent the solution with the
    // branch nearest the seed so consecutive IK calls never produce
    // multi-turn J6 unwinds — the wrist stays continuous like a real
    // controller keeping its joint readings continuous.
    function wrapQ6(q) {
      if (!qSeed) return q;
      const out = q.slice();
      let d = out[5] - qSeed[5];
      d -= Math.PI * 2 * Math.round(d / (Math.PI * 2));
      let v = qSeed[5] + d;
      if (v > Math.PI * 2) v -= Math.PI * 2;
      else if (v < -Math.PI * 2) v += Math.PI * 2;
      out[5] = v;
      return out;
    }

    function record(res) {
      if (!best || res.err < best.err) best = { q: res.q.slice(), err: res.err };
    }

    // tracking mode: single descent from the seed (Cartesian path following)
    if (opts.track && qSeed) {
      const r = dlsDescend(qSeed.slice(), targetP, targetR, 1.0, 140);
      return { ok: r.ok || r.err < 3e-3, q: wrapQ6(r.q), err: r.err };
    }

    // ---- analytic IK first (exact closed-form branches) ----
    const cands = analyticIk(targetP, targetR);
    if (cands.length && cands[0].err < 1e-3) {
      // pick the exact branch with the best posture / continuity
      let exact = cands.filter((c) => c.err < 1e-3 && clearEnough(c.q));
      if (side && exact.length) {
        const hinted = exact.filter((c) => Math.sign(c.q[4]) === side);
        if (hinted.length) exact = hinted;
      }
      if (exact.length) {
        const ref = qSeed || HOME;
        let best = exact[0], bc = Infinity;
        for (const c of exact) {
          const pc = postureCost(c.q, ref);
          if (pc < bc) { bc = pc; best = c; }
        }
        return { ok: true, q: wrapQ6(best.q), err: best.err, analytic: true };
      }
      // no clear exact branch: fall through to the DLS search below
    }
    for (const c of cands) if (c.err < 0.5) record(c);

    const seeds = [];
    if (qSeed) seeds.push(qSeed.slice());
    seeds.push(HOME.slice());
    const v1 = HOME.slice(); v1[0] += 0.6; seeds.push(v1);
    const v2 = HOME.slice(); v2[0] -= 0.6; seeds.push(v2);
    const v3 = HOME.slice(); v3[1] -= 0.4; v3[2] -= 0.4; seeds.push(v3);
    const v4 = [0, -1.2, -1.4, -0.4, -1.2, 0]; seeds.push(v4);
    // analytic candidates double as DLS seeds
    for (const c of cands) if (c.err < 0.5) seeds.push(c.q);
    if (opts.extraSeeds) for (const s of opts.extraSeeds) seeds.push(s);
    // elbow-up biased random seeds (posture family used by this arm)
    const nBias = opts.randomRestarts == null ? 4 : Math.max(2, opts.randomRestarts);
    for (let i = 0; i < nBias; i++) {
      seeds.push([
        (Math.random() - 0.5) * 1.6,
        -0.4 - Math.random() * 1.2,          // shoulder up-forward
        (Math.random() < 0.5 ? -1 : 1) * (0.5 + Math.random() * 1.6), // elbow either branch
        (Math.random() - 0.5) * 2.4,
        (Math.random() - 0.5) * 3.4,
        (Math.random() - 0.5) * 3.0,
      ]);
    }
    // a few uniform random seeds
    for (let i = 0; i < 2; i++) {
      seeds.push(LIMITS.map((L) => L[0] + Math.random() * (L[1] - L[0])));
    }

    // pool every converged solution, then pick the best posture
    const pool = [];
    const ref = qSeed || HOME;
    for (const q0 of seeds) {
      // stage A: position-priority descent (wide basin)
      const rA = dlsDescend(q0.slice(), targetP, targetR, 0.05, 60);
      record(rA);
      // stage B: full pose descent from stage-A result
      const rB = dlsDescend(rA.q, targetP, targetR, 1.0, 260);
      record(rB);
      if (rB.ok || rB.err < 6e-4) pool.push(rB.q);
      if (pool.length >= 4) break; // enough good postures to choose from
    }
    if (pool.length) {
      const clearPool = needClear ? pool.filter((q) => clearEnough(q)) : pool;
      if (clearPool.length) {
        let bq = clearPool[0], bc = Infinity;
        for (const q of clearPool) {
          const pc = postureCost(q, ref);
          if (pc < bc) { bc = pc; bq = q; }
        }
        return { ok: true, q: wrapQ6(bq), err: 0, analytic: false };
      }
      // only colliding postures available and clearance was required
      if (needClear) return { ok: false, q: null, err: Infinity };
    }
    if (needClear && best && clearEnough(best.q)) {
      return { ok: best.err < 6e-4, q: wrapQ6(best.q), err: best.err };
    }
    return { ok: !!(best && best.err < 6e-4), q: best ? wrapQ6(best.q) : null, err: best ? best.err : Infinity };
  }

  function solve6(A, b) {
    const M = A.map((r, i) => r.slice().concat([b[i]]));
    for (let c = 0; c < 6; c++) {
      let piv = c, mx = Math.abs(M[c][c]);
      for (let r = c + 1; r < 6; r++) {
        const v = Math.abs(M[r][c]);
        if (v > mx) { mx = v; piv = r; }
      }
      if (mx < 1e-12) return null;
      if (piv !== c) { const t = M[piv]; M[piv] = M[c]; M[c] = t; }
      for (let r = 0; r < 6; r++) {
        if (r === c) continue;
        const f = M[r][c] / M[c][c];
        if (f === 0) continue;
        for (let k = c; k <= 6; k++) M[r][k] -= f * M[c][k];
      }
    }
    return [0, 1, 2, 3, 4, 5].map((i) => M[i][6] / M[i][i]);
  }

  // ---------------- trapezoidal joint trajectories ----------------
  // Minimum-time profile for one joint, then synchronized duration.
  function jointTime(dq, vmax, amax) {
    const d = Math.abs(dq);
    if (d < 1e-9) return 0;
    if (d >= vmax * vmax / amax) return d / vmax + vmax / amax;
    return 2 * Math.sqrt(d / amax);
  }

  function planJointMove(q0, q1) {
    const T = Math.max(0.035);
    let tMax = 0.035;
    for (let j = 0; j < 6; j++) tMax = Math.max(tMax, jointTime(q1[j] - q0[j], VMAX[j], AMAX[j]));
    // per-joint trapezoid with common duration tMax
    const seg = [];
    for (let j = 0; j < 6; j++) {
      const dq = q1[j] - q0[j];
      const disc = tMax * tMax - 4 * Math.abs(dq) / AMAX[j];
      if (Math.abs(dq) < 1e-12) { seg.push({ dq: 0, vc: 0, tr: 0 }); continue; }
      const tr = disc <= 0 ? tMax / 2 : (tMax - Math.sqrt(Math.max(disc, 0))) / 2;
      const vc = Math.abs(dq) / (tMax - tr);
      seg.push({ dq: dq, vc: vc, tr: tr });
    }
    return { T: tMax, seg: seg, q0: q0.slice(), q1: q1.slice() };
  }

  // position on the synchronized trapezoid profile at time t
  function evalJointMove(plan, t) {
    const q = [0, 0, 0, 0, 0, 0];
    const T = plan.T;
    const tc = Math.min(Math.max(t, 0), T);
    for (let j = 0; j < 6; j++) {
      const s = plan.seg[j];
      if (s.dq === 0) { q[j] = plan.q0[j]; continue; }
      const a = s.vc / s.tr; // accel magnitude (unsigned)
      const d = Math.sign(s.dq);
      let dist;   // unsigned distance travelled along the move
      if (tc < s.tr) dist = 0.5 * a * tc * tc;
      else if (tc < T - s.tr) dist = s.vc * tc - 0.5 * s.vc * s.tr;
      else {
        // deceleration ramp: remaining distance to the target
        const t2 = T - tc;
        dist = Math.abs(s.dq) - 0.5 * a * t2 * t2;
        if (dist < 0) dist = 0;
      }
      q[j] = plan.q0[j] + d * dist;
    }
    return q;
  }

  function clampQ(q) {
    return q.map((v, j) => Math.min(Math.max(v, LIMITS[j][0]), LIMITS[j][1]));
  }

  function inLimits(q) {
    for (let j = 0; j < 6; j++) if (q[j] < LIMITS[j][0] - 1e-9 || q[j] > LIMITS[j][1] + 1e-9) return false;
    return true;
  }

  // RPY (fixed-axis XYZ) for telemetry display
  function rotToRPY(R) {
    const sy = -R[2][0];
    const y = Math.asin(Math.min(1, Math.max(-1, sy)));
    let p, r;
    if (Math.abs(sy) < 0.99999) {
      p = Math.atan2(R[2][1], R[2][2]);
      r = Math.atan2(R[1][0], R[0][0]);
    } else {
      p = Math.atan2(-R[1][2], R[1][1]);
      r = 0;
    }
    return [r, p, y];
  }

  return {
    DH: DH, TOOL: TOOL, LIMITS: LIMITS, VMAX: VMAX, AMAX: AMAX, HOME: HOME,
    BASE_R: BASE_R, BASE_P: BASE_P, DEG: DEG,
    setWorld: setWorld, getWorld: getWorld, linkClearance: linkClearance, LINK_SEG: LINK_SEG,
    fk: fk, ik: ik, analyticIk: analyticIk, positionBranches: positionBranches,
    planJointMove: planJointMove, evalJointMove: evalJointMove,
    clampQ: clampQ, inLimits: inLimits, rotToRPY: rotToRPY,
    quatFromMat: quatFromMat, matFromQuat: matFromQuat, slerpR: slerpR,
    matMul: matMul, matVec: matVec,
  };
});
