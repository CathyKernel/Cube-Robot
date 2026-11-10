/* =============================================================
 * planner.js — robot task planner
 *
 * Converts a solution (list of quarter turns on canonical faces)
 * into an executable robot program (action list) for the
 * 6-DOF arm + two-finger adaptive gripper.
 *
 * Manipulation strategy (fast single-hand cell):
 *   - Face turns are executed as DIRECT layer twists wherever the face
 *     is presented: the gripper approaches along the face normal (base
 *     centred on the normal ray, J6 as the twist axis), pinches the
 *     layer between the two jaw pads on its opposite side faces, and
 *     rolls J6 in place on the spike fixture. The TOP face uses the
 *     classic overhead grip; the four SIDE faces use a compact
 *     closer-base grip that stays inside the arm workspace on the far
 *     side of the cube.
 *   - Faces that point DOWN (or geometrically unreachable sides) are
 *     brought into reach by whole-cube reorientations: grip the cube at
 *     its equator, lift, roll +/-90 deg about a horizontal world axis
 *     (grip faces chosen parallel to the rotation axis so they stay on
 *     the sides), place back, release.
 *   - WHERE to roll is decided by a dynamic program over the 24-element
 *     orientation group: it schedules the globally MINIMUM number of
 *     rolls for the whole solution (including the final restore to the
 *     canonical orientation) instead of re-orienting greedily per move.
 *
 * Gripper grasp planning: each jaw is a planar MCP-PIP two-phalanx
 * chain; closed-form IK places the jaw pad exactly on the target
 * contact point.
 * ============================================================= */
(function (root, factory) {
  const CubeCoreRef = (typeof root.CubeCore !== 'undefined')
    ? root.CubeCore
    : require('./cube-core.js');
  const api = factory(CubeCoreRef);
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
  else root.RobotPlan = api;
})(typeof self !== 'undefined' ? self : globalThis, function (CubeCore) {
  'use strict';

  // ---------------- shared robot parameters (metres) ----------------
  // Workcell layout: the arm is mounted on a 0.12 m riser next to a
  // 0.13 m pedestal, so the cube is worked at chest height (y = 0.16)
  // and the arm stays in compact elbow-up postures.
  const ROBOT = {
    basePos: [0, 0.12, 0],        // robot riser top (matches kinematics)
    cubePos: [0.19, 0.16, 0],     // cube centre atop the pedestal
    cubeSize: 0.06,
    hover: 0.030,                  // pre-approach height above a top grasp
    hoverSide: 0.020,              // pre-approach standoff for side grasps
    extract: 0.0,                  // (retired) side layers now twist in place
                                     // on the spike fixture — no pull-out needed
    lift: 0.024,                   // whole-cube lift height
    palmH_side: 0.056,            // base height for compact side layer grasps
    // eye-in-hand camera: compact smart camera bracketed to the
    // gripper base on the +x side (clear of both jaws), boresight
    // tilted inward toward the tool axis — the classic wrist-camera
    // arrangement that watches the workspace around the gripper.
    handCam: {
      off: [0.058, 0.004, -0.012],     // mount point in the hand frame
      tilt: (16 * Math.PI) / 180,      // boresight tilt about the hand y axis
      L: 0.155,                        // standoff: optical centre -> face centre
      fov: 52, w: 320, h: 240,         // sensor model for the live feed
    },
    palmH_layer: 0.088,           // base centre above cube centre, layer grasp
    palmH_body: 0.058,            // base centre above cube centre, body grasp
    hand: {
      // two-finger adaptive gripper (Robotiq-2F class): a machined
      // base housing on the J6 flange carries two mirrored jaws,
      // each a pivoting MCP knuckle driving a two-phalanx finger
      // (proximal + distal) with a rubber volar pad — one jaw presses
      // each opposite side face of the gripped layer/body.
      baseW: 0.076, baseD: 0.050, baseT: 0.024,  // base housing (x, y, z)
      fingerL: [0.034, 0.030],   // proximal, distal phalanx per jaw
      padR: 0.0035,               // jaw pad half-thickness
      mcpFront: 0.026,            // knuckle offset from base centre along ±y
      mcpDrop: 0.009,             // knuckle offset toward the object side (+z)
    },
  };
  const H = ROBOT.hand;
  const HALF = ROBOT.cubeSize / 2;

  const DEG = Math.PI / 180;

  // ---------------- gripper: 2-phalanx jaw IK ----------------
  // Jaw local frame (attached to the gripper base): x = cross-axis,
  // y = jaw travel (toward the object side faces), z = object side
  // (base normal direction). dir(theta) = (0, cos theta, sin theta).
  // Jaw A mounts at the +y edge, jaw B at the -y edge (mirrored in y);
  // both curl toward the object side and press the two opposite
  // side faces of the gripped layer/body at the same height.
  //
  // Each jaw is a closed-form 2R chain (MCP knuckle + PIP hinge), so
  // the jaw pad lands EXACTLY on the planned contact point.
  function fingerIk(mcp, target, L1, L2, mirrored) {
    const s = mirrored ? -1 : 1;
    // work in the (y,z) plane
    const dy = (target[1] - mcp[1]) * s;
    const dz = target[2] - mcp[2];
    const d = Math.hypot(dy, dz);
    const dMin = Math.abs(L1 - L2) + 0.0015;
    const dMax = L1 + L2 - 0.0015;
    const dc = Math.min(Math.max(d, dMin), dMax);
    const dy2 = dy * (dc / (d || 1)), dz2 = dz * (dc / (d || 1));
    const phi = Math.atan2(dz2, dy2);
    // interior angle at PIP
    const cosInt = (L1 * L1 + L2 * L2 - dc * dc) / (2 * L1 * L2);
    const Int = Math.acos(Math.min(1, Math.max(-1, cosInt)));
    const theta2 = Math.PI - Int; // positive = curl toward object side
    const cosA = (dc * dc + L1 * L1 - L2 * L2) / (2 * dc * L1);
    const A = Math.acos(Math.min(1, Math.max(-1, cosA)));
    // choose the branch that keeps the knuckle forward (natural curl)
    const theta1 = phi - A;
    // t1 = MCP knuckle reading, t2 = PIP hinge reading (relative)
    return { t1: theta1, t2: Math.max(0, theta2), clamped: d < dMin || d > dMax };
  }

  // 2R jaw FK (for contact verification)
  function fingerFk(mcp, a, L1, L2, mirrored) {
    const s = mirrored ? -1 : 1;
    const p1 = [mcp[0], mcp[1] + s * L1 * Math.cos(a.t1), mcp[2] + L1 * Math.sin(a.t1)];
    const tip = [mcp[0], mcp[1] + s * (L1 * Math.cos(a.t1) + L2 * Math.cos(a.t1 + a.t2)),
                 mcp[2] + L1 * Math.sin(a.t1) + L2 * Math.sin(a.t1 + a.t2)];
    return { p1: p1, tip: tip };
  }

  // Compute jaw joint angles for a grasp mode, in the gripper-local
  // frame. mode 'layer': pads on the two opposite side faces of the
  //   TOP layer (contact height = cube centre + a/3).  mode 'body':
  //   pads on the equator faces of the whole cube.  `palmH` overrides
  //   the base height above the cube centre (compact side grasps pull
  //   the base in so the far-side approach stays in workspace).
  function planGrip(mode, palmH) {
    const ph = mode === 'layer'
      ? (palmH != null ? palmH : ROBOT.palmH_layer)
      : ROBOT.palmH_body;
    const contactAlongFront = HALF + H.padR;
    const contactDown = mode === 'layer'
      ? ph - ROBOT.cubeSize / 3
      : ph;
    const angles = { fingers: [], mode: mode, contacts: [] };
    // jaw A (+y edge) and jaw B (mirrored at the -y edge): a symmetric
    // two-jaw pinch — both pads press the layer's opposite side faces
    // at the same height
    const mcpA = [0, H.mcpFront, H.mcpDrop];
    const solA = fingerIk(mcpA, [0, contactAlongFront, contactDown], H.fingerL[0], H.fingerL[1], false);
    angles.fingers.push(solA);
    angles.contacts.push(fingerFk(mcpA, solA, H.fingerL[0], H.fingerL[1], false).tip);
    const mcpB = [0, -H.mcpFront, H.mcpDrop];
    const solB = fingerIk(mcpB, [0, -contactAlongFront, contactDown], H.fingerL[0], H.fingerL[1], true);
    angles.fingers.push(solB);
    angles.contacts.push(fingerFk(mcpB, solB, H.fingerL[0], H.fingerL[1], true).tip);
    return angles;
  }

  // relaxed open gripper: jaws extended to full stroke, slight PIP fold
  const OPEN_POSE = { t1: 0.06, t2: 0.14 };

  // standby (guard) pose: gripper parked in front-left of the cube with
  // the same orientation family as the working grasps — homing to this
  // pose never reconfigures the wrist. (Mirrors RobotKin.HOME.)
  const GUARD = {
    p: [0.105, 0.245, 0.128],
    R: rotFromBasis([1, 0, 0], [0, 0, 1], [0, -1, 0]),
  };

  // ---------------- grasp poses (TCP = gripper base centre) ----------------
  // R columns = [e_x (cross), e_y (jaw travel), e_z (base normal / approach)]
  function rotFromBasis(ex, ey, ez) {
    return [[ex[0], ey[0], ez[0]], [ex[1], ey[1], ez[1]], [ex[2], ey[2], ez[2]]];
  }
  function vadd(a, b) { return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]; }
  function vsub(a, b) { return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]; }
  function vmul(a, s) { return [a[0] * s, a[1] * s, a[2] * s]; }
  function vdot(a, b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
  function cross(a, b) {
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
  }

  // Layer-turn grasp for ANY world face direction w: the gripper base
  // centre rides the face-normal ray, base normal ez = -w points at the
  // face, the two jaws run along ey (perpendicular to the twist axis)
  // and J6 IS the twist axis.
  //   - top (w = +Y): classic overhead layer grip, jaws toward +Z
  //   - sides: compact grip (closer base) with the jaws wrapping the
  //     layer's side faces; hover and extraction distances tuned per family.
  function layerTwistPose(w) {
    w = w || [0, 1, 0];
    const top = w[1] !== 0;
    const ez = vmul(w, -1);
    const ey = top ? [0, 0, 1] : cross(w, CubeCore.UP);
    const ex = cross(ey, ez);
    const palmH = top ? ROBOT.palmH_layer : ROBOT.palmH_side;
    const hover = top ? ROBOT.hover : ROBOT.hoverSide;
    const p = vadd(ROBOT.cubePos, vmul(w, palmH));
    return {
      p: p, R: rotFromBasis(ex, ey, ez), w: w, top: top,
      hoverP: vadd(p, vmul(w, hover)),
      extP: vadd(p, vmul(w, ROBOT.extract)),
    };
  }

  // canonical overhead grasp (kept for compatibility / tests)
  function layerGraspPose() {
    const g = layerTwistPose([0, 1, 0]);
    return { p: g.p, R: g.R };
  }

  // Body grasp for a roll about world axis ('X' or 'Z'):
  // grip the face pair whose normals are parallel to the rotation axis
  // (those faces are invariant under the roll, so no pad ends up under
  // the cube).
  function bodyGraspPose(axis) {
    const ez = [0, -1, 0];
    const ey = axis === 'X' ? [1, 0, 0] : [0, 0, 1];
    const ex = cross(ey, ez);
    return {
      p: vadd(ROBOT.cubePos, [0, ROBOT.palmH_body, 0]),
      R: rotFromBasis(ex, ey, ez),
    };
  }

  // ---------------- action emitters ----------------
  // Action kinds:
  //  movej {p,R}            coordinated joint move to IK(p,R)
  //  movel {p,R,gripTo?}    linear Cartesian move (jaws may blend to
  //                          gripTo while moving — real controllers overlap
  //                          gripper actuation with motion)
  //  grip {mode|'open'}     standalone jaw animation
  //  attachLayer / attachCube / snapLayer {face,q} / snapCube {M}
  //  twist {dq}             J6-only roll while holding the layer
  //  move {face,q,fast}     logical (and fast visual) face turn, no robot
  //  setOrient {M}          update tracked cube orientation
  //  home / phase {name} / done

  // Layer turn for ANY world face direction w: the gripper base centre
  // rides the face-normal ray, base normal ez = -w points at the face,
  // the jaws run along ey (perpendicular to the twist axis) and J6 IS
  // the twist axis. The two jaw pads pinch the layer's opposite side
  // faces and roll J6 in place — the slim spike fixture keeps the
  // swept layer clear of the support, so the layer NEVER leaves the cube
  // (no extract / reinsert cycle: fewer actions, visibly cleaner twist).
  // A CCW quarter turn drives J6 by -90 degrees (equivalent to +270 for
  // the settled layer) — the short way round, like a real controller's
  // shortest-path roll. The release is folded into the NEXT approach:
  // the jaws open while the gripper flies off (gripper/motion overlap),
  // so there is no separate retract move.
  function emitTwist(acts, face, q, w) {
    w = w || [0, 1, 0];
    const g = layerTwistPose(w);
    const dir = w[1] === 1 ? 'top'
      : (w[0] === 1 ? 'right' : (w[0] === -1 ? 'left' : (w[2] === 1 ? 'front' : 'back')));
    const spins = q === 2 ? '180' : (q === 1 ? '90 CW' : '90 CCW');
    const dq = q === 3 ? -Math.PI / 2 : q * Math.PI / 2;
    acts.push({ k: 'movej', p: g.hoverP, R: g.R, gripTo: 'open', tag: 'APPROACH',
                label: 'Approach ' + dir + ' layer' });
    acts.push({ k: 'movel', p: g.p, R: g.R, gripTo: g.top ? 'layer' : 'side', tag: 'GRASP',
                label: 'Descend & grip ' + dir + ' layer' });
    acts.push({ k: 'attachLayer', w: w, tag: 'GRASP' });
    acts.push({ k: 'twist', dq: dq, tag: 'TWIST',
                label: 'Twist ' + dir + ' layer ' + spins });
    acts.push({ k: 'snapLayer', face: face, q: q, tag: 'TWIST', label: 'Settle layer' });
  }

  // ---------------- whole-cube reorientation ----------------
  // Feasible roll generators (workspace analysis): X+/- rolls park the
  // wrist comfortably on the front/back side; Z- rolls orbit the gripper to
  // the RIGHT side and stay clear in the flipped wrist branch (J5 > 0).
  // A Z+ roll ends with the wrist 80 mm from the base axis — inside the
  // arm's folded dead zone where every IK branch sweeps a link through
  // the held cube — so Z+ is excluded and the orientation scheduler
  // plans around it (the X+/- and Z- generators still present every face
  // within two rolls, verified over the whole 24-orientation group).
  const ROLL_GENS = ['X+', 'X-', 'Z-'];

  // BFS over the feasible generators (used by the safety fallback)
  function bfsRolls(M0, isGoal) {
    if (isGoal(M0)) return [];
    const info = new Map();
    const key = (M) => M[0].join(',') + ';' + M[1].join(',') + ';' + M[2].join(',');
    info.set(key(M0), { parent: null, gen: null });
    let frontier = [M0];
    for (let depth = 1; depth <= 4; depth++) {
      const next = [];
      for (const M of frontier) {
        for (const g of ROLL_GENS) {
          const M2 = CubeCore.matMul(CubeCore.GEN[g], M);
          const k = key(M2);
          if (info.has(k)) continue;
          info.set(k, { parent: key(M), gen: g });
          if (isGoal(M2)) {
            const seq = [];
            let cur = k;
            while (cur !== key(M0)) {
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
    return null;
  }

  // rotation about a world axis by an arbitrary angle (for roll checkpoints)
  function rotAxisMat(axis, ang) {
    const c = Math.cos(ang), s = Math.sin(ang);
    if (axis === 'X') return [[1, 0, 0], [0, c, -s], [0, s, c]];
    return [[c, -s, 0], [s, c, 0], [0, 0, 1]];
  }

  function emitReorient(acts, gen, M) {
    // gen: one of 'X+','X-','Z-' — world rotation applied to the cube.
    // Z- rolls keep the links clear only in the flipped wrist branch,
    // so the approach + grasp are planned with a J5 > 0 branch hint.
    const axis = gen[0];
    const G = CubeCore.GEN[gen];
    const wristSide = gen === 'Z-' ? 1 : 0;
    const g = bodyGraspPose(axis);
    const hover = { p: vadd(g.p, [0, ROBOT.hover, 0]), R: g.R };
    const liftP = vadd(g.p, [0, ROBOT.lift, 0]);
    // rotated pose about the lifted cube centre:
    //   p_rot = P_lift + G * (p_lift - P_lift),  R_rot = G * R
    const pLiftCube = vadd(ROBOT.cubePos, [0, ROBOT.lift, 0]);
    const rel = [0, ROBOT.palmH_body, 0];
    const relRot = CubeCore.matVec(G, rel);
    const pRot = vadd(pLiftCube, relRot);
    const RRot = CubeCore.matMul(G, g.R);
    const pPlace = vsub(pRot, [0, ROBOT.lift, 0]);
    const M2 = CubeCore.matMul(G, M);

    acts.push({ k: 'movej', p: hover.p, R: hover.R, tag: 'APPROACH', label: 'Approach cube body',
                wristSide: wristSide || undefined });
    acts.push({ k: 'movel', p: g.p, R: g.R, gripTo: 'body', tag: 'GRASP',
                label: 'Descend & grip cube body', wristSide: wristSide || undefined });
    acts.push({ k: 'attachCube', tag: 'GRASP' });
    acts.push({ k: 'movel', p: liftP, R: g.R, tag: 'LIFT', label: 'Lift cube' });
    // the roll runs as a chain of exact-IK joint moves through 15-degree
    // checkpoints: consecutive targets stay in the same posture family, so
    // the held cube tracks the circular arc closely (a single long joint
    // move sags inward toward the fixture mid-roll)
    const NSTEP = 6;
    for (let s = 1; s <= NSTEP; s++) {
      const Gs = rotAxisMat(axis, (gen[1] === '+' ? 1 : -1) * (Math.PI / 2) * s / NSTEP);
      const relS = CubeCore.matVec(Gs, rel);
      acts.push({ k: 'movej', p: vadd(pLiftCube, relS), R: CubeCore.matMul(Gs, g.R),
                  tag: 'REORIENT', label: 'Roll cube ' + gen + ' (' + Math.round(90 * s / NSTEP) + '°)' });
    }
    acts.push({ k: 'movel', p: pPlace, R: RRot, tag: 'PLACE', label: 'Place cube' });
    acts.push({ k: 'setOrient', M: M2, tag: 'PLACE' });
    acts.push({ k: 'snapCube', tag: 'PLACE', label: 'Settle cube on pedestal' });
    // no separate retract: the next approach / vantage move opens the
    // jaws while the gripper flies off (gripper-motion overlap)
    return M2;
  }

  // ---------------- eye-in-hand vision scan ----------------
  // The camera rides on the gripper: a compact smart camera is bracketed
  // to the base housing on the +x side (clear of both jaws), boresight
  // tilted inward (handCam.tilt) toward the tool axis. Acquiring a face =
  // flying the gripper to an inspection vantage from which the boresight
  // looks straight at that face centre — classic eye-in-hand acquisition,
  // with the two jaws framing the cube in the feed.
  const HC = ROBOT.handCam;
  const B_LOCAL = [-Math.sin(HC.tilt), 0, Math.cos(HC.tilt)];

  function vnorm(v) {
    const n = Math.hypot(v[0], v[1], v[2]) || 1;
    return [v[0] / n, v[1] / n, v[2] / n];
  }

  // material face whose world normal (M . n) equals `dir`
  function slotFace(M, dir) {
    for (let f = 0; f < 6; f++) {
      const w = CubeCore.matVec(M, CubeCore.NORMALS[f]);
      if (Math.round(w[0]) === dir[0] && Math.round(w[1]) === dir[1] && Math.round(w[2]) === dir[2]) return f;
    }
    return -1;
  }

  // Eye-in-hand inspection pose for the face slot with outward world
  // normal `n`, viewed from vantage direction `cd` (face-centre ->
  // camera) at standoff L. The gripper roll is chosen so the camera-side
  // (+x) axis ends up pointing world-up: the jaws then run along
  // fy = cross(up, cd) and the feed shows the face upright.
  // Basis: boresight b = -cd (exactly), with
  //   ex = sin(t) * cd + cos(t) * e2
  //   ez = -cos(t) * cd + sin(t) * e2,   e2 = cd x ey
  // which gives b = -sin(t) ex + cos(t) ez = -cd by construction, and
  // TCP = fc - R (off + b_local * L) puts the optical centre on the
  // boresight through the face centre at distance L.
  function scanPose(n, cd, L) {
    if (L == null) L = HC.L;
    cd = vnorm(cd);
    const fy = Math.abs(cd[1]) > 0.95 ? [1, 0, 0] : vnorm(cross([0, 1, 0], cd));
    const ey = vnorm(vsub(fy, vmul(cd, vdot(fy, cd))));
    const e2 = cross(cd, ey);
    const st = Math.sin(HC.tilt), ct = Math.cos(HC.tilt);
    const ex = vadd(vmul(cd, st), vmul(e2, ct));
    const ez = vadd(vmul(cd, -ct), vmul(e2, st));
    const R = rotFromBasis(ex, ey, ez);
    const fc = vadd(ROBOT.cubePos, [n[0] * HALF, n[1] * HALF, n[2] * HALF]);
    const off = [
      HC.off[0] + B_LOCAL[0] * L,
      HC.off[1] + B_LOCAL[1] * L,
      HC.off[2] + B_LOCAL[2] * L,
    ];
    const rOff = CubeCore.matVec(R, off);
    return { p: vsub(fc, rOff), R: R };
  }

  // The three survey vantages (front / back / top). Side vantages are
  // deliberately NOT used: at this workcell scale they park the hand
  // over the robot's own shoulder or beyond the wrist reach, so the
  // hidden side faces are exposed with whole-cube reorientations
  // instead — exactly how a real eye-in-hand cell handles slots the
  // wrist camera cannot survey directly.
  const SCAN_VANTAGES = {
    front: { d: [0, 0, 1],   cd: [0, 0.62, 1] },
    back:  { d: [0, 0, -1],  cd: [0, 0.62, -1] },
    top:   { d: [0, 1, 0],   cd: [0, 1, 0] },
  };
  const SLOT_NAME = { '0,0,1': 'front', '0,0,-1': 'back', '0,1,0': 'top' };

  function emitRead(acts, M, d, cd, note) {
    const f = slotFace(M, d);
    const pose = scanPose(d, cd);
    acts.push({ k: 'movej', p: pose.p, R: pose.R, gripTo: 'open', tag: 'SCAN',
                label: 'Wrist cam to ' + SLOT_NAME[d.join(',')] + ' vantage' });
    acts.push({ k: 'read', faces: [f], dur: 0.5, tag: 'SCAN',
                label: 'Acquire ' + CubeCore.FACES[f] + ' face' + (note ? ' — ' + note : '') });
  }

  // Eye-in-hand scan program:
  //   1. survey the three directly visible slots (front / back / top)
  //   2. X- roll  bottom -> front slot  (read 4)
  //   3. Z- roll  left   -> top slot    (read 5)
  //   4. X+ roll  right  -> back slot   (read 6)
  // Each roll is the full grasp-lift-roll-place-release
  // cycle, so the scan itself is a manipulation showcase. Slot
  // coverage is orientation-independent (faces are addressed by world
  // slot) and the tracked orientation M flows through setOrient
  // actions, so the follow-up planSolve(moves, RT.M) works from any
  // scan-end orientation.
  function planScan(M0) {
    const acts = [];
    emitRead(acts, M0, SCAN_VANTAGES.front.d, SCAN_VANTAGES.front.cd, null);
    emitRead(acts, M0, SCAN_VANTAGES.back.d, SCAN_VANTAGES.back.cd, null);
    emitRead(acts, M0, SCAN_VANTAGES.top.d, SCAN_VANTAGES.top.cd, null);
    const M1 = emitReorient(acts, 'X-', M0);   // bottom face -> front slot
    emitRead(acts, M1, SCAN_VANTAGES.front.d, SCAN_VANTAGES.front.cd, 'bottom');
    const M2 = emitReorient(acts, 'Z-', M1);   // left face -> top slot
    emitRead(acts, M2, SCAN_VANTAGES.top.d, SCAN_VANTAGES.top.cd, 'left');
    const M3 = emitReorient(acts, 'X+', M2);   // right face -> back slot
    emitRead(acts, M3, SCAN_VANTAGES.back.d, SCAN_VANTAGES.back.cd, 'right');
    acts.push({ k: 'movej', p: GUARD.p, R: GUARD.R, gripTo: 'open', tag: 'RETRACT', label: 'Retract to standby' });
    return acts;
  }

  // ---------------- top-level program builders ----------------
  function planScramble(moves) {
    return moves.map((m) => ({
      k: 'move', face: m.face, q: m.q, fast: true,
      tag: 'SCRAMBLE', label: 'Scramble ' + CubeCore.moveToNotation(m),
    }));
  }

  // moves: simplified solution on canonical faces; M: current orientation;
  // opts.ik: optional reachability probe (p, R) -> bool used to verify the
  // compact side-twist grasp poses before the scheduler will use them.
  //
  // Execution scheduler: a dynamic program over the 24-element cube
  // orientation group. At every solution move the face must be presented
  // top-up (overhead twist) or sideways (direct compact side twist); a
  // face that currently points down (or an unreachable side) is brought
  // into reach by whole-cube rolls BEFORE the twist. The DP searches all
  // roll placements across the whole solution and picks the schedule
  // with the globally minimum number of rolls — including the final
  // restore to the canonical orientation — instead of greedily
  // re-orienting before every off-top move (which is what made the old
  // plan slow: ~1 roll per move, 9 robot actions each).
  function planSolve(moves, M0, opts) {
    const ik = opts && opts.ik;
    const acts = [];
    let M = CubeCore.matEq(M0, CubeCore.IDENT) ? CubeCore.IDENT.map((r) => r.slice()) : M0;
    acts.push({ k: 'movej', p: GUARD.p, R: GUARD.R, tag: 'HOMING', label: 'Standby pose' });

    // ---- workspace probe: which side directions can be twisted in place
    const SIDE_DIRS = [[0, 0, 1], [0, 0, -1], [1, 0, 0], [-1, 0, 0]];
    const sideOk = {};
    for (const w of SIDE_DIRS) {
      const g = layerTwistPose(w);
      sideOk[w.join(',')] = ik
        ? (ik(g.hoverP, g.R) && ik(g.p, g.R) && ik(g.extP, g.R))
        : true;
    }
    // world direction in which `face` is twistable at orientation Mv
    // (top always; sides only where the probe passed)
    function twistW(Mv, face) {
      const w = CubeCore.matVec(Mv, CubeCore.NORMALS[face]);
      if (w[1] === 1) return w;
      if (w[1] === 0 && sideOk[w.join(',')]) return w;
      return null;
    }

    // ---- dynamic program
    const ROLL_COST = 1.0;    // one whole-cube roll cycle
    const SIDE_COST = 0.12;   // tie-break: prefer top twists over side twists
    const keyOf = (X) => X[0].join(',') + ';' + X[1].join(',') + ';' + X[2].join(',');
    let layer = new Map();
    layer.set(keyOf(M), { M: M.map((r) => r.slice()), cost: 0, parent: null, gens: [] });
    for (const m of moves) {
      const next = new Map();
      for (const st of layer.values()) {
        const cands = [];
        const add = (M2, gens, rolls) => {
          const w = twistW(M2, m.face);
          if (w) cands.push({ M2: M2, gens: gens, rolls: rolls, side: w[1] !== 0 });
        };
        add(st.M, [], 0);
        for (const g of ROLL_GENS) {
          add(CubeCore.matMul(CubeCore.GEN[g], st.M), [g], 1);
        }
        for (const g1 of ROLL_GENS) for (const g2 of ROLL_GENS) {
          add(CubeCore.matMul(CubeCore.GEN[g2], CubeCore.matMul(CubeCore.GEN[g1], st.M)), [g1, g2], 2);
        }
        for (const cd of cands) {
          const cost = st.cost + cd.rolls * ROLL_COST + (cd.side ? SIDE_COST : 0);
          const k = keyOf(cd.M2);
          const prev = next.get(k);
          if (!prev || cost < prev.cost - 1e-9) {
            next.set(k, {
              M: cd.M2.map((r) => r.slice()), cost: cost,
              parent: st, gens: cd.gens, face: m.face, q: m.q,
            });
          }
        }
      }
      layer = next;
    }

    // terminal state: the cube may finish in ANY orientation — the solved
    // facelet state is canonical (orientation-independent), so no time is
    // wasted rolling the cube back to a presentation pose afterwards
    let best = null;
    for (const st of layer.values()) {
      if (!best || st.cost < best.total - 1e-9) best = { st: st, rest: [], total: st.cost };
    }
    if (!best) {
      // unreachable in theory (two feasible rolls bring any face up);
      // safety fallback: BFS over the feasible roll generators
      let Mf = M.map((r) => r.slice());
      for (const m of moves) {
        const n = CubeCore.NORMALS[m.face];
        const seq = bfsRolls(Mf, (X) => {
          const w = CubeCore.matVec(X, n);
          return twistW(X, m.face) != null;
        }) || [];
        for (const gen of seq) Mf = emitReorient(acts, gen, Mf);
        emitTwist(acts, m.face, m.q, CubeCore.matVec(Mf, CubeCore.NORMALS[m.face]));
      }
      acts.push({ k: 'movej', p: GUARD.p, R: GUARD.R, gripTo: 'open', tag: 'HOMING', label: 'Standby pose' });
      acts.push({ k: 'done', tag: 'DONE', label: 'Solved' });
      return acts;
    }

    // backtrack the per-move roll decisions and emit the optimized program
    const chain = [];
    for (let s = best.st; s.parent; s = s.parent) chain.push(s);
    chain.reverse();
    for (const s of chain) {
      for (const g of s.gens) M = emitReorient(acts, g, M);
      const w = CubeCore.matVec(M, CubeCore.NORMALS[s.face]);
      emitTwist(acts, s.face, s.q, w);
    }

    acts.push({ k: 'movej', p: GUARD.p, R: GUARD.R, gripTo: 'open', tag: 'HOMING', label: 'Standby pose' });
    acts.push({ k: 'done', tag: 'DONE', label: 'Solved' });
    return acts;
  }

  return {
    ROBOT: ROBOT,
    GUARD: GUARD,
    fingerIk: fingerIk,
    fingerFk: fingerFk,
    planGrip: planGrip,
    OPEN_POSE: OPEN_POSE,
    layerGraspPose: layerGraspPose,
    layerTwistPose: layerTwistPose,
    bodyGraspPose: bodyGraspPose,
    slotFace: slotFace,
    scanPose: scanPose,
    planScramble: planScramble,
    planScan: planScan,
    planSolve: planSolve,
  };
});
