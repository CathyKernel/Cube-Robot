/* =============================================================
 * app.js — 3D visualization application
 *
 * Scene: 6-DOF articulated arm (built directly from the DH table)
 * with a two-finger adaptive gripper (mirrored two-phalanx jaws)
 * manipulating a Rubik's cube on a pedestal, plus a wrist-mounted
 * eye-in-hand camera that acquires the cube state (facelet scan)
 * before the solve.
 *
 * Runtime: interprets the planner's action list — joint moves on
 * trapezoidal profiles, Cartesian moves with per-frame IK
 * tracking, jaw grips, layer twists driven by J6 (tool roll),
 * whole-cube reorientations with rigid attach/detach and exact
 * lattice snapping after every turn.
 * ============================================================= */
(function (root) {
  'use strict';
  const CubeCore = root.CubeCore;
  const RobotKin = root.RobotKin;
  const RobotPlan = root.RobotPlan;
  const SimpleOrbit = root.SimpleOrbit;
  if (!root.THREE || !CubeCore || !RobotKin || !RobotPlan) {
    console.error('Missing dependencies'); return;
  }
  const DEG = Math.PI / 180;
  const R = RobotPlan.ROBOT;
  const S = R.cubeSize / 3;               // cubie lattice pitch

  // ------------------------------------------------------------------
  // keep-out world model (shared by the IK posture selector and the
  // joint-path guard): the parked cube, the spike fixture + its foot,
  // and the robot riser column. Exactly the keep-out-zone concept a real
  // controller loads for smart collision avoidance.
  // ------------------------------------------------------------------
  const CUBE_OB = { type: 'aabb', c: R.cubePos, h: R.cubeSize / 2 + 0.001, name: 'cube' };
  const FIXTURE_OB = [
    { type: 'cap', a: [R.cubePos[0], 0.012, R.cubePos[2]],
      b: [R.cubePos[0], R.cubePos[1] - R.cubeSize / 2 - 0.0016, R.cubePos[2]], r: 0.011, name: 'spike' },
    { type: 'cap', a: [R.cubePos[0], 0.006, R.cubePos[2]],
      b: [R.cubePos[0], 0.006, R.cubePos[2]], r: 0.054, name: 'fixture-foot' },
    { type: 'cap', a: [0, 0, 0], b: [0, 0.122, 0], r: 0.060, name: 'riser' },
  ];
  // while the whole cube is held, the keep-out volume is the cube at the
  // CURRENT action target (the hand carries it there) — each roll
  // checkpoint re-registers the box at its own target position
  function heldCubeOb(p, Rt) {
    const ez = [Rt[0][2], Rt[1][2], Rt[2][2]];
    return {
      type: 'aabb',
      c: [p[0] + R.palmH_body * ez[0], p[1] + R.palmH_body * ez[1], p[2] + R.palmH_body * ez[2]],
      h: R.cubeSize / 2 + 0.0015, name: 'held-cube',
    };
  }
  function setWorldCube(on) {
    RobotKin.setWorld(on ? [CUBE_OB].concat(FIXTURE_OB) : FIXTURE_OB);
  }

  // ------------------------------------------------------------------
  // renderer / scene / camera
  // ------------------------------------------------------------------
  const view = document.getElementById('view');
  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
  renderer.setSize(view.clientWidth, view.clientHeight);
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  renderer.outputEncoding = THREE.sRGBEncoding;
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  renderer.toneMappingExposure = 1.05;
  view.appendChild(renderer.domElement);

  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x0b0f14);
  scene.fog = new THREE.Fog(0x0b0f14, 1.0, 2.6);

  const camera = new THREE.PerspectiveCamera(42, view.clientWidth / view.clientHeight, 0.02, 20);
  const orbit = SimpleOrbit.create(camera, renderer.domElement, {
    // tight framing on the cube + gripper working region
    target: new THREE.Vector3(0.175, 0.205, 0.005),
    theta: Math.PI * 0.17, phi: 0.95, radius: 0.50,
  });

  // lights
  scene.add(new THREE.HemisphereLight(0x8fb4d9, 0x14181d, 0.55));
  const key = new THREE.DirectionalLight(0xfff1dc, 1.7);
  key.position.set(0.5, 1.0, 0.45);
  key.castShadow = true;
  key.shadow.mapSize.set(2048, 2048);
  key.shadow.camera.left = -0.6; key.shadow.camera.right = 0.6;
  key.shadow.camera.top = 0.6; key.shadow.camera.bottom = -0.6;
  key.shadow.camera.near = 0.2; key.shadow.camera.far = 2.6;
  key.shadow.bias = -0.0004;
  scene.add(key);
  const fill = new THREE.DirectionalLight(0x9fc5ff, 0.35);
  fill.position.set(-0.9, 0.7, -0.5);
  scene.add(fill);
  const rim = new THREE.PointLight(0x22d3ee, 0.22, 3.0);
  rim.position.set(0.25, 0.4, -0.7);
  scene.add(rim);

  // ------------------------------------------------------------------
  // materials
  // ------------------------------------------------------------------
  const MAT = {
    alu: new THREE.MeshStandardMaterial({ color: 0xc9ced4, metalness: 0.85, roughness: 0.34 }),
    aluDark: new THREE.MeshStandardMaterial({ color: 0x9aa1a8, metalness: 0.85, roughness: 0.42 }),
    joint: new THREE.MeshStandardMaterial({ color: 0x23272c, metalness: 0.6, roughness: 0.5 }),
    accent: new THREE.MeshStandardMaterial({ color: 0x0e7490, metalness: 0.5, roughness: 0.4 }),
    rubber: new THREE.MeshStandardMaterial({ color: 0x15181b, metalness: 0.1, roughness: 0.9 }),
    steel: new THREE.MeshStandardMaterial({ color: 0x5b636b, metalness: 0.9, roughness: 0.3 }),
    table: new THREE.MeshStandardMaterial({ color: 0x1c222a, metalness: 0.35, roughness: 0.75 }),
    cubie: new THREE.MeshStandardMaterial({ color: 0x0e1114, metalness: 0.25, roughness: 0.45 }),
    pads: new THREE.MeshStandardMaterial({ color: 0x1a1e22, metalness: 0.05, roughness: 0.95 }),
  };
  const CUBE_COLORS = [0xf2f4f6, 0xc62535, 0x12a05c, 0xffd21e, 0xff7a1a, 0x1263d6];
  const stickerMats = CUBE_COLORS.map((c) => new THREE.MeshStandardMaterial({
    color: c, metalness: 0.15, roughness: 0.28,
  }));

  function mesh(geo, mat, x, y, z) {
    const m = new THREE.Mesh(geo, mat);
    m.position.set(x || 0, y || 0, z || 0);
    m.castShadow = true; m.receiveShadow = true;
    return m;
  }
  const cyl = (r, h) => new THREE.CylinderGeometry(r, r, h, 32);
  const box = (a, b, c) => new THREE.BoxGeometry(a, b, c);
  const cap = (r, l) => new THREE.CapsuleGeometry(r, l, 6, 14);

  // ------------------------------------------------------------------
  // environment: table, grid, pedestal
  // ------------------------------------------------------------------
  {
    const table = mesh(box(1.7, 0.018, 1.25), MAT.table, 0, -0.009, 0);
    table.receiveShadow = true;
    scene.add(table);
    const grid = new THREE.GridHelper(1.7, 34, 0x2b3947, 0x1b232c);
    grid.position.y = 0.0008;
    grid.material.transparent = true;
    grid.material.opacity = 0.55;
    scene.add(grid);
    // tall work pedestal: cube presented at chest height (y = 0.16).
    // Slim SPIKE fixture: a twisting side layer sweeps its cubies past the
    // cube underside, and only the central column (r < S - cubie
    // half-diagonal ~ 10.3 mm) stays clear of every layer's swept volume.
    // A turned spike + seat disc therefore supports the cube while every
    // outer layer twists fully seated — no extract/reinsert cycle.
    const pedH = R.cubePos[1] - R.cubeSize / 2 - 0.0016;   // seat top plane
    const seat = mesh(cyl(0.0089, 0.0089, 0.0016), MAT.rubber, R.cubePos[0], pedH + 0.0008, R.cubePos[2]);
    scene.add(seat);
    const neckH = pedH - 0.012;
    const neck = mesh(new THREE.CylinderGeometry(0.0085, 0.020, neckH, 24), MAT.steel,
      R.cubePos[0], 0.012 + neckH / 2, R.cubePos[2]);
    scene.add(neck);
    const pedFoot = mesh(cyl(0.052, 0.052, 0.012), MAT.joint, R.cubePos[0], 0.006, R.cubePos[2]);
    scene.add(pedFoot);
    const ring = mesh(new THREE.TorusGeometry(0.030, 0.0022, 10, 40), MAT.accent, R.cubePos[0], 0.0122, R.cubePos[2]);
    ring.rotation.x = Math.PI / 2;
    scene.add(ring);
    // robot riser column (mounts the arm base 0.12 m above the table)
    const riser = mesh(cyl(0.044, 0.058, RobotKin.BASE_P[1]), MAT.steel, 0, RobotKin.BASE_P[1] / 2, 0);
    scene.add(riser);
    const riserFoot = mesh(cyl(0.088, 0.088, 0.012), MAT.joint, 0, 0.006, 0);
    scene.add(riserFoot);
  }

  // ------------------------------------------------------------------
  // eye-in-hand vision camera: the sensor pod is bracketed to the
  // gripper itself (built together with the gripper below), so
  // the picture-in-picture feed is the live wrist-camera point of
  // view with a machine-vision HUD overlay drawn on top of it.
  // ------------------------------------------------------------------
  const VIS = R.handCam;
  const visionCam = new THREE.PerspectiveCamera(VIS.fov, VIS.w / VIS.h, 0.03, 20);

  // PIP feed: dedicated small canvas + renderer (a separate GL surface
  // for the wrist-camera monitor — never interleaved with the main
  // view), with the 2D machine-vision HUD canvas drawn on top of it.
  const camGLCanvas = document.getElementById('camGL');
  const pipRenderer = camGLCanvas ? new THREE.WebGLRenderer({ canvas: camGLCanvas, antialias: true }) : null;
  if (pipRenderer) {
    pipRenderer.outputEncoding = THREE.sRGBEncoding;
    pipRenderer.toneMapping = THREE.ACESFilmicToneMapping;
    pipRenderer.toneMappingExposure = 1.05;
    pipRenderer.setPixelRatio(2);
  }
  const overlayCanvas = document.getElementById('camOverlay');
  const octx = overlayCanvas ? overlayCanvas.getContext('2d') : null;
  let pipRect = null;
  function measurePip() {
    if (!overlayCanvas) return;
    const r = overlayCanvas.getBoundingClientRect();
    pipRect = { w: r.width, h: r.height };
    if (pipRenderer && r.width > 8) {
      pipRenderer.setSize(r.width, r.height, false);
    }
    overlayCanvas.width = Math.max(2, Math.round(r.width * 2));
    overlayCanvas.height = Math.max(2, Math.round(r.height * 2));
  }
  function renderPip() {
    if (pipRenderer && pipRect && pipRect.w > 8) {
      pipRenderer.render(scene, visionCam);
    }
  }

  // ---- vision HUD overlay drawing ----
  const _vw = new THREE.Vector3();
  const _cw = new THREE.Vector3();
  const _cq = new THREE.Quaternion();
  const _camDir = new THREE.Vector3();
  const COLOR_LETTERS = ['W', 'R', 'G', 'Y', 'O', 'B'];
  const FACE_TANG = [];    // per face: [t1 (row dir), t2 (col dir)] unit vectors
  for (let f = 0; f < 6; f++) {
    const a = CubeCore.faceletPos(f, 0, 0), b = CubeCore.faceletPos(f, 1, 0), c = CubeCore.faceletPos(f, 0, 1);
    FACE_TANG.push([[b[0] - a[0], b[1] - a[1], b[2] - a[2]], [c[0] - a[0], c[1] - a[1], c[2] - a[2]]]);
  }
  function cssColor(i) { return '#' + CUBE_COLORS[i].toString(16).padStart(6, '0'); }

  function projectPt(v) {
    _vw.copy(v).project(visionCam);
    return [(_vw.x * 0.5 + 0.5) * overlayCanvas.width, (0.5 - _vw.y * 0.5) * overlayCanvas.height];
  }

  function drawVisionOverlay(now) {
    if (!octx || !overlayCanvas.width) return;
    const W = overlayCanvas.width, Hh = overlayCanvas.height;
    octx.clearRect(0, 0, W, Hh);
    // live cube transform — valid on the pedestal and while held in-hand
    cubeGroup.updateWorldMatrix(true, false);
    cubeGroup.getWorldPosition(_cw);
    cubeGroup.getWorldQuaternion(_cq);
    visionCam.getWorldPosition(_camDir).sub(_cw).normalize();
    const act = Vision.active;

    // ---- per-sticker detections ----
    for (let f = 0; f < 6; f++) {
      const n = CubeCore.NORMALS[f];
      _vw.set(n[0], n[1], n[2]).applyQuaternion(_cq);
      const vis = _vw.dot(_camDir);
      const reading = act && act.faces.indexOf(f) >= 0;
      // faces being read always overlay; other faces only when clearly
      // visible — grazing views (or faces hidden behind the base/jaws
      // during an in-hand hold) get no overlay so boxes never sit on top
      // of the robot's own jaws.
      if (vis < (reading ? 0.34 : 0.60)) continue;
      const rd = Vision.faces[f];
      const t1 = FACE_TANG[f][0], t2 = FACE_TANG[f][1];
      let bbox = null;
      for (let r = 0; r < 3; r++) for (let c = 0; c < 3; c++) {
        const fp = CubeCore.faceletPos(f, r, c);
        const cx = fp[0] * S + n[0] * S * 0.49;
        const cy = fp[1] * S + n[1] * S * 0.49;
        const cz = fp[2] * S + n[2] * S * 0.49;
        let revealed = !!rd;
        if (reading) revealed = act.u > 0.07 + (r * 3 + c) * 0.094;
        const e = S * 0.44;
        const pts = [];
        for (const s of [[-1, -1], [1, -1], [1, 1], [-1, 1]]) {
          _vw.set(cx + s[0] * e * t1[0] + s[1] * e * t2[0],
                  cy + s[0] * e * t1[1] + s[1] * e * t2[1],
                  cz + s[0] * e * t1[2] + s[1] * e * t2[2])
            .applyQuaternion(_cq).add(_cw);
          pts.push(projectPt(_vw));
          if (!bbox) bbox = [pts[0][0], pts[0][1], pts[0][0], pts[0][1]];
          bbox[0] = Math.min(bbox[0], pts[pts.length - 1][0]);
          bbox[1] = Math.min(bbox[1], pts[pts.length - 1][1]);
          bbox[2] = Math.max(bbox[2], pts[pts.length - 1][0]);
          bbox[3] = Math.max(bbox[3], pts[pts.length - 1][1]);
        }
        _vw.set(cx, cy, cz).applyQuaternion(_cq).add(_cw);
        const cpt = projectPt(_vw);
        const colIdx = RT.state[f * 9 + r * 3 + c];
        octx.beginPath();
        octx.moveTo(pts[0][0], pts[0][1]);
        for (let k = 1; k < 4; k++) octx.lineTo(pts[k][0], pts[k][1]);
        octx.closePath();
        if (revealed) {
          octx.fillStyle = cssColor(colIdx) + '46';
          octx.fill();
          octx.strokeStyle = reading ? 'rgba(34,211,238,0.95)' : 'rgba(34,211,238,0.5)';
          octx.lineWidth = 1.6;
          octx.stroke();
          octx.textAlign = 'center'; octx.textBaseline = 'middle';
          // dark halo so the letter stays readable on white/yellow stickers
          octx.shadowColor = 'rgba(0,0,0,0.85)';
          octx.shadowBlur = 3;
          octx.fillStyle = 'rgba(255,255,255,0.96)';
          octx.font = 'bold 20px ui-monospace, monospace';
          octx.fillText(COLOR_LETTERS[colIdx], cpt[0], cpt[1] - 7);
          const conf = 96.4 + ((f * 9 + r * 3 + c) * 53 % 34) / 10;
          octx.font = '13px ui-monospace, monospace';
          octx.fillStyle = 'rgba(180,215,235,0.9)';
          octx.fillText(conf.toFixed(1), cpt[0], cpt[1] + 13);
          octx.shadowBlur = 0;
        } else {
          octx.strokeStyle = 'rgba(150,170,190,0.32)';
          octx.lineWidth = 1;
          octx.stroke();
          if (!rd) {
            octx.fillStyle = 'rgba(150,170,190,0.5)';
            octx.font = '12px ui-monospace, monospace';
            octx.textAlign = 'center'; octx.textBaseline = 'middle';
            octx.shadowColor = 'rgba(0,0,0,0.8)';
            octx.shadowBlur = 2;
            octx.fillText('?', cpt[0], cpt[1]);
            octx.shadowBlur = 0;
          }
        }
      }
      // acquisition sweep line across the face being read
      if (reading && bbox) {
        const sy = bbox[1] + Math.min(1, act.u * 1.12) * (bbox[3] - bbox[1]);
        octx.save();
        octx.strokeStyle = 'rgba(34,211,238,0.9)';
        octx.lineWidth = 2;
        octx.shadowColor = 'rgba(34,211,238,0.9)';
        octx.shadowBlur = 9;
        octx.beginPath();
        octx.moveTo(bbox[0] - 6, sy); octx.lineTo(bbox[2] + 6, sy);
        octx.stroke();
        octx.restore();
      }
    }

    // ---- cube ROI corner brackets ----
    {
      const H2 = R.cubeSize / 2;
      let bb = null;
      for (const s of [[-1, -1, -1], [1, -1, -1], [1, 1, -1], [-1, 1, -1], [-1, -1, 1], [1, -1, 1], [1, 1, 1], [-1, 1, 1]]) {
        _vw.set(s[0] * H2, s[1] * H2, s[2] * H2).applyQuaternion(_cq).add(_cw);
        const p = projectPt(_vw);
        if (!bb) bb = [p[0], p[1], p[0], p[1]];
        bb[0] = Math.min(bb[0], p[0]); bb[1] = Math.min(bb[1], p[1]);
        bb[2] = Math.max(bb[2], p[0]); bb[3] = Math.max(bb[3], p[1]);
      }
      const L = 14;
      octx.strokeStyle = 'rgba(34,211,238,0.85)';
      octx.lineWidth = 2;
      for (const [cx, cy, dx, dy] of [[bb[0], bb[1], 1, 1], [bb[2], bb[1], -1, 1], [bb[2], bb[3], -1, -1], [bb[0], bb[3], 1, -1]]) {
        octx.beginPath();
        octx.moveTo(cx + dx * L, cy); octx.lineTo(cx, cy); octx.lineTo(cx, cy + dy * L);
        octx.stroke();
      }
    }

    // ---- HUD chrome ----
    octx.font = '13px ui-monospace, monospace';
    octx.textAlign = 'left'; octx.textBaseline = 'top';
    octx.fillStyle = 'rgba(34,211,238,0.9)';
    octx.fillText('WRIST-CAM · EYE-IN-HAND · 30 FPS', 9, 7);
    octx.textAlign = 'right';
    octx.fillText('ROI: CUBE · LOCK', W - 9, 7);
    if (Math.floor(now / 600) % 2 === 0) {
      octx.fillStyle = '#f87171';
      octx.beginPath(); octx.arc(W - 12, 30, 4.5, 0, 7); octx.fill();
    }
    octx.fillStyle = 'rgba(248,113,113,0.95)';
    octx.textAlign = 'right';
    octx.fillText('REC', W - 23, 26);
    octx.textAlign = 'left'; octx.textBaseline = 'bottom';
    octx.fillStyle = 'rgba(205,224,238,0.9)';
    const acq = Vision.acquired();
    const tail = act ? ' · ACQUIRING ' + act.faces.map((f) => CubeCore.FACES[f]).join('+')
                     : (acq === 6 ? ' · STATE LOCKED' : ' · TRACKING');
    octx.fillText('FACELETS ' + (acq * 9) + '/54' + tail, 9, Hh - 8);
    octx.textAlign = 'right';
    octx.fillText('T+' + UI.fmtTime(visionClock), W - 9, Hh - 8);
    // centre crosshair
    octx.strokeStyle = 'rgba(34,211,238,0.35)';
    octx.lineWidth = 1;
    octx.beginPath();
    octx.moveTo(W / 2 - 9, Hh / 2); octx.lineTo(W / 2 - 3, Hh / 2);
    octx.moveTo(W / 2 + 3, Hh / 2); octx.lineTo(W / 2 + 9, Hh / 2);
    octx.moveTo(W / 2, Hh / 2 - 9); octx.lineTo(W / 2, Hh / 2 - 3);
    octx.moveTo(W / 2, Hh / 2 + 3); octx.lineTo(W / 2, Hh / 2 + 9);
    octx.stroke();
  }

  // ------------------------------------------------------------------
  // robot visual model, built straight from the DH parameters
  // ------------------------------------------------------------------
  const DH = RobotKin.DH;
  const baseGroup = new THREE.Group();
  {
    const m = new THREE.Matrix4().set(
      RobotKin.BASE_R[0][0], RobotKin.BASE_R[0][1], RobotKin.BASE_R[0][2], 0,
      RobotKin.BASE_R[1][0], RobotKin.BASE_R[1][1], RobotKin.BASE_R[1][2], 0,
      RobotKin.BASE_R[2][0], RobotKin.BASE_R[2][1], RobotKin.BASE_R[2][2], 0,
      0, 0, 0, 1);
    baseGroup.setRotationFromMatrix(m);
    baseGroup.position.set(RobotKin.BASE_P[0], RobotKin.BASE_P[1], RobotKin.BASE_P[2]);
    scene.add(baseGroup);
  }

  const jointRots = [];    // g_rot: rotation.z = theta_i (drives joint i)
  const linkStatics = [];  // g_static: (a_i, 0, d_i) + Rx(alpha_i)
  {
    let parent = baseGroup;
    for (let i = 0; i < 6; i++) {
      const gRot = new THREE.Group();
      parent.add(gRot);
      const gStatic = new THREE.Group();
      gStatic.position.set(DH[i][1], 0, DH[i][0]);
      gStatic.rotation.x = DH[i][2];
      gRot.add(gStatic);
      jointRots[i] = gRot;
      linkStatics[i] = gStatic;
      parent = gStatic;
    }
    // fixed tool transform: flange -> gripper base centre
    const toolNode = new THREE.Group();
    toolNode.position.set(0, 0, RobotKin.TOOL);
    linkStatics[5].add(toolNode);
    root.__toolNode = toolNode;
  }
  const toolNode = root.__toolNode;

  // static column + J1 stator (mounted on top of the riser)
  {
    const plate = mesh(cyl(0.075, 0.014), MAT.aluDark, 0, 0.007, 0);
    baseGroup.add(plate);
    const column = mesh(cyl(0.031, 0.068), MAT.joint, 0, 0.048, 0);
    baseGroup.add(column);
    const j1stator = mesh(cyl(0.041, 0.022), MAT.joint, 0, 0.081, 0);
    baseGroup.add(j1stator);
    const led = mesh(new THREE.TorusGeometry(0.043, 0.0025, 8, 40), MAT.accent, 0, 0.0918, 0);
    led.rotation.x = Math.PI / 2;
    baseGroup.add(led);
  }
  // J1 rotor: shoulder bracket
  {
    const a1 = DH[0][1], d1 = DH[0][0];
    const br = mesh(box(a1 + 0.03, 0.054, 0.036), MAT.alu, a1 / 2, 0, d1 - 0.014);
    jointRots[0].add(br);
  }
  // J2 shoulder motor + upper arm (arm link rotates with J2)
  {
    const motor = mesh(cyl(0.036, 0.055), MAT.joint, 0, 0, 0);
    motor.rotation.x = Math.PI / 2;
    linkStatics[0].add(motor);
    const a2 = DH[1][1];
    const arm = mesh(cap(0.0265, a2 - 0.048), MAT.alu, a2 / 2, 0, 0);
    arm.rotation.z = -Math.PI / 2;
    jointRots[1].add(arm);
    const stripe = mesh(box(a2 - 0.065, 0.004, 0.033), MAT.accent, a2 / 2, 0.025, 0);
    jointRots[1].add(stripe);
  }
  // J3 elbow motor + forearm (rotates with J3)
  {
    const motor = mesh(cyl(0.031, 0.048), MAT.joint, 0, 0, 0);
    motor.rotation.x = Math.PI / 2;
    linkStatics[1].add(motor);
    const a3 = DH[2][1];
    const arm = mesh(cap(0.0235, a3 - 0.044), MAT.alu, a3 / 2, 0, 0);
    arm.rotation.z = -Math.PI / 2;
    jointRots[2].add(arm);
  }
  // J4 wrist pitch motor (at the wrist point)
  {
    const motor = mesh(cyl(0.025, 0.040), MAT.joint, 0, 0, 0);
    motor.rotation.x = Math.PI / 2;
    jointRots[3].add(motor);
  }
  // J5 yoke + motor
  {
    const d5 = DH[4][0];
    const yoke = mesh(cap(0.017, d5 - 0.032), MAT.alu, 0, 0, d5 / 2);
    yoke.rotation.x = Math.PI / 2;
    jointRots[4].add(yoke);
    const motor = mesh(cyl(0.021, 0.033), MAT.joint, 0, 0, 0);
    motor.rotation.x = Math.PI / 2;
    jointRots[4].add(motor);
  }
  // J6 roll motor + flange + gripper stem
  {
    const motor = mesh(cyl(0.016, 0.028), MAT.joint, 0, 0, 0);
    motor.rotation.x = Math.PI / 2;
    jointRots[5].add(motor);
    const flange = mesh(cyl(0.017, 0.007), MAT.steel, 0, 0, 0.001);
    flange.rotation.x = Math.PI / 2;
    linkStatics[5].add(flange);
    const stem = mesh(cap(0.010, 0.016), MAT.alu, 0, 0, 0.011);
    stem.rotation.x = Math.PI / 2;
    linkStatics[5].add(stem);
  }

  // ------------------------------------------------------------------
  // two-finger adaptive gripper (Robotiq-2F class): machined base
  // housing bolted to the J6 flange, two mirrored jaws — each a
  // pivoting MCP knuckle driving a two-phalanx finger (proximal +
  // distal link) whose rubber toe pad presses one opposite side face
  // of the gripped layer/body
  // ------------------------------------------------------------------
  const H = R.hand;
  const fingerJoints = [];  // [{ mcp, pip }] x 2 (jaw A at +y, jaw B at -y)
  {
    const L1 = H.fingerL[0], L2 = H.fingerL[1];

    // ---- base housing: bolted block between the wrist cuff and the
    // jaw knuckles, with drive + status details ----
    toolNode.add(mesh(box(H.baseW, H.baseD, H.baseT - 0.005), MAT.alu, 0, 0, -0.017));
    toolNode.add(mesh(box(H.baseW + 0.002, H.baseD + 0.002, 0.004), MAT.aluDark, 0, 0, -0.0295));  // back cover
    toolNode.add(mesh(box(H.baseW + 0.002, H.baseD + 0.002, 0.004), MAT.aluDark, 0, 0, -0.0045));  // front plate
    // teal status strips on both flanks
    toolNode.add(mesh(box(0.0025, H.baseD * 0.62, 0.011), MAT.accent, H.baseW / 2 + 0.0015, 0, -0.017));
    toolNode.add(mesh(box(0.0025, H.baseD * 0.62, 0.011), MAT.accent, -H.baseW / 2 - 0.0015, 0, -0.017));
    // socket screws on the back cover
    for (const sc of [[1, 1], [1, -1], [-1, 1], [-1, -1]]) {
      const scr = mesh(cyl(0.0022, 0.0022, 0.002), MAT.joint,
        sc[0] * (H.baseW / 2 - 0.007), sc[1] * (H.baseD / 2 - 0.007), -0.0316);
      scr.rotation.x = Math.PI / 2;
      toolNode.add(scr);
    }
    // wrist cuff: ISO flange neck joining the housing to the J6 stem
    const cuff = mesh(cyl(0.0165, 0.0215, 0.012), MAT.aluDark, 0, 0, -0.033);
    cuff.rotation.x = Math.PI / 2;
    toolNode.add(cuff);
    const cuffRing = mesh(new THREE.TorusGeometry(0.0205, 0.0016, 8, 30), MAT.accent, 0, 0, -0.0275);
    toolNode.add(cuffRing);

    // ---- static knuckle saddles + pivot shafts (both sides) ----
    function addSaddle(side) {
      const sad = mesh(box(0.026, 0.015, 0.021), MAT.aluDark, 0, side * (H.mcpFront - 0.002), 0.0015);
      toolNode.add(sad);
      const shaft = mesh(cyl(0.0042, 0.0042, 0.027), MAT.joint, 0, side * H.mcpFront, H.mcpDrop);
      shaft.rotation.z = Math.PI / 2;
      toolNode.add(shaft);
      for (const sx of [-1, 1]) {
        const capD = mesh(cyl(0.0062, 0.0062, 0.0028), MAT.steel, sx * 0.0145, side * H.mcpFront, H.mcpDrop);
        capD.rotation.z = Math.PI / 2;
        toolNode.add(capD);
      }
    }
    addSaddle(1);
    addSaddle(-1);

    // ---- jaw factory: mirrored two-phalanx finger ----
    function buildJaw(side) {
      const rootG = new THREE.Group();
      rootG.position.set(0, side * H.mcpFront, H.mcpDrop);
      if (side < 0) rootG.rotation.z = Math.PI;   // mirror the chain toward -y
      toolNode.add(rootG);

      const mcp = new THREE.Group();
      rootG.add(mcp);
      // knuckle block riding the pivot shaft
      mcp.add(mesh(box(0.0185, 0.0165, 0.0155), MAT.alu, 0, 0.0035, 0));
      // proximal phalanx: machined link + accent rib
      mcp.add(mesh(box(0.0125, L1 + 0.007, 0.0115), MAT.alu, 0, (L1 + 0.007) / 2 - 0.0035, 0));
      mcp.add(mesh(box(0.0135, L1 * 0.5, 0.0045), MAT.accent, 0, L1 * 0.5, 0.0042));

      const pip = new THREE.Group();
      pip.position.set(0, L1, 0);
      mcp.add(pip);
      // PIP hinge pin (axis across the jaw)
      const pin = mesh(cyl(0.0032, 0.0032, 0.0145), MAT.joint, 0, 0, 0);
      pin.rotation.z = Math.PI / 2;
      pip.add(pin);
      // distal phalanx + end plate (slim + set back from the face
      // plane so only the rubber toe touches the cube)
      pip.add(mesh(box(0.0105, L2 - 0.0055, 0.0088), MAT.alu, 0, (L2 - 0.0055) / 2, 0));
      pip.add(mesh(box(0.0115, 0.005, 0.0092), MAT.aluDark, 0, L2 - 0.0078, 0));
      // rubber toe pad: capsule across the jaw, centred on the planned
      // contact point (contact = face + padR, so the pad touches the
      // layer's side face exactly)
      const toe = mesh(cap(H.padR, 0.0085), MAT.pads, 0, L2 - 0.0002, 0.0006);
      toe.rotation.z = Math.PI / 2;
      pip.add(toe);

      fingerJoints.push({ mcp: mcp, pip: pip });
    }
    buildJaw(1);   // jaw A: +y edge
    buildJaw(-1);  // jaw B: mirrored -y edge
  }

  // ------------------------------------------------------------------
  // eye-in-hand camera pod: compact smart camera bracketed to the
  // gripper base housing on the +x side (clear of both jaws),
  // boresight tilted inward toward the tool axis. The THREE camera
  // rides at the pod's optical centre, parented to the gripper, so the
  // picture-in-picture feed is the live wrist-camera point of view.
  // ------------------------------------------------------------------
  let camRingMat = null, camLedMat = null;
  {
    const HC = R.handCam;
    const mount = new THREE.Group();
    mount.position.fromArray(HC.off);
    mount.rotation.y = -HC.tilt;             // boresight tilts toward the tool axis
    toolNode.add(mount);
    root.__camMount = mount;
    // L-bracket tying the pod to the gripper base flank
    const brk = mesh(box(0.036, 0.009, 0.006), MAT.aluDark, -0.026, 0, -0.008);
    mount.add(brk);
    // sensor body (behind the lens plane) + back cap
    const body = mesh(box(0.020, 0.020, 0.014), MAT.joint, 0, 0, -0.008);
    mount.add(body);
    const capB = mesh(box(0.023, 0.023, 0.004), MAT.aluDark, 0, 0, -0.0165);
    mount.add(capB);
    // lens barrel + conical hood along the boresight
    const barrel = mesh(cyl(0.0062, 0.011), MAT.aluDark, 0, 0, 0.0065);
    barrel.rotation.x = Math.PI / 2;
    mount.add(barrel);
    const hood = mesh(new THREE.CylinderGeometry(0.0086, 0.0064, 0.007, 20), MAT.joint, 0, 0, 0.015);
    hood.rotation.x = Math.PI / 2;
    mount.add(hood);
    const glass = mesh(cyl(0.0048, 0.003),
      new THREE.MeshStandardMaterial({ color: 0x0a2c40, metalness: 0.9, roughness: 0.12 }), 0, 0, 0.0195);
    glass.rotation.x = Math.PI / 2;
    mount.add(glass);
    // ring light around the hood front (pulses while acquiring)
    camRingMat = new THREE.MeshStandardMaterial({
      color: 0x22d3ee, emissive: 0x22d3ee, emissiveIntensity: 0.6, metalness: 0.2, roughness: 0.4,
    });
    const ring = mesh(new THREE.TorusGeometry(0.0075, 0.0014, 8, 32), camRingMat, 0, 0, 0.018);
    mount.add(ring);
    // status LED on the body + short cable back to the wrist
    camLedMat = new THREE.MeshStandardMaterial({
      color: 0x34d399, emissive: 0x34d399, emissiveIntensity: 1.6, metalness: 0.1, roughness: 0.3,
    });
    const led = mesh(new THREE.SphereGeometry(0.0022, 10, 8), camLedMat, 0.008, 0.0108, -0.009);
    mount.add(led);
    const cable = mesh(cap(0.0016, 0.022), MAT.rubber, -0.030, -0.004, -0.012);
    cable.rotation.z = Math.PI / 2 - 0.4;
    mount.add(cable);
    // the view camera at the pod's optical centre: THREE cameras look
    // along local -z, so aim it along the mount +z (the boresight)
    // with the image up on the mount +x side — upright face views at
    // every inspection vantage.
    visionCam.quaternion.setFromRotationMatrix(new THREE.Matrix4().makeBasis(
      new THREE.Vector3(0, 1, 0), new THREE.Vector3(1, 0, 0), new THREE.Vector3(0, 0, -1)));
    mount.add(visionCam);
  }

  function setFingerAngles(f) {
    for (let i = 0; i < fingerJoints.length; i++) {
      fingerJoints[i].mcp.rotation.x = f.fingers[i].t1;
      fingerJoints[i].pip.rotation.x = f.fingers[i].t2;
    }
  }

  function setQ(q) {
    for (let i = 0; i < 6; i++) jointRots[i].rotation.z = q[i] + DH[i][3];
  }

  // ------------------------------------------------------------------
  // swept-collision guard for coordinated joint moves: samples the
  // planned trapezoid path against the keep-out world (arm link
  // capsules + gripper boxes); if anything penetrates a zone mid-path,
  // the move is re-planned through a high clearance via point (a fly-by
  // detour, like a controller routing around a keep-out zone). Disabled
  // while the whole cube is in the hand — the roll arc is planned
  // explicitly via its 45-degree checkpoints.
  // ------------------------------------------------------------------
  // gripper envelope as oriented boxes in the TCP frame (jaws open):
  //   base housing + cuff, the two open jaws, wrist camera pod
  const HAND_BOXES = [
    { c: [0, 0.000, -0.018], h: [0.040, 0.027, 0.016] },     // base housing + cuff
    { c: [0, 0.052, 0.020], h: [0.010, 0.040, 0.026] },      // jaw A (open stroke)
    { c: [0, -0.052, 0.020], h: [0.010, 0.040, 0.026] },     // jaw B (open stroke)
    { c: [0.055, 0.002, -0.012], h: [0.026, 0.014, 0.011] }, // cam pod
  ];
  function obbClearance(f, ob) {
    let worst = 1e9;
    for (const b of HAND_BOXES) {
      for (let ci = 0; ci < 8; ci++) {
        const lx = (ci & 1 ? 1 : -1) * b.h[0], ly = (ci & 2 ? 1 : -1) * b.h[1], lz = (ci & 4 ? 1 : -1) * b.h[2];
        const p = [
          f.p[0] + f.R[0][0] * (b.c[0] + lx) + f.R[0][1] * (b.c[1] + ly) + f.R[0][2] * (b.c[2] + lz),
          f.p[1] + f.R[1][0] * (b.c[0] + lx) + f.R[1][1] * (b.c[1] + ly) + f.R[1][2] * (b.c[2] + lz),
          f.p[2] + f.R[2][0] * (b.c[0] + lx) + f.R[2][1] * (b.c[1] + ly) + f.R[2][2] * (b.c[2] + lz),
        ];
        let d;
        if (ob.type === 'aabb') {
          d = Math.max(Math.abs(p[0] - ob.c[0]) - ob.h, 0) ** 2
            + Math.max(Math.abs(p[1] - ob.c[1]) - ob.h, 0) ** 2
            + Math.max(Math.abs(p[2] - ob.c[2]) - ob.h, 0) ** 2;
          d = Math.sqrt(d);
        } else {
          const ab = [ob.b[0] - ob.a[0], ob.b[1] - ob.a[1], ob.b[2] - ob.a[2]];
          const ap = [p[0] - ob.a[0], p[1] - ob.a[1], p[2] - ob.a[2]];
          const ab2 = ab[0] * ab[0] + ab[1] * ab[1] + ab[2] * ab[2];
          let t = ab2 > 1e-12 ? (ap[0] * ab[0] + ap[1] * ab[1] + ap[2] * ab[2]) / ab2 : 0;
          t = Math.max(0, Math.min(1, t));
          d = Math.hypot(p[0] - (ob.a[0] + t * ab[0]), p[1] - (ob.a[1] + t * ab[1]), p[2] - (ob.a[2] + t * ab[2])) - ob.r;
        }
        if (d < worst) worst = d;
      }
    }
    return worst;
  }
  function configClear(q, withHand) {
    let cl = RobotKin.linkClearance(q);
    if (cl < 0.0005) return cl;
    // hand boxes only matter for free transitions (jaws open); on
    // gripper-blend moves the jaws are touching the cube by design at
    // the start, and the corner-sampled box model is coarse anyway — only
    // a VISIBLE penetration (>= 3 mm) counts as a violation
    if (!withHand) return cl;
    const f = RobotKin.fk(q);
    for (const ob of RobotKin.getWorld()) {
      if (ob.type !== 'aabb' || ob.name !== 'cube') continue;
      const d = obbClearance(f, ob);
      if (d < -0.003) return d;
    }
    return cl;
  }
  function pathClear(plan, handFrom) {
    // handFrom: sample index from which the hand boxes are checked (on
    // gripper-blend moves the jaws still touch the cube early on)
    const N = 16;
    for (let i = 0; i <= N; i++) {
      const q = RobotKin.evalJointMove(plan, plan.T * i / N);
      if (configClear(q, i >= handFrom) < 0.0005) return false;
    }
    return true;
  }
  const VIA_OFFS = [
    [-0.075, 0.115, 0.075], [0.075, 0.115, -0.075],
    [0.0, 0.140, 0.100], [0.0, 0.140, -0.100],
    [-0.100, 0.100, 0.0], [0.100, 0.100, 0.0],
  ];
  function planMovejGuard(a, qEnd) {
    a._planA = null; a._planB = null;
    if (RT.attached === 'cube') return;            // roll arc is explicit
    // hand boxes: checked from the start on free moves, from mid-move on
    // gripper-opening moves (jaws release the cube early in the path),
    // skipped while the jaws close onto the cube
    const handFrom = !a.gripTo ? 0 : (a.gripTo === 'open' ? 8 : 99);
    if (pathClear(a._plan, handFrom)) return;      // direct path clear
    // candidate vias: a lifted midpoint (shortest detour) then fixed
    // high waypoints around the cube
    const f0 = RobotKin.fk(RT.q);
    const mid = [(f0.p[0] + a.p[0]) / 2, (f0.p[1] + a.p[1]) / 2, (f0.p[2] + a.p[2]) / 2];
    let dx = mid[0] - R.cubePos[0], dz = mid[2] - R.cubePos[2];
    const dn = Math.hypot(dx, dz) || 1;
    dx /= dn; dz /= dn;
    const vias = [];
    for (const lift of [0.05, 0.09]) {
      vias.push([mid[0] + dx * 0.05, Math.max(mid[1], R.cubePos[1]) + lift, mid[2] + dz * 0.05]);
    }
    for (const off of VIA_OFFS) {
      vias.push([R.cubePos[0] + off[0], R.cubePos[1] + off[1], R.cubePos[2] + off[2]]);
    }
    for (const vp of vias) {
      const rv = RobotKin.ik(vp, a.R, RT.q);
      if (!rv.ok) continue;
      const pA = RobotKin.planJointMove(RT.q, rv.q);
      const pB = RobotKin.planJointMove(rv.q, qEnd);
      if (pathClear(pA, handFrom) && pathClear(pB, handFrom)) {
        a._planA = pA; a._planB = pB;
        a._dur = pA.T + pB.T;
        return;
      }
    }
    console.warn('movej enters keep-out zone, no via found:', a.label || a.k);
  }

  // jaw blending during motion (gripTo): real controllers overlap
  // gripper actuation with the approach / retreat motion
  function armGrip(a) {
    if (!a.gripTo) return;
    a._gFrom = RT.fingers;
    a._gTo = gripAnglesFor(a.gripTo === 'open' ? 'open' : a.gripTo);
    RT.gripMode = a.gripTo.toUpperCase();
  }
  function gripEase(u) {
    return u < 0.5 ? 2 * u * u : 1 - Math.pow(-2 * u + 2, 2) / 2;
  }

  // ------------------------------------------------------------------
  // cube visual model
  // ------------------------------------------------------------------
  const cubeGroup = new THREE.Group();
  cubeGroup.position.set(R.cubePos[0], R.cubePos[1], R.cubePos[2]);
  scene.add(cubeGroup);

  const cubies = [];
  const STICKER_NORMALS = [[0, 1, 0], [1, 0, 0], [0, 0, 1], [0, -1, 0], [-1, 0, 0], [0, 0, -1]];
  for (let i = -1; i <= 1; i++) for (let j = -1; j <= 1; j++) for (let k = -1; k <= 1; k++) {
    const g = new THREE.Group();
    g.position.set(i * S, j * S, k * S);
    const body = mesh(box(S * 0.965, S * 0.965, S * 0.965), MAT.cubie, 0, 0, 0);
    g.add(body);
    const p = [i, j, k];
    for (let f = 0; f < 6; f++) {
      const n = STICKER_NORMALS[f];
      if (p[0] * n[0] + p[1] * n[1] + p[2] * n[2] !== 1) continue;
      const st = mesh(new THREE.PlaneGeometry(S * 0.84, S * 0.84), stickerMats[f],
        n[0] * S * 0.49, n[1] * S * 0.49, n[2] * S * 0.49);
      if (n[0] === 1) st.rotation.y = Math.PI / 2;
      else if (n[0] === -1) st.rotation.y = -Math.PI / 2;
      else if (n[1] === 1) st.rotation.x = -Math.PI / 2;
      else if (n[1] === -1) st.rotation.x = Math.PI / 2;
      else if (n[2] === -1) st.rotation.y = Math.PI;
      st.castShadow = false;
      g.add(st);
    }
    cubeGroup.add(g);
    cubies.push(g);
  }

  // 24-cell quaternions for exact snapping
  function intMatToQuat(m) {
    const tm = new THREE.Matrix4().set(
      m[0][0], m[0][1], m[0][2], 0,
      m[1][0], m[1][1], m[1][2], 0,
      m[2][0], m[2][1], m[2][2], 0,
      0, 0, 0, 1);
    return new THREE.Quaternion().setFromRotationMatrix(tm);
  }
  const Q24 = CubeCore.allOrientations().map(intMatToQuat);
  function snapQuat(q) {
    let best = Q24[0], bd = -1;
    for (const c of Q24) {
      const d = Math.abs(c.x * q.x + c.y * q.y + c.z * q.z + c.w * q.w);
      if (d > bd) { bd = d; best = c; }
    }
    return best;
  }
  function snapCubiesToLattice() {
    for (const c of cubies) {
      c.position.set(Math.round(c.position.x / S) * S, Math.round(c.position.y / S) * S, Math.round(c.position.z / S) * S);
      c.quaternion.copy(snapQuat(c.quaternion));
    }
  }

  // Authoritative visual reconstruction: replays the logical move history
  // with exact 90-degree rotations in the cube frame. The rendered cube is
  // therefore consistent with the logical state BY CONSTRUCTION, independent
  // of any IK residual in the hand path.
  const _qTmp = new THREE.Quaternion();
  function rebuildCubeFromHistory() {
    let idx = 0;
    for (let i = -1; i <= 1; i++) for (let j = -1; j <= 1; j++) for (let k = -1; k <= 1; k++) {
      const c = cubies[idx++];
      c.position.set(i * S, j * S, k * S);
      c.quaternion.identity();
    }
    for (const m of RT.moveHistory) {
      const n = CubeCore.NORMALS[m.face];
      const axis = new THREE.Vector3(n[0], n[1], n[2]);
      _qTmp.setFromAxisAngle(axis, -(Math.PI / 2) * m.q);
      for (const c of cubies) {
        const sel = Math.round(c.position.x / S) * n[0] +
                    Math.round(c.position.y / S) * n[1] +
                    Math.round(c.position.z / S) * n[2];
        if (sel === 1) {
          c.position.applyQuaternion(_qTmp);
          c.quaternion.premultiply(_qTmp);
        }
      }
    }
    snapCubiesToLattice();
  }

  const tmpV = new THREE.Vector3();
  // cubies of the layer whose outward normal is the world direction w
  // (top twists use [0,1,0]; direct side twists use the face's world normal)
  function layerCubies(w) {
    const out = [];
    for (const c of cubies) {
      c.getWorldPosition(tmpV);
      const dot = (tmpV.x - R.cubePos[0]) * w[0] +
                  (tmpV.y - R.cubePos[1]) * w[1] +
                  (tmpV.z - R.cubePos[2]) * w[2];
      if (dot > 0.006) out.push(c);
    }
    return out;
  }

  // ------------------------------------------------------------------
  // solver service (Kociemba two-phase, in a worker when possible)
  // ------------------------------------------------------------------
  const Solver = {
    worker: null, ready: false, mode: 'init', reqId: 0, pending: null,
    init() {
      const src = root.__CUBEJS_SRC__;
      if (src && typeof Worker !== 'undefined') {
        try {
          const blob = new Blob([src + WORKER_GLUE], { type: 'text/javascript' });
          this.worker = new Worker(URL.createObjectURL(blob));
          this.mode = 'worker';
          this.worker.onmessage = (e) => {
            const d = e.data;
            if (d.type === 'ready') {
              this.ready = true;
            } else if (d.type === 'solution' && this.pending && d.id === this.reqId) {
              const p = this.pending; this.pending = null;
              p(d.sol);
            } else if (d.type === 'error' && this.pending) {
              const p = this.pending; this.pending = null;
              p(null);
            }
          };
          this.worker.onerror = () => { this.mode = 'main'; this.initMain(); };
          return;
        } catch (e) { /* fall through to main thread */ }
      }
      this.initMain();
    },
    initMain() {
      this.mode = 'main';
      setTimeout(() => {
        try {
          root.Cube.initSolver();
          this.ready = true;
        } catch (e) { /* solver unavailable in this environment */ }
      }, 40);
    },
    solve(facelets) {
      return new Promise((resolve) => {
        if (this.mode === 'worker' && this.worker) {
          if (!this.ready) {
            // wait for tables
            const wait = setInterval(() => {
              if (this.ready) { clearInterval(wait); this._ask(facelets, resolve); }
            }, 120);
            setTimeout(() => { clearInterval(wait); if (!this.pending) this._ask(facelets, resolve); }, 20000);
            return;
          }
          this._ask(facelets, resolve);
        } else {
          setTimeout(() => {
            try {
              const sol = root.Cube.fromString(facelets).solve(22);
              resolve(sol);
            } catch (e) { resolve(null); }
          }, 30);
        }
      });
    },
    _ask(facelets, resolve) {
      this.reqId++;
      this.pending = resolve;
      this.worker.postMessage({ cmd: 'solve', id: this.reqId, facelets: facelets });
    },
  };
  const WORKER_GLUE = `
self.onmessage = function (e) {
  var d = e.data;
  if (d.cmd === 'solve') {
    try {
      var c = Cube.fromString(d.facelets);
      var sol = c.solve(22);
      postMessage({ type: 'solution', id: d.id, sol: sol });
    } catch (err) {
      postMessage({ type: 'error', id: d.id, message: String(err) });
    }
  }
};
try { Cube.initSolver(); postMessage({ type: 'ready' }); }
catch (err) { postMessage({ type: 'error', id: -1, message: String(err) }); }
`;

  // ------------------------------------------------------------------
  // vision pipeline — facelet acquisition from the wrist-mounted
  // eye-in-hand camera. Each `read` action commits the 9 stickers of
  // the faces presented to the camera; the assembled 54-facelet string
  // is the ONLY input handed to the Kociemba solver (exactly like a
  // real vision-driven cube cell).
  // ------------------------------------------------------------------
  const Vision = {
    faces: [null, null, null, null, null, null],
    active: null,
    reset() {
      this.faces = [null, null, null, null, null, null];
      this.active = null;
      UI.refreshScanPanel();
      UI.setCamStatus('STREAMING');
    },
    acquired() {
      let n = 0;
      for (const f of this.faces) if (f) n++;
      return n;
    },
    beginRead(faces) {
      this.active = { faces: faces.filter((f) => f != null && f >= 0), u: 0 };
      if (this.active.faces.length) {
        UI.setCamStatus('ACQUIRE ' + this.active.faces.map((f) => CubeCore.FACES[f]).join('+'));
      }
    },
    update(u) {
      if (this.active) this.active.u = Math.min(1, Math.max(0, u));
    },
    commit(faces) {
      for (const f of faces) {
        if (f == null || f < 0) continue;
        const colors = [], conf = [];
        for (let i = 0; i < 9; i++) {
          colors.push(RT.state[f * 9 + i]);
          conf.push(96.4 + ((f * 9 + i) * 53 % 34) / 10);
        }
        this.faces[f] = { colors: colors, conf: conf };
      }
      this.active = null;
      UI.refreshScanPanel();
      UI.setCamStatus(this.acquired() === 6 ? 'STATE LOCKED' : 'TRACKING');
    },
    faceletString() {
      let s = '';
      for (let f = 0; f < 6; f++) {
        const d = this.faces[f];
        if (!d) return null;
        for (let i = 0; i < 9; i++) s += CubeCore.FACES[d.colors[i]];
      }
      return s;
    },
  };

  // ------------------------------------------------------------------
  // runtime — interprets the planner's action list
  // ------------------------------------------------------------------
  const OPEN_ANGLES = {
    fingers: [0, 1].map(() => ({ t1: RobotPlan.OPEN_POSE.t1, t2: RobotPlan.OPEN_POSE.t2 })),
  };
  const GRIP_CACHE = {
    layer: RobotPlan.planGrip('layer'),
    side: RobotPlan.planGrip('layer', RobotPlan.ROBOT.palmH_side),
    body: RobotPlan.planGrip('body'),
  };

  const RT = {
    program: null, idx: 0, act: null, t: 0, dur: 0,
    running: false, paused: false,
    q: RobotKin.HOME.slice(),
    prevQ: RobotKin.HOME.slice(),
    fingers: OPEN_ANGLES,
    fingerFrom: null, fingerTo: null,
    gripMode: 'OPEN',
    state: CubeCore.solvedState(),
    M: CubeCore.IDENT.map((r) => r.slice()),
    phase: 'IDLE',
    twistsDone: 0, twistsTotal: 0, movesDone: 0,
    execTime: 0,
    scanMode: false,
    scrambleMeta: '',
    attached: null,
    error: null,
    scrambleText: '',
    solutionText: '',
    onDone: null,
    moveHistory: [],   // logical moves replayed geometrically for the visuals
  };

  function gripAnglesFor(mode) {
    if (mode === 'open') return OPEN_ANGLES;
    return { fingers: GRIP_CACHE[mode].fingers.map((f) => ({ t1: f.t1, t2: f.t2 })) };
  }
  function fingersLerp(a, b, u) {
    const out = { fingers: [] };
    for (let i = 0; i < 2; i++) {
      out.fingers.push({
        t1: a.fingers[i].t1 + (b.fingers[i].t1 - a.fingers[i].t1) * u,
        t2: a.fingers[i].t2 + (b.fingers[i].t2 - a.fingers[i].t2) * u,
      });
    }
    return out;
  }

  // tactile-servo micro-motion: while the gripper holds the cube the
  // jaw joints keep making tiny measured corrections around the grip
  // setpoint (compliance control), instead of freezing at a static pose.
  function wiggleFingers(base, t, amp, freq) {
    const out = { fingers: [] };
    for (let i = 0; i < 2; i++) {
      out.fingers.push({
        t1: base.fingers[i].t1 + (i ? -1 : 1) * amp * Math.sin(t * freq + i * 2.1),
        t2: base.fingers[i].t2 + amp * Math.sin(t * freq + i * 0.9 + 1.3),
      });
    }
    return out;
  }

  function setPhase(name) {
    RT.phase = name;
    UI.setPhase(name);
  }

  function startAction(a) {
    RT.t = 0;
    RT.act = a;
    switch (a.k) {
      case 'movej': {
        // while the cube is in the hand, the keep-out volume follows the
        // action target (the hand carries the cube to exactly this pose)
        if (RT.attached === 'cube' && a.p && a.R) {
          RobotKin.setWorld([heldCubeOb(a.p, a.R)].concat(FIXTURE_OB));
        }
        const res = RobotKin.ik(a.p, a.R, RT.q, a.wristSide ? { wristSide: a.wristSide } : undefined);
        if (!res.ok) { failProgram('IK failed on movej (' + (a.tag || '') + ')'); return; }
        a._qEnd = res.q.slice();   // exact analytic solution for the final pose
        a._plan = RobotKin.planJointMove(RT.q, res.q);
        a._dur = a._plan.T;
        planMovejGuard(a, res.q);
        armGrip(a);
        break;
      }
      case 'movel': {
        const res = RobotKin.ik(a.p, a.R, RT.q, a.wristSide ? { wristSide: a.wristSide } : undefined);
        if (!res.ok) { failProgram('IK failed on movel target (' + (a.tag || '') + ')'); return; }
        a._qEnd = res.q.slice();   // exact analytic solution for the final pose
        const f0 = RobotKin.fk(RT.q);
        const d = Math.hypot(a.p[0] - f0.p[0], a.p[1] - f0.p[1], a.p[2] - f0.p[2]);
        let ang = 0;
        { const qe = RobotKin.quatFromMat(a.R), qc = RobotKin.quatFromMat(f0.R);
          const dot = Math.abs(qe[0] * qc[0] + qe[1] * qc[1] + qe[2] * qc[2] + qe[3] * qc[3]);
          ang = 2 * Math.acos(Math.min(1, dot)); }
        a._dur = Math.max(d / 0.42, ang / 2.6, a.gripTo ? 0.12 : 0.045);
        a._p0 = f0.p.slice();
        a._R0 = f0.R.map((r) => r.slice());
        a._warned = false;
        armGrip(a);
        break;
      }
      case 'twist': {
        const q1 = RT.q.slice();
        // J6 travel wrap: keep the tool roll inside its +/-2*pi limits —
        // shifting the target by a whole turn reaches the SAME final
        // tool pose (2*pi roll invariance), so the layer still settles
        // exactly; the hand may simply take the long way round
        let t6 = q1[5] + a.dq;
        if (t6 > Math.PI * 2) t6 -= Math.PI * 2;
        else if (t6 < -Math.PI * 2) t6 += Math.PI * 2;
        q1[5] = t6;
        a._plan = RobotKin.planJointMove(RT.q, q1);
        a._dur = a._plan.T;
        a._f0 = RT.fingers;
        break;
      }
      case 'read': {
        Vision.beginRead(a.faces);
        a._f0 = RT.fingers;
        a._dur = a.dur || 0.5;
        break;
      }
      case 'grip': {
        a._from = RT.fingers;
        a._to = gripAnglesFor(a.mode === 'open' ? 'open' : a.mode);
        a._dur = 0.16;
        RT.gripMode = (a.mode || 'open').toUpperCase();
        break;
      }
      case 'attachLayer': {
        const list = layerCubies(a.w || [0, 1, 0]);
        for (const c of list) toolNode.attach(c);
        RT.attached = 'layer';
        a._dur = 0.02;
        break;
      }
      case 'attachCube': {
        toolNode.attach(cubeGroup);
        RT.attached = 'cube';
        setWorldCube(false);          // cube rides with the hand now
        a._dur = 0.02;
        break;
      }
      case 'snapLayer': {
        // release from the hand, apply logically, rebuild visuals exactly
        for (const c of cubies) if (c.parent === toolNode) cubeGroup.attach(c);
        RT.state = CubeCore.applyMove(RT.state, a.face, a.q);
        RT.moveHistory.push({ face: a.face, q: a.q });
        rebuildCubeFromHistory();
        RT.attached = null;
        RT.twistsDone++;
        UI.drawNet();
        UI.updateProgress();
        a._dur = 0.03;
        break;
      }
      case 'setOrient': {
        RT.M = a.M.map((r) => r.slice());
        a._dur = 0.02;
        break;
      }
      case 'snapCube': {
        scene.attach(cubeGroup);
        cubeGroup.position.set(R.cubePos[0], R.cubePos[1], R.cubePos[2]);
        cubeGroup.quaternion.copy(intMatToQuat(RT.M));
        RT.attached = null;
        setWorldCube(true);          // cube parked on the fixture again
        a._dur = 0.045;
        break;
      }
      case 'move': {
        // fast scramble turn (no robot): spin the layer about the face axis
        const w = CubeCore.matVec(RT.M, CubeCore.NORMALS[a.face]);
        const list = [];
        for (const c of cubies) {
          c.getWorldPosition(tmpV);
          const dot = (tmpV.x - R.cubePos[0]) * w[0] + (tmpV.y - R.cubePos[1]) * w[1] + (tmpV.z - R.cubePos[2]) * w[2];
          if (dot > 0.006) list.push(c);
        }
        a._pivot = new THREE.Group();
        scene.add(a._pivot);
        a._pivot.position.set(R.cubePos[0], R.cubePos[1], R.cubePos[2]);
        for (const c of list) a._pivot.attach(c);
        a._axis = new THREE.Vector3(w[0], w[1], w[2]);
        a._angle = -(Math.PI / 2) * a.q;
        a._dur = 0.16;
        break;
      }
      case 'home': {
        a._plan = RobotKin.planJointMove(RT.q, RobotKin.HOME);
        a._dur = a._plan.T;
        break;
      }
      case 'done': {
        a._dur = 0.01;
        break;
      }
      default:
        a._dur = 0.01;
    }
    RT.dur = a._dur != null ? a._dur : 0.01;
    if (a.tag && a.tag !== RT.phase) setPhase(a.tag);
    if (a.label) UI.setProgramActive(RT.idx);
  }

  function finishAction(a) {
    switch (a.k) {
      case 'movej': case 'movel': case 'home':
        // final q already set by the last update
        if (a.gripTo) RT.fingers = a._gTo;   // fingers settle exactly
        break;
      case 'twist':
        RT.fingers = a._f0;
        break;
      case 'read':
        RT.fingers = a._f0;
        Vision.commit(a.faces);
        RT.twistsDone++;
        UI.updateProgress();
        break;
      case 'grip':
        RT.fingers = a._to;
        break;
      case 'move': {
        for (const c of cubies) if (c.parent === a._pivot) cubeGroup.attach(c);
        scene.remove(a._pivot);
        RT.state = CubeCore.applyMove(RT.state, a.face, a.q);
        RT.moveHistory.push({ face: a.face, q: a.q });
        rebuildCubeFromHistory();
        RT.twistsDone++;
        UI.drawNet();
        UI.updateProgress();
        break;
      }
      case 'done': {
        RT.running = false;
        RT.program = null;
        UI.programFinished();
        if (RT.onDone) { const cb = RT.onDone; RT.onDone = null; cb(); }
        return;
      }
    }
    RT.idx++;
    if (RT.idx >= (RT.program ? RT.program.length : 0)) {
      RT.running = false;
      UI.programFinished();
      if (RT.onDone) { const cb = RT.onDone; RT.onDone = null; cb(); }
    } else {
      startAction(RT.program[RT.idx]);
    }
  }

  function updateAction(dt) {
    const a = RT.act;
    if (!a) return;
    RT.t += dt;
    const u = Math.min(1, RT.t / RT.dur);
    switch (a.k) {
      case 'movej': case 'home': {
        if (a._planA) {
          // via-point detour: two synchronized trapezoids back to back
          RT.q = RobotKin.evalJointMove(a._planA, Math.min(RT.t, a._planA.T));
          if (RT.t > a._planA.T) RT.q = RobotKin.evalJointMove(a._planB, RT.t - a._planA.T);
        } else {
          RT.q = RobotKin.evalJointMove(a._plan, RT.t);
        }
        if (a.gripTo) RT.fingers = fingersLerp(a._gFrom, a._gTo, gripEase(Math.min(1, RT.t / a._dur)));
        break;
      }
      case 'twist': {
        RT.q = RobotKin.evalJointMove(a._plan, RT.t);
        RT.fingers = wiggleFingers(a._f0, RT.t, 0.008, 15);
        break;
      }
      case 'read': {
        Vision.update(u);
        RT.fingers = wiggleFingers(a._f0, RT.t, 0.017, 11);
        break;
      }
      case 'movel': {
        if (u >= 1 && a._qEnd) {
          // guarantee an exact final pose regardless of speed multiplier
          RT.q = a._qEnd.slice();
          if (a.gripTo) RT.fingers = a._gTo;
          break;
        }
        const p = [
          a._p0[0] + (a.p[0] - a._p0[0]) * u,
          a._p0[1] + (a.p[1] - a._p0[1]) * u,
          a._p0[2] + (a.p[2] - a._p0[2]) * u,
        ];
        const Rm = RobotKin.slerpR(a._R0, a.R, u);
        const res = RobotKin.ik(p, Rm, RT.q, { track: true });
        RT.q = res.q;
        if (a.gripTo) RT.fingers = fingersLerp(a._gFrom, a._gTo, gripEase(u));
        if (!res.ok && res.err > 0.004 && !a._warned) {
          a._warned = true;
          console.warn('movel tracking residual', res.err.toFixed(4), a.tag);
        }
        break;
      }
      case 'grip': {
        const e = u < 0.5 ? 2 * u * u : 1 - Math.pow(-2 * u + 2, 2) / 2;
        RT.fingers = fingersLerp(a._from, a._to, e);
        break;
      }
      case 'move': {
        a._pivot.quaternion.setFromAxisAngle(a._axis, a._angle * u);
        break;
      }
    }
    if (RT.t >= RT.dur) finishAction(a);
  }

  function failProgram(msg) {
    RT.running = false;
    RT.program = null;
    RT.error = msg;
    setPhase('ERROR');
    UI.toast('Motion planning fault: ' + msg);
    UI.programFinished();
  }

  function runProgram(program, opts) {
    opts = opts || {};
    RT.program = program;
    RT.idx = 0;
    RT.running = true;
    RT.paused = false;
    RT.error = null;
    RT.twistsDone = 0;
    RT.twistsTotal = opts.twistsTotal || program.filter((a) => a.k === 'snapLayer' || a.k === 'move').length;
    RT.movesDone = 0;
    RT.execTime = 0;
    RT.onDone = opts.onDone || null;
    UI.buttons();          // enable PAUSE, lock the program-start buttons
    UI.buildProgramList(program);
    UI.updateProgress();
    startAction(program[0]);
  }

  function stopProgram() {
    RT.running = false;
    RT.program = null;
    RT.act = null;
    RT.paused = false;
    Vision.active = null;
    // safely re-home any attached objects
    if (RT.attached === 'cube') {
      scene.attach(cubeGroup);
      cubeGroup.position.set(R.cubePos[0], R.cubePos[1], R.cubePos[2]);
      cubeGroup.quaternion.copy(intMatToQuat(RT.M));
    } else if (RT.attached === 'layer') {
      for (const c of cubies) if (c.parent === toolNode) cubeGroup.attach(c);
      snapCubiesToLattice();
    }
    RT.attached = null;
    setWorldCube(true);
  }

  function resetAll() {
    stopProgram();
    RT.moveHistory = [];
    RT.state = CubeCore.solvedState();
    RT.twistsDone = 0;
    RT.twistsTotal = 0;
    RT.execTime = 0;
    RT.scanMode = false;
    RT.M = CubeCore.IDENT.map((r) => r.slice());
    setWorldCube(true);
    Vision.reset();
    UI.setFaceletString('—');
    // rebuild cube exactly
    let idx = 0;
    for (let i = -1; i <= 1; i++) for (let j = -1; j <= 1; j++) for (let k = -1; k <= 1; k++) {
      const c = cubies[idx++];
      if (c.parent !== cubeGroup) cubeGroup.attach(c);
      c.position.set(i * S, j * S, k * S);
      c.quaternion.identity();
    }
    cubeGroup.position.set(R.cubePos[0], R.cubePos[1], R.cubePos[2]);
    cubeGroup.quaternion.copy(intMatToQuat(CubeCore.IDENT));
    RT.fingers = gripAnglesFor('open');
    RT.gripMode = 'OPEN';
    RT.q = RobotKin.HOME.slice();
    RT.scrambleText = '';
    RT.scrambleMeta = '';
    RT.solutionText = '';
    setPhase('IDLE');
    UI.drawNet();
    UI.clearSolution();
    UI.programFinished();
    UI.setScramble('—');
    UI.updateProgress();
    UI.clearProgramList();
    UI.buttons();
  }

  // ------------------------------------------------------------------
  // UI layer
  // ------------------------------------------------------------------
  const $ = (id) => document.getElementById(id);
  const UI = (function () {
    const jointRows = [];
    const PHASES = ['IDLE', 'SCAN', 'APPROACH', 'DESCEND', 'GRASP', 'LIFT', 'REORIENT', 'PLACE', 'TWIST', 'RELEASE', 'RETRACT', 'HOMING', 'SCRAMBLE', 'COMPUTING', 'DONE', 'ERROR'];
    const phaseEls = {};

    // build joint telemetry rows
    {
      const jt = $('jointTable');
      for (let j = 0; j < 6; j++) {
        const row = document.createElement('div');
        row.className = 'jrow';
        row.innerHTML =
          '<span class="jl">J' + (j + 1) + '</span>' +
          '<span class="jbar"><i></i></span>' +
          '<span class="jv">0.0°</span>' +
          '<span class="jw">0°/s</span>';
        jt.appendChild(row);
        jointRows.push({
          bar: row.querySelector('.jbar i'),
          v: row.querySelector('.jv'),
          w: row.querySelector('.jw'),
        });
      }
    }
    // phase list
    {
      const pl = $('phaseList');
      for (const ph of PHASES) {
        const el = document.createElement('div');
        el.className = 'ph';
        el.textContent = ph;
        pl.appendChild(el);
        phaseEls[ph] = el;
      }
    }
    // gripper jaw bars
    const fingerRows = [];
    {
      const hb = $('handBox');
      const names = ['JAW A', 'JAW B'];
      for (const n of names) {
        const row = document.createElement('div');
        row.className = 'jrow small';
        row.innerHTML = '<span class="jl">' + n + '</span><span class="jbar"><i></i></span><span class="jv">0°</span>';
        hb.appendChild(row);
        fingerRows.push({ bar: row.querySelector('.jbar i'), v: row.querySelector('.jv') });
      }
      const modeRow = document.createElement('div');
      modeRow.className = 'gripmode';
      modeRow.innerHTML = 'GRIP <b id="gripModeVal">OPEN</b>';
      hb.appendChild(modeRow);
    }

    // vision face-acquisition mini grids
    const scanGridCells = [];
    {
      const host = $('scanGrids');
      if (host) {
        for (let f = 0; f < 6; f++) {
          const blk = document.createElement('div');
          blk.className = 'sgrid';
          const lab = document.createElement('span');
          lab.className = 'slabel';
          lab.textContent = CubeCore.FACES[f];
          blk.appendChild(lab);
          const cells = document.createElement('div');
          cells.className = 'scells';
          scanGridCells.push([]);
          for (let i = 0; i < 9; i++) {
            const c = document.createElement('i');
            cells.appendChild(c);
            scanGridCells[f].push(c);
          }
          blk.appendChild(cells);
          host.appendChild(blk);
        }
      }
    }

    const netCanvas = $('netCanvas');
    const nctx = netCanvas ? netCanvas.getContext('2d') : null;
    const chipsBox = $('solutionChips');
    const progList = $('programList');
    const progFill = $('progressFill');

    const NET_LAYOUT = {
      0: { x: 1, y: 0 }, 4: { x: 0, y: 1 }, 2: { x: 1, y: 1 }, 1: { x: 2, y: 1 }, 5: { x: 3, y: 1 }, 3: { x: 1, y: 2 },
    };

    function drawNet() {
      if (!nctx) return;
      const CELL = 14, GAP = 3, M = 8;
      const faceS = CELL * 3 + GAP * 2;
      nctx.clearRect(0, 0, netCanvas.width, netCanvas.height);
      for (let f = 0; f < 6; f++) {
        const L = NET_LAYOUT[f];
        const ox = M + L.x * (faceS + GAP), oy = M + L.y * (faceS + GAP);
        for (let r = 0; r < 3; r++) for (let c = 0; c < 3; c++) {
          nctx.fillStyle = '#' + CUBE_COLORS[RT.state[f * 9 + r * 3 + c]].toString(16).padStart(6, '0');
          nctx.fillRect(ox + c * (CELL + GAP), oy + r * (CELL + GAP), CELL, CELL);
        }
      }
    }

    let chipEls = [];
    function buildChips(moves) {
      chipsBox.innerHTML = '';
      chipEls = [];
      for (const m of moves) {
        const el = document.createElement('span');
        el.className = 'chip';
        el.textContent = CubeCore.moveToNotation(m);
        chipsBox.appendChild(el);
        chipEls.push(el);
      }
    }
    function updateChips() {
      for (let i = 0; i < chipEls.length; i++) {
        const el = chipEls[i];
        el.className = 'chip' + (i < RT.twistsDone ? ' done' : i === RT.twistsDone ? ' active' : '');
      }
      const act = chipEls[RT.twistsDone];
      if (act && act.scrollIntoView) act.scrollIntoView({ block: 'nearest', inline: 'nearest' });
    }

    let progEls = [];
    function buildProgramList(program) {
      progList.innerHTML = '';
      progEls = [];
      program.forEach((a, i) => {
        const el = document.createElement('div');
        el.className = 'prow';
        el.textContent = (i + 1) + '. ' + (a.label || a.k.toUpperCase());
        progList.appendChild(el);
        progEls.push(el);
      });
    }
    function setProgramActive(i) {
      for (let k = 0; k < progEls.length; k++) progEls[k].className = 'prow' + (k === i ? ' active' : k < i ? ' past' : '');
      const el = progEls[i];
      if (el && el.scrollIntoView) el.scrollIntoView({ block: 'nearest' });
    }

    function updateProgress() {
      const pct = RT.twistsTotal ? Math.min(100, 100 * RT.twistsDone / RT.twistsTotal) : 0;
      progFill.style.width = pct.toFixed(1) + '%';
      $('progressText').textContent = RT.scanMode
        ? RT.twistsDone + ' / ' + RT.twistsTotal + ' face reads'
        : RT.twistsDone + ' / ' + RT.twistsTotal + ' turns';
      updateChips();
    }

    function refreshScanPanel() {
      for (let f = 0; f < 6; f++) {
        const rd = Vision.faces[f];
        for (let i = 0; i < 9; i++) {
          scanGridCells[f][i].style.background = rd ? cssColor(rd.colors[i]) : '';
        }
      }
      const n = Vision.acquired();
      $('scanStat').textContent = 'FACES ' + n + '/6 · FACELETS ' + (n * 9) + '/54';
    }
    function setFaceletString(s) { $('faceletStr').textContent = s; }
    function setCamStatus(t) {
      const el = $('camPipStatus');
      if (el) el.textContent = t;
    }

    function setPhase(name) {
      for (const k in phaseEls) phaseEls[k].className = 'ph' + (k === name ? ' active' : '');
    }
    function setScramble(t) { $('scrambleBox').textContent = t; }
    function clearSolution() {
      chipsBox.innerHTML = '<span class="dim">— no solution computed —</span>';
      chipEls = [];
      $('statMoves').textContent = '0';
      $('statTime').textContent = '0:00';
    }
    let toastTimer = null;
    function toast(msg) {
      const t = $('toast');
      t.textContent = msg;
      t.className = 'toast show';
      clearTimeout(toastTimer);
      toastTimer = setTimeout(() => { t.className = 'toast'; }, 3400);
    }
    function buttons() {
      const busy = RT.running;
      $('btnScramble').disabled = busy;
      $('btnSolve').disabled = busy;
      $('btnReset').disabled = false;
      $('btnApply').disabled = busy;
      $('btnPause').disabled = !busy;
      $('btnPause').textContent = RT.paused ? 'RESUME' : 'PAUSE';
    }
    function programFinished() {
      buttons();
      setProgramActive(-2);
    }
    function clearProgramList() {
      progList.innerHTML = '<div class="dim" style="padding:2px 7px">— idle —</div>';
      progEls = [];
    }

    let lastTelemetry = 0;
    function telemetry(now, dt, speed) {
      if (now - lastTelemetry < 100) return;
      lastTelemetry = now;
      const f = RobotKin.fk(RT.q);
      for (let j = 0; j < 6; j++) {
        const lo = RobotKin.LIMITS[j][0], hi = RobotKin.LIMITS[j][1];
        const frac = (RT.q[j] - lo) / (hi - lo);
        jointRows[j].bar.style.width = (100 * Math.min(1, Math.max(0, frac))).toFixed(1) + '%';
        jointRows[j].v.textContent = (RT.q[j] / DEG).toFixed(1) + '°';
        const w = Math.abs(RT.q[j] - RT.prevQ[j]) / Math.max(dt, 1e-3);
        jointRows[j].w.textContent = (w / DEG).toFixed(0) + '°/s';
      }
      const rpy = RobotKin.rotToRPY(f.R);
      $('tcpVals').innerHTML =
        'X ' + (f.p[0] * 1000).toFixed(0) + ' &nbsp;Y ' + (f.p[1] * 1000).toFixed(0) + ' &nbsp;Z ' + (f.p[2] * 1000).toFixed(0) + ' mm<br>' +
        'R ' + (rpy[0] / DEG).toFixed(1) + '° &nbsp;P ' + (rpy[1] / DEG).toFixed(1) + '° &nbsp;Y ' + (rpy[2] / DEG).toFixed(1) + '°';
      for (let i = 0; i < 2; i++) {
        const src = RT.fingers.fingers[i];
        const flex = (src.t1 + src.t2) / DEG;
        fingerRows[i].bar.style.width = Math.min(100, flex / 1.5).toFixed(1) + '%';
        fingerRows[i].v.textContent = flex.toFixed(0) + '°';
      }
      const gv = $('gripModeVal');
      if (gv) gv.textContent = RT.gripMode;
      $('statMoves').textContent = String(RT.twistsDone);
      $('statTime').textContent = fmtTime(RT.execTime);
    }
    function fmtTime(t) {
      const m = Math.floor(t / 60), s = Math.floor(t % 60);
      return m + ':' + String(s).padStart(2, '0');
    }

    return {
      drawNet: drawNet, buildChips: buildChips, updateChips: updateChips,
      buildProgramList: buildProgramList, setProgramActive: setProgramActive,
      updateProgress: updateProgress, setPhase: setPhase,
      setScramble: setScramble,
      refreshScanPanel: refreshScanPanel, setFaceletString: setFaceletString,
      setCamStatus: setCamStatus,
      clearSolution: clearSolution, toast: toast, buttons: buttons,
      clearProgramList: clearProgramList,
      programFinished: programFinished, telemetry: telemetry, fmtTime: fmtTime,
    };
  })();

  // ------------------------------------------------------------------
  // control flow: scramble / solve / pause / reset
  // ------------------------------------------------------------------
  let speedMult = 1.0, turbo = false;
  function effectiveSpeed() { return speedMult * (turbo ? 4 : 1); }

  function doScramble(moves, meta) {
    if (RT.running) return;
    if (CubeCore.isSolved(RT.state) === false && moves) { /* fine */ }
    RT.scrambleMeta = meta || '';
    RT.scrambleText = CubeCore.algToString(moves);
    UI.setScramble((RT.scrambleMeta ? RT.scrambleMeta + ' — ' : '') + RT.scrambleText);
    setPhase('SCRAMBLE');
    Vision.reset();
    UI.setFaceletString('—');
    const program = RobotPlan.planScramble(moves);
    runProgram(program, {
      onDone: () => {
        setPhase('IDLE');
        UI.toast('Scramble applied: ' + (RT.scrambleMeta ? RT.scrambleMeta + ' — ' : '') + RT.scrambleText);
      },
    });
  }

  // workspace probe handed to the task planner: the scheduler only
  // plans direct side-layer twists whose compact grasp poses verify
  // reachable AND collision-free (keep-out zones active), otherwise it
  // falls back to whole-cube rolls — the offline reachability check a
  // real cell runs before committing to a grasp strategy
  const IK_PROBE_SEEDS = [
    RobotKin.HOME,
    [0.4, -0.6, 1.0, -0.4, -1.0, 0],
    [-0.4, -0.6, 1.0, -0.4, -1.0, 0],
    [1.2, -0.9, 1.6, -0.5, -1.2, 0],
    [-1.2, -0.9, 1.6, -0.5, -1.2, 0],
  ];
  function ikReach(p, Rt) {
    for (const s of IK_PROBE_SEEDS) {
      if (RobotKin.ik(p, Rt, s.slice(), { requireClear: true }).ok) return true;
    }
    return false;
  }

  // Full autonomous pipeline, exactly like a real vision-driven cell:
  //   SCAN (the wrist camera flies eye-in-hand inspection vantages
  //         around the cube and acquires all 6 faces)
  //   -> SOLVE (Kociemba two-phase on the scanned facelet string)
  //   -> EXECUTE (planned grasps, in-hand rolls and layer twists).
  function doSolve() {
    if (RT.running) return;
    if (CubeCore.isSolved(RT.state)) { UI.toast('The cube is already solved.'); return; }
    setPhase('SCAN');
    RT.scanMode = true;
    UI.toast('Eye-in-hand scan: wrist camera acquiring facelets…');
    const scanProgram = RobotPlan.planScan(RT.M);
    runProgram(scanProgram, {
      twistsTotal: scanProgram.filter((a) => a.k === 'read').length,
      onDone: () => {
        RT.scanMode = false;
        const facelets = Vision.faceletString();
        if (!facelets || facelets.length !== 54) {
          setPhase('ERROR');
          UI.toast('Vision acquisition incomplete — cannot solve.');
          UI.buttons();
          return;
        }
        UI.setFaceletString(facelets);
        // integrity gate: the scanned state must match the physical cube
        if (facelets !== CubeCore.toFaceletString(RT.state)) {
          setPhase('ERROR');
          UI.toast('Vision state failed validation against the cube.');
          UI.buttons();
          return;
        }
        setPhase('COMPUTING');
        Solver.solve(facelets).then((sol) => {
      if (sol == null) {
        setPhase('ERROR');
        UI.toast('Solver failed to produce a solution.');
        UI.buttons();
        return;
      }
      let moves = null;
      try { moves = CubeCore.simplify(CubeCore.parseAlg(sol)); } catch (e) { moves = null; }
      if (!moves || !moves.length) {
        // empty solution -> already solved logically
        UI.toast('Solver: already solved.');
        setPhase('IDLE');
        return;
      }
      // verify the solution against the logical state before execution
      const check = CubeCore.applyMoves(RT.state, moves);
      if (!CubeCore.isSolved(check)) {
        UI.toast('Internal check failed: solution did not verify.');
        setPhase('ERROR');
        return;
      }
      RT.solutionText = sol;
      UI.buildChips(moves);
      const program = RobotPlan.planSolve(moves, RT.M, { ik: ikReach });
      runProgram(program, {
        twistsTotal: moves.length,
        onDone: () => {
          setPhase('DONE');
          UI.toast('Cube solved — ' + moves.length + ' turns in ' + UI.fmtTime(RT.execTime) + ' (sim time).');
        },
        });
      });
    },
  });
  }

  function doPause() {
    if (!RT.running) return;
    RT.paused = !RT.paused;
    UI.buttons();
  }

  function doReset() {
    resetAll();
    UI.toast('System reset.');
  }

  function applyCustomScramble() {
    if (RT.running) return;
    const txt = ($('scrambleInput').value || '').trim();
    if (!txt) { UI.toast('Enter a scramble sequence first, e.g. "R U R\' U\'"'); return; }
    let moves;
    try { moves = CubeCore.parseAlg(txt); } catch (e) {
      UI.toast('Invalid notation: ' + e.message);
      return;
    }
    if (CubeCore.isSolved(RT.state) === false) {
      // reset first so the custom scramble applies to a solved cube
      resetAll();
    }
    doScramble(moves);
  }

  // ------------------------------------------------------------------
  // scramble library — 512 verified-distinct scrambles. The SCRAMBLE
  // button draws from a shuffle bag (no repetition until all 512 have
  // been used), the selector can pin any single entry, or switch to
  // unlimited true-random generation. Selecting an entry also copies
  // its sequence into the custom-scramble input for hand editing.
  // ------------------------------------------------------------------
  const libSel = $('libSel');
  let libBag = [], libBagPos = 0;

  function buildLibSelect() {
    if (!libSel || !root.ScrambleLib) return;
    const L = root.ScrambleLib;
    libSel.innerHTML = '';
    const mode = document.createElement('optgroup');
    mode.label = 'MODE';
    for (const [v, t] of [
      ['shuffle', 'LIBRARY SHUFFLE · ' + L.count + ' distinct scrambles'],
      ['random', '∞ TRUE RANDOM · fresh 25-move every time'],
    ]) {
      const o = document.createElement('option');
      o.value = v; o.textContent = t; mode.appendChild(o);
    }
    libSel.appendChild(mode);
    let group = null, groupCat = null;
    for (const e of L.entries) {                       // [id, cat, name, seq]
      if (groupCat !== e[1]) {
        group = document.createElement('optgroup');
        group.label = e[1];
        libSel.appendChild(group);
        groupCat = e[1];
      }
      const o = document.createElement('option');
      o.value = String(e[0]);
      o.textContent = '#' + e[0] + ' · ' + e[2];
      group.appendChild(o);
    }
    libSel.value = 'shuffle';
  }
  buildLibSelect();

  if (libSel) libSel.addEventListener('change', () => {
    const v = libSel.value;
    if (v === 'shuffle' || v === 'random' || !root.ScrambleLib) return;
    const e = root.ScrambleLib.entries[+v - 1];
    if (e) $('scrambleInput').value = e[3];
  });

  function libNext() {
    const L = root.ScrambleLib;
    if (libBagPos >= libBag.length) {
      // refill + Fisher-Yates shuffle: no repeat until the bag empties
      libBag = L.entries.map((_, i) => i);
      for (let i = libBag.length - 1; i > 0; i--) {
        const j = Math.floor(Math.random() * (i + 1));
        const t = libBag[i]; libBag[i] = libBag[j]; libBag[j] = t;
      }
      libBagPos = 0;
    }
    return L.entries[libBag[libBagPos++]];
  }

  function applyLibEntry(e) {
    doScramble(CubeCore.parseAlg(e[3]), '#' + e[0] + '/' + root.ScrambleLib.count + ' · ' + e[2]);
  }

  function scrambleFromLibrary() {
    if (!root.ScrambleLib) { doScramble(CubeCore.randomScramble(25)); return; }
    const v = libSel ? libSel.value : 'shuffle';
    if (v === 'random') { doScramble(CubeCore.randomScramble(25), '∞ RANDOM'); return; }
    if (v === 'shuffle') { applyLibEntry(libNext()); return; }
    const e = root.ScrambleLib.entries[+v - 1];
    if (e) applyLibEntry(e); else doScramble(CubeCore.randomScramble(25));
  }

  // wire up buttons
  $('btnScramble').addEventListener('click', () => {
    if (RT.running) return;
    if (!CubeCore.isSolved(RT.state)) resetAll();
    scrambleFromLibrary();
  });
  $('btnSolve').addEventListener('click', doSolve);
  $('btnPause').addEventListener('click', doPause);
  $('btnReset').addEventListener('click', doReset);
  $('btnApply').addEventListener('click', applyCustomScramble);
  $('scrambleInput').addEventListener('keydown', (e) => { if (e.key === 'Enter') applyCustomScramble(); });

  const speedRange = $('speedRange');
  function updateSpeedLabel() {
    speedMult = Math.pow(2, -2 + 6 * (speedRange.value / 100));
    $('speedVal').textContent = '×' + (speedMult >= 1 ? speedMult.toFixed(1) : speedMult.toFixed(2));
  }
  speedRange.addEventListener('input', updateSpeedLabel);
  updateSpeedLabel();
  $('chkTurbo').addEventListener('change', (e) => { turbo = e.target.checked; });
  $('chkOrbit').addEventListener('change', (e) => { orbit.state.autoRotate = e.target.checked; });

  $('camIso').addEventListener('click', () => orbit.setView(Math.PI * 0.17, 0.95, 0.50));
  $('camHand').addEventListener('click', () => orbit.setView(Math.PI * 0.17, 0.88, 0.34));
  $('camTop').addEventListener('click', () => orbit.setView(Math.PI * 0.5, 0.18, 0.70));
  $('camFront').addEventListener('click', () => orbit.setView(Math.PI * 0.5, 1.32, 0.68));

  const dhPanel = $('dhPanel');
  const dhToggleBtn = $('dhToggle');
  dhPanel.style.display = 'none';
  dhToggleBtn.addEventListener('click', () => {
    const show = dhPanel.style.display === 'none';
    dhPanel.style.display = show ? 'block' : 'none';
    dhToggleBtn.textContent = show ? 'HIDE KINEMATICS (DH)' : 'SHOW KINEMATICS (DH)';
  });

  // ------------------------------------------------------------------
  // main loop
  // ------------------------------------------------------------------
  setWorldCube(true);
  setQ(RT.q);
  setFingerAngles(RT.fingers);
  UI.drawNet();
  UI.buttons();
  Solver.init();
  measurePip();

  let lastT = performance.now();
  let visionClock = 0;
  function frame() {
    requestAnimationFrame(frame);
    const now = performance.now();
    let dt = Math.min(0.05, (now - lastT) / 1000);
    lastT = now;
    visionClock += dt;

    const sp = effectiveSpeed();
    const sdt = (RT.running && !RT.paused) ? dt * sp : 0;
    RT.prevQ = RT.q.slice();
    if (sdt > 0) {
      RT.execTime += sdt;
      // sub-step the action clock: short actions (attach / snap / tiny
      // moves) complete within a single frame even at high speed
      // multipliers, so the measured exec time tracks the planned
      // trajectory durations instead of the frame quantum
      let rem = sdt;
      while (rem > 1e-9 && RT.running && !RT.paused) {
        const need = RT.dur - RT.t;
        if (need > rem) { updateAction(rem); rem = 0; }
        else { updateAction(need); rem -= need; }
      }
    }
    setQ(RT.q);
    setFingerAngles(RT.fingers);

    orbit.update(dt);
    renderer.render(scene, camera);
    // live wrist-camera feed (picture-in-picture) + machine-vision HUD
    renderPip();
    drawVisionOverlay(now);
    // camera status lights: ring light pulses while acquiring, standby
    // LED blinks slowly otherwise
    {
      const acquiring = !!Vision.active;
      if (camRingMat) camRingMat.emissiveIntensity = acquiring
        ? 1.9 + 0.9 * Math.sin(visionClock * 14) : 0.55;
      if (camLedMat) {
        const on = acquiring ? (visionClock * 6 % 1 < 0.6) : (visionClock * 1.3 % 1 < 0.55);
        camLedMat.emissiveIntensity = on ? 1.8 : 0.25;
        camLedMat.color.setHex(acquiring ? 0xf59e0b : 0x34d399);
      }
      const rd = $('recDot');
      if (rd) rd.style.opacity = Math.floor(now / 600) % 2 === 0 ? '1' : '0.25';
    }
    UI.telemetry(now, sdt, sp);
  }
  frame();

  window.addEventListener('resize', () => {
    const w = view.clientWidth, h = view.clientHeight;
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
    renderer.setSize(w, h);
    measurePip();
  });

  // expose debug handles
  root.__rubikApp = { RT: RT, runProgram: runProgram, Vision: Vision };
  root.__scene = scene;
  root.__camera = camera;
  root.__visionCam = visionCam;
  root.__renderer = renderer;
  root.__pip = { renderPip: renderPip, measurePip: measurePip, getRect: () => pipRect };
  root.__orbit2 = orbit;
  root.__robot = { jointRots: jointRots, linkStatics: linkStatics, toolNode: toolNode,
    setQ: setQ, cubeGroup: cubeGroup, cubies: cubies, fingerJoints: fingerJoints };
})(window);
