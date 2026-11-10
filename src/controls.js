/* =============================================================
 * controls.js — lightweight orbit camera controls
 * (drag to orbit, wheel to zoom, right-drag / two-finger to pan,
 *  smoothed with exponential damping)
 * ============================================================= */
(function (root, factory) {
  const api = factory();
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
  else root.SimpleOrbit = api;
})(typeof self !== 'undefined' ? self : globalThis, function () {
  'use strict';

  function create(camera, dom, opts) {
    opts = opts || {};
    const state = {
      target: opts.target ? opts.target.clone() : new THREE.Vector3(0, 0.15, 0),
      theta: opts.theta != null ? opts.theta : Math.PI * 0.15,   // azimuth
      phi: opts.phi != null ? opts.phi : 1.05,                   // polar from +Y
      radius: opts.radius != null ? opts.radius : 0.95,
      minRadius: opts.minRadius || 0.28,
      maxRadius: opts.maxRadius || 2.2,
      autoRotate: false,
      autoSpeed: 0.12,
    };
    const goal = {
      theta: state.theta, phi: state.phi, radius: state.radius,
      target: state.target.clone(),
    };

    let dragging = false, panning = false, lastX = 0, lastY = 0;
    let pinchDist = 0;

    function clampState() {
      goal.phi = Math.min(Math.max(goal.phi, 0.12), Math.PI - 0.12);
      goal.radius = Math.min(Math.max(goal.radius, state.minRadius), state.maxRadius);
    }

    function onDown(e) {
      if (e.button === 2 || e.ctrlKey) panning = true;
      else dragging = true;
      lastX = e.clientX; lastY = e.clientY;
      dom.style.cursor = 'grabbing';
      e.preventDefault();
    }
    function onMove(e) {
      if (!dragging && !panning) return;
      const dx = e.clientX - lastX, dy = e.clientY - lastY;
      lastX = e.clientX; lastY = e.clientY;
      if (dragging) {
        goal.theta -= dx * 0.006;
        goal.phi -= dy * 0.006;
      } else {
        // pan in camera space
        const scale = goal.radius * 0.0016;
        const right = new THREE.Vector3(), up = new THREE.Vector3();
        camera.matrixWorld.extractBasis(right, up, new THREE.Vector3());
        goal.target.addScaledVector(right, -dx * scale).addScaledVector(up, dy * scale);
      }
      clampState();
      e.preventDefault();
    }
    function onUp() { dragging = false; panning = false; dom.style.cursor = 'grab'; }
    function onWheel(e) {
      goal.radius *= (e.deltaY > 0 ? 1.09 : 0.92);
      clampState();
      e.preventDefault();
    }
    function onTouch(e) {
      if (e.touches.length === 1) {
        const t = e.touches[0];
        if (dragging) {
          goal.theta -= (t.clientX - lastX) * 0.007;
          goal.phi -= (t.clientY - lastY) * 0.007;
          clampState();
        }
        lastX = t.clientX; lastY = t.clientY;
        dragging = true; panning = false;
      } else if (e.touches.length === 2) {
        const d = Math.hypot(
          e.touches[0].clientX - e.touches[1].clientX,
          e.touches[0].clientY - e.touches[1].clientY);
        if (pinchDist > 0) {
          goal.radius *= pinchDist / Math.max(d, 1);
          clampState();
        }
        pinchDist = d;
        dragging = false; panning = true;
      }
      e.preventDefault();
    }
    function onTouchEnd() { dragging = false; panning = false; pinchDist = 0; }

    dom.addEventListener('mousedown', onDown);
    window.addEventListener('mousemove', onMove);
    window.addEventListener('mouseup', onUp);
    dom.addEventListener('wheel', onWheel, { passive: false });
    dom.addEventListener('contextmenu', (e) => e.preventDefault());
    dom.addEventListener('touchstart', onTouch, { passive: false });
    dom.addEventListener('touchmove', onTouch, { passive: false });
    dom.addEventListener('touchend', onTouchEnd);

    const sph = new THREE.Spherical();
    const tmp = new THREE.Vector3();

    function update(dt) {
      if (state.autoRotate) goal.theta += state.autoSpeed * dt;
      const k = 1 - Math.pow(0.0001, dt); // exponential smoothing
      state.theta += (goal.theta - state.theta) * k;
      state.phi += (goal.phi - state.phi) * k;
      state.radius += (goal.radius - state.radius) * k;
      state.target.lerp(goal.target, k);
      sph.set(state.radius, state.phi, state.theta);
      tmp.setFromSpherical(sph).add(state.target);
      camera.position.copy(tmp);
      camera.lookAt(state.target);
    }

    function setView(theta, phi, radius, immediate) {
      goal.theta = theta; goal.phi = phi;
      if (radius) goal.radius = radius;
      clampState();
      if (immediate) {
        state.theta = theta; state.phi = phi;
        if (radius) state.radius = radius;
      }
    }

    return { update: update, state: state, setView: setView, goal: goal };
  }

  return { create: create };
});
