/* ------------------------------------------------------------------
 * build.js — assembles the self-contained demo HTML.
 *
 * Usage:    node build.js        (or: npm run build)
 * Requires: Node.js >= 14, no npm packages.
 *
 * Reads the modules from src/, inlines them into src/template.html
 * (three.js r147 + the cubejs Kociemba solver are inlined from
 * src/vendor/), and writes a single dependency-free index.html to
 * the repository root — ready for GitHub Pages or a static host.
 * ------------------------------------------------------------------ */
'use strict';
const fs = require('fs');
const path = require('path');

const ROOT = __dirname;
const read = (f) => fs.readFileSync(path.join(ROOT, 'src', f), 'utf8');

const template = read('template.html');
const three = read('vendor/three.min.js');
const cubejsCore = read('vendor/cubejs-core.js');
const cubejsSolve = read('vendor/cubejs-solve.js');
const cubeCore = read('cube-core.js');
const scrambles = read('scrambles.js');
const kinematics = read('kinematics.js');
const planner = read('planner.js');
const controls = read('controls.js');
const app = read('app.js');

// guard: no </script> may appear inside inline scripts
function guard(name, src) {
  const bad = ['</script', '<!--'].filter((tok) => src.includes(tok));
  if (bad.length) throw new Error(`${name} contains forbidden token(s): ${bad}`);
}
guard('three', three);
guard('cubejsCore', cubejsCore);
guard('cubejsSolve', cubejsSolve);
for (const [n, s] of [['cubeCore', cubeCore], ['scrambles', scrambles], ['kinematics', kinematics], ['planner', planner], ['controls', controls], ['app', app]]) guard(n, s);

// worker source (cubejs core + solve), JSON-stringified with escaped slashes
const cubejsSrc = JSON.stringify(cubejsCore + '\n' + cubejsSolve).replace(/<\//g, '<\\/');

function inject(tpl, tag, content) {
  return tpl.replace(tag, () => content);
}

let html = template;
html = inject(html, '{{THREE}}', three);
html = inject(html, '{{CUBEJS_CORE}}', cubejsCore);
html = inject(html, '{{CUBEJS_SOLVE}}', cubejsSolve);
html = inject(html, '{{CUBEJS_SRC}}', cubejsSrc);
html = inject(html, '{{CUBE_CORE}}', cubeCore);
html = inject(html, '{{SCRAMBLES}}', scrambles);
html = inject(html, '{{KINEMATICS}}', kinematics);
html = inject(html, '{{PLANNER}}', planner);
html = inject(html, '{{CONTROLS}}', controls);
html = inject(html, '{{APP}}', app);

const outFile = path.join(ROOT, 'index.html');
fs.writeFileSync(outFile, html);
console.log('built', path.relative(process.cwd(), outFile), (html.length / 1024).toFixed(0) + ' KB');
