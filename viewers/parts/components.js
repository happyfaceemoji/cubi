/* Detailed models of Cubi's bought components, placed where the fit check put them.
   Everything is in millimetres in robot coordinates (x right, y up, z forward). */
window.buildCubiComponents = function (THREE, D) {
  const C = D.C, G = D.G;
  const M = (color, o) => new THREE.MeshStandardMaterial(Object.assign({ color, roughness: 0.55, metalness: 0 }, o || {}));
  const mat = {
    servo: M('#18181C', { roughness: 0.3 }), horn: M('#EFEDE6', { roughness: 0.45 }),
    brass: M('#C9A24A', { metalness: 0.9, roughness: 0.3 }), steel: M('#C9CCD3', { metalness: 1, roughness: 0.22 }),
    dsteel: M('#4C4F57', { metalness: 0.9, roughness: 0.35 }), hole: M('#060608', { roughness: 1 }),
    black: M('#141416', { roughness: 0.5 }), gold: M('#D8B45A', { metalness: 1, roughness: 0.25 }),
    term: M('#2F8A4A', { roughness: 0.5 }), glass: M('#07070A', { roughness: 0.05, metalness: 0.2 }),
    paper: M('#1A1A1C', { roughness: 0.95 }), plug: M('#1C1C1F', { roughness: 0.6 })
  };
  const Box = (w, h, d) => new THREE.BoxGeometry(w, h, d);
  const Cyl = (r1, r2, h, s) => new THREE.CylinderGeometry(r1, r2, h, s || 24);
  function add(p, g, m, x, y, z) { const o = new THREE.Mesh(g, m); o.position.set(x || 0, y || 0, z || 0); p.add(o); return o; }
  function wire(p, pts, color, r) {
    const c = new THREE.CatmullRomCurve3(pts.map(q => new THREE.Vector3(q[0], q[1], q[2])));
    return add(p, new THREE.TubeGeometry(c, 20, r, 8, false), M(color, { roughness: 0.6 }));
  }
  function rounded(w, h, d, bevel, r) {
    const s = new THREE.Shape(), W = w - 2 * bevel, H = h - 2 * bevel, x = -W / 2, y = -H / 2;
    s.moveTo(x + r, y); s.lineTo(x + W - r, y); s.quadraticCurveTo(x + W, y, x + W, y + r);
    s.lineTo(x + W, y + H - r); s.quadraticCurveTo(x + W, y + H, x + W - r, y + H);
    s.lineTo(x + r, y + H); s.quadraticCurveTo(x, y + H, x, y + H - r); s.lineTo(x, y + r); s.quadraticCurveTo(x, y, x + r, y);
    const g = new THREE.ExtrudeGeometry(s, { depth: d - 2 * bevel, bevelEnabled: true, bevelThickness: bevel, bevelSize: bevel, bevelSegments: 3, curveSegments: 6 });
    g.center(); return g;
  }
  function tex(w, h, draw) {
    const c = document.createElement('canvas'); c.width = w; c.height = h; draw(c.getContext('2d'), w, h);
    const t = new THREE.CanvasTexture(c); t.encoding = THREE.sRGBEncoding; t.anisotropy = 4; return t;
  }
  function pcbTex(w, d, base, trace, label) {
    return tex(Math.max(32, Math.round(w * 8)), Math.max(32, Math.round(d * 8)), (x, W, H) => {
      x.fillStyle = base; x.fillRect(0, 0, W, H);
      let seed = Math.round(w * 31 + d * 7); const r = () => (seed = (seed * 9301 + 49297) % 233280) / 233280;
      x.strokeStyle = trace; x.lineWidth = 1.4; x.globalAlpha = 0.6;
      for (let i = 0; i < 46; i++) { let px = r() * W, py = r() * H; x.beginPath(); x.moveTo(px, py); for (let k = 0; k < 3; k++) { if (r() < 0.5) px = r() * W; else py = r() * H; x.lineTo(px, py); } x.stroke(); }
      x.globalAlpha = 1; x.fillStyle = '#D9B65E';
      for (let i = 0; i < 40; i++) { x.beginPath(); x.arc(r() * W, r() * H, 1.6, 0, Math.PI * 2); x.fill(); }
      x.strokeStyle = 'rgba(255,255,255,0.75)'; x.lineWidth = 1; x.strokeRect(4, 4, W - 8, H - 8);
      if (label) { x.fillStyle = 'rgba(255,255,255,0.85)'; x.font = '600 14px monospace'; x.fillText(label, 8, H - 9); }
    });
  }
  function board(w, d, base, trace, label) {
    const g = new THREE.Group(), side = M(base, { roughness: 0.6 });
    const top = new THREE.MeshStandardMaterial({ map: pcbTex(w, d, base, trace, label), roughness: 0.45 });
    add(g, Box(w, 1.6, d), [side, side, top, side, side, side], 0, 0.8, 0);
    return g;
  }
  const pinGeo = Box(0.64, 8.5, 0.64);
  function header(g, n, x, z, alongX, down, rows) {
    rows = rows || 1;
    const len = n * 2.54, wid = rows * 2.54, y = down ? -1.25 : 1.6 + 1.25;
    add(g, alongX ? Box(len, 2.5, wid) : Box(wid, 2.5, len), mat.black, x, y, z);
    const inst = new THREE.InstancedMesh(pinGeo, mat.gold, n * rows), m4 = new THREE.Matrix4(); let k = 0;
    for (let i = 0; i < n; i++) for (let j = 0; j < rows; j++) {
      const a = (i - (n - 1) / 2) * 2.54, b = (j - (rows - 1) / 2) * 2.54;
      m4.makeTranslation(x + (alongX ? a : b), down ? -2.6 : 1.6 + 2.6, z + (alongX ? b : a)); inst.setMatrixAt(k++, m4);
    }
    g.add(inst);
  }
  const chip = (g, w, d, h, x, z, m) => add(g, Box(w, h, d), m || mat.black, x, 1.6 + h / 2, z);
  function usbC(g, x, y, z, rotY) {
    const u = add(g, rounded(8.9, 3.2, 7.4, 0.3, 1.2), mat.steel, x, y, z); u.rotation.y = rotY || 0;
    add(u, Box(6.8, 1.2, 0.4), mat.hole, 0, 0, 3.6); return u;
  }
  const servoLabel = new THREE.MeshStandardMaterial({ roughness: 0.4, map: tex(256, 160, (x) => {
    x.fillStyle = '#18181C'; x.fillRect(0, 0, 256, 160); x.fillStyle = '#E8E4DA'; x.textAlign = 'center';
    x.font = '700 52px monospace'; x.fillText('3.7g', 128, 70); x.font = '500 22px monospace'; x.fillText('MICRO SERVO', 128, 112);
  }) });

  // 3.7 g micro servo: origin on the shaft axis at the case top face; shaft +y; case runs toward -x (cable end)
  function servo() {
    const g = new THREE.Group();
    const Lup = C.SERVO_L - C.SHAFT_FROM_END, Ldn = C.SHAFT_FROM_END, H = C.SERVO_H, Tz = C.SERVO_T;
    add(g, Box(C.SERVO_L, H, Tz), mat.servo, (Ldn - Lup) / 2, -H / 2, 0);
    add(g, Box(C.SERVO_L + 2 * C.TAB_OVER, 1.5, Tz), mat.servo, (Ldn - Lup) / 2, -C.TAB_DEPTH - 0.75, 0);
    [-(Lup + 1.9), Ldn + 1.9].forEach(x => add(g, Cyl(0.6, 0.6, 1.7, 10), mat.hole, x, -C.TAB_DEPTH - 0.75, 0));
    add(g, Cyl(C.TOWER_R, C.TOWER_R, C.TOWER_H, 24), mat.servo, 0, C.TOWER_H / 2, 0);
    add(g, Cyl(1.9, 1.9, 2.5, 16), mat.steel, 0, C.TOWER_H + 1.25, 0);
    add(g, new THREE.PlaneGeometry(11, 6.9), servoLabel, (Ldn - Lup) / 2, -10, Tz / 2 + 0.01);
    add(g, new THREE.PlaneGeometry(11, 6.9), servoLabel, (Ldn - Lup) / 2, -10, -Tz / 2 - 0.01).rotation.y = Math.PI;
    ['#5A3A22', '#C8312B', '#E8892E'].forEach((c, i) => wire(g, [[-Lup, -H + 1.5, (i - 1) * 0.7], [-Lup - 2.5, -H + 1.5, (i - 1) * 0.7], [-Lup - 4, -H + 3, (i - 1) * 0.8]], c, 0.3));
    return g;
  }
  // micro single-arm horn: origin at the hub base, hub axis +y, arm along +x
  function horn(len) {
    const g = new THREE.Group();
    add(g, Cyl(2.6, 2.6, 2.6, 24), mat.horn, 0, 1.3, 0);
    add(g, rounded(len, 1.3, 3.4, 0.2, 1.1), mat.horn, len / 2, 1.95, 0);
    [4.5, 7, 9.5].forEach(x => { if (x < len - 1) add(g, Cyl(0.4, 0.4, 1.4, 10), mat.hole, x, 1.95, 0); });
    add(g, Cyl(1.0, 1.0, 0.5, 16), mat.steel, 0, 2.85, 0);
    return g;
  }
  function battery() {   // 602035 standing on its long edge: 35 long (x), 20 tall (y), 6 thick (z)
    const g = new THREE.Group();
    const label = new THREE.MeshStandardMaterial({ roughness: 0.3, map: tex(256, 150, (x) => {
      x.fillStyle = '#2B5FA8'; x.fillRect(0, 0, 256, 150); x.fillStyle = '#F2F4F8'; x.textAlign = 'center';
      x.font = '700 34px monospace'; x.fillText('3.7V 400mAh', 128, 66); x.font = '500 24px monospace'; x.fillText('602035  1S LiPo', 128, 106);
    }) });
    const wrap = M('#C9CCD3', { metalness: 0.6, roughness: 0.35 });
    add(g, Box(34, 20, 6), [wrap, wrap, wrap, wrap, label, wrap]);
    add(g, Box(2, 18, 5), M('#E2C25A', { roughness: 0.6 }), 18, 0, 0);   // protection board, taped
    return g;
  }
  function pca() {
    const g = board(62, 25, '#1F5FA8', '#6FA2E0', 'PCA9685 16CH');
    chip(g, 9.7, 4.4, 1.2, 8, -4);
    header(g, 16, -6, 7.3, true, false, 3);
    add(g, Box(10, 9, 7.6), mat.term, 24, 1.6 + 4.5, -6);
    add(g, Cyl(4, 4, 9, 24), M('#1A1A1E', { roughness: 0.3 }), 15, 6.1, -7);
    add(g, Cyl(4.05, 4.05, 0.6, 24), mat.steel, 15, 10.9, -7);
    header(g, 6, -29.5, 0, false, false); header(g, 6, 29.5, 0, false, false);
    // the 8 servo plugs, with their cables rising toward the battery tray
    for (let i = 0; i < 8; i++) {
      const x = -6 + (i - 7.5) * 2.54;
      add(g, Box(2.4, 14, 7.6), mat.plug, x, 1.6 + 2.5 + 7, 7.3);
      ['#5A3A22', '#C8312B', '#E8892E'].forEach((c, j) => wire(g, [[x, 18, 7.3 + (j - 1) * 1.1], [x, 22, 7.3 + (j - 1) * 1.1], [x + (i < 4 ? -4 : 4), 24, 7.3 + (j - 1) * 1.1]], c, 0.45));
    }
    return g;
  }
  function display() {   // 1.3" ST7789 240x240 module, portrait, 7-pin header along the top edge (local -z = up)
    const W = C.DISP_W, H = C.DISP_H;
    const g = board(W, H, '#1A2E5A', '#4A6EB0', '');
    add(g, Box(26, C.DISP_T, 33), mat.glass, 0, 1.6 + C.DISP_T / 2, 3);
    header(g, 7, 0, -H / 2 + 2.3, true, true);
    return g;
  }
  function mini560() {
    const g = board(22, 17, '#1F3F73', '#5A82C2', '5V');
    add(g, rounded(8, 4.5, 8, 0.5, 1.2), M('#55585E', { roughness: 0.6 }), 2, 1.6 + 2.25, 0);
    chip(g, 3, 3, 1, -6, 3);
    [[-6, -4], [8, -5], [8, 5]].forEach(p => chip(g, 3.2, 1.6, 1.4, p[0], p[1], M('#B88A5A')));
    return g;
  }
  function charger() {   // 3.7 V -> 5 V DC-DC boost module (plain, no USB, no auto-off), about 22 x 12 mm
    const g = board(22, 12, '#1E3F73', '#5A82C2', '5V BOOST');
    add(g, rounded(5.5, 3, 5.5, 0.4, 1), M('#3A3A40'), -2, 3.1, 0);                         // inductor
    chip(g, 3, 1.6, 1.0, 4, -2.5);                                                          // boost IC
    [[7, 3], [-8, 3], [-8, -3]].forEach(p => chip(g, 3.2, 1.6, 1.2, p[0], p[1], M('#B88A5A')));
    return g;
  }
  function nano() {   // Waveshare ESP32-S3-Zero, 23.5 x 18 mm
    const g = board(23.5, 18, '#1B1B20', '#3A3A44', 'S3-ZERO');
    chip(g, 7, 7, 0.9, 3, 0, M('#232326', { roughness: 0.4 }));
    chip(g, 2.6, 2.6, 0.9, -4, 4.5, M('#F4F4F4'));                                       // WS2812 LED
    chip(g, 3, 2, 0.8, -4, -4.5, M('#2A2A2E'));                                            // BOOT/RESET
    usbC(g, -9.4, 1.6 + 1.6, 0, -Math.PI / 2);
    [-7.6, 7.6].forEach(z => { for (let i = 0; i < 9; i++) add(g, Cyl(0.7, 0.7, 1.7, 10), mat.gold, -10.15 + 2.54 * i, 0.8, z); });
    return g;
  }
  function amp() {
    const g = board(18, 18, '#3E2A6E', '#7A62B8', 'MAX98357A');
    chip(g, 3, 3, 0.8, 0, -2);
    chip(g, 2.4, 1.4, 1.0, -4, 5, M('#B88A5A')); chip(g, 2.4, 1.4, 1.0, 4, 5, M('#B88A5A'));   // no screw terminal: speaker wires soldered
    return g;
  }
  function speaker() {   // 1511 rectangular speaker, 15 x 11 x 3.5, sound side +x
    const g = new THREE.Group();
    add(g, rounded(15, 11, 3.5, 0.3, 2), mat.black).rotation.y = Math.PI / 2;
    add(g, Box(0.2, 8, 12), mat.paper, 1.8, 0, 0);
    return g;
  }
  function mic() {
    const g = new THREE.Group();
    add(g, Cyl(7, 7, 1.2, 32), M('#2B2440', { roughness: 0.5 }), 0, 0.6, 0);
    add(g, Box(3.5, 1.0, 2.7), mat.steel, 0, 1.7, 0);
    return g;
  }
  function slideSwitch() {   // E-Switch 100SP1T1B4M2QE: smooth bushing + lever outside, body + 3 PCB pins inside (local z+ = into the robot, z 0 = inner wall)
    const g = new THREE.Group();
    const r = (o) => { o.rotation.x = Math.PI / 2; return o; };
    r(add(g, Cyl(3, 3, 6.5, 24), mat.steel, 0, 0, -3.25));                                  // threaded bushing through the wall
    const lever = r(add(g, Cyl(1.1, 1.6, 9, 16), mat.steel, 0, 0, -10.5)); lever.rotation.x = Math.PI / 2 + 0.25;   // lever, flipped up = on
    add(g, Box(8, 13, 10.5), M('#1A1A1E', { roughness: 0.6 }), 0, 0, 5.25);                 // body
    [-4.7, 0, 4.7].forEach(y => add(g, Box(2.6, 0.6, 5), mat.gold, 0, y, 13));              // solder lugs
    return g;
  }

  function xiao() {   // Seeed XIAO ESP32S3 Sense stack, built facing +y (camera up); USB-C at local -x
    const g = new THREE.Group();
    const main = board(C.XIAO_W, C.XIAO_H, '#1E3A5F', '#4A78B0', 'XIAO S3');
    main.position.y = 0; g.add(main);
    usbC(main, -C.XIAO_W / 2 + 3.2, 1.6 + 1.6, 0, -Math.PI / 2);
    chip(main, 10, 10, 1.6, 2.5, 0, mat.steel);                                               // RF shield can
    [-1, 1].forEach(sz => { for (let i = 0; i < 7; i++) add(main, Cyl(0.7, 0.7, 1.7, 10), mat.gold, -7.62 + 2.54 * i, 0.8, sz * 7.6); });
    const exp = board(C.XIAO_W, C.XIAO_H, '#20242A', '#4A4F58', 'SENSE');
    exp.position.y = 4.5; g.add(exp);
    add(exp, Box(4, 1.5, 3), mat.black, -6, 1.6 + 0.75, -5);                                  // on-board mic
    add(exp, Box(11, 1.6, 12), M('#C9CCD3', { metalness: 0.7, roughness: 0.35 }), 3.5, 1.6 + 0.8, 0);   // SD slot
    const cam = new THREE.Group(); cam.position.set(0, 4.5 + 1.6 + 1.6, 0); g.add(cam);
    add(cam, Box(8.5, 3.2, 8.5), M('#222226', { roughness: 0.5 }), 0, 1.6, 0);                // camera module
    add(cam, Cyl(3.1, 3.1, 3.3, 24), mat.black, 0, 3.2 + 1.65, 0);                           // lens barrel
    add(cam, Cyl(1.6, 1.6, 0.2, 20), mat.glass, 0, 3.2 + 3.35, 0);
    add(cam, Box(5, 0.3, 6), M('#C98A2E', { roughness: 0.4 }), -5.5, 0.3, 0);                 // flex
    return g;
  }
  function tof() {   // VL53L0X module, sensor facing +y, header along local +z (bottom edge)
    const g = board(C.TOF_W, C.TOF_H, '#5B2C82', '#9A6CC8', 'VL53L0X');
    add(g, Box(4.4, 1.0, 2.4), mat.black, 0, 1.6 + 0.5, -2);
    [-1.2, 1.2].forEach(x => add(g, Box(1.0, 0.2, 1.0), mat.glass, x, 2.15, -2));
    for (let i = 0; i < 6; i++) add(g, Cyl(0.6, 0.6, 1.7, 10), mat.gold, (i - 2.5) * 2.0, 0.8, C.TOF_H / 2 - 1.5);
    return g;
  }

  // ---------- placement ----------
  const root = new THREE.Group();
  const info = {};
  function place(obj, name, title, note, matrix) {
    if (matrix) { obj.matrixAutoUpdate = false; obj.matrix.copy(matrix); }
    obj.traverse(o => { if (o.isMesh || o.isInstancedMesh) { o.castShadow = true; o.receiveShadow = true; o.userData.part = 'c:' + name; } });
    info['c:' + name] = [title, note];
    root.add(obj); return obj;
  }
  const m4 = () => new THREE.Matrix4();
  const T = (x, y, z) => m4().makeTranslation(x, y, z);
  const Rx = d => m4().makeRotationX(d * Math.PI / 180), Ry = d => m4().makeRotationY(d * Math.PI / 180), Rz = d => m4().makeRotationZ(d * Math.PI / 180);
  const S = (x, y, z) => m4().makeScale(x, y, z);
  const mul = (...ms) => ms.reduce((a, b) => a.multiply(b), m4());

  place(battery(), 'battery', '1S LiPo 602035, 400 mAh', 'Stands on its long edge in the fences on the bottom plate, strapped at the left end. Unscrew the plate and unplug it to take it out.',
    T(C.BATT_X, -C.HY + 0.3 + 10, 0));
  place(speaker(), 'speaker', '1511 speaker, 8 Ω 1 W', 'Glued between the two ledges on the right wall, behind the grille.', T(C.IX - 1.8, C.SPK[0], C.SPK[1]));
  const zf = C.IX - 0.5;
  place(xiao(), 'xiao', 'Seeed XIAO ESP32S3 Sense (left eye)', 'The brain, the camera and the battery charger: its lens looks out of the left eye. The battery is soldered to its BAT pads, and its USB-C (out the left side) both programs Cubi and charges it. Its on-board mic hears through the shell wall and the open seams.',
    mul(T(-C.EYE_X, C.EYE_Y, zf - C.XIAO_T), Rx(90)));
  place(tof(), 'tof', 'VL53L0X distance sensor (right eye)', 'Glued onto the tilted pad behind the right eye, looking 30° down through it. It watches the desk ~10 cm ahead: a sudden long reading = desk edge, a short one = wall. Physics-tested.',
    mul(T(...C.TOF_SENSOR), Rx(C.TOF_TILT), T(0, 0, -1.6), Rx(90)));
  place(amp(), 'amp', 'MAX98357A amplifier', 'Foam-taped to the right half of the battery’s front face, chips facing forward. Leave the screw terminal off and solder the speaker wires.', mul(T(9.5, -C.HY + 25.25, 3.5), Rx(90)));
  place(slideSwitch(), 'switch', 'E-Switch 100SP1T1B4M2QE power toggle', 'Mini toggle in the battery + line: up = on, down = battery fully disconnected. Must be ON to charge. Smooth bushing (no nut): pushed through the snug back-wall hole from inside, body seated in a shallow pocket, glued.', T(C.SWITCH[0], C.SWITCH[1], -C.IX));

  // hip servos: top face on the inner wall, shaft out through the side, case running up
  D.legs.forEach(L => {
    const sx = L.sx, sz = L.sz;
    const side = (sx > 0 ? 'R' : 'L') + (sz > 0 ? 'F' : 'B');
    place(servo(), 'hip' + side, 'GH-S37D 3.7 g micro servo (hip)', 'GH-S37D, one of the 4. Its tabs sit on two solid pads on the side wall and the screws self-tap into them; shaft through the side wall.',
      mul(T(sx * C.IX, C.HIP_Y, sz * C.HIP_Z), S(sx, 1, 1), Rz(-90)));
    const base = m4().fromArray(L.base);
    place(horn(12), 'hipHorn' + side, 'Hip servo horn', 'Glued into the recess on the leg, arm pointing down the leg; the servo screw goes through the centre.',
      mul(base.clone(), T(G.hip_horn_x0, 0, 0), Rz(-90)));
  });

  // eye lenses: dark glass behind each bezel
  [-1, 1].forEach(sx => {
    const [ow, oh] = C.EYE_OPEN || [C.EYE_HOLE, C.EYE_HOLE], ey = C.EYE_Y + (C.EYE_DY || 0);
    const lens = new THREE.Mesh(new THREE.CircleGeometry(0.5, 48), new THREE.MeshStandardMaterial({ color: '#07070B', roughness: 0.08, metalness: 0.3 }));
    if (sx < 0 && C.CAM_LENS_D) { lens.scale.set(C.CAM_LENS_D, C.CAM_LENS_D, 1); lens.position.set(sx * C.EYE_X, C.EYE_Y, C.IX + 0.6); }   // camera lens in its collar
    else { lens.scale.set(ow, oh, 1); lens.position.set(sx * C.EYE_X, ey, C.IX - 0.4); }
    root.add(lens);
    const glint = new THREE.Mesh(new THREE.CircleGeometry(1.1, 20), new THREE.MeshBasicMaterial({ color: '#FFFFFF' }));
    glint.position.set(sx * C.EYE_X - 1.6, ey + 2.6, C.IX - 0.35); root.add(glint);
  });

  return { group: root, info };
};
