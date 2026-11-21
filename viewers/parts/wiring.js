/* CUBI (camera eyes) wiring: every wire between the real pin positions (robot coords, mm; x right, y up, z forward).
   Board: Seeed XIAO ESP32S3 Sense. Pins are named by GPIO (GPn); XPIN maps them to the XIAO's D-labels. */
window.CUBI_NETS = [
  { id: 'batt', name: 'Battery (3.7 V, switched)', color: '#B3261E', r: 0.5 },
  { id: 'gnd', name: 'Ground', color: '#1C1B20', r: 0.38 },
  { id: 'v5', name: 'Servo + amp power (switched battery)', color: '#F2545B', r: 0.4 },
  { id: 'v33', name: '3.3 V', color: '#F28CC6', r: 0.3 },
  { id: 'servo', name: 'Servo signals', color: '#F2D13A', r: 0.3 },
  { id: 'spi', name: 'Distance sensor (I²C)', color: '#4F8EF7', r: 0.28 },
  { id: 'i2s', name: 'Audio bus (I²S)', color: '#4CC38A', r: 0.28 },
  { id: 'spk', name: 'Speaker', color: '#9AA0A6', r: 0.32 },
  { id: 'ctrl', name: 'Touch pad + battery sense', color: '#7FE3F0', r: 0.28 }
];

window.buildCubiWiring = function (THREE, D) {
  const C = D.C;
  const V = (x, y, z) => new THREE.Vector3(x, y, z);
  const TOP = C.TOP, IX = C.IX;
  const PIN_Y = TOP - 1.6 - 0.3;
  const BUS = 9.0;                 // free layer between the servo tops and the ceiling boards: long runs go here
  const P = {};

  // XIAO ESP32S3 Sense behind the left eye: board parallel to the front wall, pins along its top and
  // bottom edges, wires soldered on the back face. USB-C at the left end (left-wall hole).
  const EX = -C.EYE_X, EY = C.EYE_Y, EZ = C.IX - 0.5 - C.XIAO_T - 0.3;
  const top = [['D0', 1], ['D1', 2], ['D2', 3], ['D3', 4], ['D4', 5], ['D5', 6], ['D6', 43]];
  const bot = [['5V'], ['GND'], ['3V3'], ['D10', 9], ['D9', 8], ['D8', 7], ['D7', 44]];
  const GPIO = {}, XPIN = {};
  const nameOf = (d, g) => (g === undefined ? d : 'GP' + g);
  top.forEach(([d, g], i) => { const n = nameOf(d, g); P['esp.' + n] = V(EX - 7.62 + 2.54 * i, EY + 7.6, EZ); XPIN[n] = d; if (g !== undefined) GPIO[n] = g; });
  bot.forEach(([d, g], i) => { const n = nameOf(d, g); P['esp.' + n] = V(EX - 7.62 + 2.54 * i, EY - 7.6, EZ); XPIN[n] = d; if (g !== undefined) GPIO[n] = g; });

  // XIAO BAT pads on the back of the main board, near the USB end. The XIAO charges the battery through them.
  P['esp.BAT+'] = V(EX - 7.5, EY + 2.5, EZ); P['esp.BAT-'] = V(EX - 7.5, EY - 2.5, EZ);
  // battery-voltage divider (2 x 100k) into D2 (GPIO3, ADC)
  P['rdiv.top'] = V(-4, 2.5, -8); P['rdiv.mid'] = V(-4, 2.5, -4); P['rdiv.bot'] = V(-4, 2.5, 0);
  // MAX98357A on the battery's front face (right half), chips forward, pads along its top edge
  const AY = -C.HY + 33.5;
  ['LRC', 'BCLK', 'DIN', 'GAIN', 'SD', 'GND', 'VIN'].forEach((n, i) => { P['amp.' + n] = V(9.5 + (i - 3) * 2.54, AY, 6.6); });
  P['amp.out+'] = V(8, -C.HY + 17, 6.6); P['amp.out-'] = V(11, -C.HY + 17, 6.6);
  // VL53L0X distance sensor behind the right eye: 6-pin header along its bottom edge
  const tofM = new THREE.Matrix4().makeTranslation(...C.TOF_SENSOR).multiply(new THREE.Matrix4().makeRotationX(C.TOF_TILT * Math.PI / 180));
  ['VIN', 'GND', 'SCL', 'SDA', 'GPIO1', 'XSHUT'].forEach((n, i) => { P['tof.' + n] = V((i - 2.5) * 2.54, -C.TOF_H / 2 + 1.5, -2.2).applyMatrix4(tofM); });
  const TY = P['tof.VIN'].y;
  // 1511 speaker on the right wall
  const [SY, SZ] = C.SPK;
  P['spk.+'] = V(IX - 3.8, SY + 1.5, SZ - 7.2); P['spk.-'] = V(IX - 3.8, SY - 1.5, SZ - 7.2);
  // E-Switch 100-series toggle in the back wall (centre pin = common)
  const [bx, by] = C.SWITCH;
  P['sw.1'] = V(bx, by, -IX + 15.5); P['sw.2'] = V(bx, by + 4.7, -IX + 15.5);
  // battery standing on edge; lead at its right end, plug just past it
  const bxe = C.BATT_X + C.BATT_L / 2 - 0.5;
  P['bat.+'] = V(bxe, -C.HY + 8, 1); P['bat.-'] = V(bxe, -C.HY + 8, -1);
  P['plug.+'] = V(bxe + 3.5, -C.HY + 14, 1); P['plug.-'] = V(bxe + 3.5, -C.HY + 14, -1);
  // servo cables leave the top end of each case
  const servoTag = { LF: [-1, 1], RF: [1, 1], LB: [-1, -1], RB: [1, -1] };
  Object.entries(servoTag).forEach(([k, [sx, sz]]) => { P['srv' + k] = V(sx * (IX - C.SERVO_H + 2), C.HIP_Y + 15 + 2.5, sz * C.HIP_Z); });
  P['splice+'] = V(9, 2.5, -2.5); P['splice-'] = V(9, -0.5, -2.5);
  P['cap+'] = V(-3.5, 3.6, -3); P['cap-'] = V(-15.5, 3.6, -3);
  P['cap2+'] = V(13.5, C.TOP - 4.3, -4.5); P['cap2-'] = V(13.5, C.TOP - 4.3, -17);
  P['touch'] = V(0, C.CEIL - 0.3, 7);

  // ---------- wires: [net, from, to, label, via points, nudge] ----------
  const W = [];
  const w = (net, a, b, label, via, nudge) => W.push({ net, a, b, label, via: via || null, nudge: nudge || 0 });
  // default route: up/down to the bus layer, across, then to the pin
  const route = (a, b, y) => [V(a.x, y, a.z), V(b.x, y, b.z)];

  // battery -> plug -> power switch -> XIAO BAT (it charges the cell) and the servo/amp splice
  w('batt', 'bat.+', 'plug.+', 'Battery + to plug', []);
  w('gnd', 'bat.-', 'plug.-', 'Battery − to plug', []);
  w('batt', 'plug.+', 'sw.1', 'Battery + to the power switch (middle lug), doubled 26 AWG', [V(20, 0, 1), V(8, -2, -4), V(bx + 1, by - 2, -IX + 19)]);
  w('gnd', 'plug.-', 'splice-', 'Battery − to the ground splice, doubled 26 AWG', [V(19, -3, -1), V(12, -1.5, -2.5)]);
  w('v5', 'sw.2', 'splice+', 'Switched battery + to the power splice (servos + amp), doubled 26 AWG', [V(bx, by + 7, -IX + 18), V(6, BUS + 1, -8), V(6, 2.5, -3)]);
  w('batt', 'sw.2', 'esp.BAT+', 'Switched battery + to the XIAO BAT+ pad (the XIAO charges the battery through this)', [V(bx - 1, by + 7, -IX + 17), V(-6, BUS, -2), V(P['esp.BAT+'].x, P['esp.BAT+'].y, P['esp.BAT+'].z - 3)]);
  w('batt', 'sw.2', 'cap2+', '1000 µF bulk capacitor + on the switched battery line', [V(bx + 2, by + 7, -IX + 17), V(13.5, BUS + 3, -15)]);
  w('gnd', 'cap2-', 'splice-', '1000 µF capacitor − (stripe) to the ground splice', [V(13.5, BUS + 2, -18), V(10, 1, -6)]);
  w('v5', 'cap+', 'splice+', '470 µF capacitor + to the power splice', [V(2, 4.5, -3)]);
  w('gnd', 'cap-', 'splice-', '470 µF capacitor − (stripe) to the ground splice', [V(-15.5, 1, -1), V(2, 1, -2)]);
  w('gnd', 'splice-', 'esp.BAT-', 'Ground splice to the XIAO BAT− pad', [V(2, 0, 0), V(P['esp.BAT-'].x + 1, P['esp.BAT-'].y, P['esp.BAT-'].z - 3)]);
  w('v5', 'splice+', 'amp.VIN', 'Power splice to amp VIN (battery voltage)', [V(12, 3, 1), V(P['amp.VIN'].x, AY + 1.5, 6)]);
  w('gnd', 'splice-', 'amp.GND', 'Ground splice to amp GND', [V(12, 0, 1.5), V(P['amp.GND'].x, AY + 2, 5.5)]);
  // battery voltage sense: switched battery -> 100k -> D2 -> 100k -> ground (reads half the battery voltage)
  w('batt', 'sw.2', 'rdiv.top', '100 kΩ resistor, top of the battery-voltage divider', [V(bx - 2, by + 6, -IX + 17)]);
  w('ctrl', 'rdiv.mid', 'esp.GP3', 'Divider middle → XIAO D2 (GPIO3, ADC): battery voltage ÷ 2', [V(-4, 4, -2), V(P['esp.GP3'].x, P['esp.GP3'].y + 1.5, 4.5)]);
  w('gnd', 'rdiv.bot', 'splice-', '100 kΩ bottom of the divider to ground', [V(2, 1, -1)]);

  // servos: red/brown to the splices, orange signal to the ESP32
  const servoPin = { LF: 'GP2', RF: 'GP4', LB: 'GP7', RB: 'GP8' };
  const servoName = { LF: 'front-left', RF: 'front-right', LB: 'rear-left', RB: 'rear-right' };
  Object.entries(servoTag).forEach(([k, [sx, sz]], idx) => {
    const s = 'srv' + k, n0 = (idx - 1.5) * 0.7;
    const mid = [V(sx * 6, 6.5, sz * 6)];
    w('v5', s, 'splice+', `Red: ${servoName[k]} servo + to the power splice (battery, 3.5-4.2 V)`, mid.concat([V(6, 5.5, -2)]), n0);
    w('gnd', s, 'splice-', `Brown: ${servoName[k]} servo − to the ground splice`, mid.concat([V(6, 2.5, -2)]), n0 + 0.3);
    const pin = servoPin[k];
    w('servo', s, 'esp.' + pin, `Orange: ${servoName[k]} servo signal to XIAO ${XPIN[pin]} (${pin})`, mid.concat([V(P['esp.' + pin].x, 4.6 + idx * 0.3, 2 + idx * 0.4)]), n0 + 0.6);
  });

  // VL53L0X: power + I2C (GPIO1 and XSHUT left open)
  [['VIN', 'esp.3V3', 'v33', '3.3 V'], ['GND', 'esp.GND', 'gnd', 'ground'], ['SCL', 'esp.GP6', 'spi', 'I²C clock'], ['SDA', 'esp.GP5', 'spi', 'I²C data']]
    .forEach(([n, from, net, what], k) => {
      w(net, from, 'tof.' + n, `Distance sensor ${n} (${what}) ← XIAO ${XPIN[from.slice(4)]}`, [V(P[from].x, P[from].y + (from === 'esp.3V3' || from === 'esp.GND' ? -1.2 : 1.2), 5.5), V(0, 15 + k * 0.4, 8 + k * 0.4), V(P['tof.' + n].x, TY - 1.5, P['tof.' + n].z - 2)]);
    });
  // shared audio: the XIAO has its own mic on the camera board, so the I²S pins only go to the amp
  [['BCLK', 'esp.GP43', 'bit clock'], ['LRC', 'esp.GP44', 'word select'], ['DIN', 'esp.GP9', 'audio data into the amp']]
    .forEach(([n, from, what], k) => {
      w('i2s', from, 'amp.' + n, `Amp ${n} (${what}) ← XIAO ${XPIN[from.slice(4)]}`, [V(P[from].x, P[from].y + (from === 'esp.GP9' ? -1.2 : 1.2), 5.2), V(3, 9 + k * 0.4, 4.5), V(P['amp.' + n].x, AY + 1.5, 7.5)]);
    });
  w('spk', 'amp.out+', 'spk.+', 'Amp + to speaker +', [V(-1.5, -6, 8), V(10, -2, 5), V(17, SY, SZ - 8)]);
  w('spk', 'amp.out-', 'spk.-', 'Amp − to speaker −', [V(1.5, -6.5, 8), V(11, -2.5, 4.5), V(17, SY - 2, SZ - 8)]);

  // touch pad
  w('ctrl', 'touch', 'esp.GP1', 'Head-tap copper pad → XIAO D0 (GPIO1, touch)', [V(-4, C.CEIL - 1, 4), V(P['esp.GP1'].x, P['esp.GP1'].y + 1.5, 5)]);

  // ---------- build meshes ----------
  const nets = {}; window.CUBI_NETS.forEach(n => { nets[n.id] = n; });
  const group = new THREE.Group(), info = {}, byNet = {}, mats = {};
  const matFor = id => mats[id] || (mats[id] = new THREE.MeshStandardMaterial({ color: nets[id].color, roughness: 0.5, emissive: nets[id].color, emissiveIntensity: id === 'gnd' ? 0 : 0.12 }));
  W.forEach((wr, i) => {
    const a = P[wr.a], b = P[wr.b];
    if (!a || !b) { console.warn('missing pin', wr.a, wr.b); return; }
    const via = wr.via === null ? route(a, b, BUS) : wr.via;
    const pts = [a.clone()].concat(via.map(v => v.clone().add(V(0, 0, wr.nudge)))).concat([b.clone()]);
    const curve = new THREE.CatmullRomCurve3(pts, false, 'centripetal');
    const m = new THREE.Mesh(new THREE.TubeGeometry(curve, Math.max(24, pts.length * 14), nets[wr.net].r, 8, false), matFor(wr.net));
    m.userData.part = 'w:' + i; m.userData.net = wr.net;
    info['w:' + i] = [wr.label, nets[wr.net].name];
    group.add(m);
    (byNet[wr.net] = byNet[wr.net] || []).push(m);
  });
  const dot = new THREE.SphereGeometry(0.45, 10, 8), dotMat = new THREE.MeshStandardMaterial({ color: '#C9CCD3', metalness: 0.9, roughness: 0.3 });
  const seen = new Set();
  W.forEach(wr => [wr.a, wr.b].forEach(k => { if (!seen.has(k) && P[k]) { seen.add(k); const s = new THREE.Mesh(dot, dotMat); s.position.copy(P[k]); group.add(s); } }));
  const shrink = new THREE.MeshStandardMaterial({ color: '#2A2A30', roughness: 0.8 });
  [['splice+', 'Power splice (heat-shrink)', 'Switched battery +, all 4 servo reds, the 470 µF capacitor and the amp VIN joined here. 3.5-4.2 V, inside the servos\' 3.6-4.8 V rating.'],
   ['splice-', 'Ground splice (heat-shrink)', 'Battery −, all 4 servo browns, both capacitors, amp, divider and XIAO BAT− joined here.']]
    .forEach(([k, t, n]) => {
      const s = new THREE.Mesh(new THREE.CylinderGeometry(1.1, 1.1, 5, 14), shrink);
      s.position.copy(P[k]); s.rotation.z = Math.PI / 2; s.userData.part = 'w:' + k; info['w:' + k] = [t, n]; group.add(s);
    });
  [['rdiv', 'Battery-voltage divider (2 × 100 kΩ)', 'Two 100 kΩ resistors in series from the switched battery to ground; the middle goes to D2 so the firmware can read the battery level and warn before it runs flat.']].forEach(([k, t, n]) => {
    const r = new THREE.Mesh(new THREE.CylinderGeometry(0.9, 0.9, 7, 12), new THREE.MeshStandardMaterial({ color: '#C9A06A', roughness: 0.5 }));
    r.position.copy(P['rdiv.mid']); r.rotation.x = Math.PI / 2; r.userData.part = 'w:' + k; info['w:' + k] = [t, n]; group.add(r);
  });
  const capMat = new THREE.MeshStandardMaterial({ color: '#2B3A8C', roughness: 0.45 });
  const cap = new THREE.Mesh(new THREE.CylinderGeometry(3.15, 3.15, 11, 20), capMat);
  cap.position.set(-9.5, 3.6, -3); cap.rotation.z = Math.PI / 2; cap.userData.part = 'w:cap';
  info['w:cap'] = ['470 µF 10 V capacitor', 'Across the servo power at the splices. Stops the supply dipping when the servos start. Stripe (−) to ground.']; group.add(cap);
  const cap2 = new THREE.Mesh(new THREE.CylinderGeometry(4, 4, 10.5, 20), capMat);
  cap2.position.set(13.5, C.TOP - 4.3, -10.75); cap2.rotation.x = Math.PI / 2; cap2.userData.part = 'w:cap2';
  info['w:cap2'] = ['1000 µF 6.3 V capacitor (battery bulk)', 'Across the switched battery line (under the top, back-right). Smooths the battery when the servos pull hard, so the XIAO never browns out.']; group.add(cap2);
  const plug = new THREE.Mesh(new THREE.BoxGeometry(3, 5, 6), new THREE.MeshStandardMaterial({ color: '#E8E4DA', roughness: 0.5 }));
  plug.position.copy(P['plug.+']).lerp(P['plug.-'], 0.5); plug.userData.part = 'w:plug';
  info['w:plug'] = ['Battery plug (PH2.0)', 'Unplug here to take the bottom plate (and battery) out.']; group.add(plug);
  const pad = new THREE.Mesh(new THREE.BoxGeometry(12, 0.2, 8), new THREE.MeshStandardMaterial({ color: '#C77B3A', metalness: 0.9, roughness: 0.25 }));
  pad.position.set(0, C.CEIL - 0.1, 7); pad.userData.part = 'w:touch';
  info['w:touch'] = ['Head-tap pad (copper tape)', 'Stuck under the top. Tap Cubi’s head to talk; the ESP32 senses it through the shell.']; group.add(pad);

  return { group, info, byNet, pins: P, GPIO, XPIN, count: W.length, wires: W };
};
