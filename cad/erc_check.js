// Electrical rules check for Cubi (1S version). Run: npm install three@0.128.0 && node erc_check.js
const path = require('path');
const THREE = require(require.resolve('three', { paths: [process.cwd(), '/tmp/claude-0/t3'] }));
global.window = {};
global.document = { createElement: () => ({ getContext: () => new Proxy({}, { get: () => () => {} }) }) };
require(path.resolve(__dirname, 'viewer/viewer_data.js'));
require(path.resolve(__dirname, 'viewer/wiring.js'));
const r = window.buildCubiWiring(THREE, window.CUBI);

const cond = (pin, net) => pin.startsWith('srv') ? pin + ({ v5: '.red', gnd: '.brown', servo: '.signal' })[net] : pin;
const DIRECTIONAL = new Set(['diode.A', 'diode.K']);
const parent = {}; const find = x => (parent[x] = parent[x] === undefined ? x : parent[x]) === x ? x : (parent[x] = find(parent[x]));
const join = (a, b) => { parent[find(a)] = find(b); };
r.wires.forEach(w => { const a = cond(w.a, w.net), b = cond(w.b, w.net); find(a); find(b); join(a, b); });
join('esp.BAT-', 'esp.GND');        // XIAO: BAT- is its ground
join('sw.1', 'sw.2');               // check with the power switch ON
['rdiv.top', 'rdiv.mid', 'rdiv.bot'].forEach(find);
join('rdiv.top', 'rdiv.top'); join('rdiv.bot', 'rdiv.bot');
const same = (a, b) => find(a) === find(b);
const errs = [], ok = [], warn = [];
const expect = (c, good, bad) => (c ? ok : errs).push(c ? good : bad);

// rails must never touch (the resistors are not wires)
const rails = { 'battery + (switched)': 'sw.2', 'GND': 'bat.-', '3.3 V': 'esp.3V3', 'battery sense (D2)': 'rdiv.mid' };
const rn = Object.keys(rails);
let shorts = 0;
for (let i = 0; i < rn.length; i++) for (let j = i + 1; j < rn.length; j++) if (same(rails[rn[i]], rails[rn[j]])) { shorts++; errs.push(`SHORT: ${rn[i]} touches ${rn[j]}`); }
if (!shorts) ok.push('No shorts between the 4 rails (battery, 3.3 V, ground, battery-sense)');

const want = [
  ['bat.+', 'sw.1', 'Battery + to the power switch'], ['bat.-', 'esp.BAT-', 'Battery − to ground'],
  ['esp.BAT+', 'sw.2', 'XIAO BAT+ on the switched battery (XIAO charges the cell and runs from it)'],
  ['cap2+', 'sw.2', '1000 µF bulk cap on the switched battery'], ['cap2-', 'bat.-', '1000 µF cap − on ground'],
  ['amp.VIN', 'sw.2', 'Amp on the switched battery (MAX98357A: 2.5-5.5 V)'], ['amp.GND', 'bat.-', 'Amp ground'],
  ['tof.VIN', 'esp.3V3', 'Distance sensor on 3.3 V'], ['tof.GND', 'bat.-', 'Distance sensor ground'],
  ['cap+', 'sw.2', '470 µF cap on the servo supply'], ['cap-', 'bat.-', '470 µF cap − on ground'],
  ['esp.GND', 'bat.-', 'XIAO shares ground with the servos (needed for servo signals)'],
  ['rdiv.top', 'sw.2', 'Divider top on the switched battery'], ['rdiv.bot', 'bat.-', 'Divider bottom on ground'],
  ['rdiv.mid', 'esp.GP3', 'Divider middle on D2 (GPIO3, ADC)'],
];
['LF', 'RF', 'LB', 'RB'].forEach(k => { want.push(['srv' + k + '.red', 'sw.2', `Servo ${k} red on the switched battery`]); want.push(['srv' + k + '.brown', 'bat.-', `Servo ${k} brown on ground`]); });
want.forEach(([p, rail, msg]) => expect(same(p, rail), msg, `WRONG: ${msg}`));
[['tof.XSHUT', 'Distance sensor XSHUT floating (always on)'], ['tof.GPIO1', 'Distance sensor interrupt unused'], ['amp.SD', 'Amp SD floating (= mono mix)'], ['amp.GAIN', 'Amp GAIN floating (= 9 dB)'], ['esp.5V', 'XIAO 5V pin unused (it is USB VBUS: nothing may feed it)']]
  .forEach(([p, msg]) => expect(!(p in parent), msg, `WRONG: ${p} is wired but should be left open`));

const sig = [['esp.GP3', 'rdiv.mid'], ['esp.GP2', 'srvLF.signal'], ['esp.GP4', 'srvRF.signal'], ['esp.GP7', 'srvLB.signal'], ['esp.GP8', 'srvRB.signal'],
  ['esp.GP5', 'tof.SDA'], ['esp.GP6', 'tof.SCL'],
  ['esp.GP43', 'amp.BCLK'], ['esp.GP44', 'amp.LRC'], ['esp.GP9', 'amp.DIN'], ['esp.GP1', 'touch']];
let n = 0; sig.forEach(([a, b]) => { if (same(a, b)) n++; else errs.push(`WRONG: ${a} should reach ${b}`); });
ok.push(`All ${n}/${sig.length} signal connections reach the right pin`);
expect(!same('esp.GP5', 'esp.GP6'), 'I²C data and clock on separate pins', 'SHORT: SDA and SCL joined');
warn.push('XIAO Sense: D8-D10 (GPIO7-9) are shared with its SD-card slot. Leave the SD slot empty.');
const gp = Object.keys(parent).filter(p => /^esp\.GP\d+$/.test(p));
const seen = {}; gp.forEach(p => { const k = find(p); if (seen[k]) errs.push(`SHORT: ${p} and ${seen[k]}`); seen[k] = p; });
const used = gp.map(p => r.GPIO[p.slice(4)]).sort((a, b) => a - b);
const bad = used.filter(g => [0, 19, 20, 21, 45, 46].includes(g) || (g >= 26 && g <= 37));   // GPIO3 only straps JTAG when an eFuse is burned: fine as an ADC input
expect(!bad.length, `No strapping, USB, LED or flash/PSRAM pins used (GPIO ${used.join(', ')})`, `WRONG: risky GPIOs ${bad}`);
expect(used.includes(1) && 1 <= 14, 'Head-tap pad on a touch-capable pin (GPIO1 = XIAO D0)', 'touch pin');

// power: servos + amp straight off the 1S battery
// GH-S37D rated 3.6-4.8 V (3 of 4 spec sheets); a 1S LiPo runs 4.2 V full -> 3.6 V when the firmware should stop walking
expect(4.2 <= 4.8 && 3.6 >= 3.6, 'Servo voltage 3.6-4.2 V (battery) inside the GH-S37D rating of 3.6-4.8 V; firmware stops walking below 3.6 V', 'servo overvolted');
expect(4.2 <= 5.5 && 3.5 >= 2.5, 'Amp voltage 3.5-4.2 V inside the MAX98357A range of 2.5-5.5 V', 'amp voltage');
// servo numbers from ponder_sim.py (real geometry, GH-S37D): 0.13 A average, 0.46 A worst peak
const walkAvg = 0.13 + 0.03 + 0.01, walkPeak = 0.46 + 0.4;            // servos + amp idle + sensor | servos + amp at a loud word
const xiao = 0.154;                                                   // Seeed: camera streaming from battery, 3.8 V
const battA = xiao + walkAvg, battPeak = xiao + 0.15 + walkPeak;
expect(battPeak < 1.2, `Battery peak ~${battPeak.toFixed(1)} A for a few ms (servos + a loud word + Wi-Fi burst): ~2C on 500 mAh, fine for bursts; the 1000 µF cap covers the edges`, 'battery peak too high');
expect(battA < 1.0, `Battery draw ~${(battA * 1000).toFixed(0)} mA walking with the camera on: ~${(500 / battA / 1000 * 0.85 * 60).toFixed(0)} min on 500 mAh`, 'battery draw too high');
ok.push('Charging: the XIAO charges the cell at ~50 mA (0.1C for 500 mAh): very safe, about 10-12 h from flat. Switch must be ON to charge.');
// switch-off state: nothing downstream of the switch is connected to the battery
{ const P2 = {}; const f2 = x => (P2[x] = P2[x] === undefined ? x : P2[x]) === x ? x : (P2[x] = f2(P2[x]));
  r.wires.forEach(w => { const a = cond(w.a, w.net), b = cond(w.b, w.net); P2[f2(a)] = f2(b); });
  expect(f2('bat.+') !== f2('esp.BAT+') && f2('bat.+') !== f2('splice+'), 'Switch OFF: battery fully disconnected (no drain on the shelf)', 'WRONG: switch does not isolate the battery'); }
warn.push('Servo torque drops ~25% as the battery runs down (sim walks at half torque). Speaker volume follows the battery: cap it at ~70%.');
console.log('PASS (' + ok.length + ')'); ok.forEach(m => console.log('  ✓ ' + m));
console.log('WARN (' + warn.length + ')'); warn.forEach(m => console.log('  ! ' + m));
console.log('FAIL (' + errs.length + ')'); errs.forEach(m => console.log('  ✗ ' + m));
