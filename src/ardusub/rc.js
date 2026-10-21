// Port of RC_Channel scaling used by ArduSub (integer maths kept as in C++).

const trunc = Math.trunc;
const clampI = (v, lo, hi) => (v < lo ? lo : v > hi ? hi : v);

export class RCChannel {
  constructor({ min = 1100, max = 1900, trim = 1500, dz = 0, reversed = false } = {}) {
    this.min = min; this.max = max; this.trim = trim; this.deadZone = dz; this.reversed = reversed;
    this.type = 'angle'; this.highIn = 4500;
    this.radioIn = trim;
    this.controlIn = 0;
  }
  configure({ min, max, trim, dz, reversed }) {
    if (min !== undefined) this.min = min;
    if (max !== undefined) this.max = max;
    if (trim !== undefined) this.trim = trim;
    if (dz !== undefined) this.deadZone = dz;
    if (reversed !== undefined) this.reversed = !!reversed;
  }
  setAngle(a) { this.type = 'angle'; this.highIn = a; }
  setRange(h) { this.type = 'range'; this.highIn = h; }
  setOverride(pwm) { this.radioIn = trunc(pwm); this.update(); }
  update() {
    this.controlIn = this.type === 'range' ? this.pwmToRangeDz(this.deadZone) : this.pwmToAngleDzTrim(this.deadZone, this.trim);
    return this.controlIn;
  }
  pwmToAngleDzTrim(dz, trim) {
    const hi = trim + dz, lo = trim - dz;
    const rev = this.reversed ? -1 : 1;
    const r = clampI(this.radioIn, this.min, this.max);
    if (r > hi && this.max !== hi) return trunc((rev * trunc(this.highIn * (r - hi))) / (this.max - hi));
    if (r < lo && lo !== this.min) return trunc((rev * trunc(this.highIn * (r - lo))) / (lo - this.min));
    return 0;
  }
  pwmToRangeDz(dz) {
    let r = clampI(this.radioIn, this.min, this.max);
    if (this.reversed) r = this.max - (r - this.min);
    const lo = this.min + dz;
    if (r > lo) return trunc((this.highIn * (r - lo)) / (this.max - lo));
    return 0;
  }
  getControlMid() {
    if (this.type === 'range') {
      const r = trunc((this.min + this.max) / 2);
      const lo = this.min + this.deadZone;
      return trunc((this.highIn * (r - lo)) / (this.max - lo));
    }
    return 0;
  }
  normInput() {
    const rev = this.reversed ? -1 : 1;
    let ret;
    if (this.radioIn < this.trim) {
      if (this.min >= this.trim) return 0;
      ret = (rev * (this.radioIn - this.trim)) / (this.trim - this.min);
    } else {
      if (this.max <= this.trim) return 0;
      ret = (rev * (this.radioIn - this.trim)) / (this.max - this.trim);
    }
    return Math.max(-1, Math.min(1, ret));
  }
}

// ArduSub channel map: 1 pitch, 2 roll, 3 throttle, 4 yaw, 5 forward, 6 lateral (+7 pan, 8 tilt)
export function makeSubChannels(P) {
  const ch = [];
  for (let i = 1; i <= 8; i++) ch.push(new RCChannel({ min: P[`RC${i}_MIN`], max: P[`RC${i}_MAX`], trim: P[`RC${i}_TRIM`], dz: P[`RC${i}_DZ`], reversed: P[`RC${i}_REVERSED`] }));
  const [pitch, roll, throttle, yaw, forward, lateral] = ch;
  roll.setAngle(4500); pitch.setAngle(4500); yaw.setAngle(4500); forward.setAngle(4500); lateral.setAngle(4500);
  throttle.setRange(1000);
  ch.forEach((c) => c.setOverride(1500));
  return { all: ch, pitch, roll, throttle, yaw, forward, lateral };
}

export function configureSubChannels(rc, P) {
  rc.all.forEach((c, i) => c.configure({ min: P[`RC${i + 1}_MIN`], max: P[`RC${i + 1}_MAX`], trim: P[`RC${i + 1}_TRIM`], dz: P[`RC${i + 1}_DZ`], reversed: P[`RC${i + 1}_REVERSED`] }));
  rc.all.forEach((c) => c.update());
}
