const PI = 3.14159265;
const PADDING = 5;

class PolarChart {
  constructor(canvas, x, y, r, num_j, num_samp, colors, cb) {
    this.colors = colors;
    this.canvas = canvas;
    this.ctx = canvas.getContext("2d");
    this.x = x;
    this.y = y;
    this.r = r * 0.96;
    this.tr = r * 0.02;
    this.num_j = num_j;
    this.num_samp = num_samp;
    this.nticks = 30;
    this.data = [...Array(num_j)].map(e => Array(num_samp));
    this.targets = Array(num_j);

    canvas.addEventListener('mousedown', this.on_mousedown.bind(this));
    canvas.addEventListener('mousemove', this.on_mousemove.bind(this));
    canvas.addEventListener('mouseup', this.on_mouseup.bind(this));
    canvas.addEventListener('mouseout', this.on_mouseout.bind(this));
    this.cb = cb;
    this.mousing = false;
  }

  getCursorAngle(evt) {
    const rect = this.canvas.getBoundingClientRect();
    const x = evt.clientX - rect.left;
    const y = evt.clientY - rect.top;
    return Math.atan2(y-this.r, x-this.r);
  }

  on_mousedown(evt) {
    this.mousing = true;
    this.cb(this.getCursorAngle(evt));
  }
  on_mousemove(evt) {
    if (!this.mousing) {
      return;
    }
    this.cb(this.getCursorAngle(evt));
  }
  on_mouseup(evt) {
    this.mousing = false;
    this.cb(this.getCursorAngle(evt));
  }
  on_mouseout(evt) {
    this.mousing = false;
  }

  draw_bg() {
    this.ctx.beginPath();
    this.ctx.arc(this.x, this.y, this.r - PADDING, 0, 2*PI);
    this.ctx.stroke();

    let irad = (2*PI)/this.nticks;
    for (let i = 0; i < this.nticks; i++) {
      this.ctx.beginPath();
      this.ctx.moveTo(this.x+(0.1*this.r*Math.cos(i*irad)), this.y+(0.1*this.r*Math.sin(i*irad)));
      this.ctx.lineTo(this.x+this.r*Math.cos(i*irad), this.y+this.r*Math.sin(i*irad));
      this.ctx.stroke();
    }
  }

  draw_joint(i) {
    let dr = this.r / this.num_samp;
    this.ctx.beginPath();
    let j = 0;
    while (this.data[i][j] === undefined && j < this.num_samp) {
      j++;
    }

    this.ctx.moveTo(this.x + j*dr*Math.cos(this.data[i][j]), this.y + j*dr*Math.sin(this.data[i][j]));

    for (; j < this.num_samp; j++) {
      let v = this.data[i][j];
      this.ctx.lineTo(this.x + j*dr*Math.cos(v), this.y + j*dr*Math.sin(v));
    }
    this.ctx.stroke();

    // Draw target bubble
    let tx = this.x + this.r*Math.cos(this.targets[i]);
    let ty = this.y + this.r*Math.sin(this.targets[i]);
    this.ctx.beginPath();
    this.ctx.arc(tx, ty, this.tr, 0, 2*Math.PI);
    this.ctx.fill();
  }

  add_data(jointvals) {
    for (let i = 0; i < this.num_j; i++) {
      this.data[i].shift();
      this.data[i].push(jointvals[i]);
    }
  }

  set_targets(jointvals) {
    for (let i = 0; i < this.num_j; i++) {
      this.targets[i] = jointvals[i];
    }

  }

  draw() {
    this.ctx.clearRect(0, 0, this.ctx.canvas.width, this.ctx.canvas.height);

    this.ctx.strokeStyle = "#ccc";
    this.ctx.lineWidth = 0.8;
    this.draw_bg();

    this.ctx.linewidth = 1.0;
    for (let i = 0; i < this.num_j; i++) {
      this.ctx.strokeStyle = this.colors[i] || "#000";
      this.ctx.fillStyle = this.ctx.strokeStyle;
      this.draw_joint(i);
    }
  }
}
