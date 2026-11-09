const boidsInput = document.getElementById('boids');
const stepsInput = document.getElementById('steps');
const seedInput = document.getElementById('seed');
const simulateBtn = document.getElementById('simulate-btn');
const resetBtn = document.getElementById('reset-btn');
const compareBtn = document.getElementById('compare-btn');
const traceBtn = document.getElementById('trace-btn');
const simMeta = document.getElementById('sim-meta');
const compareMeta = document.getElementById('compare-meta');
const traceMeta = document.getElementById('trace-meta');
const canvas = document.getElementById('boids-canvas');
const ctx = canvas.getContext('2d');
const fpgaCanvas = document.getElementById('boids-fpga-canvas');
const fpgaCtx = fpgaCanvas.getContext('2d');
const fpgaFlockMeta = document.getElementById('fpga-flock-meta');
const runtimeSummary = document.getElementById('runtime-summary');

const cpuTraceCanvas = document.getElementById('cpu-trace-canvas');
const fpgaTraceCanvas = document.getElementById('fpga-trace-canvas');
const cpuTraceCtx = cpuTraceCanvas.getContext('2d');
const fpgaTraceCtx = fpgaTraceCanvas.getContext('2d');

const benchMode = document.getElementById('bench-mode');
const elementsInput = document.getElementById('elements');
const iterationsInput = document.getElementById('iterations');
const deviceInput = document.getElementById('device');
const kernelInput = document.getElementById('kernel');
const xclbinInput = document.getElementById('xclbin');
const statusBtn = document.getElementById('status-btn');
const programBtn = document.getElementById('program-btn');
const benchBtn = document.getElementById('bench-btn');
const logEl = document.getElementById('runtime-log');
const protectedRangeSlider = document.getElementById('flock-protected-range');
const visualRangeSlider = document.getElementById('flock-visual-range');
const centeringRecipSlider = document.getElementById('flock-centering-recip');
const avoidRecipSlider = document.getElementById('flock-avoid-recip');
const matchingRecipSlider = document.getElementById('flock-matching-recip');
const protectedRangeVal = document.getElementById('flock-protected-range-val');
const visualRangeVal = document.getElementById('flock-visual-range-val');
const centeringRecipVal = document.getElementById('flock-centering-recip-val');
const avoidRecipVal = document.getElementById('flock-avoid-recip-val');
const matchingRecipVal = document.getElementById('flock-matching-recip-val');

const PREVIEW_MAX_BOIDS = 700;
const SOFT_MARGIN = 100;
const BOUNCE_NUDGE = 15;
const FPGA_QUANT_STEP = 1 / 16;

const flockRules = {
  visualRange: 75,
  protectedRange: 8,
  centeringFactor: 0.0005,
  matchingFactor: 0.05,
  avoidFactor: 0.05,
  turnFactor: 0.2,
  minSpeed: 3.0,
  maxSpeed: 6.0,
};

let boids = [];
let fpgaBoids = [];
let running = true;
let lastTs = 0;
let fps = 0;
let frameMs = 0;
let fpgaFrameMs = 0;
let frameCount = 0;
let frameAccum = 0;
let previewSourceCount = 0;

function appendLog(msg) {
  const now = new Date().toLocaleTimeString();
  logEl.textContent = `[${now}] ${msg}\n${logEl.textContent}`.slice(0, 60000);
}

function mulberry32(seed) {
  let t = seed >>> 0;
  return function rand() {
    t += 0x6D2B79F5;
    let x = t;
    x = Math.imul(x ^ (x >>> 15), x | 1);
    x ^= x + Math.imul(x ^ (x >>> 7), x | 61);
    return ((x ^ (x >>> 14)) >>> 0) / 4294967296;
  };
}

function syncWorkloadFields() {
  const b = Math.max(1, Number(boidsInput.value || 1));
  const s = Math.max(1, Number(stepsInput.value || 1));
  elementsInput.value = String(b);
  iterationsInput.value = String(s);
}

function reciprocalToFactor(value, fallback) {
  const asNumber = Number(value);
  if (!Number.isFinite(asNumber) || asNumber <= 0) return fallback;
  return 1 / asNumber;
}

function factorToReciprocal(value, fallback) {
  const asNumber = Number(value);
  if (!Number.isFinite(asNumber) || asNumber <= 0) return fallback;
  return Math.max(1, Math.round(1 / asNumber));
}

function formatFactor(value) {
  if (!Number.isFinite(value)) return '0';
  if (Math.abs(value) < 0.001) return value.toExponential(3);
  return value.toFixed(4);
}

function seedFlockControlValues() {
  protectedRangeSlider.value = String(Math.round(flockRules.protectedRange));
  visualRangeSlider.value = String(Math.round(flockRules.visualRange));
  centeringRecipSlider.value = String(factorToReciprocal(flockRules.centeringFactor, 2000));
  avoidRecipSlider.value = String(factorToReciprocal(flockRules.avoidFactor, 20));
  matchingRecipSlider.value = String(factorToReciprocal(flockRules.matchingFactor, 20));
}

function syncFlockControlLabels() {
  protectedRangeVal.textContent = `${Math.round(flockRules.protectedRange)}`;
  visualRangeVal.textContent = `${Math.round(flockRules.visualRange)}`;
  centeringRecipVal.textContent = `${Math.round(Number(centeringRecipSlider.value || 2000))} (${formatFactor(flockRules.centeringFactor)})`;
  avoidRecipVal.textContent = `${Math.round(Number(avoidRecipSlider.value || 20))} (${formatFactor(flockRules.avoidFactor)})`;
  matchingRecipVal.textContent = `${Math.round(Number(matchingRecipSlider.value || 20))} (${formatFactor(flockRules.matchingFactor)})`;
}

function applyFlockControlValues() {
  const protectedRange = Math.max(1, Number(protectedRangeSlider.value || flockRules.protectedRange));
  const visualRangeRaw = Math.max(2, Number(visualRangeSlider.value || flockRules.visualRange));
  const visualRange = Math.max(visualRangeRaw, protectedRange + 1);
  const centeringFactor = reciprocalToFactor(centeringRecipSlider.value, flockRules.centeringFactor);
  const avoidFactor = reciprocalToFactor(avoidRecipSlider.value, flockRules.avoidFactor);
  const matchingFactor = reciprocalToFactor(matchingRecipSlider.value, flockRules.matchingFactor);

  flockRules.protectedRange = protectedRange;
  flockRules.visualRange = visualRange;
  flockRules.centeringFactor = centeringFactor;
  flockRules.avoidFactor = avoidFactor;
  flockRules.matchingFactor = matchingFactor;

  if (Number(visualRangeSlider.value || 0) !== visualRange) {
    visualRangeSlider.value = String(Math.round(visualRange));
  }

  syncFlockControlLabels();
}

function initFlock() {
  const totalRequested = Math.max(1, Number(boidsInput.value || 1));
  const seed = Number(seedInput.value || 1);
  const previewCount = Math.min(totalRequested, PREVIEW_MAX_BOIDS);
  const rand = mulberry32(seed >>> 0);

  boids = [];
  fpgaBoids = [];
  boids.length = previewCount;
  fpgaBoids.length = previewCount;
  const minX = SOFT_MARGIN;
  const maxX = canvas.width - SOFT_MARGIN;
  const minY = SOFT_MARGIN;
  const maxY = canvas.height - SOFT_MARGIN;

  for (let i = 0; i < previewCount; i++) {
    const x = minX + rand() * (maxX - minX);
    const y = minY + rand() * (maxY - minY);
    const angle = rand() * Math.PI * 2;
    const speed = flockRules.minSpeed + rand() * (flockRules.maxSpeed - flockRules.minSpeed);
    boids[i] = {
      x,
      y,
      vx: Math.cos(angle) * speed,
      vy: Math.sin(angle) * speed,
    };

    fpgaBoids[i] = { ...boids[i] };
  }

  previewSourceCount = totalRequested;
  lastTs = 0;
  frameCount = 0;
  frameAccum = 0;
  fps = 0;
  frameMs = 0;
  fpgaFrameMs = 0;
}

function quantize(val, step) {
  return Math.round(val / step) * step;
}

function updateFlockModel(targetBoids, drawCanvas, quantStep = 0) {
  const n = targetBoids.length;
  if (!n) return;

  const vr2 = flockRules.visualRange * flockRules.visualRange;
  const pr2 = flockRules.protectedRange * flockRules.protectedRange;
  const nextVX = new Array(n);
  const nextVY = new Array(n);

  for (let i = 0; i < n; i++) {
    const bi = targetBoids[i];

    let closeX = 0;
    let closeY = 0;
    let avgVX = 0;
    let avgVY = 0;
    let avgX = 0;
    let avgY = 0;
    let neighbors = 0;

    for (let j = 0; j < n; j++) {
      if (i === j) continue;
      const bj = targetBoids[j];
      const dx = bi.x - bj.x;
      const dy = bi.y - bj.y;
      if (Math.abs(dx) < flockRules.visualRange && Math.abs(dy) < flockRules.visualRange) {
        const dist2 = dx * dx + dy * dy;
        if (dist2 < pr2) {
          closeX += dx;
          closeY += dy;
        } else if (dist2 < vr2) {
          avgVX += bj.vx;
          avgVY += bj.vy;
          avgX += bj.x;
          avgY += bj.y;
          neighbors += 1;
        }
      }
    }

    if (neighbors > 0) {
      avgVX /= neighbors;
      avgVY /= neighbors;
      avgX /= neighbors;
      avgY /= neighbors;

      bi.vx += (avgX - bi.x) * flockRules.centeringFactor;
      bi.vy += (avgY - bi.y) * flockRules.centeringFactor;
      bi.vx += (avgVX - bi.vx) * flockRules.matchingFactor;
      bi.vy += (avgVY - bi.vy) * flockRules.matchingFactor;
    }

    bi.vx += closeX * flockRules.avoidFactor;
    bi.vy += closeY * flockRules.avoidFactor;

    if (bi.x < SOFT_MARGIN) bi.vx += flockRules.turnFactor;
    if (bi.x > drawCanvas.width - SOFT_MARGIN) bi.vx -= flockRules.turnFactor;
    if (bi.y < SOFT_MARGIN) bi.vy += flockRules.turnFactor;
    if (bi.y > drawCanvas.height - SOFT_MARGIN) bi.vy -= flockRules.turnFactor;

    const speed = Math.hypot(bi.vx, bi.vy) || 1.0;
    if (speed < flockRules.minSpeed) {
      const scale = flockRules.minSpeed / speed;
      bi.vx *= scale;
      bi.vy *= scale;
    } else if (speed > flockRules.maxSpeed) {
      const scale = flockRules.maxSpeed / speed;
      bi.vx *= scale;
      bi.vy *= scale;
    }

    if (quantStep > 0) {
      bi.vx = quantize(bi.vx, quantStep);
      bi.vy = quantize(bi.vy, quantStep);
    }

    nextVX[i] = bi.vx;
    nextVY[i] = bi.vy;
  }

  for (let i = 0; i < n; i++) {
    const b = targetBoids[i];
    b.vx = nextVX[i];
    b.vy = nextVY[i];
    b.x += b.vx;
    b.y += b.vy;

    if (quantStep > 0) {
      b.x = quantize(b.x, quantStep);
      b.y = quantize(b.y, quantStep);
    }

    if (b.x < 0) {
      b.x += BOUNCE_NUDGE;
      b.vx = Math.abs(b.vx);
    } else if (b.x > drawCanvas.width) {
      b.x -= BOUNCE_NUDGE;
      b.vx = -Math.abs(b.vx);
    }

    if (b.y < 0) {
      b.y += BOUNCE_NUDGE;
      b.vy = Math.abs(b.vy);
    } else if (b.y > drawCanvas.height) {
      b.y -= BOUNCE_NUDGE;
      b.vy = -Math.abs(b.vy);
    }

    if (b.x < 0) b.x = 0;
    else if (b.x > drawCanvas.width) b.x = drawCanvas.width;
    if (b.y < 0) b.y = 0;
    else if (b.y > drawCanvas.height) b.y = drawCanvas.height;
    }
  }

function updateFlock() {
  updateFlockModel(boids, canvas, 0);
}

function drawFlock() {
  ctx.fillStyle = '#000000';
  ctx.fillRect(0, 0, canvas.width, canvas.height);

  ctx.fillStyle = '#ffffff';
  for (let i = 0; i < boids.length; i++) {
    const b = boids[i];
    ctx.fillRect(b.x - 1, b.y - 1, 2.25, 2.25);
  }
}

function updateFpgaFlock() {
  updateFlockModel(fpgaBoids, fpgaCanvas, FPGA_QUANT_STEP);
}

function drawFpgaFlock() {
  fpgaCtx.fillStyle = '#000000';
  fpgaCtx.fillRect(0, 0, fpgaCanvas.width, fpgaCanvas.height);
  fpgaCtx.fillStyle = '#ffffff';
  for (let i = 0; i < fpgaBoids.length; i++) {
    const b = fpgaBoids[i];
    fpgaCtx.fillRect(b.x - 1, b.y - 1, 2.25, 2.25);
  }
}

function updateSimMeta() {
  const capped = previewSourceCount > boids.length;
  const capNote = capped ? ` | preview capped to ${boids.length}` : '';
  const state = running ? 'running' : 'paused';
  simMeta.textContent = `CPU flock ${state} | source=${previewSourceCount}${capNote} | fps=${fps.toFixed(1)} | frame=${frameMs.toFixed(2)} ms`;
  if (fpgaFlockMeta) {
    fpgaFlockMeta.textContent = `FPGA flock ${state} | source=${previewSourceCount}${capNote} | frame=${fpgaFrameMs.toFixed(2)} ms`;
  }
}

function animate(ts) {
  if (!lastTs) lastTs = ts;
  const dt = ts - lastTs;
  lastTs = ts;

  if (running) {
    const t0 = performance.now();
    updateFlock();
    drawFlock();
    frameMs = performance.now() - t0;

    const t1 = performance.now();
    updateFpgaFlock();
    drawFpgaFlock();
    fpgaFrameMs = performance.now() - t1;

    frameCount += 1;
    frameAccum += dt;
    if (frameAccum >= 300) {
      fps = (1000 * frameCount) / frameAccum;
      frameCount = 0;
      frameAccum = 0;
      updateSimMeta();
    }
  }

  requestAnimationFrame(animate);
}

function setRunState(nextRunning) {
  running = !!nextRunning;
  simulateBtn.textContent = running ? 'Pause CPU Flock' : 'Resume CPU Flock';
  updateSimMeta();
}

function mapTracePoint(x, y, bounds, traceCanvas) {
  const px = ((x + bounds.x) / (2 * bounds.x)) * traceCanvas.width;
  const py = traceCanvas.height - ((y + bounds.y) / (2 * bounds.y)) * traceCanvas.height;
  return [px, py];
}

function renderTraceCanvas(traceCtx, traceCanvas, traces, bounds) {
  traceCtx.fillStyle = '#000';
  traceCtx.fillRect(0, 0, traceCanvas.width, traceCanvas.height);

  traceCtx.strokeStyle = 'rgba(255,255,255,0.22)';
  traceCtx.lineWidth = 1;
  for (const tr of traces) {
    if (!tr || tr.length < 2) continue;
    traceCtx.beginPath();
    const [x0, y0] = mapTracePoint(tr[0][0], tr[0][1], bounds, traceCanvas);
    traceCtx.moveTo(x0, y0);
    for (let i = 1; i < tr.length; i++) {
      const [x, y] = mapTracePoint(tr[i][0], tr[i][1], bounds, traceCanvas);
      traceCtx.lineTo(x, y);
    }
    traceCtx.stroke();
  }

  traceCtx.fillStyle = '#fff';
  for (const tr of traces) {
    if (!tr || !tr.length) continue;
    const p = tr[tr.length - 1];
    const [x, y] = mapTracePoint(p[0], p[1], bounds, traceCanvas);
    traceCtx.fillRect(x - 1.1, y - 1.1, 2.4, 2.4);
  }
}

async function runTraceCompare() {
  traceBtn.disabled = true;
  traceMeta.textContent = 'Generating traces...';

  try {
    const payload = {
      boids: Math.max(1, Number(boidsInput.value || 1)),
      steps: Math.max(1, Number(stepsInput.value || 1)),
      seed: Number(seedInput.value || 1),
      trace_count: 24,
      visual_range: flockRules.visualRange,
      protected_range: flockRules.protectedRange,
      centering_factor: flockRules.centeringFactor,
      avoid_factor: flockRules.avoidFactor,
      matching_factor: flockRules.matchingFactor,
      turn_factor: flockRules.turnFactor,
      min_speed: flockRules.minSpeed,
      max_speed: flockRules.maxSpeed,
    };

    const resp = await fetch('/api/boids/trace-compare', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(payload),
    });

    const data = await resp.json();
    if (!resp.ok) {
      throw new Error(data.detail || 'Trace generation failed');
    }

    renderTraceCanvas(cpuTraceCtx, cpuTraceCanvas, data.cpu_trace || [], data.bounds);
    renderTraceCanvas(fpgaTraceCtx, fpgaTraceCanvas, data.fpga_trace || [], data.bounds);

    const notes = (data.notes || []).join(' | ');
    traceMeta.textContent =
      `Trace generated: boids=${data.boids}, steps=${data.steps}, tracked=${data.trace_count} | ` +
      `CPU trace ${Number(data.cpu_elapsed_ms || 0).toFixed(2)} ms | ` +
      `FPGA-model trace ${Number(data.fpga_model_elapsed_ms || 0).toFixed(2)} ms` +
      (notes ? ` | ${notes}` : '');

    appendLog(`Trace compare generated: boids=${data.boids} steps=${data.steps} tracked=${data.trace_count}`);
    if (runtimeSummary) {
      runtimeSummary.textContent =
        `Trace compare complete | CPU trace ${Number(data.cpu_elapsed_ms || 0).toFixed(2)} ms | ` +
        `FPGA-model trace ${Number(data.fpga_model_elapsed_ms || 0).toFixed(2)} ms.`;
    }
  } catch (err) {
    traceMeta.textContent = `Trace error: ${err.message}`;
    appendLog(`Trace error: ${err.message}`);
  } finally {
    traceBtn.disabled = false;
  }
}

async function runStatus() {
  appendLog('Querying FPGA status...');
  const resp = await fetch('/api/fpga/status');
  const data = await resp.json();
  if (!resp.ok) throw new Error(data.detail || 'Status failed');
  appendLog(data.output || '(no output)');
}

async function runProgram() {
  appendLog('Programming FPGA...');
  const payload = {
    xclbin_path: xclbinInput.value.trim(),
    device: deviceInput.value.trim() || '0',
  };
  const resp = await fetch('/api/fpga/program', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(payload),
  });
  const data = await resp.json();
  if (!resp.ok) throw new Error(data.detail || 'Program failed');
  appendLog(data.output || '(program command completed)');
}

async function runBenchmark() {
  benchBtn.disabled = true;
  appendLog('Running benchmark...');
  try {
    const payload = {
      mode: benchMode.value,
      elements: Math.max(1, Number(elementsInput.value || 1)),
      iterations: Math.max(1, Number(iterationsInput.value || 1)),
      device: deviceInput.value.trim() || '0',
      kernel: kernelInput.value.trim() || 'boids_step',
      seed: Number(seedInput.value || 1),
      xclbin_path: xclbinInput.value.trim(),
    };

    const resp = await fetch('/api/benchmark', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(payload),
    });
    const data = await resp.json();
    if (!resp.ok) throw new Error(data.detail || 'Benchmark failed');

    appendLog(data.cmd);
    appendLog(data.summary || '(no summary line found)');
    if (data.output_tail) appendLog(data.output_tail);
  } catch (err) {
    appendLog(`Benchmark error: ${err.message}`);
  } finally {
    benchBtn.disabled = false;
  }
}

async function runCompare() {
  compareBtn.disabled = true;
  compareMeta.textContent = 'Running CPU + FPGA comparison...';

  try {
    const payload = {
      boids: Math.max(1, Number(boidsInput.value || 1)),
      steps: Math.max(1, Number(stepsInput.value || 1)),
      device: deviceInput.value.trim() || '0',
      kernel: kernelInput.value.trim() || 'boids_step',
      seed: Number(seedInput.value || 1),
      xclbin_path: xclbinInput.value.trim(),
    };

    const resp = await fetch('/api/compare', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(payload),
    });
    const data = await resp.json();
    if (!resp.ok) throw new Error(data.detail || 'Compare failed');

    const cpu = data.cpu?.metrics || {};
    const fpga = data.fpga?.metrics || {};
    const sx = data.speedup || {};

    const cpuTotal = Number(cpu.avg_total_ms || 0);
    const fpgaTotal = Number(fpga.avg_total_ms || 0);

    compareMeta.textContent =
      `CPU total ${cpuTotal.toFixed(3)} ms | FPGA total ${fpgaTotal.toFixed(3)} ms | ` +
      `total speedup ${Number(sx.total_x || 0).toFixed(2)}x | kernel speedup ${Number(sx.kernel_x || 0).toFixed(2)}x`;

    appendLog(`COMPARE workload boids=${payload.boids} steps=${payload.steps}`);
    appendLog(`CPU: ${data.cpu?.summary || '(no summary)'}`);
    appendLog(`FPGA: ${data.fpga?.summary || '(no summary)'}`);
    appendLog(`Speedup total=${Number(sx.total_x || 0).toFixed(3)}x kernel=${Number(sx.kernel_x || 0).toFixed(3)}x`);
    if (runtimeSummary) {
      runtimeSummary.textContent =
        `Compare complete | CPU total ${cpuTotal.toFixed(3)} ms | FPGA total ${fpgaTotal.toFixed(3)} ms | ` +
        `speedup ${Number(sx.total_x || 0).toFixed(2)}x.`;
    }

    elementsInput.value = String(payload.boids);
    iterationsInput.value = String(payload.steps);
  } catch (err) {
    compareMeta.textContent = `Compare error: ${err.message}`;
    appendLog(`Compare error: ${err.message}`);
  } finally {
    compareBtn.disabled = false;
  }
}

simulateBtn.addEventListener('click', () => {
  setRunState(!running);
});

resetBtn.addEventListener('click', () => {
  syncWorkloadFields();
  initFlock();
  setRunState(true);
  drawFlock();
});

compareBtn.addEventListener('click', () => {
  runCompare().catch((err) => {
    compareMeta.textContent = `Compare error: ${err.message}`;
  });
});

traceBtn.addEventListener('click', () => {
  runTraceCompare().catch((err) => {
    traceMeta.textContent = `Trace error: ${err.message}`;
  });
});

statusBtn.addEventListener('click', () => runStatus().catch((err) => appendLog(`Status error: ${err.message}`)));
programBtn.addEventListener('click', () => runProgram().catch((err) => appendLog(`Program error: ${err.message}`)));
benchBtn.addEventListener('click', () => runBenchmark().catch((err) => appendLog(`Benchmark error: ${err.message}`)));

boidsInput.addEventListener('change', syncWorkloadFields);
stepsInput.addEventListener('change', syncWorkloadFields);
[
  protectedRangeSlider,
  visualRangeSlider,
  centeringRecipSlider,
  avoidRecipSlider,
  matchingRecipSlider,
].forEach((slider) => {
  slider.addEventListener('input', () => {
    applyFlockControlValues();
  });
});

syncWorkloadFields();
seedFlockControlValues();
applyFlockControlValues();
initFlock();
setRunState(true);
drawFlock();
drawFpgaFlock();
runTraceCompare().catch(() => {});
requestAnimationFrame(animate);
