/* ================================================================
   Dual Crank-Slider Design Tool - script.js
   Ported from MATLAB prototype: follows the same search logic,
   same sign conventions (0 deg = right, + = clockwise,
   lower semicircle means y = -r*sin(angle) in math coords).
   ================================================================ */

'use strict';

// -----------------------------------------------------------------
// Small math helpers (degrees)
// -----------------------------------------------------------------
const DEG2RAD = Math.PI / 180;
const sind = (d) => Math.sin(d * DEG2RAD);
const cosd = (d) => Math.cos(d * DEG2RAD);
const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));

// Slider position measured from O_B along the +x axis
// x_B(phi) = r_B*cos(phi) + sqrt(l_B^2 - r_B^2*sin(phi)^2)
function sliderPosB(phi_deg, rB, lB) {
    const inside = lB * lB - (rB * sind(phi_deg)) ** 2;
    if (inside < -1e-9) return NaN;
    return rB * cosd(phi_deg) + Math.sqrt(Math.max(inside, 0));
}

// --- Crank A geometry ------------------------------------------------
// Angle convention for Crank A matches Crank B:
//   theta_A = 0   => pin points to the RIGHT of O_A  (away from slider)
//   theta_A = 180 => pin points to the LEFT of O_A   (toward slider)
//   positive angle = clockwise, lower semicircle => y = -rA*sin(theta)
// Because the slider lies on the negative-x side of O_A (between O_B and O_A),
// we compute xSlider directly from pin geometry:
//   pin     = (D + rA*cos(theta), -rA*sin(theta))
//   slider  = pin - (sqrt(lA^2 - pinY^2), 0)    // slider to the LEFT of pin
function xSliderFromThetaA(theta_deg, D, rA, lA) {
    const pinX = D + rA * cosd(theta_deg);
    const pinY = -rA * sind(theta_deg);
    const inside = lA * lA - pinY * pinY;
    if (inside < 0) return NaN;
    return pinX - Math.sqrt(inside);
}

// Solve theta_A in [0, 360) such that xSliderFromThetaA(theta, D, rA, lA) = xSlider_target
// Picks the root closest to preferredTheta_deg.
function solveThetaAFromSlider(xSlider_target, D, rA, lA, preferredTheta_deg) {
    const f = (th) => xSliderFromThetaA(th, D, rA, lA) - xSlider_target;
    const scanN = 721; // 0.5-deg resolution over [0, 360]
    const roots = [];
    let thPrev = 0;
    let fPrev = f(thPrev);
    for (let i = 1; i < scanN; i++) {
        const th = (i * 360) / (scanN - 1);
        const fCur = f(th);
        if (!isFinite(fPrev) || !isFinite(fCur)) {
            thPrev = th; fPrev = fCur; continue;
        }
        if (Math.abs(fPrev) < 1e-9) roots.push(thPrev);
        else if (fPrev * fCur < 0) {
            let a = thPrev, b = th, fa = fPrev, fb = fCur;
            for (let k = 0; k < 40; k++) {
                const m = 0.5 * (a + b);
                const fm = f(m);
                if (!isFinite(fm)) break;
                if (fa * fm <= 0) { b = m; fb = fm; }
                else { a = m; fa = fm; }
                if (Math.abs(b - a) < 1e-7) break;
            }
            roots.push(0.5 * (a + b));
        }
        thPrev = th; fPrev = fCur;
    }
    if (roots.length === 0) return { theta: NaN, solved: false };
    // Pick the root closest to preferredTheta_deg (with 360° wrap-around)
    let bestTh = roots[0], bestDelta = Infinity;
    for (const th of roots) {
        let d = Math.abs(th - preferredTheta_deg);
        d = Math.min(d, 360 - d);
        if (d < bestDelta) { bestDelta = d; bestTh = th; }
    }
    return { theta: bestTh, solved: true };
}

// Solve phi_B from slider x_B (in [0, 180])
function solvePhiBFromSlider(xB_target, rB, lB) {
    const f = (p) => sliderPosB(p, rB, lB) - xB_target;
    const scanN = 361;
    let pPrev = 0, fPrev = f(pPrev);
    const roots = [];
    for (let i = 1; i < scanN; i++) {
        const p = (i * 180) / (scanN - 1);
        const fCur = f(p);
        if (!isFinite(fPrev) || !isFinite(fCur)) {
            pPrev = p; fPrev = fCur; continue;
        }
        if (Math.abs(fPrev) < 1e-9) roots.push(pPrev);
        else if (fPrev * fCur < 0) {
            let a = pPrev, b = p, fa = fPrev, fb = fCur;
            for (let k = 0; k < 40; k++) {
                const m = 0.5 * (a + b);
                const fm = f(m);
                if (!isFinite(fm)) break;
                if (fa * fm <= 0) { b = m; fb = fm; }
                else { a = m; fa = fm; }
                if (Math.abs(b - a) < 1e-6) break;
            }
            roots.push(0.5 * (a + b));
        }
        pPrev = p; fPrev = fCur;
    }
    if (roots.length === 0) return { phi: NaN, solved: false };
    // For the crank-slider with l>r, there is typically one root in [0,180]
    return { phi: roots[0], solved: true };
}

// -----------------------------------------------------------------
// ROM window selection (per case, for a given rB,lB candidate)
// -----------------------------------------------------------------
function chooseBestROMWindow(deltaPhi_deg, rB, lB, phiBaseline_deg,
    phiStopMin_deg, phiStopMax_deg, prevCenter,
    wBaseline, wPrev, wStroke, nCenterScan, stepIndex) {

    const centerMin = phiStopMin_deg + deltaPhi_deg / 2;
    const centerMax = phiStopMax_deg - deltaPhi_deg / 2;
    if (centerMax < centerMin) return { solved: false };

    let bestObj = Infinity;
    let best = null;

    for (let i = 0; i < nCenterScan; i++) {
        const t = nCenterScan === 1 ? 0 : i / (nCenterScan - 1);
        const phiC = centerMin + t * (centerMax - centerMin);
        const phi1 = phiC - deltaPhi_deg / 2;
        const phi2 = phiC + deltaPhi_deg / 2;
        if (phi1 < phiStopMin_deg || phi2 > phiStopMax_deg) continue;
        const x1 = sliderPosB(phi1, rB, lB);
        const x2 = sliderPosB(phi2, rB, lB);
        if (!isFinite(x1) || !isFinite(x2)) continue;
        const thisStroke = Math.abs(x2 - x1);
        const thisLo = Math.min(x1, x2);
        const thisHi = Math.max(x1, x2);

        let obj;
        if (stepIndex === 0) {
            obj = wBaseline * Math.abs(phiC - phiBaseline_deg) + wStroke * thisStroke;
        } else {
            obj = wPrev * Math.abs(phiC - prevCenter) + wStroke * thisStroke;
        }

        if (obj < bestObj) {
            bestObj = obj;
            best = {
                phiMin: phi1, phiMax: phi2, phiCenter: phiC,
                stroke: thisStroke, xLo: thisLo, xHi: thisHi,
                obj: obj
            };
        }
    }
    if (best === null) return { solved: false };
    return { solved: true, ...best };
}

function evaluateCandidateRelaxed(rB, lB, deltaPhiList, params) {
    const n = deltaPhiList.length;
    const out = {
        feasible: false,
        phiMin: new Array(n), phiMax: new Array(n),
        phiCenter: new Array(n), centerDrift: new Array(n),
        stroke: new Array(n), xLo: new Array(n), xHi: new Array(n),
        windowObj: new Array(n)
    };
    let prevCenter = NaN;
    for (let k = 0; k < n; k++) {
        const r = chooseBestROMWindow(
            deltaPhiList[k], rB, lB, params.phiBaseline,
            params.phiStopMin, params.phiStopMax,
            prevCenter,
            params.wBaseline, params.wPrev, params.wStroke,
            params.nCenterScan, k
        );
        if (!r.solved) return out;
        out.phiMin[k] = r.phiMin;
        out.phiMax[k] = r.phiMax;
        out.phiCenter[k] = r.phiCenter;
        out.centerDrift[k] = r.phiCenter - params.phiBaseline;
        out.stroke[k] = r.stroke;
        out.xLo[k] = r.xLo;
        out.xHi[k] = r.xHi;
        out.windowObj[k] = r.obj;
        prevCenter = r.phiCenter;
    }
    if (Math.abs(out.centerDrift[0]) > params.maxSmallDrift) return out;
    if (Math.abs(out.centerDrift[n - 1]) > params.maxDrift) return out;
    out.feasible = true;
    return out;
}

// -----------------------------------------------------------------
// Main search
// -----------------------------------------------------------------
function searchDesign(params) {
    const {
        deltaPhiMin, deltaPhiMax, nSteps,
        phiStopMin, phiStopMax, phiBaseline,
        rBmin, rBmax, rBstep,
        lambdaBmin, lambdaBmax, lambdaBstep,
        marginFactor, dMarginFactor
    } = params;

    // Spread delta phi linearly
    const deltaPhiList = [];
    for (let i = 0; i < nSteps; i++) {
        const t = nSteps === 1 ? 0 : i / (nSteps - 1);
        deltaPhiList.push(deltaPhiMin + t * (deltaPhiMax - deltaPhiMin));
    }

    const lambdaBminClearance = 2 + marginFactor;
    const lambdaBmin_eff = Math.max(lambdaBmin, lambdaBminClearance);

    if (deltaPhiMax >= (phiStopMax - phiStopMin)) {
        throw new Error('Largest Δφ is too big for the hard-stop range.');
    }
    if (lambdaBmin_eff > lambdaBmax) {
        throw new Error('No λ_B candidates exist. Increase λ_B max or reduce margin factor.');
    }

    const rBlist = [];
    for (let v = rBmin; v <= rBmax + 1e-9; v += rBstep) rBlist.push(+v.toFixed(8));
    const lambdaBlist = [];
    for (let v = lambdaBmin_eff; v <= lambdaBmax + 1e-9; v += lambdaBstep) lambdaBlist.push(+v.toFixed(8));

    let bestDesign = null;
    let bestObjective = Infinity;
    let feasibleCount = 0;

    for (const rB of rBlist) {
        for (const lambdaB of lambdaBlist) {
            const lB = lambdaB * rB;
            // clearance / envelope condition
            if (lB <= 2 * rB + marginFactor * rB) continue;

            const cand = evaluateCandidateRelaxed(rB, lB, deltaPhiList, params);
            if (!cand.feasible) continue;

            const rA_list = cand.stroke.map(s => s / 2);
            const rA_max = Math.max(...rA_list);
            if (rA_max > rB) continue;

            const lA = 2 * rA_max + marginFactor * rA_max;
            const d_min_theoretical = lA + lB + rA_max - rB;
            const d_margin = dMarginFactor * Math.max(rA_max, rB);
            const d = d_min_theoretical + d_margin;
            const d_upper = lA + lB - rA_max + rB;
            if (d > d_upper) continue;

            feasibleCount++;

            const design = {
                rB, lB, lambdaB,
                rA_list, rA_max, lA, d, d_min_theoretical, d_margin,
                railLength: d - rB,
                deltaPhiList: deltaPhiList.slice(),
                phiMin: cand.phiMin.slice(),
                phiMax: cand.phiMax.slice(),
                phiCenter: cand.phiCenter.slice(),
                centerDrift: cand.centerDrift.slice(),
                stroke: cand.stroke.slice(),
                strokeLeft: cand.xLo.slice(),
                strokeRight: cand.xHi.slice(),
                windowObj: cand.windowObj.slice(),
                xMid_ref: sliderPosB(phiBaseline, rB, lB)
            };
            const objective = d + 1e-3 * lambdaB + 1e-4 * rB;
            if (objective < bestObjective) {
                bestObjective = objective;
                bestDesign = design;
            }
        }
    }
    return { bestDesign, feasibleCount };
}

// -----------------------------------------------------------------
// UI STATE
// -----------------------------------------------------------------
const state = {
    design: null,           // baseline suggested design (immutable snapshot)
    working: null,          // editable working copy (what is drawn / animated)
    selectedCase: 0,        // 0-based index
    phiStopMin: 35,
    phiStopMax: 170,
    preferredThetaA: 110,
    // animation
    animating: false,
    animTheta: 0,
    lastT: 0,
    reqId: null
};

// -----------------------------------------------------------------
// DOM helpers
// -----------------------------------------------------------------
const $ = (id) => document.getElementById(id);
const numIn = (id) => parseFloat($(id).value);
const intIn = (id) => parseInt($(id).value, 10);
const setStatus = (txt, cls) => {
    const el = $('status-pill');
    el.textContent = txt;
    el.className = 'status-pill ' + (cls || 'ready');
};
const setMsg = (txt, cls) => {
    const el = $('searchMsg');
    el.textContent = txt;
    el.className = 'search-msg ' + (cls || '');
};

// -----------------------------------------------------------------
// SUGGEST DESIGN (onclick handler)
// -----------------------------------------------------------------
$('suggestBtn').addEventListener('click', () => {
    try {
        const params = {
            deltaPhiMin: numIn('deltaPhiMin'),
            deltaPhiMax: numIn('deltaPhiMax'),
            nSteps: intIn('nSteps'),
            phiStopMin: numIn('phiStopMin'),
            phiStopMax: numIn('phiStopMax'),
            phiBaseline: numIn('phiBaseline'),
            rBmin: numIn('rBmin'),
            rBmax: numIn('rBmax'),
            rBstep: numIn('rBstep'),
            lambdaBmin: numIn('lambdaBmin'),
            lambdaBmax: numIn('lambdaBmax'),
            lambdaBstep: numIn('lambdaBstep'),
            marginFactor: numIn('marginFactor'),
            dMarginFactor: numIn('dMarginFactor'),
            maxDrift: numIn('maxDrift'),
            maxSmallDrift: numIn('maxSmallDrift'),
            wBaseline: numIn('wBaseline'),
            wPrev: numIn('wPrev'),
            wStroke: numIn('wStroke'),
            nCenterScan: intIn('nCenterScan')
        };
        state.phiStopMin = params.phiStopMin;
        state.phiStopMax = params.phiStopMax;
        state.preferredThetaA = numIn('preferredThetaA');

        setStatus('Searching…', 'searching');
        setMsg('Running search over ' +
            Math.round((params.rBmax - params.rBmin) / params.rBstep + 1) + ' × ' +
            Math.round((params.lambdaBmax - params.lambdaBmin) / params.lambdaBstep + 1) +
            ' candidates…');

        // Yield to browser so the status can repaint before the search blocks
        setTimeout(() => {
            try {
                const { bestDesign, feasibleCount } = searchDesign(params);
                if (!bestDesign) {
                    setStatus('No feasible design', 'err');
                    setMsg('No feasible design found. Try relaxing search ranges, drift tolerances, or margin factor.', 'error');
                    return;
                }
                state.design = bestDesign;
                state.working = cloneDesign(bestDesign);
                state.selectedCase = 0;
                populateOverrideFields();
                buildCaseTabs();
                buildCasesTable();
                enableOverrideInputs(true);
                $('playBtn').disabled = false;
                $('resetBtn').disabled = false;
                setStatus('Design found', 'ok');
                setMsg(`Found ${feasibleCount} feasible design${feasibleCount === 1 ? '' : 's'}. Most compact selected.`, 'ok');
                redraw();
            } catch (err) {
                console.error(err);
                setStatus('Error', 'err');
                setMsg(err.message || 'Search failed.', 'error');
            }
        }, 10);
    } catch (err) {
        console.error(err);
        setStatus('Error', 'err');
        setMsg(err.message || 'Invalid inputs.', 'error');
    }
});

// -----------------------------------------------------------------
// Override fields
// -----------------------------------------------------------------
const overrideGlobalIds = ['ovr_rB', 'ovr_lB', 'ovr_lA', 'ovr_d'];
const overrideCaseIds = ['ovr_rA', 'ovr_phiMin', 'ovr_phiMax'];

function enableOverrideInputs(en) {
    overrideGlobalIds.concat(overrideCaseIds).forEach(id => $(id).disabled = !en);
}

function populateOverrideFields() {
    const d = state.working;
    const k = state.selectedCase;
    $('ovr_rB').value = d.rB.toFixed(4);
    $('ovr_lB').value = d.lB.toFixed(4);
    $('ovr_lA').value = d.lA.toFixed(4);
    $('ovr_d').value = d.d.toFixed(4);
    $('ovr_rA').value = d.rA_list[k].toFixed(4);
    $('ovr_phiMin').value = d.phiMin[k].toFixed(2);
    $('ovr_phiMax').value = d.phiMax[k].toFixed(2);
    updateReadonlyFields();
}

function updateReadonlyFields() {
    const d = state.working;
    const k = state.selectedCase;
    $('rd_rail').textContent = (d.d - d.rB).toFixed(4);
    $('rd_dphi').textContent = (d.phiMax[k] - d.phiMin[k]).toFixed(2);
    $('rd_center').textContent = (0.5 * (d.phiMin[k] + d.phiMax[k])).toFixed(2);
    $('rd_stroke').textContent = (d.stroke[k]).toFixed(4);
}

// Listen to override edits → commit into working design and redraw
overrideGlobalIds.forEach(id => $(id).addEventListener('input', () => {
    if (!state.working) return;
    const d = state.working;
    d.rB = parseFloat($('ovr_rB').value);
    d.lB = parseFloat($('ovr_lB').value);
    d.lA = parseFloat($('ovr_lA').value);
    d.d = parseFloat($('ovr_d').value);
    d.railLength = d.d - d.rB;
    // recompute stroke per case from phiMin/phiMax (rB/lB may have changed)
    recomputeStrokesFromPhi(d);
    updateReadonlyFields();
    redraw();
}));

overrideCaseIds.forEach(id => $(id).addEventListener('input', () => {
    if (!state.working) return;
    const d = state.working;
    const k = state.selectedCase;
    if (id === 'ovr_rA') {
        d.rA_list[k] = parseFloat($('ovr_rA').value);
        d.rA_max = Math.max(...d.rA_list);
    } else {
        d.phiMin[k] = parseFloat($('ovr_phiMin').value);
        d.phiMax[k] = parseFloat($('ovr_phiMax').value);
        d.phiCenter[k] = 0.5 * (d.phiMin[k] + d.phiMax[k]);
        d.deltaPhiList[k] = d.phiMax[k] - d.phiMin[k];
        recomputeStrokesFromPhi(d);
        // When user edits phi, follow the MATLAB convention: rA = stroke/2
        d.rA_list[k] = d.stroke[k] / 2;
        $('ovr_rA').value = d.rA_list[k].toFixed(4);
        d.rA_max = Math.max(...d.rA_list);
    }
    updateReadonlyFields();
    buildCasesTable();
    redraw();
}));

function recomputeStrokesFromPhi(d) {
    for (let i = 0; i < d.phiMin.length; i++) {
        const x1 = sliderPosB(d.phiMin[i], d.rB, d.lB);
        const x2 = sliderPosB(d.phiMax[i], d.rB, d.lB);
        d.strokeLeft[i] = Math.min(x1, x2);
        d.strokeRight[i] = Math.max(x1, x2);
        d.stroke[i] = Math.abs(x2 - x1);
    }
}

$('resetBtn').addEventListener('click', () => {
    if (!state.design) return;
    state.working = cloneDesign(state.design);
    populateOverrideFields();
    buildCaseTabs();
    buildCasesTable();
    redraw();
});

function cloneDesign(d) {
    return {
        rB: d.rB, lB: d.lB, lambdaB: d.lambdaB,
        rA_list: d.rA_list.slice(), rA_max: d.rA_max,
        lA: d.lA, d: d.d, d_min_theoretical: d.d_min_theoretical,
        d_margin: d.d_margin, railLength: d.railLength,
        deltaPhiList: d.deltaPhiList.slice(),
        phiMin: d.phiMin.slice(), phiMax: d.phiMax.slice(),
        phiCenter: d.phiCenter.slice(), centerDrift: d.centerDrift.slice(),
        stroke: d.stroke.slice(),
        strokeLeft: d.strokeLeft.slice(), strokeRight: d.strokeRight.slice(),
        windowObj: d.windowObj.slice(),
        xMid_ref: d.xMid_ref
    };
}

// -----------------------------------------------------------------
// Case tabs + cases table
// -----------------------------------------------------------------
function buildCaseTabs() {
    const container = $('caseTabs');
    container.innerHTML = '';
    const n = state.working.rA_list.length;
    for (let i = 0; i < n; i++) {
        const btn = document.createElement('button');
        btn.type = 'button';
        btn.className = 'case-tab' + (i === state.selectedCase ? ' active' : '');
        btn.textContent = `Case ${i + 1}`;
        btn.addEventListener('click', () => {
            if (state.animating) stopAnimation();
            state.selectedCase = i;
            document.querySelectorAll('.case-tab').forEach((b, j) =>
                b.classList.toggle('active', j === i));
            populateOverrideFields();
            buildCasesTable();
            redraw();
        });
        container.appendChild(btn);
    }
}

function buildCasesTable() {
    const tbody = $('allCasesTable').querySelector('tbody');
    tbody.innerHTML = '';
    const d = state.working;
    for (let i = 0; i < d.rA_list.length; i++) {
        const tr = document.createElement('tr');
        if (i === state.selectedCase) tr.classList.add('active');
        // Columns (must match thead order):
        //  # | r_B  l_B  l_A  d | Δφ  φ_min φ_max φ_ctr | x_L  x_R  stroke  r_A
        tr.innerHTML = `
            <td class="cat-idx">${i + 1}</td>
            <td class="cat-shared">${d.rB.toFixed(3)}</td>
            <td class="cat-shared">${d.lB.toFixed(3)}</td>
            <td class="cat-shared">${d.lA.toFixed(3)}</td>
            <td class="cat-shared">${d.d.toFixed(3)}</td>
            <td class="cat-B">${(d.phiMax[i] - d.phiMin[i]).toFixed(1)}</td>
            <td class="cat-B">${d.phiMin[i].toFixed(1)}</td>
            <td class="cat-B">${d.phiMax[i].toFixed(1)}</td>
            <td class="cat-B">${d.phiCenter[i].toFixed(1)}</td>
            <td class="cat-slider">${d.strokeLeft[i].toFixed(3)}</td>
            <td class="cat-slider">${d.strokeRight[i].toFixed(3)}</td>
            <td class="cat-slider">${d.stroke[i].toFixed(3)}</td>
            <td class="cat-slider">${d.rA_list[i].toFixed(3)}</td>
        `;
        tr.addEventListener('click', () => {
            if (state.animating) stopAnimation();
            state.selectedCase = i;
            document.querySelectorAll('.case-tab').forEach((b, j) =>
                b.classList.toggle('active', j === i));
            populateOverrideFields();
            buildCasesTable();
            redraw();
        });
        tbody.appendChild(tr);
    }
}

// -----------------------------------------------------------------
// CANVAS DRAWING
// -----------------------------------------------------------------
const canvas = $('vizCanvas');
const ctx = canvas.getContext('2d');

function resizeCanvas() {
    const rect = canvas.getBoundingClientRect();
    const dpr = window.devicePixelRatio || 1;
    canvas.width = Math.max(600, Math.floor(rect.width * dpr));
    canvas.height = Math.max(400, Math.floor(rect.height * dpr));
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    redraw();
}
window.addEventListener('resize', resizeCanvas);

function computeLimits(d) {
    // include: full B envelope, full A envelope, all stroke ranges
    let xs = [], ys = [];
    const t360 = Array.from({ length: 73 }, (_, i) => (i * Math.PI) / 36);
    for (const th of t360) {
        xs.push(d.rB * Math.cos(th)); ys.push(d.rB * Math.sin(th));
        xs.push(d.d + d.rA_max * Math.cos(th)); ys.push(d.rA_max * Math.sin(th));
    }
    for (let k = 0; k < d.phiMin.length; k++) {
        xs.push(d.strokeLeft[k], d.strokeRight[k]);
        ys.push(0, 0);
    }
    xs.push(0, d.d);
    ys.push(0);
    const xMin = Math.min(...xs), xMax = Math.max(...xs);
    const yMin = Math.min(...ys), yMax = Math.max(...ys);
    const xR = xMax - xMin || 1;
    const yR = yMax - yMin || 1;
    const pad = 0.18;
    return {
        xMin: xMin - pad * xR,
        xMax: xMax + pad * xR,
        yMin: yMin - 0.55 * Math.max(d.rA_max, d.rB, 1),
        yMax: yMax + 0.70 * Math.max(d.rA_max, d.rB, 1)
    };
}

function makeTransform(lim) {
    const rect = canvas.getBoundingClientRect();
    const cw = rect.width;
    const ch = rect.height;
    const sx = cw / (lim.xMax - lim.xMin);
    const sy = ch / (lim.yMax - lim.yMin);
    const s = Math.min(sx, sy) * 0.92;
    // Center drawing
    const xPxCenter = cw / 2;
    const yPxCenter = ch / 2;
    const xMathCenter = 0.5 * (lim.xMin + lim.xMax);
    const yMathCenter = 0.5 * (lim.yMin + lim.yMax);
    // map math(x,y) to canvas px(X,Y); math y up → canvas y down
    const toPx = (mx, my) => ({
        x: xPxCenter + (mx - xMathCenter) * s,
        y: yPxCenter - (my - yMathCenter) * s
    });
    return { toPx, s };
}

function clearCanvas() {
    const rect = canvas.getBoundingClientRect();
    ctx.save();
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    const dpr = window.devicePixelRatio || 1;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.restore();
    ctx.fillStyle = '#fcfcfd';
    ctx.fillRect(0, 0, rect.width, rect.height);
}

function drawGrid(lim, toPx) {
    const rect = canvas.getBoundingClientRect();
    ctx.strokeStyle = '#edf0f5';
    ctx.lineWidth = 1;
    const xStep = niceStep((lim.xMax - lim.xMin) / 10);
    const yStep = niceStep((lim.yMax - lim.yMin) / 8);
    ctx.beginPath();
    for (let x = Math.ceil(lim.xMin / xStep) * xStep; x <= lim.xMax; x += xStep) {
        const p1 = toPx(x, lim.yMin);
        const p2 = toPx(x, lim.yMax);
        ctx.moveTo(p1.x, p1.y); ctx.lineTo(p2.x, p2.y);
    }
    for (let y = Math.ceil(lim.yMin / yStep) * yStep; y <= lim.yMax; y += yStep) {
        const p1 = toPx(lim.xMin, y);
        const p2 = toPx(lim.xMax, y);
        ctx.moveTo(p1.x, p1.y); ctx.lineTo(p2.x, p2.y);
    }
    ctx.stroke();
}

function niceStep(raw) {
    if (raw <= 0) return 1;
    const p = Math.pow(10, Math.floor(Math.log10(raw)));
    const r = raw / p;
    let m;
    if (r < 1.5) m = 1;
    else if (r < 3) m = 2;
    else if (r < 7) m = 5;
    else m = 10;
    return m * p;
}

function drawCircle(toPx, cx, cy, r, style) {
    const p0 = toPx(cx + r, cy);
    const pC = toPx(cx, cy);
    const pxR = Math.hypot(p0.x - pC.x, p0.y - pC.y);
    ctx.beginPath();
    ctx.arc(pC.x, pC.y, pxR, 0, 2 * Math.PI);
    if (style.fill) { ctx.fillStyle = style.fill; ctx.fill(); }
    if (style.stroke) {
        ctx.strokeStyle = style.stroke;
        ctx.setLineDash(style.dash || []);
        ctx.lineWidth = style.width || 1;
        ctx.stroke();
        ctx.setLineDash([]);
    }
}

function drawSector(toPx, cx, cy, r, a1_deg, a2_deg, fillStyle, flipYForLowerSemicircle) {
    // MATLAB convention: y = -r*sin(angle) for lower semicircle
    const step = 2; // deg
    const pts = [];
    pts.push(toPx(cx, cy));
    const steps = Math.max(2, Math.floor(Math.abs(a2_deg - a1_deg) / step));
    for (let i = 0; i <= steps; i++) {
        const t = a1_deg + (a2_deg - a1_deg) * (i / steps);
        const px = cx + r * cosd(t);
        const py = cy + (flipYForLowerSemicircle ? -r * sind(t) : r * sind(t));
        pts.push(toPx(px, py));
    }
    ctx.beginPath();
    ctx.moveTo(pts[0].x, pts[0].y);
    for (let i = 1; i < pts.length; i++) ctx.lineTo(pts[i].x, pts[i].y);
    ctx.closePath();
    ctx.fillStyle = fillStyle;
    ctx.fill();
}

function drawLine(toPx, x1, y1, x2, y2, style) {
    const p1 = toPx(x1, y1);
    const p2 = toPx(x2, y2);
    ctx.beginPath();
    ctx.moveTo(p1.x, p1.y);
    ctx.lineTo(p2.x, p2.y);
    ctx.strokeStyle = style.stroke || '#000';
    ctx.lineWidth = style.width || 1;
    ctx.setLineDash(style.dash || []);
    ctx.stroke();
    ctx.setLineDash([]);
}

function drawDot(toPx, x, y, r_px, fill, stroke) {
    const p = toPx(x, y);
    ctx.beginPath();
    ctx.arc(p.x, p.y, r_px, 0, 2 * Math.PI);
    if (fill) { ctx.fillStyle = fill; ctx.fill(); }
    if (stroke) { ctx.strokeStyle = stroke; ctx.lineWidth = 1; ctx.stroke(); }
}

function drawText(toPx, x, y, text, style) {
    const p = toPx(x, y);
    ctx.fillStyle = style.color || '#222';
    ctx.font = style.font || '12px -apple-system, Segoe UI, sans-serif';
    ctx.textAlign = style.align || 'center';
    ctx.textBaseline = style.baseline || 'middle';
    ctx.fillText(text, p.x, p.y);
}

/**
 * Draw text with proper subscript rendering on canvas.
 * segments: array of { text: string, sub?: string }
 * style: { color, font, subFont, align ('left'|'center'|'right'), baseline }
 * Example:
 *   drawTextSub(toPx, x, y, [
 *     { text: 'φ', sub: 'B' },
 *     { text: ' = 45.0°' }
 *   ], { color: '#e65a1a', font: 'bold 12px sans-serif' })
 */
function drawTextSub(toPx, x, y, segments, style) {
    const p = toPx(x, y);
    const mainFont = style.font || 'bold 12px -apple-system, Segoe UI, sans-serif';
    // Subscript: ~72% size, offset down
    const subFont = style.subFont || mainFont.replace(/(\d+(?:\.\d+)?)px/, (_, n) =>
        Math.max(8, Math.round(parseFloat(n) * 0.72)) + 'px');
    const subDy = style.subDy !== undefined ? style.subDy : 3;

    // Measure total width
    let totalW = 0;
    for (const seg of segments) {
        ctx.font = mainFont;
        totalW += ctx.measureText(seg.text).width;
        if (seg.sub) {
            ctx.font = subFont;
            totalW += ctx.measureText(seg.sub).width;
        }
    }

    // Baseline alignment: use 'alphabetic' for consistent subscript offset
    ctx.textBaseline = style.baseline || 'middle';

    // Starting x depending on alignment
    const align = style.align || 'center';
    let cursorX;
    if (align === 'left') cursorX = p.x;
    else if (align === 'right') cursorX = p.x - totalW;
    else cursorX = p.x - totalW / 2;

    ctx.fillStyle = style.color || '#222';
    ctx.textAlign = 'left';
    for (const seg of segments) {
        ctx.font = mainFont;
        ctx.fillText(seg.text, cursorX, p.y);
        cursorX += ctx.measureText(seg.text).width;
        if (seg.sub) {
            ctx.font = subFont;
            ctx.fillText(seg.sub, cursorX, p.y + subDy);
            cursorX += ctx.measureText(seg.sub).width;
        }
    }
}

// -----------------------------------------------------------------
// Main redraw (uses state.working + state.selectedCase)
// -----------------------------------------------------------------
function redraw(liveOverrides) {
    clearCanvas();
    if (!state.working) {
        ctx.fillStyle = '#9ca3af';
        ctx.font = '14px -apple-system, Segoe UI, sans-serif';
        ctx.textAlign = 'center';
        ctx.textBaseline = 'middle';
        const rect = canvas.getBoundingClientRect();
        ctx.fillText('Fill in design inputs, then click "Suggest Design".',
            rect.width / 2, rect.height / 2);
        return;
    }
    const d = state.working;
    const k = state.selectedCase;
    const lim = computeLimits(d);
    const { toPx } = makeTransform(lim);

    drawGrid(lim, toPx);

    const rB = d.rB, rA = d.rA_list[k];
    const lA = d.lA, lB = d.lB;
    const D = d.d;
    const phiMean = d.phiCenter[k];
    const phiMin = d.phiMin[k];
    const phiMax = d.phiMax[k];

    // Determine current theta_A & slider position
    // In static mode: pick theta_A from phi_mean
    // In animation mode: theta_A is driven externally; compute slider & phi_B
    let thetaA, xSliderGlobal, phiB;
    if (state.animating && liveOverrides && liveOverrides.thetaA !== undefined) {
        thetaA = liveOverrides.thetaA;
        xSliderGlobal = xSliderFromThetaA(thetaA, D, rA, lA);
        const r = solvePhiBFromSlider(xSliderGlobal, rB, lB);
        phiB = r.solved ? r.phi : phiMean;
    } else {
        phiB = phiMean;
        const xB = sliderPosB(phiB, rB, lB);
        xSliderGlobal = xB;
        const r = solveThetaAFromSlider(xB, D, rA, lA, state.preferredThetaA);
        thetaA = r.solved ? r.theta : NaN;
    }

    // --- Rail
    drawLine(toPx, lim.xMin, 0, lim.xMax, 0,
        { stroke: '#8a93a6', width: 1.4 });

    // --- Hard-stop sector (red) — lower semicircle, from phiStopMin to phiStopMax
    drawSector(toPx, 0, 0, rB,
        state.phiStopMin, state.phiStopMax,
        'rgba(251, 144, 144, 0.28)', true);

    // --- Selected ROM sector (yellow)
    drawSector(toPx, 0, 0, 0.94 * rB,
        phiMin, phiMax,
        'rgba(255, 209, 71, 0.55)', true);

    // --- Hard-stop rays (dashed red)
    const pH1x = rB * cosd(state.phiStopMin);
    const pH1y = -rB * sind(state.phiStopMin);
    const pH2x = rB * cosd(state.phiStopMax);
    const pH2y = -rB * sind(state.phiStopMax);
    drawLine(toPx, 0, 0, pH1x, pH1y,
        { stroke: '#c0392b', width: 1.2, dash: [4, 3] });
    drawLine(toPx, 0, 0, pH2x, pH2y,
        { stroke: '#c0392b', width: 1.2, dash: [4, 3] });

    // --- Crank envelopes (dashed)
    drawCircle(toPx, 0, 0, rB,
        { stroke: '#b5bbc7', width: 1, dash: [4, 3] });
    drawCircle(toPx, D, 0, rA,
        { stroke: '#c9ced9', width: 1, dash: [4, 3] });

    // --- Stroke bar (blue) along the rail
    const xLo = d.strokeLeft[k], xHi = d.strokeRight[k];
    const yScale = Math.max(rA, rB, 1);
    const strokeBarH = 0.10 * yScale;
    const p1 = toPx(xLo, -strokeBarH / 2);
    const p2 = toPx(xHi, strokeBarH / 2);
    ctx.fillStyle = 'rgba(191, 215, 255, 0.85)';
    ctx.fillRect(
        Math.min(p1.x, p2.x),
        Math.min(p1.y, p2.y),
        Math.abs(p2.x - p1.x),
        Math.abs(p2.y - p1.y)
    );

    // Stroke end markers
    drawDot(toPx, xLo, 0, 4, '#1f4bbf', '#fff');
    drawDot(toPx, xHi, 0, 4, '#1f4bbf', '#fff');

    // --- B-crank pin
    const PBx = rB * cosd(phiB);
    const PBy = -rB * sind(phiB);
    // --- A-crank pin
    let PAx = NaN, PAy = NaN;
    if (isFinite(thetaA)) {
        PAx = D + rA * cosd(thetaA);
        PAy = -rA * sind(thetaA);
    }

    // --- B crank (orange) + coupler
    drawLine(toPx, 0, 0, PBx, PBy,
        { stroke: '#e65a1a', width: 4 });
    drawLine(toPx, PBx, PBy, xSliderGlobal, 0,
        { stroke: '#5c5c5c', width: 2 });

    // --- A crank (green) + coupler
    if (isFinite(thetaA)) {
        drawLine(toPx, D, 0, PAx, PAy,
            { stroke: '#27a042', width: 4 });
        drawLine(toPx, PAx, PAy, xSliderGlobal, 0,
            { stroke: '#2e7d32', width: 2 });
    }

    // --- Slider (block) on rail
    const sliderW = 0.12 * yScale;
    const sliderH = 0.12 * yScale;
    const sp1 = toPx(xSliderGlobal - sliderW / 2, sliderH / 2);
    const sp2 = toPx(xSliderGlobal + sliderW / 2, -sliderH / 2);
    ctx.fillStyle = '#2558d6';
    ctx.strokeStyle = '#17378a';
    ctx.lineWidth = 1.2;
    roundRect(sp1.x, sp1.y,
        sp2.x - sp1.x, sp2.y - sp1.y, 4, true, true);

    // --- Pivots & pins
    drawDot(toPx, 0, 0, 5, '#111', '#111');      // O_B
    drawDot(toPx, D, 0, 5, '#111', '#111');      // O_A
    drawDot(toPx, PBx, PBy, 5, '#e65a1a', '#333');
    if (isFinite(thetaA)) drawDot(toPx, PAx, PAy, 5, '#27a042', '#333');

    // --- Labels (with proper subscripts)
    const pivotLabelStyle = { color: '#111', font: 'bold 12px -apple-system, Segoe UI, sans-serif' };
    drawTextSub(toPx, 0, 0.18 * yScale, [{ text: 'O', sub: 'B' }], pivotLabelStyle);
    drawTextSub(toPx, D, 0.18 * yScale, [{ text: 'O', sub: 'A' }], pivotLabelStyle);

    // φ_B label near crank B pin
    drawTextSub(toPx,
        0.65 * rB * cosd(phiB),
        -0.65 * rB * sind(phiB) - 0.10 * yScale,
        [
            { text: 'φ', sub: 'B' },
            { text: ` = ${phiB.toFixed(1)}°` }
        ],
        { color: '#8a3b0e', font: 'bold 12px -apple-system, Segoe UI, sans-serif' });

    // θ_A label near crank A
    if (isFinite(thetaA)) {
        drawTextSub(toPx, D + 0.25 * rA, 0.25 * yScale,
            [
                { text: 'θ', sub: 'A' },
                { text: ` = ${thetaA.toFixed(1)}°` }
            ],
            { color: '#1f7d2a', font: 'bold 12px -apple-system, Segoe UI, sans-serif',
              align: 'left' });
    }

    // Stroke dimension text
    drawText(toPx, 0.5 * (xLo + xHi), -0.22 * yScale,
        `stroke = ${(xHi - xLo).toFixed(3)}`,
        { color: '#1f4bbf', font: 'bold 12px -apple-system, sans-serif' });

    // d dimension line
    const dimY = 0.30 * yScale;
    drawLine(toPx, 0, dimY, D, dimY, { stroke: '#444', width: 1 });
    drawLine(toPx, 0, dimY - 0.04 * yScale, 0, dimY + 0.04 * yScale,
        { stroke: '#444', width: 1 });
    drawLine(toPx, D, dimY - 0.04 * yScale, D, dimY + 0.04 * yScale,
        { stroke: '#444', width: 1 });
    drawText(toPx, D / 2, dimY + 0.07 * yScale, `d = ${D.toFixed(3)}`,
        { color: '#222', font: 'bold 12px -apple-system, sans-serif' });

    // rail length (from rightmost edge of B envelope to O_A)
    const railY = 0.46 * yScale;
    drawLine(toPx, rB, railY, D, railY, { stroke: '#2558d6', width: 1 });
    drawLine(toPx, rB, railY - 0.04 * yScale, rB, railY + 0.04 * yScale,
        { stroke: '#2558d6', width: 1 });
    drawLine(toPx, D, railY - 0.04 * yScale, D, railY + 0.04 * yScale,
        { stroke: '#2558d6', width: 1 });
    drawText(toPx, 0.5 * (rB + D), railY + 0.07 * yScale,
        `rail = ${(D - rB).toFixed(3)}`,
        { color: '#1f4bbf', font: 'bold 12px -apple-system, sans-serif' });

    // Allowable-ROM hard-stop angle labels
    drawTextSub(toPx, 1.10 * pH1x, 1.10 * pH1y,
        [
            { text: 'φ', sub: 'stop,min' },
            { text: ` = ${state.phiStopMin.toFixed(0)}°` }
        ],
        { color: '#a8321f', font: '11px -apple-system, Segoe UI, sans-serif',
          align: 'left' });
    drawTextSub(toPx, 1.10 * pH2x, 1.10 * pH2y,
        [
            { text: 'φ', sub: 'stop,max' },
            { text: ` = ${state.phiStopMax.toFixed(0)}°` }
        ],
        { color: '#a8321f', font: '11px -apple-system, Segoe UI, sans-serif',
          align: 'right' });

    // Live readouts
    $('thetaADisp').textContent = isFinite(thetaA) ? thetaA.toFixed(1) + '°' : '—';
    $('phiBDisp').textContent = isFinite(phiB) ? phiB.toFixed(1) + '°' : '—';
    $('xSliderDisp').textContent = isFinite(xSliderGlobal) ? xSliderGlobal.toFixed(3) : '—';
}

function roundRect(x, y, w, h, r, fill, stroke) {
    if (w < 0) { x += w; w = -w; }
    if (h < 0) { y += h; h = -h; }
    r = Math.min(r, w / 2, h / 2);
    ctx.beginPath();
    ctx.moveTo(x + r, y);
    ctx.arcTo(x + w, y, x + w, y + h, r);
    ctx.arcTo(x + w, y + h, x, y + h, r);
    ctx.arcTo(x, y + h, x, y, r);
    ctx.arcTo(x, y, x + w, y, r);
    ctx.closePath();
    if (fill) ctx.fill();
    if (stroke) ctx.stroke();
}

// -----------------------------------------------------------------
// ANIMATION (2 Hz rotation of crank A)
// -----------------------------------------------------------------
$('playBtn').addEventListener('click', () => {
    if (state.animating) stopAnimation();
    else startAnimation();
});

function startAnimation() {
    if (!state.working) return;
    state.animating = true;
    state.animTheta = 0;
    state.lastT = performance.now();
    $('playBtn').textContent = '⏸ Pause';
    step(state.lastT);
}

function stopAnimation() {
    state.animating = false;
    if (state.reqId) cancelAnimationFrame(state.reqId);
    state.reqId = null;
    $('playBtn').textContent = '▶ Play 2 Hz';
    // Settle back to mean pose
    redraw();
}

function step(now) {
    if (!state.animating) return;
    const dt = (now - state.lastT) / 1000;
    state.lastT = now;
    // 2 Hz = 720 deg/sec
    state.animTheta = (state.animTheta + 720 * dt) % 360;
    redraw({ thetaA: state.animTheta });
    state.reqId = requestAnimationFrame(step);
}

// -----------------------------------------------------------------
// Init
// -----------------------------------------------------------------
enableOverrideInputs(false);
resizeCanvas();
redraw();
