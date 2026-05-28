/**
 * QPIkSolver — QP-based task-space IK solver faithfully porting PLACO's approach.
 *
 * Formulates IK as a constrained Quadratic Program:
 *   min  0.5 * dq^T * H * dq + g^T * dq
 *   s.t. lower <= dq <= upper   (joint limits)
 *
 * H and g encode weighted position + orientation tasks plus regularization.
 *
 * Design matching PLACO (placo/problem/problem.cpp, placo/kinematics/):
 *   - Tasks assembled as soft equality: min w * ||A*dq - b||^2
 *   - Regularization = baseline + adaptive (Nakamura–Hanafusa manipulability)
 *   - Joint pre-locking at limits (matches DLS _detectLockedJoints)
 *   - Post-solve null-space posture correction (matches DLS approach)
 *   - Uniform step scaling (matches DLS maxJointDelta)
 *   - Box-constrained QP solved via iterative clamping
 */

const DEFAULT_OPTIONS = {
    maxIterations: 50,
    tolerance: 1e-3,
    positionWeight: 1.0,
    orientationWeight: 1.0,
    regularizationBaseline: 1e-8,
    lambdaMax: 0.05,
    manipulabilityThreshold: 0.04,
    maxStepPerJoint: 0.15,
    nullSpaceGain: 0.05,
    limitMarginRatio: 0.25,
    stepSize: 1.0
};

export class QPIkSolver {
    /**
     * @param {import('./RobotKinematics.js').RobotKinematics} kinematics
     * @param {object} [options]
     */
    constructor(kinematics, options = {}) {
        this.kinematics = kinematics;
        this.options = { ...DEFAULT_OPTIONS, ...options };
    }

    /**
     * Solve IK for a single tip link. Returns joint angle deltas.
     * Same interface as JacobianIkSolver.solve().
     */
    solve(tipName, targetPos, targetQuat = null) {
        const chain = this.kinematics.getChain(tipName);
        if (!chain) return null;

        const { joints } = chain;
        const nj = joints.length;
        if (nj === 0) return null;

        const { tolerance, stepSize, positionWeight, orientationWeight,
                regularizationBaseline, lambdaMax, manipulabilityThreshold,
                maxStepPerJoint, nullSpaceGain, limitMarginRatio } = this.options;

        const errResult = this.kinematics.computePoseError(tipName, targetPos, targetQuat);
        if (!errResult) return null;

        const { error, dim } = errResult;
        const errNorm = vecNorm(error);

        if (errNorm < tolerance) {
            return { deltas: [], converged: true, iterations: 1 };
        }

        const jacResult = this.kinematics.computeJacobian(tipName, dim);
        if (!jacResult) return null;

        const { J, rows, cols } = jacResult;

        // --- Joint pre-locking (matches DLS _detectLockedJoints) ---
        const locked = detectLockedJoints(joints, J, error, rows, cols);

        const lambda = computeAdaptiveLambda(
            J, rows, cols, lambdaMax, manipulabilityThreshold, locked
        );
        const reg = lambda * lambda + regularizationBaseline;

        // --- Build QP: min 0.5 * dq^T * H * dq + g^T * dq ---
        const H = new Float64Array(nj * nj);
        const g = new Float64Array(nj);

        for (let i = 0; i < nj; i++) {
            H[i * nj + i] = reg;
        }

        if (targetQuat != null && dim === 6) {
            addTask(H, g, J, error, nj, cols, 0, 3, positionWeight, locked);
            addTask(H, g, J, error, nj, cols, 3, 6, orientationWeight, locked);
        } else {
            addTask(H, g, J, error, nj, cols, 0, Math.min(3, rows), positionWeight, locked);
        }

        // --- Box constraints from joint limits ---
        const lower = new Float64Array(nj);
        const upper = new Float64Array(nj);
        for (let i = 0; i < nj; i++) {
            if (locked[i]) {
                lower[i] = 0;
                upper[i] = 0;
                continue;
            }
            const joint = joints[i];
            const q = joint.currentValue;
            if (joint.limits) {
                lower[i] = joint.limits.lower - q;
                upper[i] = joint.limits.upper - q;
            } else {
                lower[i] = -1e10;
                upper[i] = 1e10;
            }
        }

        // --- Solve box-constrained QP ---
        const dq = solveBoxQP(H, g, nj, lower, upper);

        // --- Post-solve null-space posture correction ---
        // Compute posture push into null-space BEFORE step limiting,
        // then limit the TOTAL (task + posture) uniformly — matching DLS.
        const nsDq = new Float64Array(nj);
        if (nj > rows) {
            computeNullSpacePosture(nsDq, J, joints, rows, cols, locked,
                                   nullSpaceGain, limitMarginRatio, lambda);
        }

        // --- Uniform step limiting on total step (matches DLS exactly) ---
        let maxAbs = 0;
        for (let i = 0; i < nj; i++) {
            if (locked[i]) continue;
            const v = Math.abs(stepSize * dq[i] + nsDq[i]);
            if (v > maxAbs) maxAbs = v;
        }
        const scale = (maxAbs > maxStepPerJoint)
            ? maxStepPerJoint / maxAbs
            : 1;

        const deltas = [];
        for (let i = 0; i < nj; i++) {
            if (locked[i]) continue;
            const delta = scale * (stepSize * dq[i] + nsDq[i]);
            if (Math.abs(delta) > 1e-10) {
                deltas.push({ name: joints[i].name, delta });
            }
        }

        if (deltas.length === 0) {
            return { deltas: [], converged: errNorm < tolerance, iterations: 1 };
        }

        return { deltas, converged: false, iterations: 1 };
    }
}

/**
 * Detect joints at their limit with gradient pushing further into the limit.
 */
function detectLockedJoints(joints, J, error, rows, cols) {
    const locked = new Array(cols).fill(false);

    for (let i = 0; i < cols; i++) {
        const joint = joints[i];
        if (!joint.limits) continue;

        const q = joint.currentValue;
        const lo = joint.limits.lower;
        const hi = joint.limits.upper;
        if (hi - lo < 1e-8) { locked[i] = true; continue; }

        const atLower = q <= lo + 1e-6;
        const atUpper = q >= hi - 1e-6;
        if (!atLower && !atUpper) continue;

        let gradient = 0;
        for (let r = 0; r < rows; r++) {
            gradient += J[r * cols + i] * error[r];
        }

        if (atLower && gradient < 0) locked[i] = true;
        if (atUpper && gradient > 0) locked[i] = true;
    }

    return locked;
}

/**
 * Adaptive damping (Nakamura & Hanafusa).
 * Locked columns are excluded from manipulability computation.
 */
function computeAdaptiveLambda(J, rows, cols, lambdaMax, threshold, locked) {
    const JJt = new Float64Array(rows * rows);
    for (let i = 0; i < rows; i++) {
        for (let j = 0; j < rows; j++) {
            let sum = 0;
            for (let k = 0; k < cols; k++) {
                if (locked[k]) continue;
                sum += J[i * cols + k] * J[j * cols + k];
            }
            JJt[i * rows + j] = sum;
        }
    }

    const det = matDet(JJt, rows);
    const w = Math.sqrt(Math.max(0, det));

    if (w >= threshold) return 1e-4;
    const ratio = w / threshold;
    return lambdaMax * (1 - ratio * ratio) + 1e-4;
}

function matDet(M, n) {
    if (n === 1) return M[0];
    if (n === 2) return M[0] * M[3] - M[1] * M[2];
    if (n === 3) {
        return M[0] * (M[4] * M[8] - M[5] * M[7])
             - M[1] * (M[3] * M[8] - M[5] * M[6])
             + M[2] * (M[3] * M[7] - M[4] * M[6]);
    }
    const work = new Float64Array(n * n);
    work.set(M);
    let det = 1;
    for (let col = 0; col < n; col++) {
        let maxRow = col;
        let maxVal = Math.abs(work[col * n + col]);
        for (let row = col + 1; row < n; row++) {
            const v = Math.abs(work[row * n + col]);
            if (v > maxVal) { maxVal = v; maxRow = row; }
        }
        if (maxVal < 1e-12) return 0;
        if (maxRow !== col) {
            for (let k = 0; k < n; k++) {
                const tmp = work[col * n + k];
                work[col * n + k] = work[maxRow * n + k];
                work[maxRow * n + k] = tmp;
            }
            det = -det;
        }
        det *= work[col * n + col];
        const pivot = work[col * n + col];
        for (let row = col + 1; row < n; row++) {
            const factor = work[row * n + col] / pivot;
            for (let k = col; k < n; k++) {
                work[row * n + k] -= factor * work[col * n + k];
            }
        }
    }
    return det;
}

/**
 * Add a task: min w * ||J_task * dq - e_task||^2
 * Locked joints are excluded from the task assembly.
 */
function addTask(H, g, J, error, nj, cols, rowStart, rowEnd, weight, locked) {
    const taskRows = rowEnd - rowStart;
    if (taskRows <= 0) return;

    for (let i = 0; i < nj; i++) {
        if (locked[i]) continue;
        for (let j = 0; j < nj; j++) {
            if (locked[j]) continue;
            let sum = 0;
            for (let r = rowStart; r < rowEnd; r++) {
                sum += J[r * cols + i] * J[r * cols + j];
            }
            H[i * nj + j] += weight * sum;
        }
        let ge = 0;
        for (let r = rowStart; r < rowEnd; r++) {
            ge += J[r * cols + i] * error[r];
        }
        g[i] += -weight * ge;
    }
}

/**
 * Post-solve null-space posture correction.
 * Same algorithm as DLS's _computeNullSpaceLimitAvoidance:
 *   1. Compute limit-avoidance gradient (push away from limits in margin zone)
 *   2. Project into null space of J via N = I - J^T (J J^T + eps I)^-1 J
 *   3. Add to dq with gain factor
 *
 * Writes result into the output array.
 */
function computeNullSpacePosture(out, J, joints, rows, cols, locked, gain, marginRatio, lambda) {
    const nj = cols;

    const gradient = new Float64Array(nj);
    let hasGradient = false;
    for (let i = 0; i < nj; i++) {
        if (locked[i]) continue;
        const joint = joints[i];
        if (!joint.limits) continue;

        const q = joint.currentValue;
        const lo = joint.limits.lower;
        const hi = joint.limits.upper;
        const range = hi - lo;
        if (range < 1e-8) continue;

        const margin = range * marginRatio;

        if (q < lo + margin) {
            gradient[i] = (lo + margin - q) / margin;
            hasGradient = true;
        } else if (q > hi - margin) {
            gradient[i] = (hi - margin - q) / margin;
            hasGradient = true;
        }
    }

    if (!hasGradient) return;

    // Build J_working with locked columns zeroed
    // Compute JJt = J * J^T (rows x rows)
    const JJt = new Float64Array(rows * rows);
    for (let i = 0; i < rows; i++) {
        for (let j = 0; j < rows; j++) {
            let sum = 0;
            for (let k = 0; k < nj; k++) {
                if (locked[k]) continue;
                sum += J[i * nj + k] * J[j * nj + k];
            }
            JJt[i * rows + j] = sum;
        }
    }
    const eps = lambda * lambda + 1e-4;
    for (let i = 0; i < rows; i++) JJt[i * rows + i] += eps;

    // Solve JJt * y = J * gradient  (reuse Cholesky)
    const Jg = new Float64Array(rows);
    for (let r = 0; r < rows; r++) {
        let sum = 0;
        for (let k = 0; k < nj; k++) {
            if (locked[k]) continue;
            sum += J[r * nj + k] * gradient[k];
        }
        Jg[r] = sum;
    }

    const y = choleskySolve(JJt, negateVec(Jg), rows);

    // result = gain * (gradient - J^T * y) = gain * N * gradient
    for (let i = 0; i < nj; i++) {
        if (locked[i]) continue;
        let JtY = 0;
        for (let r = 0; r < rows; r++) {
            JtY += J[r * nj + i] * y[r];
        }
        out[i] = gain * (gradient[i] - JtY);
    }
}

function negateVec(v) {
    const r = new Float64Array(v.length);
    for (let i = 0; i < v.length; i++) r[i] = -v[i];
    return r;
}

/**
 * Solve a box-constrained QP via iterative clamping.
 */
function solveBoxQP(H, g, n, lower, upper) {
    const x = new Float64Array(n);
    const clamped = new Uint8Array(n);

    for (let outerIter = 0; outerIter < n + 1; outerIter++) {
        const free = [];
        for (let i = 0; i < n; i++) {
            if (!clamped[i]) free.push(i);
        }
        if (free.length === 0) break;

        const nf = free.length;
        const Hf = new Float64Array(nf * nf);
        const gf = new Float64Array(nf);

        for (let i = 0; i < nf; i++) {
            const fi = free[i];
            gf[i] = g[fi];
            for (let j = 0; j < nf; j++) {
                Hf[i * nf + j] = H[fi * n + free[j]];
            }
            for (let k = 0; k < n; k++) {
                if (clamped[k]) {
                    gf[i] += H[fi * n + k] * x[k];
                }
            }
        }

        const xf = choleskySolve(Hf, gf, nf);

        let newClamp = false;
        for (let i = 0; i < nf; i++) {
            const fi = free[i];
            if (xf[i] < lower[fi]) {
                x[fi] = lower[fi];
                clamped[fi] = 1;
                newClamp = true;
            } else if (xf[i] > upper[fi]) {
                x[fi] = upper[fi];
                clamped[fi] = 2;
                newClamp = true;
            } else {
                x[fi] = xf[i];
            }
        }

        if (!newClamp) {
            let released = false;
            for (let i = 0; i < n; i++) {
                if (!clamped[i]) continue;
                let grad = g[i];
                for (let j = 0; j < n; j++) grad += H[i * n + j] * x[j];

                if (clamped[i] === 1 && grad < -1e-10) {
                    clamped[i] = 0;
                    released = true;
                } else if (clamped[i] === 2 && grad > 1e-10) {
                    clamped[i] = 0;
                    released = true;
                }
            }
            if (!released) break;
        }
    }

    return x;
}

/**
 * Solve H * x = -g via modified Cholesky (diagonal perturbation for robustness).
 */
function choleskySolve(H, g, n) {
    const L = new Float64Array(n * n);
    const FLOOR = 1e-10;

    for (let i = 0; i < n; i++) {
        for (let j = 0; j <= i; j++) {
            let sum = 0;
            for (let k = 0; k < j; k++) {
                sum += L[i * n + k] * L[j * n + k];
            }
            if (i === j) {
                let diag = H[i * n + i] - sum;
                if (diag < FLOOR) diag = FLOOR;
                L[i * n + j] = Math.sqrt(diag);
            } else {
                L[i * n + j] = (H[i * n + j] - sum) / L[j * n + j];
            }
        }
    }

    const y = new Float64Array(n);
    for (let i = 0; i < n; i++) {
        let sum = 0;
        for (let k = 0; k < i; k++) sum += L[i * n + k] * y[k];
        y[i] = (-g[i] - sum) / L[i * n + i];
    }

    const x = new Float64Array(n);
    for (let i = n - 1; i >= 0; i--) {
        let sum = 0;
        for (let k = i + 1; k < n; k++) sum += L[k * n + i] * x[k];
        x[i] = (y[i] - sum) / L[i * n + i];
    }

    return x;
}

function vecNorm(v) {
    let sum = 0;
    for (let i = 0; i < v.length; i++) sum += v[i] * v[i];
    return Math.sqrt(sum);
}
