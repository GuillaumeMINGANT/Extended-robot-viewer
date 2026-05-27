/**
 * JacobianIkSolver — Damped Least-Squares pseudo-inverse IK with
 * null-space limit avoidance and joint locking at boundaries.
 *
 * Pure math module: reads model state via RobotKinematics, returns
 * Δq deltas. Never writes joint state directly.
 */

const DEFAULT_OPTIONS = {
    maxIterations: 50,
    tolerance: 1e-3,
    stepSize: 1.0,
    lambdaMax: 0.05,
    manipulabilityThreshold: 0.04,
    nullSpaceGain: 0.3,
    limitMarginRatio: 0.25,
    maxJointDelta: 0.15
};

export class JacobianIkSolver {
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
     *
     * @param {string} tipName
     * @param {THREE.Vector3} targetPos
     * @param {THREE.Quaternion} [targetQuat]
     * @returns {{ deltas: { name: string, delta: number }[], converged: boolean, iterations: number } | null}
     */
    solve(tipName, targetPos, targetQuat = null) {
        const chain = this.kinematics.getChain(tipName);
        if (!chain) return null;

        const { joints } = chain;
        const nj = joints.length;
        if (nj === 0) return null;

        const { maxIterations, tolerance, stepSize, lambdaMax, manipulabilityThreshold } = this.options;

        let converged = false;
        let iterations = 0;

        for (let iter = 0; iter < maxIterations; iter++) {
            iterations++;

            const errResult = this.kinematics.computePoseError(tipName, targetPos, targetQuat);
            if (!errResult) return null;

            const { error, dim } = errResult;
            const errNorm = vecNorm(error);

            if (errNorm < tolerance) {
                converged = true;
                break;
            }

            const jacResult = this.kinematics.computeJacobian(tipName, dim);
            if (!jacResult) return null;

            const { J, rows, cols } = jacResult;

            const lockedColumns = this._detectLockedJoints(joints, J, error, rows, cols);

            const lambda = this._computeAdaptiveLambda(J, rows, cols, lambdaMax, manipulabilityThreshold);

            const dq = this._computeDq(J, error, rows, cols, lambda, lockedColumns);

            const nullSpaceDq = this._computeNullSpaceLimitAvoidance(J, dq, joints, rows, cols, lockedColumns);

            // Clamp maximum joint delta to prevent explosive steps near singularities
            const { maxJointDelta } = this.options;
            let maxAbs = 0;
            for (let i = 0; i < nj; i++) {
                if (lockedColumns[i]) continue;
                const raw = Math.abs(stepSize * dq[i] + nullSpaceDq[i]);
                if (raw > maxAbs) maxAbs = raw;
            }
            const scale = (maxAbs > maxJointDelta) ? maxJointDelta / maxAbs : 1;

            const deltas = [];
            for (let i = 0; i < nj; i++) {
                if (lockedColumns[i]) continue;
                const delta = scale * (stepSize * dq[i] + nullSpaceDq[i]);
                deltas.push({ name: joints[i].name, delta });
            }

            if (deltas.length === 0) break;

            return { deltas, converged: false, iterations };
        }

        return { deltas: [], converged, iterations };
    }

    /**
     * Iterative multi-step solve. Applies deltas externally between steps.
     * Returns the final result after convergence or max iterations.
     *
     * @param {string} tipName
     * @param {THREE.Vector3} targetPos
     * @param {THREE.Quaternion} [targetQuat]
     * @param {(deltas: {name: string, delta: number}[]) => void} applyFn
     * @returns {{ converged: boolean, iterations: number }}
     */
    solveIterative(tipName, targetPos, targetQuat, applyFn) {
        const { maxIterations, tolerance } = this.options;
        let totalIter = 0;

        for (let iter = 0; iter < maxIterations; iter++) {
            totalIter++;

            const errResult = this.kinematics.computePoseError(tipName, targetPos, targetQuat);
            if (!errResult) return { converged: false, iterations: totalIter };

            if (vecNorm(errResult.error) < tolerance) {
                return { converged: true, iterations: totalIter };
            }

            const result = this._singleStep(tipName, targetPos, targetQuat);
            if (!result || result.deltas.length === 0) break;

            applyFn(result.deltas);
        }

        const finalErr = this.kinematics.computePoseError(tipName, targetPos, targetQuat);
        const converged = finalErr ? vecNorm(finalErr.error) < tolerance : false;
        return { converged, iterations: totalIter };
    }

    _singleStep(tipName, targetPos, targetQuat) {
        const chain = this.kinematics.getChain(tipName);
        if (!chain) return null;

        const { joints } = chain;
        const nj = joints.length;

        const errResult = this.kinematics.computePoseError(tipName, targetPos, targetQuat);
        if (!errResult) return null;

        const { error, dim } = errResult;
        const jacResult = this.kinematics.computeJacobian(tipName);
        if (!jacResult) return null;

        const { J, rows, cols } = jacResult;
        const { stepSize, lambdaMax, manipulabilityThreshold } = this.options;

        const lockedColumns = this._detectLockedJoints(joints, J, error, rows, cols);
        const lambda = this._computeAdaptiveLambda(J, rows, cols, lambdaMax, manipulabilityThreshold);
        const dq = this._computeDq(J, error, rows, cols, lambda, lockedColumns);
        const nullSpaceDq = this._computeNullSpaceLimitAvoidance(J, dq, joints, rows, cols, lockedColumns);

        const { maxJointDelta } = this.options;
        let maxAbs = 0;
        for (let i = 0; i < nj; i++) {
            if (lockedColumns[i]) continue;
            const raw = Math.abs(stepSize * dq[i] + nullSpaceDq[i]);
            if (raw > maxAbs) maxAbs = raw;
        }
        const scale = (maxAbs > maxJointDelta) ? maxJointDelta / maxAbs : 1;

        const deltas = [];
        for (let i = 0; i < nj; i++) {
            if (lockedColumns[i]) continue;
            deltas.push({ name: joints[i].name, delta: scale * (stepSize * dq[i] + nullSpaceDq[i]) });
        }

        return { deltas, converged: false, iterations: 1 };
    }

    /**
     * Lock joints that are at their limit and the Jacobian gradient pushes further in.
     */
    _detectLockedJoints(joints, J, error, rows, cols) {
        const locked = new Array(cols).fill(false);

        for (let i = 0; i < cols; i++) {
            const joint = joints[i];
            if (!joint.limits) continue;

            const q = joint.currentValue;
            const lo = joint.limits.lower;
            const hi = joint.limits.upper;
            const range = hi - lo;
            if (range < 1e-8) { locked[i] = true; continue; }

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
     * Adaptive damping (Nakamura & Hanafusa):
     * w = sqrt(det(J Jᵀ))
     * λ = λ_max * (1 - (w/threshold)²) if w < threshold, else 0
     */
    _computeAdaptiveLambda(J, rows, cols, lambdaMax, threshold) {
        const JJt = matMulABt(J, J, rows, cols, rows);
        const det = matDet(JJt, rows);
        const w = Math.sqrt(Math.max(0, det));

        if (w >= threshold) return 1e-4;
        const ratio = w / threshold;
        return lambdaMax * (1 - ratio * ratio) + 1e-4;
    }

    /**
     * Compute Δq = Jᵀ (J Jᵀ + λ² I)⁻¹ e
     * (Right pseudo-inverse / damped least squares)
     */
    _computeDq(J, error, rows, cols, lambda, lockedColumns) {
        const Jw = new Float64Array(rows * cols);
        for (let r = 0; r < rows; r++) {
            for (let c = 0; c < cols; c++) {
                Jw[r * cols + c] = lockedColumns[c] ? 0 : J[r * cols + c];
            }
        }

        const JJt = matMulABt(Jw, Jw, rows, cols, rows);

        for (let i = 0; i < rows; i++) {
            JJt[i * rows + i] += lambda * lambda;
        }

        const JJtInv = matInverse(JJt, rows);
        if (!JJtInv) {
            return new Float64Array(cols);
        }

        const JJtInvE = matVecMul(JJtInv, error, rows, rows);

        const dq = new Float64Array(cols);
        for (let c = 0; c < cols; c++) {
            if (lockedColumns[c]) continue;
            let sum = 0;
            for (let r = 0; r < rows; r++) {
                sum += Jw[r * cols + c] * JJtInvE[r];
            }
            dq[c] = sum;
        }

        return dq;
    }

    /**
     * Null-space limit avoidance: push joints away from limits
     * in the outer margin zone, projected into the null space of J.
     */
    _computeNullSpaceLimitAvoidance(J, dq, joints, rows, cols, lockedColumns) {
        const { nullSpaceGain, limitMarginRatio } = this.options;
        const nj = cols;

        if (nj <= rows) return new Float64Array(nj);

        const gradient = new Float64Array(nj);
        for (let i = 0; i < nj; i++) {
            if (lockedColumns[i]) continue;
            const joint = joints[i];
            if (!joint.limits) continue;

            const q = joint.currentValue;
            const lo = joint.limits.lower;
            const hi = joint.limits.upper;
            const range = hi - lo;
            if (range < 1e-8) continue;

            const margin = range * limitMarginRatio;
            const midpoint = (lo + hi) / 2;

            if (q < lo + margin) {
                gradient[i] = (lo + margin - q) / margin;
            } else if (q > hi - margin) {
                gradient[i] = (hi - margin - q) / margin;
            }
        }

        const Jw = new Float64Array(rows * nj);
        for (let r = 0; r < rows; r++) {
            for (let c = 0; c < nj; c++) {
                Jw[r * nj + c] = lockedColumns[c] ? 0 : J[r * nj + c];
            }
        }

        const JJt = matMulABt(Jw, Jw, rows, nj, rows);
        for (let i = 0; i < rows; i++) JJt[i * rows + i] += 1e-4;
        const JJtInv = matInverse(JJt, rows);
        if (!JJtInv) return new Float64Array(nj);

        const pinv = new Float64Array(nj * rows);
        for (let c = 0; c < nj; c++) {
            for (let r = 0; r < rows; r++) {
                let sum = 0;
                for (let k = 0; k < rows; k++) {
                    sum += Jw[k * nj + c] * JJtInv[k * rows + r];
                }
                pinv[c * rows + r] = sum;
            }
        }

        const pinvJ = new Float64Array(nj * nj);
        for (let i = 0; i < nj; i++) {
            for (let j = 0; j < nj; j++) {
                let sum = 0;
                for (let k = 0; k < rows; k++) {
                    sum += pinv[i * rows + k] * Jw[k * nj + j];
                }
                pinvJ[i * nj + j] = sum;
            }
        }

        const result = new Float64Array(nj);
        for (let i = 0; i < nj; i++) {
            let sum = 0;
            for (let j = 0; j < nj; j++) {
                const Nij = (i === j ? 1 : 0) - pinvJ[i * nj + j];
                sum += Nij * gradient[j];
            }
            result[i] = nullSpaceGain * sum;
        }

        return result;
    }
}

function vecNorm(v) {
    let sum = 0;
    for (let i = 0; i < v.length; i++) sum += v[i] * v[i];
    return Math.sqrt(sum);
}

function matMulABt(A, B, rowsA, colsA, rowsB) {
    const C = new Float64Array(rowsA * rowsB);
    for (let i = 0; i < rowsA; i++) {
        for (let j = 0; j < rowsB; j++) {
            let sum = 0;
            for (let k = 0; k < colsA; k++) {
                sum += A[i * colsA + k] * B[j * colsA + k];
            }
            C[i * rowsB + j] = sum;
        }
    }
    return C;
}

function matVecMul(M, v, rows, cols) {
    const result = new Float64Array(rows);
    for (let i = 0; i < rows; i++) {
        let sum = 0;
        for (let j = 0; j < cols; j++) {
            sum += M[i * cols + j] * v[j];
        }
        result[i] = sum;
    }
    return result;
}

/**
 * In-place Gauss-Jordan matrix inverse for small dense matrices (≤ 6×6).
 */
function matInverse(M, n) {
    const aug = new Float64Array(n * 2 * n);
    for (let i = 0; i < n; i++) {
        for (let j = 0; j < n; j++) {
            aug[i * 2 * n + j] = M[i * n + j];
        }
        aug[i * 2 * n + (n + i)] = 1;
    }

    for (let col = 0; col < n; col++) {
        let maxRow = col;
        let maxVal = Math.abs(aug[col * 2 * n + col]);
        for (let row = col + 1; row < n; row++) {
            const v = Math.abs(aug[row * 2 * n + col]);
            if (v > maxVal) { maxVal = v; maxRow = row; }
        }

        if (maxVal < 1e-12) return null;

        if (maxRow !== col) {
            for (let k = 0; k < 2 * n; k++) {
                const tmp = aug[col * 2 * n + k];
                aug[col * 2 * n + k] = aug[maxRow * 2 * n + k];
                aug[maxRow * 2 * n + k] = tmp;
            }
        }

        const pivot = aug[col * 2 * n + col];
        for (let k = 0; k < 2 * n; k++) {
            aug[col * 2 * n + k] /= pivot;
        }

        for (let row = 0; row < n; row++) {
            if (row === col) continue;
            const factor = aug[row * 2 * n + col];
            for (let k = 0; k < 2 * n; k++) {
                aug[row * 2 * n + k] -= factor * aug[col * 2 * n + k];
            }
        }
    }

    const inv = new Float64Array(n * n);
    for (let i = 0; i < n; i++) {
        for (let j = 0; j < n; j++) {
            inv[i * n + j] = aug[i * 2 * n + (n + j)];
        }
    }
    return inv;
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
