/**
 * ActiveSetQP — Goldfarb-Idnani active-set QP solver for small dense problems.
 *
 * Solves:
 *   min  0.5 * x^T H x + g^T x
 *   s.t. A_eq  x + b_eq  = 0   (meq equality constraints)
 *        A_ineq x + b_ineq >= 0 (inequality constraints)
 *
 * Based on the Goldfarb-Idnani dual method (eiquadprog convention).
 * Designed for robot IK problems with <= ~30 variables and <= ~100 constraints.
 *
 * All matrices are stored as flat Float64Array in row-major order.
 */

export class ActiveSetQP {
    /**
     * Solve the QP problem.
     *
     * @param {Float64Array} H   - (n x n) positive-definite Hessian, row-major
     * @param {Float64Array} g   - (n) linear cost
     * @param {Float64Array|null} Aeq  - (meq x n) equality constraint matrix, row-major
     * @param {Float64Array|null} beq  - (meq) equality rhs
     * @param {Float64Array|null} Ain  - (min x n) inequality constraint matrix, row-major
     * @param {Float64Array|null} bin  - (min) inequality rhs
     * @param {number} n   - number of variables
     * @param {number} meq - number of equality constraints
     * @param {number} min - number of inequality constraints
     * @returns {{ x: Float64Array, cost: number, success: boolean }}
     */
    static solve(H, g, Aeq, beq, Ain, bin, n, meq, min_ineq) {
        const m = meq + min_ineq;

        const L = new Float64Array(n * n);
        if (!choleskyDecomp(H, L, n)) {
            return { x: new Float64Array(n), cost: Infinity, success: false };
        }

        const x = new Float64Array(n);
        solveUnconstrained(L, g, x, n);

        let f0 = 0;
        for (let i = 0; i < n; i++) {
            f0 += 0.5 * g[i] * x[i];
        }

        if (m === 0) {
            return { x, cost: f0, success: true };
        }

        const J = new Float64Array(n * n);
        invertCholesky(L, J, n);

        const activeSet = new Int32Array(m).fill(-1);
        let activeCount = 0;

        const R = new Float64Array(n * n);
        let rSize = 0;

        const np_ = new Float64Array(n);
        const u = new Float64Array(m + 1);
        const z = new Float64Array(n);
        const r = new Float64Array(m + 1);
        const d = new Float64Array(n);

        for (let eqIdx = 0; eqIdx < meq; eqIdx++) {
            for (let i = 0; i < n; i++) {
                np_[i] = Aeq[eqIdx * n + i];
            }

            computeD(J, np_, d, n);
            updateR(R, d, rSize, n);

            const step = computeStepEq(R, r, u, activeCount, rSize, np_, x, n, Aeq, beq, eqIdx);

            if (Math.abs(step.t2) < 1e-15) {
                return { x: new Float64Array(n), cost: Infinity, success: false };
            }

            for (let i = 0; i < n; i++) {
                x[i] += step.t2 * z[i];
            }
            computeZ(J, d, z, n, rSize);

            u[activeCount] = step.t2;
            for (let k = activeCount - 1; k >= 0; k--) {
                u[k] -= step.t2 * r[k];
            }

            activeSet[activeCount] = eqIdx;
            activeCount++;
            rSize++;
        }

        const s = new Float64Array(min_ineq);

        let maxIter = 2 * (m + n);
        let iter = 0;

        while (iter < maxIter) {
            iter++;

            computeSlacks(Ain, bin, x, s, n, min_ineq);

            let worstIdx = -1;
            let worstVal = 0;
            for (let i = 0; i < min_ineq; i++) {
                if (s[i] < worstVal) {
                    let alreadyActive = false;
                    for (let k = 0; k < activeCount; k++) {
                        if (activeSet[k] === meq + i) { alreadyActive = true; break; }
                    }
                    if (!alreadyActive) {
                        worstVal = s[i];
                        worstIdx = i;
                    }
                }
            }

            if (worstIdx === -1 || worstVal > -1e-10) {
                break;
            }

            for (let i = 0; i < n; i++) {
                np_[i] = Ain[worstIdx * n + i];
            }

            computeD(J, np_, d, n);
            computeZ(J, d, z, n, rSize);

            computeR_vec(R, r, d, rSize, n);

            let innerIter = 0;
            while (innerIter < maxIter) {
                innerIter++;

                let t1 = Infinity;
                let dropIdx = -1;
                for (let k = meq; k < activeCount; k++) {
                    if (r[k] > 1e-14) {
                        const ratio = u[k] / r[k];
                        if (ratio < t1) {
                            t1 = ratio;
                            dropIdx = k;
                        }
                    }
                }

                let zn = dotVec(z, z, n);
                let t2;
                if (zn > 1e-14) {
                    t2 = -computeSlack(Ain, bin, x, worstIdx, n) / dotVec(z, np_, n);
                    if (t2 < 0) t2 = Infinity;
                } else {
                    t2 = Infinity;
                }

                if (t1 === Infinity && t2 === Infinity) {
                    return { x: new Float64Array(n), cost: Infinity, success: false };
                }

                let t = Math.min(t1, t2);

                if (t2 <= t1) {
                    for (let i = 0; i < n; i++) x[i] += t * z[i];
                    for (let k = 0; k < activeCount; k++) u[k] -= t * r[k];
                    u[activeCount] = t;

                    activeSet[activeCount] = meq + worstIdx;
                    activeCount++;

                    updateR(R, d, rSize, n);
                    rSize++;
                    break;
                } else {
                    for (let i = 0; i < n; i++) x[i] += t * z[i];
                    for (let k = 0; k < activeCount; k++) u[k] -= t * r[k];
                    u[activeCount] += t;

                    dropConstraint(R, J, activeSet, u, n, rSize, activeCount, dropIdx);
                    activeCount--;
                    rSize--;

                    computeD(J, np_, d, n);
                    computeZ(J, d, z, n, rSize);
                    computeR_vec(R, r, d, rSize, n);
                }
            }
        }

        let cost = 0;
        for (let i = 0; i < n; i++) {
            let Hx_i = 0;
            for (let j = 0; j < n; j++) Hx_i += H[i * n + j] * x[j];
            cost += 0.5 * x[i] * Hx_i + g[i] * x[i];
        }

        return { x, cost, success: true };
    }
}

function choleskyDecomp(A, L, n) {
    L.fill(0);
    for (let i = 0; i < n; i++) {
        for (let j = 0; j <= i; j++) {
            let sum = 0;
            for (let k = 0; k < j; k++) {
                sum += L[i * n + k] * L[j * n + k];
            }
            if (i === j) {
                const diag = A[i * n + i] - sum;
                if (diag <= 1e-14) return false;
                L[i * n + j] = Math.sqrt(diag);
            } else {
                L[i * n + j] = (A[i * n + j] - sum) / L[j * n + j];
            }
        }
    }
    return true;
}

function solveUnconstrained(L, g, x, n) {
    const y = new Float64Array(n);
    for (let i = 0; i < n; i++) {
        let sum = 0;
        for (let k = 0; k < i; k++) sum += L[i * n + k] * y[k];
        y[i] = (g[i] - sum) / L[i * n + i]; // not negated here, we negate below
    }
    for (let i = n - 1; i >= 0; i--) {
        let sum = 0;
        for (let k = i + 1; k < n; k++) sum += L[k * n + i] * x[k];
        x[i] = -(y[i] - sum) / L[i * n + i];
    }
}

function invertCholesky(L, J, n) {
    for (let i = 0; i < n; i++) {
        J[i * n + i] = 1.0 / L[i * n + i];
        for (let j = i + 1; j < n; j++) {
            let sum = 0;
            for (let k = i; k < j; k++) {
                sum += L[j * n + k] * J[k * n + i];
            }
            J[j * n + i] = -sum / L[j * n + j];
        }
    }
}

function computeD(J, np, d, n) {
    for (let i = 0; i < n; i++) {
        d[i] = 0;
        for (let j = i; j < n; j++) {
            d[i] += J[j * n + i] * np[j];
        }
    }
}

function computeZ(J, d, z, n, rSize) {
    for (let i = 0; i < n; i++) {
        z[i] = 0;
        for (let j = rSize; j < n; j++) {
            z[i] += J[j * n + i] * d[j];
        }
    }
}

function updateR(R, d, rSize, n) {
    for (let i = 0; i < rSize; i++) {
        R[i * n + rSize] = d[i];
    }
    R[rSize * n + rSize] = d[rSize];

    for (let i = rSize - 1; i >= 0; i--) {
        const a = R[i * n + rSize];
        const b = d[i + 1];
        if (Math.abs(b) < 1e-14) continue;
        const norm = Math.sqrt(a * a + b * b);
        const c = a / norm;
        const s = b / norm;
        R[i * n + rSize] = norm;
        d[i + 1] = 0;

        for (let j = i + 1; j <= rSize; j++) {
            const ri = R[i * n + j];
            const rj = R[(i + 1) * n + j];
            R[i * n + j] = c * ri + s * rj;
            R[(i + 1) * n + j] = -s * ri + c * rj;
        }
    }
}

function computeR_vec(R, r, d, rSize, n) {
    for (let i = rSize - 1; i >= 0; i--) {
        let sum = d[i];
        for (let k = i + 1; k < rSize; k++) {
            sum -= R[i * n + k] * r[k];
        }
        r[i] = sum / R[i * n + i];
    }
}

function computeSlack(Ain, bin, x, idx, n) {
    let val = bin[idx];
    for (let j = 0; j < n; j++) val += Ain[idx * n + j] * x[j];
    return val;
}

function computeSlacks(Ain, bin, x, s, n, m) {
    for (let i = 0; i < m; i++) {
        s[i] = computeSlack(Ain, bin, x, i, n);
    }
}

function computeStepEq(R, r, u, activeCount, rSize, np, x, n, Aeq, beq, eqIdx) {
    let slack = beq[eqIdx];
    for (let j = 0; j < n; j++) slack += Aeq[eqIdx * n + j] * x[j];
    const t2 = -slack / R[rSize * n + rSize];
    return { t2 };
}

function dropConstraint(R, J, activeSet, u, n, rSize, activeCount, dropIdx) {
    for (let k = dropIdx; k < activeCount - 1; k++) {
        activeSet[k] = activeSet[k + 1];
        u[k] = u[k + 1];
    }
    activeSet[activeCount - 1] = -1;
    u[activeCount - 1] = 0;

    for (let i = dropIdx; i < rSize - 1; i++) {
        const a = R[i * n + i + 1];
        const b = R[(i + 1) * n + i + 1];
        if (Math.abs(b) < 1e-14 && Math.abs(a) < 1e-14) continue;
        const norm = Math.sqrt(a * a + b * b);
        const c = a / norm;
        const s = b / norm;

        for (let j = i + 1; j < rSize; j++) {
            const ri = R[i * n + j];
            const rj = R[(i + 1) * n + j];
            R[i * n + j] = c * ri + s * rj;
            R[(i + 1) * n + j] = -s * ri + c * rj;
        }

        for (let j = 0; j < n; j++) {
            const ji = J[i * n + j];
            const jj = J[(i + 1) * n + j];
            J[i * n + j] = c * ji + s * jj;
            J[(i + 1) * n + j] = -s * ji + c * jj;
        }
    }

    for (let i = dropIdx; i < rSize - 1; i++) {
        for (let j = 0; j < rSize - 1; j++) {
            R[i * n + j] = R[(i + 1) * n + j];
        }
    }
}

function dotVec(a, b, n) {
    let sum = 0;
    for (let i = 0; i < n; i++) sum += a[i] * b[i];
    return sum;
}
