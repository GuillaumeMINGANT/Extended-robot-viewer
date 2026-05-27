/**
 * JointPoseAnimator — smooth joint-space interpolation for Home/Random poses.
 *
 * Uses manual easing (no external tween library needed); integrates with
 * requestAnimationFrame-based render loops.
 *
 * Single write path: calls a user-provided `applyJoint(name, value)` callback.
 */

const EASE_QUAD_OUT = t => t * (2 - t);

export class JointPoseAnimator {
    /**
     * @param {{
     *   applyJoint: (name: string, value: number) => void,
     *   duration?: number
     * }} options
     */
    constructor(options) {
        this.applyJoint = options.applyJoint;
        this.duration = options.duration ?? 600;
        this._active = false;
        this._startTime = 0;
        this._startConfig = null;
        this._targetConfig = null;
        this._onComplete = null;
    }

    get isAnimating() {
        return this._active;
    }

    /**
     * Animate from the current configuration to a target.
     *
     * @param {{ name: string, value: number }[]} currentConfig
     * @param {{ name: string, value: number }[]} targetConfig
     * @param {{ duration?: number, onComplete?: () => void }} [opts]
     */
    animateTo(currentConfig, targetConfig, opts = {}) {
        if (currentConfig.length !== targetConfig.length) {
            console.warn('[JointPoseAnimator] Config length mismatch');
            return;
        }

        this._startConfig = currentConfig.map(c => ({ ...c }));
        this._targetConfig = targetConfig.map(c => ({ ...c }));
        this.duration = opts.duration ?? this.duration;
        this._onComplete = opts.onComplete ?? null;
        this._startTime = performance.now();
        this._active = true;
    }

    /**
     * Instantly set a target pose without animation.
     * @param {{ name: string, value: number }[]} config
     */
    setImmediate(config) {
        this.cancel();
        for (const { name, value } of config) {
            this.applyJoint(name, value);
        }
    }

    /**
     * Call once per frame (inside requestAnimationFrame loop).
     * Returns true if still animating.
     */
    update() {
        if (!this._active) return false;

        const elapsed = performance.now() - this._startTime;
        let t = Math.min(1, elapsed / this.duration);
        t = EASE_QUAD_OUT(t);

        for (let i = 0; i < this._startConfig.length; i++) {
            const start = this._startConfig[i].value;
            const end = this._targetConfig[i].value;
            const name = this._startConfig[i].name;
            const interpolated = start + (end - start) * t;
            this.applyJoint(name, interpolated);
        }

        if (t >= 1) {
            this._active = false;
            this._onComplete?.();
        }

        return this._active;
    }

    /**
     * Cancel any in-progress animation without snapping to target.
     */
    cancel() {
        this._active = false;
        this._onComplete = null;
    }
}
