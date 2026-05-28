/**
 * IkController — orchestrator for IK gizmos, solver loop, reachability, and animations.
 *
 * Architecture:
 *   - Single write path via `ModelLoaderFactory.setJointAngle(model, name, angle)`
 *   - TransformControls gizmos attached per tip chain
 *   - Reachability point clouds rendered as THREE.Points per tip
 *   - Home/Random smooth transitions via JointPoseAnimator
 *   - Never touches joint sliders directly; emits events for UI sync
 */
import * as THREE from 'three';
import { TransformControls } from 'three/examples/jsm/controls/TransformControls.js';
import { ModelLoaderFactory } from '../loaders/ModelLoaderFactory.js';
import { RobotKinematics } from '../kinematics/RobotKinematics.js';
import { JacobianIkSolver } from '../kinematics/JacobianIkSolver.js';
import { QPIkSolver } from '../kinematics/QPIkSolver.js';
import { JointPoseAnimator } from '../kinematics/JointPoseAnimator.js';
import { ReachabilitySampler } from '../kinematics/ReachabilitySampler.js';
import { HumanoidKinematicsAnalyzer, classifyCategories } from '../kinematics/HumanoidKinematicsAnalyzer.js';

const TIP_COLORS = [
    0x22d3ee, // cyan
    0xf97316, // orange
    0xa78bfa, // purple
    0x4ade80, // green
    0xfbbf24, // amber
    0xf472b6, // pink
    0x38bdf8, // sky
    0xe879f9, // fuchsia
];

const IK_ITERATIONS_PER_FRAME = 40;

export class IkController {
    /**
     * @param {{
     *   sceneManager: import('../renderer/SceneManager.js').SceneManager,
     *   onJointChanged?: (name: string, value: number) => void,
     *   onAnimationTick?: () => void
     * }} options
     */
    constructor(options) {
        this.sceneManager = options.sceneManager;
        this.onJointChanged = options.onJointChanged ?? (() => {});
        this.onAnimationTick = options.onAnimationTick ?? (() => {});
        this.onSolverChanged = options.onSolverChanged ?? (() => {});

        this._model = null;
        this._kinematics = null;
        this._solver = null;
        this._qpSolver = null;
        this._sampler = null;
        this._animator = null;
        this._enabled = false;
        this._solverType = 'jacobian';

        this._gizmos = new Map();
        this._tipVisibility = new Map();
        this._tipLabels = new Map();
        this._reachClouds = new Map();
        this._reachVisible = false;
        this._reachSamples = 5000;

        this._dragging = false;
        this._activeTip = null;
        this._activeMode = null; // 'translate' or 'rotate'
        this._gizmoSpace = 'local'; // 'local' or 'world'
        this._translateEnabled = true;
        this._rotateEnabled = true;
        this._lockLinks = false;

        this._onKeyDown = this._handleKeyDown.bind(this);
    }

    get enabled() { return this._enabled; }
    get model() { return this._model; }
    get kinematics() { return this._kinematics; }
    get tipLinks() { return this._kinematics?.tipLinks ?? []; }
    get isAnimating() { return this._animator?.isAnimating ?? false; }
    get translateEnabled() { return this._translateEnabled; }
    get rotateEnabled() { return this._rotateEnabled; }
    get lockLinks() { return this._lockLinks; }

    get solverType() { return this._solverType; }
    get qpAvailable() { return this._qpSolver != null; }

    /**
     * Switch active solver: 'jacobian' (DLS) or 'qp' (QP-based).
     * @param {'jacobian'|'qp'} type
     */
    setSolverType(type) {
        const prev = this._solverType;
        if (type === 'qp' && this._qpSolver) {
            this._solverType = 'qp';
        } else {
            this._solverType = 'jacobian';
        }
        if (this._solverType !== prev) this.onSolverChanged(this._solverType);
    }

    /**
     * Initialize IK for a loaded model. Call after model is added to scene.
     * @param {import('../models/UnifiedRobotModel.js').UnifiedRobotModel} model
     */
    init(model) {
        this.dispose();

        this._model = model;
        if (!model?.joints || model.joints.size === 0) return;

        this._kinematics = new RobotKinematics(model);

        const analysis = HumanoidKinematicsAnalyzer.analyze(model);
        const tipCandidates = this._selectTipLinks(model, analysis);

        this._kinematics.buildChains(tipCandidates);

        this._solver = new JacobianIkSolver(this._kinematics);
        this._qpSolver = new QPIkSolver(this._kinematics);
        this._sampler = new ReachabilitySampler(this._kinematics);
        this._animator = new JointPoseAnimator({
            applyJoint: (name, value) => this._applyJoint(name, value),
            duration: 600
        });

        this._assignTipLabels(analysis);

        for (const tip of this._kinematics.tipLinks) {
            this._tipVisibility.set(tip, true);
        }

        this._solverType = 'qp';
        this.onSolverChanged(this._solverType);
    }

    /**
     * Select IK tip links using HumanoidKinematicsAnalyzer categories.
     *
     * Strategy per limb region:
     *   Arms  → Hand/Palm/Wrist base (branch point before fingers/gripper diverge)
     *   Legs  → Foot/Ankle (deepest non-toe link in chain)
     *   Head  → Head link (if leaf)
     * Falls back to leaf-based heuristics for non-humanoid robots.
     */
    _selectTipLinks(model, analysis) {
        const linkCats = analysis.linkCategories;

        const DISTAL_ARM = new Set(['Finger', 'Gripper']);
        const EE_ARM     = new Set(['Hand', 'Wrist']);
        const DISTAL_LEG = new Set(['Toe']);
        const EE_LEG     = new Set(['Foot', 'Ankle']);
        const ARM_REGIONS = ['Left Arm', 'Right Arm'];
        const LEG_REGIONS = ['Left Leg', 'Right Leg'];

        const parentMap = new Map();
        model.joints.forEach(j => {
            if (j.parent && j.child)
                parentMap.set(j.child, { parentLink: j.parent, joint: j });
        });

        const childCountMap = new Map();
        model.joints.forEach(j => {
            if (j.parent && j.child && j.type !== 'fixed')
                childCountMap.set(j.parent, (childCountMap.get(j.parent) || 0) + 1);
        });

        const leafLinks = new Set();
        model.links.forEach((_link, name) => {
            const hasChildren = [...model.joints.values()].some(j => j.parent === name);
            if (!hasChildren) leafLinks.add(name);
        });

        const tips = new Set();

        const findEeTip = (regionTag, distalParts, eeParts) => {
            const regionLeaves = [];
            const regionLinks = [];

            linkCats.forEach((cats, name) => {
                if (!cats.includes(regionTag)) return;
                regionLinks.push(name);
                if (leafLinks.has(name)) regionLeaves.push(name);
            });
            if (regionLinks.length === 0) return;

            const distalLeaves = regionLeaves.filter(n => {
                const c = linkCats.get(n) || [];
                return c.some(t => distalParts.has(t));
            });

            if (distalLeaves.length > 0) {
                const ancestors = new Set();
                for (const leaf of distalLeaves) {
                    let current = leaf;
                    while (parentMap.has(current)) {
                        const { parentLink } = parentMap.get(current);
                        const pCats = linkCats.get(parentLink) || classifyCategories(parentLink);
                        const isBranch = (childCountMap.get(parentLink) || 0) > 1;
                        const isDistal = pCats.some(c => distalParts.has(c));
                        const isEe = pCats.some(c => eeParts.has(c));

                        if (isBranch || isEe || !isDistal) {
                            ancestors.add(parentLink);
                            break;
                        }
                        current = parentLink;
                    }
                }
                if (ancestors.size > 0) {
                    for (const a of ancestors) tips.add(a);
                    return;
                }
            }

            const eeLinks = regionLinks.filter(n => {
                const c = linkCats.get(n) || [];
                return c.some(t => eeParts.has(t));
            });
            if (eeLinks.length > 0) {
                let deepest = eeLinks[0];
                let maxDepth = 0;
                for (const name of eeLinks) {
                    let depth = 0, cur = name;
                    while (parentMap.has(cur)) { cur = parentMap.get(cur).parentLink; depth++; }
                    if (depth > maxDepth) { maxDepth = depth; deepest = name; }
                }
                tips.add(deepest);
                return;
            }

            const nonTrunkLeaves = regionLeaves.filter(n => {
                const c = linkCats.get(n) || [];
                return !c.some(t => t === 'Torso' || t === 'Waist' || t === 'Base');
            });
            if (nonTrunkLeaves.length > 0) {
                tips.add(nonTrunkLeaves[0]);
            }
        };

        for (const region of ARM_REGIONS) findEeTip(region, DISTAL_ARM, EE_ARM);
        for (const region of LEG_REGIONS) findEeTip(region, DISTAL_LEG, EE_LEG);

        // Head: find the deepest Head or Neck link.
        // Prefer Head over Neck; among equals prefer deeper in chain.
        let bestHead = null, bestHeadDepth = -1, bestIsHead = false;
        linkCats.forEach((cats, name) => {
            const isHead = cats.includes('Head');
            const isNeck = cats.includes('Neck');
            if (!isHead && !isNeck) return;
            // Skip links that are part of arm/leg regions (sensor links etc.)
            if (cats.some(c => c === 'Left Arm' || c === 'Right Arm' || c === 'Left Leg' || c === 'Right Leg')) return;
            let depth = 0, cur = name;
            while (parentMap.has(cur)) { cur = parentMap.get(cur).parentLink; depth++; }
            // Prefer Head-tagged over Neck-tagged, then deeper
            if (isHead && !bestIsHead) {
                bestHead = name; bestHeadDepth = depth; bestIsHead = true;
            } else if (isHead === bestIsHead && depth > bestHeadDepth) {
                bestHead = name; bestHeadDepth = depth; bestIsHead = isHead;
            }
        });
        if (bestHead) tips.add(bestHead);

        if (tips.size > 0) return [...tips];

        const fallbackLeaves = [...leafLinks].filter(n => {
            const cats = classifyCategories(n);
            return !(cats.includes('Other') && cats.length === 1);
        });
        return fallbackLeaves.slice(0, 8);
    }

    /**
     * Assign user-friendly labels to tips based on kinematic analysis.
     */
    _assignTipLabels(analysis) {
        this._tipLabels.clear();
        const linkCats = analysis.linkCategories;
        for (const tip of this._kinematics.tipLinks) {
            const cats = linkCats?.get(tip) ?? classifyCategories(tip);
            const meaningful = cats.filter(c => c !== 'Other');
            const label = meaningful.length > 0 ? meaningful.join(' / ') : tip;
            this._tipLabels.set(tip, label);
        }
    }

    /**
     * Get the human-readable label for a tip link.
     */
    getTipLabel(tipName) {
        return this._tipLabels.get(tipName) ?? tipName;
    }

    /**
     * Get color index for a tip.
     */
    getTipColor(tipName) {
        const idx = this._kinematics.tipLinks.indexOf(tipName);
        return TIP_COLORS[idx % TIP_COLORS.length];
    }

    // ==================== Enable / Disable ====================

    enable() {
        if (this._enabled || !this._kinematics) return;
        this._enabled = true;
        this._createGizmos();
        document.addEventListener('keydown', this._onKeyDown);
    }

    disable() {
        if (!this._enabled) return;
        this._enabled = false;
        this._removeGizmos();
        this.hideReachability();
        document.removeEventListener('keydown', this._onKeyDown);
        if (this._lockLinks && this.sceneManager?.dragControls) {
            this.sceneManager.dragControls.enabled = true;
        }
    }

    // ==================== Gizmo Management ====================

    _createGizmos() {
        const sm = this.sceneManager;
        if (!sm?.scene || !sm?.camera || !sm?.renderer) return;

        for (const tip of this._kinematics.tipLinks) {
            if (!this._tipVisibility.get(tip)) continue;
            this._createGizmoForTip(tip);
        }
    }

    _createGizmoForTip(tip) {
        if (this._gizmos.has(tip)) return;

        const sm = this.sceneManager;
        const pos = this._kinematics.getLinkWorldPosition(tip);
        const quat = this._kinematics.getLinkWorldQuaternion(tip);
        if (!pos) return;

        const anchor = new THREE.Object3D();
        anchor.position.copy(pos);
        if (quat) anchor.quaternion.copy(quat);
        anchor.name = `ik-anchor-${tip}`;
        sm.scene.add(anchor);

        const translateGizmo = new TransformControls(sm.camera, sm.renderer.domElement);
        translateGizmo.setMode('translate');
        translateGizmo.setSpace(this._gizmoSpace);
        translateGizmo.setSize(0.25);
        translateGizmo.attach(anchor);
        translateGizmo.visible = this._translateEnabled;
        translateGizmo.enabled = this._translateEnabled;
        sm.scene.add(translateGizmo);

        const rotateGizmo = new TransformControls(sm.camera, sm.renderer.domElement);
        rotateGizmo.setMode('rotate');
        rotateGizmo.setSpace(this._gizmoSpace);
        rotateGizmo.setSize(0.5);
        rotateGizmo.attach(anchor);
        rotateGizmo.visible = this._rotateEnabled;
        rotateGizmo.enabled = this._rotateEnabled;
        sm.scene.add(rotateGizmo);

        const onTranslateDragChanged = (e) => {
            sm.controls.enabled = !e.value;
            this._dragging = e.value;
            if (e.value) {
                this._activeTip = tip;
                this._activeMode = 'translate';
                rotateGizmo.enabled = false;
            } else {
                this._activeTip = null;
                this._activeMode = null;
                if (this._rotateEnabled) rotateGizmo.enabled = true;
            }
        };

        const onRotateDragChanged = (e) => {
            sm.controls.enabled = !e.value;
            this._dragging = e.value;
            if (e.value) {
                this._activeTip = tip;
                this._activeMode = 'rotate';
                translateGizmo.enabled = false;
            } else {
                this._activeTip = null;
                this._activeMode = null;
                if (this._translateEnabled) translateGizmo.enabled = true;
            }
        };

        const onTranslateChange = () => {
            if (this._dragging && this._activeTip === tip && this._activeMode === 'translate') {
                this._solveForTip(tip, anchor.position, null);
            }
        };

        const onRotateChange = () => {
            if (this._dragging && this._activeTip === tip && this._activeMode === 'rotate') {
                this._solveForTip(tip, anchor.position, anchor.quaternion);
            }
        };

        translateGizmo.addEventListener('dragging-changed', onTranslateDragChanged);
        translateGizmo.addEventListener('objectChange', onTranslateChange);
        rotateGizmo.addEventListener('dragging-changed', onRotateDragChanged);
        rotateGizmo.addEventListener('objectChange', onRotateChange);

        const idx = this._kinematics.tipLinks.indexOf(tip);
        const color = TIP_COLORS[idx % TIP_COLORS.length];
        this._tintGizmo(translateGizmo, color);
        this._tintGizmo(rotateGizmo, color);

        this._gizmos.set(tip, { translateGizmo, rotateGizmo, anchor });
    }

    // ==================== Translate / Rotate / Lock Toggles ====================

    setTranslateEnabled(enabled) {
        this._translateEnabled = enabled;
        for (const [, { translateGizmo }] of this._gizmos) {
            translateGizmo.visible = enabled;
            translateGizmo.enabled = enabled;
        }
        this.sceneManager.redraw();
    }

    setRotateEnabled(enabled) {
        this._rotateEnabled = enabled;
        for (const [, { rotateGizmo }] of this._gizmos) {
            rotateGizmo.visible = enabled;
            rotateGizmo.enabled = enabled;
        }
        this.sceneManager.redraw();
    }

    setLockLinks(locked) {
        this._lockLinks = locked;
        const sm = this.sceneManager;
        if (sm?.dragControls) {
            sm.dragControls.enabled = !locked;
        }
    }

    _tintGizmo(gizmo, color) {
        gizmo.traverse(child => {
            if (child.material && child.material.color) {
                child.material = child.material.clone();
                child.material.color.setHex(color);
            }
        });
    }

    _removeGizmos() {
        const sm = this.sceneManager;
        for (const [tip, { translateGizmo, rotateGizmo, anchor }] of this._gizmos) {
            translateGizmo.detach();
            sm.scene.remove(translateGizmo);
            translateGizmo.dispose();
            rotateGizmo.detach();
            sm.scene.remove(rotateGizmo);
            rotateGizmo.dispose();
            sm.scene.remove(anchor);
        }
        this._gizmos.clear();
        sm.controls.enabled = true;
        sm.redraw();
    }

    /**
     * Toggle visibility for a specific tip's gizmo.
     */
    setTipVisible(tipName, visible) {
        this._tipVisibility.set(tipName, visible);

        if (visible && this._enabled && !this._gizmos.has(tipName)) {
            this._createGizmoForTip(tipName);
        } else if (!visible && this._gizmos.has(tipName)) {
            const { translateGizmo, rotateGizmo, anchor } = this._gizmos.get(tipName);
            translateGizmo.detach();
            this.sceneManager.scene.remove(translateGizmo);
            translateGizmo.dispose();
            rotateGizmo.detach();
            this.sceneManager.scene.remove(rotateGizmo);
            rotateGizmo.dispose();
            this.sceneManager.scene.remove(anchor);
            this._gizmos.delete(tipName);
        }

        this.sceneManager.redraw();
    }

    isTipVisible(tipName) {
        return this._tipVisibility.get(tipName) ?? true;
    }

    // ==================== Gizmo Space ====================

    get gizmoSpace() { return this._gizmoSpace; }

    /**
     * Set gizmo space for all tips: 'local' or 'world'.
     */
    setGizmoSpace(space) {
        if (space !== 'local' && space !== 'world') return;
        this._gizmoSpace = space;
        for (const [, { translateGizmo, rotateGizmo }] of this._gizmos) {
            translateGizmo.setSpace(space);
            rotateGizmo.setSpace(space);
        }
        this.sceneManager.redraw();
    }

    toggleGizmoSpace() {
        this.setGizmoSpace(this._gizmoSpace === 'local' ? 'world' : 'local');
    }

    _handleKeyDown(e) {
        if (!this._enabled) return;
        if (e.target.tagName === 'INPUT' || e.target.tagName === 'TEXTAREA') return;

        if (e.key === 'l' || e.key === 'L') {
            this.toggleGizmoSpace();
        }
    }

    /**
     * Sync gizmo anchor positions to the current link world positions.
     * Call after joint values change externally (e.g. slider input).
     */
    syncGizmoPositions() {
        if (!this._enabled) return;

        for (const [tip, { anchor }] of this._gizmos) {
            const pos = this._kinematics.getLinkWorldPosition(tip);
            const quat = this._kinematics.getLinkWorldQuaternion(tip);
            if (pos) anchor.position.copy(pos);
            if (quat) anchor.quaternion.copy(quat);
        }
        this.sceneManager.redraw();
    }

    // ==================== IK Solving ====================

    /**
     * @param {string} tipName
     * @param {THREE.Vector3} targetPos
     * @param {THREE.Quaternion|null} targetQuat
     */
    _solveForTip(tipName, targetPos, targetQuat = null) {
        if (!this._model) return;

        const useQP = this._solverType === 'qp' && this._qpSolver;

        if (useQP) {
            this._solveWithQP(tipName, targetPos, targetQuat);
        } else if (this._solver) {
            this._solveWithJacobian(tipName, targetPos, targetQuat);
        }

        this._syncOtherGizmos(tipName);
        this.sceneManager.redraw();
    }

    _solveWithQP(tipName, targetPos, targetQuat) {
        const chain = this._kinematics.getChain(tipName);
        if (!chain) return;

        for (let i = 0; i < IK_ITERATIONS_PER_FRAME; i++) {
            const result = this._qpSolver.solve(tipName, targetPos, targetQuat);
            if (!result || result.deltas.length === 0) break;

            for (const { name, delta } of result.deltas) {
                const joint = this._model.getJoint(name);
                if (!joint) continue;

                let newValue = joint.currentValue + delta;
                if (joint.limits) {
                    newValue = Math.max(joint.limits.lower, Math.min(joint.limits.upper, newValue));
                }
                this._applyJoint(name, newValue);
            }

            if (result.converged) break;
        }
    }

    _solveWithJacobian(tipName, targetPos, targetQuat) {
        const chain = this._kinematics.getChain(tipName);
        if (!chain) return;

        for (let i = 0; i < IK_ITERATIONS_PER_FRAME; i++) {
            const result = this._solver.solve(tipName, targetPos, targetQuat);
            if (!result || result.deltas.length === 0) break;

            for (const { name, delta } of result.deltas) {
                const joint = this._model.getJoint(name);
                if (!joint) continue;

                let newValue = joint.currentValue + delta;
                if (joint.limits) {
                    newValue = Math.max(joint.limits.lower, Math.min(joint.limits.upper, newValue));
                }
                this._applyJoint(name, newValue);
            }

            if (result.converged) break;
        }
    }

    /**
     * After solving one tip, sync all gizmo anchors to match current link poses.
     */
    _syncOtherGizmos(solvedTip) {
        for (const [tip, { anchor }] of this._gizmos) {
            if (tip === solvedTip) continue;
            const pos = this._kinematics.getLinkWorldPosition(tip);
            const quat = this._kinematics.getLinkWorldQuaternion(tip);
            if (pos) anchor.position.copy(pos);
            if (quat) anchor.quaternion.copy(quat);
        }
    }

    // ==================== Single Write Path ====================

    _applyJoint(name, value) {
        if (!this._model) return;
        ModelLoaderFactory.setJointAngle(this._model, name, value);
        const joint = this._model.getJoint(name);
        if (joint) joint.currentValue = value;
        this.onJointChanged(name, value);
    }

    // ==================== Home / Random Animations ====================

    goHome(opts = {}) {
        if (!this._kinematics || !this._model) return;
        this._startPoseAnimation(this._kinematics.getHomeConfiguration(), opts);
    }

    goRandom(opts = {}) {
        if (!this._kinematics || !this._model) return;
        this._startPoseAnimation(this._kinematics.getRandomConfiguration(), opts);
    }

    _startPoseAnimation(targetConfig, opts = {}) {
        if (!this._animator) return;

        const currentConfig = targetConfig.map(({ name }) => {
            const joint = this._model.getJoint(name);
            return { name, value: joint?.currentValue ?? 0 };
        });

        this._animator.animateTo(currentConfig, targetConfig, {
            duration: opts.duration ?? 600,
            onComplete: () => {
                this.syncGizmoPositions();
                opts.onComplete?.();
            }
        });
    }

    /**
     * Call in the animation loop.
     */
    update() {
        if (this._animator?.update()) {
            this.syncGizmoPositions();
            this.sceneManager.redraw();
            this.onAnimationTick();
        }
    }

    // ==================== Reachability ====================

    get reachSamples() { return this._reachSamples; }
    set reachSamples(n) { this._reachSamples = Math.max(0, Math.min(10000, n)); }

    showReachability(samplesPerTip) {
        samplesPerTip = samplesPerTip ?? this._reachSamples;
        if (!this._sampler || !this._kinematics) return;
        this.hideReachability();

        const clouds = this._sampler.sampleAll(samplesPerTip);

        for (const [tipName, positions] of clouds) {
            if (positions.length === 0) continue;

            const geometry = new THREE.BufferGeometry();
            geometry.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));

            const idx = this._kinematics.tipLinks.indexOf(tipName);
            const color = TIP_COLORS[idx % TIP_COLORS.length];

            const material = new THREE.PointsMaterial({
                color,
                size: 0.005,
                transparent: true,
                opacity: 0.6,
                sizeAttenuation: true,
                depthWrite: false
            });

            const points = new THREE.Points(geometry, material);
            points.name = `reachability-${tipName}`;
            points.renderOrder = -1;
            this.sceneManager.scene.add(points);
            this._reachClouds.set(tipName, points);
        }

        this._reachVisible = true;
        this.sceneManager.redraw();
    }

    hideReachability() {
        for (const [, points] of this._reachClouds) {
            this.sceneManager.scene.remove(points);
            points.geometry.dispose();
            points.material.dispose();
        }
        this._reachClouds.clear();
        this._reachVisible = false;
        this.sceneManager.redraw();
    }

    get reachabilityVisible() {
        return this._reachVisible;
    }

    isTipReachVisible(tipName) {
        return this._reachClouds.has(tipName) && this._reachClouds.get(tipName).visible;
    }

    setTipReachVisible(tipName, visible) {
        const cloud = this._reachClouds.get(tipName);
        if (cloud) {
            cloud.visible = visible;
            this.sceneManager.redraw();
        }
    }

    toggleReachability(samplesPerTip) {
        if (this._reachVisible) {
            this.hideReachability();
        } else {
            this.showReachability(samplesPerTip);
        }
    }

    // ==================== Cleanup ====================

    dispose() {
        this.disable();
        this.hideReachability();
        this._animator?.cancel();
        document.removeEventListener('keydown', this._onKeyDown);
        this._qpSolver = null;
        this._solverType = 'jacobian';
        this._model = null;
        this._kinematics = null;
        this._solver = null;
        this._sampler = null;
        this._tipLabels.clear();
        this._tipVisibility.clear();
    }
}
