/**
 * RobotKinematics — kinematic chain extraction, FK, and geometric Jacobian.
 *
 * Operates on UnifiedRobotModel + Three.js scene graph.
 * Read-only w.r.t. joint state; never calls setJointAngle.
 */
import * as THREE from 'three';

const _vec3A = new THREE.Vector3();
const _vec3B = new THREE.Vector3();
const _mat4 = new THREE.Matrix4();
const _quat = new THREE.Quaternion();

export class RobotKinematics {
    /**
     * @param {import('../models/UnifiedRobotModel.js').UnifiedRobotModel} model
     */
    constructor(model) {
        this.model = model;
        this._chains = new Map();
        this._tipLinks = [];
    }

    /**
     * Build kinematic chains from root to each tip link.
     * A tip link is any leaf link (no children) with at least one movable
     * ancestor joint, or an explicitly provided list.
     *
     * @param {string[]} [explicitTips] — override auto-detected tips
     * @returns {string[]} tip link names
     */
    buildChains(explicitTips = null) {
        const model = this.model;
        if (!model?.joints || !model?.links) return [];

        const parentMap = new Map();
        const childLinksSet = new Set();

        model.joints.forEach((joint) => {
            if (!joint.parent || !joint.child) return;
            parentMap.set(joint.child, { parentLink: joint.parent, joint });
            childLinksSet.add(joint.child);
        });

        let tips;
        if (explicitTips && explicitTips.length > 0) {
            tips = explicitTips.filter(name => model.links.has(name));
        } else {
            tips = [];
            model.links.forEach((link, name) => {
                const hasChildren = [...model.joints.values()].some(j => j.parent === name);
                if (!hasChildren && parentMap.has(name)) {
                    tips.push(name);
                }
            });
        }

        this._chains.clear();
        this._tipLinks = [];

        for (const tipName of tips) {
            const chain = this._walkChainToRoot(tipName, parentMap);
            if (chain.joints.length > 0) {
                this._chains.set(tipName, chain);
                this._tipLinks.push(tipName);
            }
        }

        return this._tipLinks;
    }

    /**
     * Walk from tip link to root, collecting movable joints.
     * Returns joints ordered ROOT → TIP (for Jacobian column ordering).
     */
    _walkChainToRoot(tipName, parentMap) {
        const joints = [];
        let current = tipName;

        while (parentMap.has(current)) {
            const { parentLink, joint } = parentMap.get(current);
            if (joint.type !== 'fixed') {
                joints.push(joint);
            }
            current = parentLink;
        }

        joints.reverse();
        return { tipLink: tipName, joints };
    }

    get tipLinks() {
        return this._tipLinks;
    }

    /**
     * Get the chain for a given tip link.
     * @param {string} tipName
     * @returns {{ tipLink: string, joints: Joint[] } | null}
     */
    getChain(tipName) {
        return this._chains.get(tipName) ?? null;
    }

    /**
     * Get the world-space pose of a link via its Three.js object.
     * @param {string} linkName
     * @returns {THREE.Matrix4 | null}
     */
    getLinkWorldMatrix(linkName) {
        const link = this.model.getLink(linkName);
        if (!link?.threeObject) return null;
        link.threeObject.updateWorldMatrix(true, false);
        return link.threeObject.matrixWorld.clone();
    }

    /**
     * Get world position of a link.
     * @param {string} linkName
     * @returns {THREE.Vector3 | null}
     */
    getLinkWorldPosition(linkName) {
        const link = this.model.getLink(linkName);
        if (!link?.threeObject) return null;
        link.threeObject.updateWorldMatrix(true, false);
        const pos = new THREE.Vector3();
        pos.setFromMatrixPosition(link.threeObject.matrixWorld);
        return pos;
    }

    /**
     * Get world quaternion of a link.
     * @param {string} linkName
     * @returns {THREE.Quaternion | null}
     */
    getLinkWorldQuaternion(linkName) {
        const link = this.model.getLink(linkName);
        if (!link?.threeObject) return null;
        link.threeObject.updateWorldMatrix(true, false);
        const q = new THREE.Quaternion();
        link.threeObject.getWorldQuaternion(q);
        return q;
    }

    /**
     * Compute the geometric Jacobian for a given tip chain.
     * Returns a (taskDim × nJoints) flat Float64Array (row-major).
     *
     * @param {string} tipName
     * @param {number} [taskDim] — 3 for position-only, 6 for full pose. Auto if omitted.
     * @returns {{ J: Float64Array, rows: number, cols: number } | null}
     */
    computeJacobian(tipName, taskDim = null) {
        const chain = this._chains.get(tipName);
        if (!chain) return null;

        const { joints, tipLink } = chain;
        const nj = joints.length;
        if (nj === 0) return null;

        const tipObj = this.model.getLink(tipLink)?.threeObject;
        if (!tipObj) return null;

        tipObj.updateWorldMatrix(true, false);
        const pTip = new THREE.Vector3();
        pTip.setFromMatrixPosition(tipObj.matrixWorld);

        const fullPose = taskDim !== null ? taskDim === 6 : nj >= 6;
        const rows = fullPose ? 6 : 3;
        const J = new Float64Array(rows * nj);

        for (let col = 0; col < nj; col++) {
            const joint = joints[col];
            const jObj = joint.threeObject;
            if (!jObj) continue;

            jObj.updateWorldMatrix(true, false);

            const axisLocal = _vec3A.set(0, 0, 1);
            if (joint.axis?.xyz) {
                axisLocal.set(joint.axis.xyz[0], joint.axis.xyz[1], joint.axis.xyz[2]);
            } else if (jObj.axis?.isVector3) {
                axisLocal.copy(jObj.axis);
            }
            axisLocal.normalize();

            _quat.setFromRotationMatrix(jObj.matrixWorld);
            const zWorld = _vec3B.copy(axisLocal).applyQuaternion(_quat);

            if (joint.type === 'revolute' || joint.type === 'continuous') {
                const pJoint = new THREE.Vector3();
                pJoint.setFromMatrixPosition(jObj.matrixWorld);

                const diff = new THREE.Vector3().subVectors(pTip, pJoint);
                const linear = new THREE.Vector3().crossVectors(zWorld, diff);

                J[0 * nj + col] = linear.x;
                J[1 * nj + col] = linear.y;
                J[2 * nj + col] = linear.z;

                if (fullPose) {
                    J[3 * nj + col] = zWorld.x;
                    J[4 * nj + col] = zWorld.y;
                    J[5 * nj + col] = zWorld.z;
                }
            } else if (joint.type === 'prismatic') {
                J[0 * nj + col] = zWorld.x;
                J[1 * nj + col] = zWorld.y;
                J[2 * nj + col] = zWorld.z;

                if (fullPose) {
                    J[3 * nj + col] = 0;
                    J[4 * nj + col] = 0;
                    J[5 * nj + col] = 0;
                }
            }
        }

        return { J, rows, cols: nj };
    }

    /**
     * Compute the 6D pose error between current tip pose and target.
     * Returns [dx, dy, dz, rx, ry, rz] where rotation part is the
     * axis-angle vector (vex of skew-symmetric log).
     *
     * @param {string} tipName
     * @param {THREE.Vector3} targetPos
     * @param {THREE.Quaternion} [targetQuat] — if null, only positional error
     * @returns {{ error: Float64Array, dim: number } | null}
     */
    computePoseError(tipName, targetPos, targetQuat = null) {
        const link = this.model.getLink(this._chains.get(tipName)?.tipLink);
        if (!link?.threeObject) return null;

        link.threeObject.updateWorldMatrix(true, false);

        const currentPos = new THREE.Vector3();
        currentPos.setFromMatrixPosition(link.threeObject.matrixWorld);

        const chain = this._chains.get(tipName);
        const fullPose = targetQuat != null;
        const dim = fullPose ? 6 : 3;
        const error = new Float64Array(dim);

        error[0] = targetPos.x - currentPos.x;
        error[1] = targetPos.y - currentPos.y;
        error[2] = targetPos.z - currentPos.z;

        if (fullPose) {
            const currentQuat = new THREE.Quaternion();
            link.threeObject.getWorldQuaternion(currentQuat);

            // World-frame orientation error: q_error = q_target * q_current^-1
            const errorQuat = targetQuat.clone().multiply(currentQuat.clone().invert());
            errorQuat.normalize();
            if (errorQuat.w < 0) {
                errorQuat.x = -errorQuat.x;
                errorQuat.y = -errorQuat.y;
                errorQuat.z = -errorQuat.z;
                errorQuat.w = -errorQuat.w;
            }

            const halfAngle = Math.acos(Math.min(1, errorQuat.w));
            const sinHalf = Math.sin(halfAngle);

            if (sinHalf > 1e-8) {
                const scale = 2 * halfAngle / sinHalf;
                error[3] = errorQuat.x * scale;
                error[4] = errorQuat.y * scale;
                error[5] = errorQuat.z * scale;
            }
        }

        return { error, dim };
    }

    /**
     * Get current configuration (joint values) for a chain.
     * @param {string} tipName
     * @returns {number[] | null}
     */
    getChainConfiguration(tipName) {
        const chain = this._chains.get(tipName);
        if (!chain) return null;
        return chain.joints.map(j => j.currentValue);
    }

    /**
     * Get all controllable joint names (non-fixed).
     * @returns {string[]}
     */
    getControllableJoints() {
        const names = [];
        this.model.joints.forEach((joint, name) => {
            if (joint.type !== 'fixed') {
                names.push(name);
            }
        });
        return names;
    }

    /**
     * Get a zero (home) configuration for all controllable joints.
     * @returns {{ name: string, value: number }[]}
     */
    getHomeConfiguration() {
        const config = [];
        this.model.joints.forEach((joint, name) => {
            if (joint.type === 'fixed') return;
            config.push({ name, value: 0 });
        });
        return config;
    }

    /**
     * Get a random valid configuration within limits.
     * @returns {{ name: string, value: number }[]}
     */
    getRandomConfiguration() {
        const config = [];
        this.model.joints.forEach((joint, name) => {
            if (joint.type === 'fixed') return;
            const lo = joint.limits?.lower ?? -Math.PI;
            const hi = joint.limits?.upper ?? Math.PI;
            const value = lo + Math.random() * (hi - lo);
            config.push({ name, value });
        });
        return config;
    }
}
