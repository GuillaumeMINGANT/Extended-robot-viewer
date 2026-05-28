/**
 * ReachabilitySampler — pure-math FK sampling for reachability point clouds.
 *
 * Does NOT modify scene graph or joint state. Computes forward kinematics
 * using DH-like transforms from joint axis/origin data in UnifiedRobotModel.
 * Runs synchronously for quick point cloud generation.
 */
import * as THREE from 'three';

const _mat4 = new THREE.Matrix4();
const _pos = new THREE.Vector3();
const _quat = new THREE.Quaternion();
const _axis = new THREE.Vector3();

export class ReachabilitySampler {
    /**
     * @param {import('./RobotKinematics.js').RobotKinematics} kinematics
     */
    constructor(kinematics) {
        this.kinematics = kinematics;
    }

    /**
     * Sample reachability for a given tip chain.
     * Returns a Float32Array of [x, y, z, x, y, z, ...] world positions.
     *
     * Uses the Three.js scene graph's current base transforms but randomises
     * joint values. This is intentionally fast and approximate.
     *
     * @param {string} tipName
     * @param {number} [sampleCount=2000]
     * @returns {Float32Array}
     */
    sample(tipName, sampleCount = 2000) {
        const chain = this.kinematics.getChain(tipName);
        if (!chain) return new Float32Array(0);

        const { joints, tipLink } = chain;
        if (joints.length === 0) return new Float32Array(0);

        const baseTransform = this._getChainBaseTransform(joints);
        if (!baseTransform) return new Float32Array(0);

        const localTransforms = this._extractLocalTransforms(joints);

        const points = new Float32Array(sampleCount * 3);

        for (let s = 0; s < sampleCount; s++) {
            const randomAngles = this._randomJointValues(joints);
            const tipPos = this._computeFk(baseTransform, localTransforms, joints, randomAngles);
            points[s * 3] = tipPos.x;
            points[s * 3 + 1] = tipPos.y;
            points[s * 3 + 2] = tipPos.z;
        }

        return points;
    }

    /**
     * Sample all tip chains at once. Returns a Map<tipName, Float32Array>.
     * @param {number} [samplesPerTip=2000]
     * @returns {Map<string, Float32Array>}
     */
    sampleAll(samplesPerTip = 2000) {
        const results = new Map();
        for (const tipName of this.kinematics.tipLinks) {
            results.set(tipName, this.sample(tipName, samplesPerTip));
        }
        return results;
    }

    /**
     * Get the world transform of the first joint's parent link (chain root).
     */
    _getChainBaseTransform(joints) {
        const firstJoint = joints[0];
        if (!firstJoint?.threeObject) return null;

        const parent = firstJoint.threeObject.parent;
        if (!parent) return new THREE.Matrix4().identity();

        parent.updateWorldMatrix(true, false);
        return parent.matrixWorld.clone();
    }

    /**
     * Extract local rest transforms for each joint in the chain.
     * These represent the transform from parent to joint frame at q=0.
     */
    _extractLocalTransforms(joints) {
        return joints.map(joint => {
            if (!joint.threeObject) return new THREE.Matrix4().identity();
            return joint.threeObject.matrix.clone();
        });
    }

    /**
     * Random joint values respecting limits.
     */
    _randomJointValues(joints) {
        return joints.map(joint => {
            const lo = joint.limits?.lower ?? -Math.PI;
            const hi = joint.limits?.upper ?? Math.PI;
            return lo + Math.random() * (hi - lo);
        });
    }

    /**
     * Pure forward kinematics along the chain with given joint values.
     * Returns the world position of the tip.
     */
    _computeFk(baseTransform, localTransforms, joints, angles) {
        const accumulated = baseTransform.clone();

        for (let i = 0; i < joints.length; i++) {
            const joint = joints[i];
            const localMat = localTransforms[i];

            accumulated.multiply(localMat);

            const angle = angles[i];
            _axis.set(0, 0, 1);
            if (joint.axis?.xyz) {
                _axis.set(joint.axis.xyz[0], joint.axis.xyz[1], joint.axis.xyz[2]);
            } else if (joint.threeObject?.axis?.isVector3) {
                _axis.copy(joint.threeObject.axis);
            }
            _axis.normalize();

            if (joint.type === 'revolute' || joint.type === 'continuous') {
                _mat4.makeRotationAxis(_axis, angle);
            } else if (joint.type === 'prismatic') {
                _mat4.makeTranslation(
                    _axis.x * angle,
                    _axis.y * angle,
                    _axis.z * angle
                );
            } else {
                _mat4.identity();
            }

            accumulated.multiply(_mat4);
        }

        _pos.setFromMatrixPosition(accumulated);
        return _pos.clone();
    }
}
