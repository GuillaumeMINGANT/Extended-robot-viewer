import * as THREE from 'three';

/**
 * CoordinateAxesManager - Handles link coordinate axes and joint axes visualization
 */
export class CoordinateAxesManager {
    /** Distinct from RGB link axes (red / green / blue) */
    static JOINT_AXIS_COLOR = 0xf59e0b;
    static JOINT_ROTATION_COLOR = 0xfbbf24;

    constructor(sceneManager) {
        this.sceneManager = sceneManager;
        this.linkAxesHelpers = new Map();
        this.jointAxesHelpers = new Map();
        this.showAxesEnabled = false;
        this.showJointAxesEnabled = false;
        /** Overall robot span (max bbox dimension), set when a model loads */
        this.modelScale = 1;
    }

    setModelScale(modelScale) {
        this.modelScale = Math.max(modelScale, 1e-6);
    }

    /**
     * Max dimension of an Object3D subtree (meters in scene units).
     */
    static measureObjectScale(object3d, fallback = 1) {
        try {
            if (object3d) {
                object3d.updateMatrixWorld(true);
                const bbox = new THREE.Box3().setFromObject(object3d);
                if (!bbox.isEmpty()) {
                    const size = bbox.getSize(new THREE.Vector3());
                    const maxDim = Math.max(size.x, size.y, size.z);
                    if (maxDim > 1e-6) return maxDim;
                }
            }
        } catch (_) { /* use fallback */ }
        return fallback;
    }

    /**
     * Axis triad length: ~12% of link size, bounded by robot scale (not fixed meters).
     * Tuned so a ~1.6 m humanoid stays near the previous ~14 cm cap.
     */
    static computeLinkAxesLength(linkSize = 1, modelScale = linkSize) {
        const link = Math.max(linkSize, 1e-6);
        const model = Math.max(modelScale, link, 1e-6);

        const ideal = link * 0.12;
        const minLen = Math.max(
            link * 0.08,
            Math.min(model * 0.025, link * 0.5)
        );
        const maxLen = Math.min(link * 0.35, model * 0.0875);

        return Math.max(minLen, Math.min(ideal, maxLen));
    }

    /**
     * Solid RGB triad matching world gizmo: MeshBasicMaterial (unlit, opaque), no labels.
     * @param {number} axisLength - Total length of each axis arrow
     * @param {{ showOrigin?: boolean }} options
     * @returns {THREE.Group}
     */
    static createGizmoStyleAxesGroup(axisLength, { showOrigin = true } = {}) {
        const group = new THREE.Group();
        group.userData.isCoordinateAxes = true;
        group.renderOrder = 100;

        const headLength = axisLength * 0.22;
        const headWidth = axisLength * 0.1;
        const shaftRadius = Math.max(0.001, axisLength * 0.045);
        const shaftLen = axisLength - headLength;

        const axes = [
            { color: 0xef4444, rotZ: -Math.PI / 2 },
            { color: 0x22c55e },
            { color: 0x3b82f6, rotX: Math.PI / 2 }
        ];

        axes.forEach(({ color, rotZ, rotX }) => {
            const mat = new THREE.MeshBasicMaterial({
                color,
                toneMapped: false,
                transparent: false,
                opacity: 1,
                depthTest: true,
                depthWrite: true
            });

            const shaft = new THREE.Mesh(
                new THREE.CylinderGeometry(shaftRadius, shaftRadius, shaftLen, 8),
                mat
            );
            const cone = new THREE.Mesh(
                new THREE.ConeGeometry(headWidth, headLength, 12),
                mat
            );

            const axisGroup = new THREE.Group();
            shaft.position.y = shaftLen / 2;
            cone.position.y = shaftLen + headLength / 2;
            axisGroup.add(shaft, cone);
            if (rotZ !== undefined) axisGroup.rotation.z = rotZ;
            if (rotX !== undefined) axisGroup.rotation.x = rotX;

            axisGroup.traverse((child) => {
                child.renderOrder = 100;
                child.castShadow = false;
                child.receiveShadow = false;
                if (child.material) {
                    child.material.toneMapped = false;
                    child.material.transparent = false;
                    child.material.opacity = 1;
                    child.material.depthTest = true;
                    child.material.depthWrite = true;
                    child.material.needsUpdate = true;
                }
            });
            group.add(axisGroup);
        });

        if (showOrigin) {
            const origin = new THREE.Mesh(
                new THREE.SphereGeometry(axisLength * 0.08, 12, 12),
                new THREE.MeshBasicMaterial({
                    color: 0x888888,
                    toneMapped: false,
                    transparent: false,
                    opacity: 1,
                    depthTest: true,
                    depthWrite: true
                })
            );
            origin.renderOrder = 100;
            origin.castShadow = false;
            origin.receiveShadow = false;
            group.add(origin);
        }

        return group;
    }

    /**
     * Static method: Create coordinate axes geometry
     * @param {number} axesSize - Length of axes
     * @returns {THREE.Group} Axes group
     */
    static createAxesGeometry(axesSize) {
        return CoordinateAxesManager.createGizmoStyleAxesGroup(axesSize);
    }

    /**
     * Dashed line along joint rotation axis (same length scale as link axes).
     */
    static createJointAxisDashedLine(axisLength, axisDirection) {
        const dir = axisDirection.clone().normalize();
        const end = dir.clone().multiplyScalar(axisLength);
        const geometry = new THREE.BufferGeometry().setFromPoints([
            new THREE.Vector3(0, 0, 0),
            end
        ]);
        const material = new THREE.LineDashedMaterial({
            color: CoordinateAxesManager.JOINT_AXIS_COLOR,
            dashSize: axisLength * 0.06,
            gapSize: axisLength * 0.03,
            depthTest: true,
            transparent: false,
            toneMapped: false
        });
        const line = new THREE.Line(geometry, material);
        line.computeLineDistances();
        line.renderOrder = 100;
        return line;
    }

    /**
     * Static method: Create joint axis visualization (dashed line + rotation arc).
     */
    static createJointArrowGeometry(axisDirection, axisLength = 0.1) {
        const axisGroup = new THREE.Group();
        axisGroup.userData.isJointAxis = true;
        axisGroup.renderOrder = 100;

        axisGroup.add(CoordinateAxesManager.createJointAxisDashedLine(axisLength, axisDirection));
        axisGroup.add(CoordinateAxesManager.createRotationIndicator(
            axisDirection,
            axisLength,
            CoordinateAxesManager.JOINT_ROTATION_COLOR
        ));

        return axisGroup;
    }

    /**
     * Rotation-direction arc — proportions match createGizmoStyleAxesGroup at axisLength.
     */
    static createRotationIndicator(axisDirection, axisLength, color = CoordinateAxesManager.JOINT_ROTATION_COLOR) {
        const group = new THREE.Group();
        const radius = axisLength * 0.2;
        const tubeRadius = axisLength * 0.045;
        const arrowSize = axisLength * 0.1;
        const arrowHeadLength = arrowSize * 2;

        const arcAngle = Math.PI * 1.5;
        const curve = new THREE.EllipseCurve(0, 0, radius, radius, 0, arcAngle, false, 0);
        const points3D = curve.getPoints(50).map(p => new THREE.Vector3(p.x, p.y, 0));

        const curvePath = new THREE.CatmullRomCurve3(points3D);
        const tubeMat = new THREE.MeshBasicMaterial({
            color,
            toneMapped: false,
            transparent: false,
            depthTest: true
        });
        const tubeMesh = new THREE.Mesh(
            new THREE.TubeGeometry(curvePath, 50, tubeRadius, 8, false),
            tubeMat
        );
        group.add(tubeMesh);

        const endPoint = points3D[points3D.length - 1];
        const preEndPoint = points3D[points3D.length - 5];
        const tangent = new THREE.Vector3().subVectors(endPoint, preEndPoint).normalize();
        const coneMesh = new THREE.Mesh(
            new THREE.ConeGeometry(arrowSize, arrowHeadLength, 8),
            tubeMat
        );
        coneMesh.position.copy(endPoint);
        coneMesh.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), tangent);
        group.add(coneMesh);

        const rotQuat = new THREE.Quaternion();
        rotQuat.setFromUnitVectors(new THREE.Vector3(0, 0, 1), axisDirection.clone().normalize());
        group.quaternion.copy(rotQuat);
        group.position.copy(axisDirection.clone().normalize().multiplyScalar(axisLength * 0.75));

        group.traverse((child) => {
            child.renderOrder = 100;
            child.castShadow = false;
            child.receiveShadow = false;
        });

        return group;
    }

    /**
     * Same length as the RGB link triad for linkName (must run after createLinkAxes).
     */
    getJointAxisLength(joint, modelSize = 1) {
        for (const linkName of [joint.child, joint.parent]) {
            const len = this.getStoredLinkAxesLength(linkName);
            if (len != null) return len;
        }
        const model = this.sceneManager?.currentModel;
        const linkName = joint.child || joint.parent;
        const link = linkName && model?.links?.get(linkName);
        const scale = this.modelScale || modelSize;
        if (link) {
            return CoordinateAxesManager.computeLinkAxesLength(
                CoordinateAxesManager.measureLinkSize(link, scale),
                scale
            );
        }
        return CoordinateAxesManager.computeLinkAxesLength(scale, scale);
    }

    getStoredLinkAxesLength(linkName) {
        if (!linkName) return null;
        const axes = this.linkAxesHelpers.get(linkName);
        const size = axes?.userData?.axesSize;
        return size > 0 ? size : null;
    }

    static measureLinkSize(link, modelSize = 1) {
        let linkSize = modelSize;
        try {
            if (link?.threeObject) {
                link.threeObject.updateMatrixWorld(true);
                const bbox = new THREE.Box3().setFromObject(link.threeObject);
                if (!bbox.isEmpty()) {
                    const size = bbox.getSize(new THREE.Vector3());
                    const maxDim = Math.max(size.x, size.y, size.z);
                    if (maxDim > 0.001) linkSize = maxDim;
                }
            }
        } catch (_) { /* use modelSize */ }
        return linkSize;
    }

    /**
     * Create coordinate axes for a link
     */
    createLinkAxes(link, linkName, modelSize = 1.0) {
        const scale = this.modelScale || modelSize;
        const axesSize = CoordinateAxesManager.computeLinkAxesLength(
            CoordinateAxesManager.measureLinkSize(link, scale),
            scale
        );
        const axesGroup = CoordinateAxesManager.createGizmoStyleAxesGroup(axesSize);
        axesGroup.name = `${linkName}_axes`;
        axesGroup.userData.axesSize = axesSize;

        // Add to link's threeObject
        if (link.threeObject) {
            link.threeObject.add(axesGroup);
        }

        // Save reference
        this.linkAxesHelpers.set(linkName, axesGroup);

        // Decide whether to show based on current setting
        axesGroup.visible = this.showAxesEnabled;

        return axesGroup;
    }

    /**
     * Create joint axis visualization (dashed amber line + rotation arc).
     */
    createJointAxis(joint, jointName, modelSize = 1) {
        if (!joint.threeObject || (joint.type !== 'revolute' && joint.type !== 'continuous')) {
            return null;
        }

        const jointObject = joint.threeObject;
        const axisGroup = new THREE.Group();
        axisGroup.name = `jointAxis_${jointName}`;
        axisGroup.userData.isJointAxis = true;
        axisGroup.renderOrder = 100;

        let localAxisDirection = new THREE.Vector3(0, 0, 1);
        if (jointObject.axis) {
            localAxisDirection.copy(jointObject.axis).normalize();
        } else if (joint.axis?.xyz) {
            localAxisDirection.set(
                joint.axis.xyz[0] || 0,
                joint.axis.xyz[1] || 0,
                joint.axis.xyz[2] !== undefined ? joint.axis.xyz[2] : 1
            ).normalize();
        }

        const axisLength = this.getJointAxisLength(joint, modelSize);

        axisGroup.add(CoordinateAxesManager.createJointAxisDashedLine(axisLength, localAxisDirection));
        axisGroup.add(CoordinateAxesManager.createRotationIndicator(
            localAxisDirection,
            axisLength,
            CoordinateAxesManager.JOINT_ROTATION_COLOR
        ));

        // Save reference (but don't add to scene yet)
        this.jointAxesHelpers.set(jointName, {
            mesh: axisGroup,
            parent: jointObject,  // Add to joint object so it follows joint movement
            joint: joint,
            isAttached: false
        });

        // Decide whether to add to scene based on current setting
        if (this.showJointAxesEnabled) {
            jointObject.add(axisGroup);
            this.jointAxesHelpers.get(jointName).isAttached = true;
        }
        return axisGroup;
    }

    /**
     * Show all link axes
     */
    showAllAxes() {
        this.showAxesEnabled = true;

        // Show all link axes
        this.linkAxesHelpers.forEach((axes) => {
            axes.visible = true;
        });    }

    /**
     * Hide all link axes
     */
    hideAllAxes() {
        this.showAxesEnabled = false;

        // Hide all link axes
        this.linkAxesHelpers.forEach((axes) => {
            axes.visible = false;
        });    }

    /**
     * Show all joint axes
     */
    showAllJointAxes() {
        this.showJointAxesEnabled = true;

        // Show all joint axes (add to scene)
        this.jointAxesHelpers.forEach((axisInfo, jointName) => {
            if (!axisInfo.isAttached && axisInfo.parent) {
                axisInfo.parent.add(axisInfo.mesh);
                axisInfo.isAttached = true;
            }
        });    }

    /**
     * Hide all joint axes
     */
    hideAllJointAxes() {
        this.showJointAxesEnabled = false;

        // Hide all joint axes (remove from scene)
        this.jointAxesHelpers.forEach((axisInfo, jointName) => {
            if (axisInfo.isAttached && axisInfo.parent) {
                axisInfo.parent.remove(axisInfo.mesh);
                axisInfo.isAttached = false;
            }
        });    }

    /**
     * Temporarily show only specified joint axis (for slider drag/model drag)
     */
    showOnlyJointAxis(joint) {
        // Hide all joint axes
        this.jointAxesHelpers.forEach((axisInfo, jointName) => {
            if (axisInfo.isAttached && axisInfo.parent) {
                axisInfo.parent.remove(axisInfo.mesh);
                axisInfo.isAttached = false;
            }
        });

        // Show specified joint axis (regardless of switch state)
        this.jointAxesHelpers.forEach((axisInfo, jointName) => {
            if (axisInfo.joint === joint) {
                if (!axisInfo.isAttached && axisInfo.parent) {
                    axisInfo.parent.add(axisInfo.mesh);
                    axisInfo.isAttached = true;
                }
            }
        });
    }

    /**
     * Restore all joint axes display (called after slider drag ends)
     */
    restoreAllJointAxes() {
        // Hide all joint axes
        this.jointAxesHelpers.forEach((axisInfo, jointName) => {
            if (axisInfo.isAttached && axisInfo.parent) {
                axisInfo.parent.remove(axisInfo.mesh);
                axisInfo.isAttached = false;
            }
        });

        // If joint axes switch is on, show all axes
        if (this.showJointAxesEnabled) {
            this.jointAxesHelpers.forEach((axisInfo, jointName) => {
                if (!axisInfo.isAttached && axisInfo.parent) {
                    axisInfo.parent.add(axisInfo.mesh);
                    axisInfo.isAttached = true;
                }
            });
        }
    }

    /**
     * Ensure axes don't cast shadows
     */
    ensureAxesNoShadow() {
        // Ensure link axes don't cast shadows
        this.linkAxesHelpers.forEach((axes) => {
            axes.traverse((child) => {
                if (child.isMesh) {
                    child.castShadow = false;
                    child.receiveShadow = false;
                }
            });
        });

        // Ensure joint axes don't cast shadows
        this.jointAxesHelpers.forEach((axisInfo) => {
            axisInfo.mesh.traverse((child) => {
                if (child.isMesh) {
                    child.castShadow = false;
                    child.receiveShadow = false;
                }
            });
        });
    }

    /**
     * Clear all link axes
     */
    clearAllLinkAxes() {
        this.linkAxesHelpers.forEach((axes, linkName) => {
            if (axes.parent) {
                axes.parent.remove(axes);
            }
        });
        this.linkAxesHelpers.clear();
    }

    /**
     * Clear all joint axes
     */
    clearAllJointAxes() {
        this.jointAxesHelpers.forEach((axisInfo, jointName) => {
            if (axisInfo.isAttached && axisInfo.parent) {
                axisInfo.parent.remove(axisInfo.mesh);
            }
        });
        this.jointAxesHelpers.clear();
    }

    /**
     * Clear all axes
     */
    clear() {
        this.clearAllLinkAxes();
        this.clearAllJointAxes();
    }
}

