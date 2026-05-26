import * as THREE from 'three';
import { MathUtils } from '../utils/MathUtils.js';
import { CoordinateAxesManager } from './CoordinateAxesManager.js';

/**
 * Center-of-mass and inertia helpers for the 3D viewer.
 *
 * COM markers use a Blender-style checkerboard sphere (eight sphere octants).
 * Size is uniform for every link on a given robot: it scales with overall model
 * height (same basis as link axes), not with each link's bounding box.
 */
export class InertialVisualization {
    constructor(sceneManager) {
        this.sceneManager = sceneManager;
        /** @type {THREE.Group[]} Per-link COM marker groups */
        this.comMarkers = [];
        /** @type {THREE.Mesh[]} Inertia box meshes (Gazebo-style) */
        this.inertiaEllipsoids = [];
        this.showCOM = false;
        this.showInertia = false;
        /** Max bbox dimension of the loaded robot (meters), used for uniform COM size */
        this.modelScale = 1;
    }

    /**
     * Store overall robot span after {@link CoordinateAxesManager.measureObjectScale}.
     * @param {number} modelScale
     */
    setModelScale(modelScale) {
        this.modelScale = Math.max(modelScale, 1e-6);
    }

    /**
     * Re-measure robot span from the loaded scene graph.
     * Call after async meshes finish loading so COM size matches the final model.
     * @param {object} model - Unified robot model with threeObject
     * @returns {number} Updated modelScale
     */
    updateModelScaleFromObject(model) {
        if (!model?.threeObject) return this.modelScale;
        const root = model.threeObject;
        if (this.sceneManager?.world?.children?.includes(root)) {
            this.sceneManager.world.updateMatrixWorld(true);
        }
        root.updateMatrixWorld(true);
        this.setModelScale(CoordinateAxesManager.measureObjectScale(root, this.modelScale));
        return this.modelScale;
    }

    /**
     * Uniform COM sphere radius for all links on one robot.
     * Derived from model-span axis length (~8.75% of height) × 0.15.
     * @param {number} modelScale - Max dimension of the full robot bbox
     * @returns {number} Radius in scene units (meters)
     */
    static computeCOMRadius(modelScale = 1) {
        const model = Math.max(modelScale, 1e-6);
        const axisLen = CoordinateAxesManager.computeLinkAxesLength(model, model);
        return axisLen * 0.15;
    }

    /**
     * Radius for the Measure panel overall-COM overlay (blue/white, slightly larger).
     * @param {number} modelScale
     * @returns {number}
     */
    static computeGlobalCOMRadius(modelScale = 1) {
        return InertialVisualization.computeCOMRadius(modelScale) * 1.4;
    }

    /**
     * Keep COM markers fully opaque (not affected by robot transparency pass).
     * @param {THREE.Material} material
     */
    static applyCOMMaterial(material) {
        material.transparent = false;
        material.opacity = 1;
        material.toneMapped = false;
        material.depthTest = true;
        material.depthWrite = true;
        material.needsUpdate = true;
    }

    /**
     * Create a Blender-style checkerboard sphere (eight quarter-sphere octants).
     * @param {number} radius - Sphere radius in scene units
     * @param {{ lightColor?: number, darkColor?: number }} [options]
     * @param {number} [options.lightColor=0xffffff] - Bright octant color (toolbar: white)
     * @param {number} [options.darkColor=0x000000] - Dark octant color (toolbar: black; Measure: blue)
     * @returns {THREE.Group}
     */
    static createCOMGeometry(radius, { lightColor = 0xffffff, darkColor = 0x000000 } = {}) {
        const comGroup = new THREE.Group();
        comGroup.userData.isCenterOfMass = true;
        comGroup.userData.isCOMMarker = true;
        comGroup.renderOrder = 100;
        const segments = 16;

        // Material configuration: fully opaque, proper depth testing
        const matLight = new THREE.MeshBasicMaterial({
            color: lightColor,
            side: THREE.DoubleSide
        });
        const matDark = new THREE.MeshBasicMaterial({
            color: darkColor,
            side: THREE.DoubleSide
        });
        InertialVisualization.applyCOMMaterial(matLight);
        InertialVisualization.applyCOMMaterial(matDark);

        // Eight quarter spheres with alternating colors (checkerboard)
        const sphereParts = [
            { phiStart: 0, phiLength: Math.PI/2, thetaStart: 0, thetaLength: Math.PI/2, material: matLight },
            { phiStart: 0, phiLength: Math.PI/2, thetaStart: Math.PI/2, thetaLength: Math.PI/2, material: matDark },
            { phiStart: Math.PI/2, phiLength: Math.PI/2, thetaStart: 0, thetaLength: Math.PI/2, material: matDark },
            { phiStart: Math.PI/2, phiLength: Math.PI/2, thetaStart: Math.PI/2, thetaLength: Math.PI/2, material: matLight },
            { phiStart: Math.PI, phiLength: Math.PI/2, thetaStart: 0, thetaLength: Math.PI/2, material: matLight },
            { phiStart: Math.PI, phiLength: Math.PI/2, thetaStart: Math.PI/2, thetaLength: Math.PI/2, material: matDark },
            { phiStart: Math.PI*1.5, phiLength: Math.PI/2, thetaStart: 0, thetaLength: Math.PI/2, material: matDark },
            { phiStart: Math.PI*1.5, phiLength: Math.PI/2, thetaStart: Math.PI/2, thetaLength: Math.PI/2, material: matLight }
        ];

        sphereParts.forEach(part => {
            const geometry = new THREE.SphereGeometry(
                radius, segments, segments,
                part.phiStart, part.phiLength,
                part.thetaStart, part.thetaLength
            );
            const mesh = new THREE.Mesh(geometry, part.material);
            mesh.userData.isCenterOfMass = true;
            mesh.userData.isCOMMarker = true;
            mesh.castShadow = false;
            mesh.receiveShadow = false;
            mesh.renderOrder = 100;
            // Allow raycasting so COM can be selected for dragging
            comGroup.add(mesh);
        });

        return comGroup;
    }

    /**
     * Extract inertial data from the model and rebuild COM / inertia helpers when enabled.
     * Refreshes modelScale first so marker size matches the loaded robot height.
     * @param {object} model - Unified robot model
     */
    extractInertialProperties(model) {
        this.updateModelScaleFromObject(model);

        // Clean up: remove from parent link objects
        this.comMarkers.forEach(marker => {
            if (marker.parent) {
                marker.parent.remove(marker);
            }
        });
        this.inertiaEllipsoids.forEach(ellipsoid => {
            if (ellipsoid.parent) {
                ellipsoid.parent.remove(ellipsoid);
            }
        });
        this.comMarkers = [];
        this.inertiaEllipsoids = [];

        if (!model.links) {
            return;
        }

        model.links.forEach((link, name) => {
            if (!link.inertial) return;

            const inertial = link.inertial;

            // Try to get COM position, handling various data formats
            let comPosition;
            try {
                if (inertial.origin && inertial.origin.xyz) {
                    comPosition = MathUtils.xyzToVector3(inertial.origin.xyz);
                } else if (inertial.origin) {
                    // Might be array form
                    comPosition = new THREE.Vector3(
                        inertial.origin[0] || 0,
                        inertial.origin[1] || 0,
                        inertial.origin[2] || 0
                    );
                } else {
                    comPosition = new THREE.Vector3(0, 0, 0);
                }
            } catch (error) {
                console.error(`Failed to extract COM position for Link ${name}:`, error);
                comPosition = new THREE.Vector3(0, 0, 0);
            }

            // Create COM marker (only when display is needed)
            if (this.showCOM && inertial.mass !== undefined && inertial.mass > 0) {
                this.createCOMMarker(model, link, comPosition);
            }

            // Create inertia ellipsoid (only when display is needed)
            if (this.showInertia && (inertial.ixx !== undefined || inertial.inertia)) {
                this.createInertiaEllipsoid(model, link, comPosition, inertial);
            }
        });
    }

    /**
     * Attach one COM marker to a link (black/white checkerboard, uniform radius per robot).
     * @param {object} model
     * @param {object} link
     * @param {THREE.Vector3} position - COM in link-local coordinates (URDF inertial origin)
     */
    createCOMMarker(model, link, position) {
        const linkObject = this.findLinkObject(model.threeObject, link.name);
        if (!linkObject) {
            return;
        }

        // Same radius on every link for this robot (scales with model height, not link bbox)
        const radius = InertialVisualization.computeCOMRadius(this.modelScale);
        const comGroup = InertialVisualization.createCOMGeometry(radius);

        comGroup.position.copy(position);
        comGroup.visible = this.showCOM;

        linkObject.add(comGroup);
        this.comMarkers.push(comGroup);
    }

    /**
     * Create inertia box visualization (Gazebo style)
     */
    createInertiaEllipsoid(model, link, comPosition, inertial) {
        // For MJCF models with quat, use the original diagonal inertia values
        // (before rotation), then apply the quat rotation to the visualization
        let inertiaForCalculation;
        if (inertial.diagonalInertia) {
            // Use the original diagonal inertia from MJCF inertial frame
            inertiaForCalculation = {
                ixx: inertial.diagonalInertia.ixx,
                iyy: inertial.diagonalInertia.iyy,
                izz: inertial.diagonalInertia.izz,
                ixy: 0, // No off-diagonal components
                ixz: 0,
                iyz: 0,
                mass: inertial.mass
            };
        } else {
            // For URDF or models without diagonalInertia, use as-is
            // (may have off-diagonal components)
            inertiaForCalculation = inertial;
        }

        // Calculate inertia box (like Gazebo)
        // This will perform eigendecomposition if there are off-diagonal components
        const boxData = MathUtils.computeInertiaBox(inertiaForCalculation);

        // If boxData is null, the inertia parameters are invalid or unreasonable
        // (e.g., very small mass with large inertia), so don't display the box
        if (!boxData) {
            return; // Skip creating inertia visualization
        }

        const boxGeometry = MathUtils.createInertiaBoxGeometry(
            boxData.width,
            boxData.height,
            boxData.depth
        );

        // Use semi-transparent light blue fill box (similar to collider style)
        const boxMaterial = new THREE.MeshPhongMaterial({
            transparent: true,
            opacity: 0.35,
            shininess: 2.5,
            premultipliedAlpha: true,
            color: 0x4a9eff,  // Light blue
            polygonOffset: true,
            polygonOffsetFactor: -1,
            polygonOffsetUnits: -1,
        });

        const inertiaBox = new THREE.Mesh(boxGeometry, boxMaterial);
        inertiaBox.position.copy(comPosition);

        // Apply rotation
        if (inertial.origin && inertial.origin.quat) {
            // For MJCF: Need to transform the quat from MJCF frame to Three.js frame
            // The quat represents rotation in MJCF coordinate system
            // In MJCFAdapter.parseInertial, the inertia tensor is:
            // 1. Rotated by quat in MJCF frame
            // 2. Then transformed to Three.js frame by Y-axis 180° rotation
            //
            // For the visual ellipsoid, try applying coord conversion first:
            const quat = inertial.origin.quat;
            const mjcfQuat = new THREE.Quaternion(quat.x, quat.y, quat.z, quat.w);

            // Apply coordinate system transformation: 180° around Y axis
            // This matches the two 90° Y-rotations in MJCFAdapter.parseInertial
            const coordConversionQuat = new THREE.Quaternion();
            coordConversionQuat.setFromAxisAngle(new THREE.Vector3(0, 1, 0), Math.PI);

            // Try reversed order: first coord conversion, then quat
            const finalQuat = new THREE.Quaternion();
            finalQuat.multiplyQuaternions(mjcfQuat, coordConversionQuat);

            inertiaBox.quaternion.copy(finalQuat);
        } else if (boxData.rotation && boxData.rotation.w !== undefined) {
            // For URDF with off-diagonal components: use the rotation from eigendecomposition
            inertiaBox.quaternion.copy(boxData.rotation);
        } else if (inertial.origin && inertial.origin.rpy) {
            // For URDF with RPY (though this is usually just for COM position, not inertia orientation)
            const rpy = inertial.origin.rpy;
            inertiaBox.rotation.set(rpy[0], rpy[1], rpy[2], 'XYZ');
        } else {
            inertiaBox.rotation.set(0, 0, 0);
        }

        inertiaBox.visible = this.showInertia;
        inertiaBox.castShadow = false;
        inertiaBox.receiveShadow = false;

        // Allow raycasting so inertia box can be selected for dragging
        // Mark as inertia box
        inertiaBox.userData.isInertiaBox = true;

        const linkObject = this.findLinkObject(model.threeObject, link.name);
        if (linkObject) {
            linkObject.add(inertiaBox);
        } else {
            this.sceneManager.scene.add(inertiaBox);
        }

        this.inertiaEllipsoids.push(inertiaBox); // Although called ellipsoids, they're boxes now
    }

    /**
     * Find a link's Three.js object in the URDF/MJCF scene graph.
     * @param {THREE.Object3D} root - Model root (model.threeObject)
     * @param {string} linkName
     * @returns {THREE.Object3D|null}
     */
    findLinkObject(root, linkName) {
        let found = null;
        root.traverse((child) => {
            if (child.name === linkName || child.name === `link_${linkName}` || child.name === `body_${linkName}`) {
                found = child;
            }
        });
        return found;
    }

    /**
     * Toggle per-link COM markers. Rebuilds geometry when turned on (current model scale).
     * @param {boolean} show
     * @param {object|null} currentModel
     */
    toggleCenterOfMass(show, currentModel) {
        this.showCOM = show;

        if (show && currentModel) {
            this.extractInertialProperties(currentModel);
        } else {
            this.comMarkers.forEach(marker => {
                marker.visible = false;
            });
        }
    }

    /**
     * Toggle inertia box display.
     * @param {boolean} show
     * @param {object|null} currentModel
     */
    toggleInertia(show, currentModel) {
        this.showInertia = show;

        if (show && currentModel) {
            this.extractInertialProperties(currentModel);
        } else {
            this.inertiaEllipsoids.forEach(ellipsoid => {
                ellipsoid.visible = show;
            });
        }
    }

    /**
     * Remove all COM and inertia helpers from the scene.
     */
    clear() {
        this.comMarkers.forEach(marker => {
            if (marker.parent) marker.parent.remove(marker);
        });
        this.inertiaEllipsoids.forEach(ellipsoid => {
            if (ellipsoid.parent) ellipsoid.parent.remove(ellipsoid);
        });
        this.comMarkers = [];
        this.inertiaEllipsoids = [];
    }
}

