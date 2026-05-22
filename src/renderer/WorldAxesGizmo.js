import * as THREE from 'three';

/**
 * WorldAxesGizmo - A SolidWorks-style orientation indicator
 * rendered in the bottom-left corner of the viewport.
 * Uses its own scene, camera, and renderer so it stays
 * independent of the main scene content.
 */
export class WorldAxesGizmo {
    constructor(mainCamera, parentElement) {
        this.mainCamera = mainCamera;

        const size = 120;
        this.size = size;
        this.canvas = document.createElement('canvas');
        this.canvas.id = 'world-axes-gizmo';
        this.canvas.width = size * window.devicePixelRatio;
        this.canvas.height = size * window.devicePixelRatio;
        this.canvas.style.cssText = `
            position: fixed;
            bottom: 16px;
            left: 16px;
            width: ${size}px;
            height: ${size}px;
            z-index: 50;
            pointer-events: none;
        `;
        parentElement.appendChild(this.canvas);

        this.renderer = new THREE.WebGLRenderer({
            canvas: this.canvas,
            antialias: true,
            alpha: true
        });
        this.renderer.setPixelRatio(window.devicePixelRatio);
        this.renderer.setSize(size, size);
        this.renderer.setClearColor(0x000000, 0);

        this.scene = new THREE.Scene();
        this.axesRoot = new THREE.Group();
        this.scene.add(this.axesRoot);

        const frustum = 1.8;
        this.camera = new THREE.OrthographicCamera(
            -frustum, frustum, frustum, -frustum, 0.1, 100
        );
        this.camera.position.set(0, 0, 5);
        this.camera.lookAt(0, 0, 0);

        this._buildAxes();

        this.worldRotation = new THREE.Euler();
    }

    _buildAxes() {
        const axisLength = 1.0;
        const headLength = 0.22;
        const headWidth = 0.1;
        const shaftRadius = 0.03;

        const axes = [
            { dir: new THREE.Vector3(1, 0, 0), color: 0xef4444, label: 'X' },
            { dir: new THREE.Vector3(0, 1, 0), color: 0x22c55e, label: 'Y' },
            { dir: new THREE.Vector3(0, 0, 1), color: 0x3b82f6, label: 'Z' }
        ];

        axes.forEach(({ dir, color, label }) => {
            const shaftLen = axisLength - headLength;
            const shaftGeo = new THREE.CylinderGeometry(shaftRadius, shaftRadius, shaftLen, 8);
            const shaftMat = new THREE.MeshBasicMaterial({ color });
            const shaft = new THREE.Mesh(shaftGeo, shaftMat);

            const coneGeo = new THREE.ConeGeometry(headWidth, headLength, 12);
            const coneMat = new THREE.MeshBasicMaterial({ color });
            const cone = new THREE.Mesh(coneGeo, coneMat);

            const group = new THREE.Group();
            shaft.position.y = shaftLen / 2;
            cone.position.y = shaftLen + headLength / 2;
            group.add(shaft);
            group.add(cone);

            if (dir.x === 1) {
                group.rotation.z = -Math.PI / 2;
            } else if (dir.z === 1) {
                group.rotation.x = Math.PI / 2;
            }

            this.axesRoot.add(group);

            this._addLabel(label, dir, color, axisLength + 0.3);
        });

        const sphereGeo = new THREE.SphereGeometry(0.08, 12, 12);
        const sphereMat = new THREE.MeshBasicMaterial({ color: 0x888888 });
        this.axesRoot.add(new THREE.Mesh(sphereGeo, sphereMat));
    }

    _addLabel(text, dir, color, distance) {
        const canvas = document.createElement('canvas');
        const ctx = canvas.getContext('2d');
        canvas.width = 64;
        canvas.height = 64;

        ctx.clearRect(0, 0, 64, 64);
        ctx.font = 'bold 44px -apple-system, BlinkMacSystemFont, "SF Pro Display", sans-serif';
        ctx.textAlign = 'center';
        ctx.textBaseline = 'middle';
        ctx.fillStyle = '#' + new THREE.Color(color).getHexString();
        ctx.fillText(text, 32, 32);

        const texture = new THREE.CanvasTexture(canvas);
        texture.minFilter = THREE.LinearFilter;

        const spriteMat = new THREE.SpriteMaterial({
            map: texture,
            transparent: true,
            depthTest: false
        });
        const sprite = new THREE.Sprite(spriteMat);
        sprite.position.copy(dir.clone().multiplyScalar(distance));
        sprite.scale.set(0.5, 0.5, 1);
        this.axesRoot.add(sprite);
    }

    /**
     * Apply the same coordinate-system rotation that SceneManager.setUp()
     * applies to the world group, so the gizmo labels match the viewport.
     */
    setUpAxis(up) {
        if (!up) up = '+Z';
        up = up.toUpperCase();
        const sign = up.replace(/[^-+]/g, '')[0] || '+';
        const axis = up.replace(/[^XYZ]/gi, '')[0] || 'Z';

        const PI = Math.PI;
        const HALFPI = PI / 2;

        if (axis === 'X') {
            this.worldRotation.set(0, 0, sign === '+' ? HALFPI : -HALFPI);
        } else if (axis === 'Z') {
            this.worldRotation.set(sign === '+' ? -HALFPI : HALFPI, 0, 0);
        } else if (axis === 'Y') {
            this.worldRotation.set(sign === '+' ? 0 : PI, 0, 0);
        }
    }

    update() {
        this.axesRoot.rotation.copy(this.worldRotation);
        const q = this.mainCamera.quaternion;
        this.camera.position.set(0, 0, 5).applyQuaternion(q);
        this.camera.quaternion.copy(q);
        this.renderer.render(this.scene, this.camera);
    }

    dispose() {
        this.renderer.dispose();
        if (this.canvas.parentElement) {
            this.canvas.parentElement.removeChild(this.canvas);
        }
    }
}
