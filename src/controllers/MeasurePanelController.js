/**
 * MeasurePanelController - Measure panel controller
 * Provides robot inspection: total weight, dimensions, per-link/limb data,
 * and joint-to-joint distance measurement with 3D visualization.
 */
import * as THREE from 'three';
import { CoordinateAxesManager } from '../renderer/CoordinateAxesManager.js';
import { InertialVisualization } from '../renderer/InertialVisualization.js';

export class MeasurePanelController {
    constructor(sceneManager) {
        this.sceneManager = sceneManager;
        this.currentModel = null;
        this.activeTab = 'overview';
        this.distanceSelection = { first: null, second: null };
        this.distanceLine = null;
        this.distanceLabels = [];
        this.sortColumn = null;
        this.sortAscending = true;

        this.showGlobalCOM = false;
        this.globalCOMMarker = null;
        this.showBBox = false;
        this.bboxHelper = null;

        this._limbsCache = null;
        this._linkToLimbMap = null;
    }

    update(model) {
        this.currentModel = model;
        this.clearDistanceMeasurement();
        this.clearOverlays();
        this._limbsCache = null;
        this._linkToLimbMap = null;
        this.render();
    }

    clear() {
        this.currentModel = null;
        this.clearDistanceMeasurement();
        this.clearOverlays();
        this._limbsCache = null;
        this._linkToLimbMap = null;
        const container = document.getElementById('measure-panel-content');
        if (container) {
            const t = (k) => window.i18n?.t(k) || k;
            container.innerHTML = `<div class="empty-state">${t('noModel')}</div>`;
        }
    }

    render() {
        const container = document.getElementById('measure-panel-content');
        if (!container || !this.currentModel) return;

        const t = (k) => window.i18n?.t(k) || k;
        container.innerHTML = '';

        const tabBar = document.createElement('div');
        tabBar.className = 'measure-tab-bar';

        const tabs = [
            { id: 'overview', label: t('measureOverview') },
            { id: 'links', label: t('measureLinks') },
            { id: 'limbs', label: t('measureLimbs') },
            { id: 'distance', label: t('measureDistance') }
        ];

        tabs.forEach(tab => {
            const btn = document.createElement('button');
            btn.className = `measure-tab-btn ${this.activeTab === tab.id ? 'active' : ''}`;
            btn.textContent = tab.label;
            btn.addEventListener('click', () => {
                this.activeTab = tab.id;
                this.render();
            });
            tabBar.appendChild(btn);
        });
        container.appendChild(tabBar);

        const content = document.createElement('div');
        content.className = 'measure-tab-content';

        switch (this.activeTab) {
            case 'overview': this.renderOverview(content); break;
            case 'links': this.renderLinksTable(content); break;
            case 'limbs': this.renderLimbsTable(content); break;
            case 'distance': this.renderDistanceTool(content); break;
        }
        container.appendChild(content);
    }

    // ==================== Overview Tab ====================

    renderOverview(container) {
        const model = this.currentModel;
        const t = (k) => window.i18n?.t(k) || k;
        const stats = this.computeOverviewStats(model);

        const html = `
            <div class="measure-overview">
                <div class="measure-stat-group">
                    <div class="measure-stat-title">${t('measureTotalMass')}</div>
                    <div class="measure-stat-value">${stats.totalMass.toFixed(3)} kg</div>
                </div>
                <div class="measure-stat-group" style="position:relative;">
                    <div class="measure-stat-row-header">
                        <div class="measure-stat-title">${t('measureBoundingBox')}</div>
                        <label class="measure-toggle"><input type="checkbox" id="measure-toggle-bbox" ${this.showBBox ? 'checked' : ''}/><span>${t('measureShowVisual')}</span></label>
                    </div>
                    <div class="measure-stat-grid">
                        <span class="measure-dim-label">X:</span>
                        <span class="measure-dim-value">${(stats.bbox.x * 1000).toFixed(1)} mm</span>
                        <span class="measure-dim-label">Y:</span>
                        <span class="measure-dim-value">${(stats.bbox.y * 1000).toFixed(1)} mm</span>
                        <span class="measure-dim-label">Z:</span>
                        <span class="measure-dim-value">${(stats.bbox.z * 1000).toFixed(1)} mm</span>
                    </div>
                    <button class="measure-refresh-mini" id="measure-refresh-bbox" title="${t('measureRefresh')}">⟳</button>
                </div>
                <div class="measure-stat-group" style="position:relative;">
                    <div class="measure-stat-row-header">
                        <div class="measure-stat-title">${t('measureCenterOfMass')}</div>
                        <label class="measure-toggle"><input type="checkbox" id="measure-toggle-com" ${this.showGlobalCOM ? 'checked' : ''}/><span>${t('measureShowVisual')}</span></label>
                    </div>
                    <div class="measure-stat-grid">
                        <span class="measure-dim-label">X:</span>
                        <span class="measure-dim-value">${(stats.com.x * 1000).toFixed(1)} mm</span>
                        <span class="measure-dim-label">Y:</span>
                        <span class="measure-dim-value">${(stats.com.y * 1000).toFixed(1)} mm</span>
                        <span class="measure-dim-label">Z:</span>
                        <span class="measure-dim-value">${(stats.com.z * 1000).toFixed(1)} mm</span>
                    </div>
                    <button class="measure-refresh-mini" id="measure-refresh-com" title="${t('measureRefresh')}">⟳</button>
                </div>
                <div class="measure-stat-group">
                    <div class="measure-stat-title">${t('measureStructure')}</div>
                    <div class="measure-stat-grid">
                        <span class="measure-dim-label">${t('links')}:</span>
                        <span class="measure-dim-value">${stats.linkCount}</span>
                        <span class="measure-dim-label">${t('joints')}:</span>
                        <span class="measure-dim-value">${stats.jointCount}</span>
                        <span class="measure-dim-label">${t('controllable')}:</span>
                        <span class="measure-dim-value">${stats.controllableJoints}</span>
                    </div>
                </div>
            </div>
        `;
        container.innerHTML = html;

        container.querySelector('#measure-toggle-com')?.addEventListener('change', (e) => {
            this.showGlobalCOM = e.target.checked;
            if (this.showGlobalCOM) {
                this.addGlobalCOMMarker(stats.com);
            } else {
                this.removeGlobalCOMMarker();
            }
        });

        container.querySelector('#measure-toggle-bbox')?.addEventListener('change', (e) => {
            this.showBBox = e.target.checked;
            if (this.showBBox) {
                this.addBBoxHelper();
            } else {
                this.removeBBoxHelper();
            }
        });

        container.querySelector('#measure-refresh-bbox')?.addEventListener('click', () => {
            if (this.showBBox) this.addBBoxHelper();
            this.render();
        });

        container.querySelector('#measure-refresh-com')?.addEventListener('click', () => {
            const freshStats = this.computeOverviewStats(this.currentModel);
            if (this.showGlobalCOM) this.addGlobalCOMMarker(freshStats.com);
            this.render();
        });
    }

    computeOverviewStats(model) {
        let totalMass = 0;
        const com = new THREE.Vector3();

        model.links.forEach(link => {
            if (link.inertial && link.inertial.mass > 0) {
                const mass = link.inertial.mass;
                totalMass += mass;

                if (link.threeObject) {
                    const linkWorldPos = new THREE.Vector3();
                    link.threeObject.updateMatrixWorld(true);
                    link.threeObject.getWorldPosition(linkWorldPos);
                    com.addScaledVector(linkWorldPos, mass);
                } else {
                    com.x += mass * link.inertial.origin.xyz[0];
                    com.y += mass * link.inertial.origin.xyz[1];
                    com.z += mass * link.inertial.origin.xyz[2];
                }
            }
        });

        if (totalMass > 0) {
            com.divideScalar(totalMass);
        }

        const bbox = { x: 0, y: 0, z: 0 };
        if (model.threeObject) {
            model.threeObject.updateMatrixWorld(true);
            const box = new THREE.Box3().setFromObject(model.threeObject);
            if (!box.isEmpty()) {
                const size = box.getSize(new THREE.Vector3());
                bbox.x = size.x;
                bbox.y = size.y;
                bbox.z = size.z;
            }
        }

        const jointCount = model.joints ? model.joints.size : 0;
        const controllableJoints = model.joints
            ? Array.from(model.joints.values()).filter(j => j.type !== 'fixed').length
            : 0;

        return {
            totalMass, com, bbox,
            linkCount: model.links ? model.links.size : 0,
            jointCount, controllableJoints
        };
    }

    // ==================== Overview 3D Overlays ====================

    /**
     * Show overall center of mass in the scene (blue/white checkerboard, larger than per-link COM).
     */
    addGlobalCOMMarker(comPos) {
        this.removeGlobalCOMMarker();
        const model = this.currentModel;
        const iv = this.sceneManager?.inertialVisualization;
        if (iv && model) {
            iv.updateModelScaleFromObject(model);
        }
        const modelScale = iv?.modelScale ?? 1;
        const radius = InertialVisualization.computeGlobalCOMRadius(modelScale);
        const marker = InertialVisualization.createCOMGeometry(radius, {
            lightColor: 0xffffff,
            darkColor: 0x3b82f6 // Distinct from toolbar COM (black/white)
        });
        marker.userData.isGlobalCOM = true;
        marker.position.copy(comPos);
        marker.name = 'measureGlobalCOM';
        marker.renderOrder = 998;
        this.sceneManager.scene.add(marker);
        this.globalCOMMarker = marker;
        this._setModelTransparency(true);
        this.sceneManager.redraw();
        this.sceneManager.render();
    }

    removeGlobalCOMMarker() {
        if (this.globalCOMMarker) {
            this.sceneManager.scene.remove(this.globalCOMMarker);
            this.globalCOMMarker = null;
            if (!this._isOtherTransparencyActive()) {
                this._setModelTransparency(false);
            }
            this.sceneManager.redraw();
        }
    }

    _setModelTransparency(transparent) {
        this.sceneManager?.setTransparencyEnabled(!!transparent);
    }

    _isOtherTransparencyActive() {
        const iv = this.sceneManager?.inertialVisualization;
        const am = this.sceneManager?.axesManager;
        const vm = this.sceneManager?.visualizationManager;
        return (iv?.showCOM) || (am?.showAxesEnabled) || (am?.showJointAxesEnabled) ||
            (vm?.transparencyEnabled);
    }

    addBBoxHelper() {
        this.removeBBoxHelper();
        const model = this.currentModel;
        if (!model || !model.threeObject) return;

        model.threeObject.updateMatrixWorld(true);
        const box = new THREE.Box3().setFromObject(model.threeObject);
        if (box.isEmpty()) return;

        const size = box.getSize(new THREE.Vector3());
        const center = box.getCenter(new THREE.Vector3());

        const hx = size.x / 2, hy = size.y / 2, hz = size.z / 2;
        const corners = [
            new THREE.Vector3(-hx, -hy, -hz), new THREE.Vector3( hx, -hy, -hz),
            new THREE.Vector3( hx,  hy, -hz), new THREE.Vector3(-hx,  hy, -hz),
            new THREE.Vector3(-hx, -hy,  hz), new THREE.Vector3( hx, -hy,  hz),
            new THREE.Vector3( hx,  hy,  hz), new THREE.Vector3(-hx,  hy,  hz)
        ];
        const edgePairs = [
            [0,1],[1,2],[2,3],[3,0],
            [4,5],[5,6],[6,7],[7,4],
            [0,4],[1,5],[2,6],[3,7]
        ];

        const bboxGroup = new THREE.Group();
        bboxGroup.name = 'measureBBox';
        bboxGroup.renderOrder = 997;

        edgePairs.forEach(([a, b]) => {
            const geo = new THREE.BufferGeometry().setFromPoints([corners[a], corners[b]]);
            const mat = new THREE.LineDashedMaterial({
                color: 0x0a84ff,
                dashSize: 0.015,
                gapSize: 0.008,
                transparent: true,
                opacity: 0.7,
                depthTest: true
            });
            const line = new THREE.Line(geo, mat);
            line.computeLineDistances();
            bboxGroup.add(line);
        });

        bboxGroup.position.copy(center);
        this.bboxHelper = bboxGroup;
        this.sceneManager.scene.add(this.bboxHelper);
        this.sceneManager.redraw();
        this.sceneManager.render();
    }

    removeBBoxHelper() {
        if (this.bboxHelper) {
            this.sceneManager.scene.remove(this.bboxHelper);
            this.bboxHelper.traverse(child => {
                if (child.geometry) child.geometry.dispose();
                if (child.material) child.material.dispose();
            });
            this.bboxHelper = null;
            this.sceneManager.redraw();
        }
    }

    clearOverlays() {
        this.removeGlobalCOMMarker();
        this.removeBBoxHelper();
        this.showGlobalCOM = false;
        this.showBBox = false;
    }

    // ==================== Limb Identification ====================

    /**
     * Classify a link/joint name into an anatomical category using
     * standardized ROS / human-anatomy keywords.
     */
    classifyLimbCategory(linkNames, jointNames) {
        const allNames = [...linkNames, ...jointNames].map(n => n.toLowerCase());
        const text = allNames.join(' ');

        const patterns = [
            { category: 'Right Hand', keywords: ['right_hand', 'r_hand', 'rhand', 'right_finger', 'r_finger', 'right_gripper', 'r_gripper'] },
            { category: 'Left Hand',  keywords: ['left_hand', 'l_hand', 'lhand', 'left_finger', 'l_finger', 'left_gripper', 'l_gripper'] },
            { category: 'Right Arm',  keywords: ['right_shoulder', 'right_elbow', 'right_wrist', 'r_shoulder', 'r_elbow', 'r_wrist', 'right_arm', 'r_arm', 'rarm', 'right_upper_arm', 'right_forearm'] },
            { category: 'Left Arm',   keywords: ['left_shoulder', 'left_elbow', 'left_wrist', 'l_shoulder', 'l_elbow', 'l_wrist', 'left_arm', 'l_arm', 'larm', 'left_upper_arm', 'left_forearm'] },
            { category: 'Right Leg',  keywords: ['right_hip', 'right_knee', 'right_ankle', 'r_hip', 'r_knee', 'r_ankle', 'right_leg', 'r_leg', 'rleg', 'right_thigh', 'right_shin', 'right_foot', 'r_foot'] },
            { category: 'Left Leg',   keywords: ['left_hip', 'left_knee', 'left_ankle', 'l_hip', 'l_knee', 'l_ankle', 'left_leg', 'l_leg', 'lleg', 'left_thigh', 'left_shin', 'left_foot', 'l_foot'] },
            { category: 'Neck',       keywords: ['neck', 'head'] },
            { category: 'Waist',      keywords: ['waist', 'torso', 'trunk', 'chest', 'spine', 'back'] },
            { category: 'Base',       keywords: ['base', 'pelvis', 'hip_link', 'root', 'body'] }
        ];

        for (const { category, keywords } of patterns) {
            for (const kw of keywords) {
                if (text.includes(kw)) return category;
            }
        }

        if (text.match(/\bright\b/) || text.match(/_r_/) || text.match(/_r$/)) {
            if (text.match(/shoulder|elbow|wrist|arm/)) return 'Right Arm';
            if (text.match(/hip|knee|ankle|leg|foot|thigh/)) return 'Right Leg';
            if (text.match(/hand|finger|gripper/)) return 'Right Hand';
        }
        if (text.match(/\bleft\b/) || text.match(/_l_/) || text.match(/_l$/)) {
            if (text.match(/shoulder|elbow|wrist|arm/)) return 'Left Arm';
            if (text.match(/hip|knee|ankle|leg|foot|thigh/)) return 'Left Leg';
            if (text.match(/hand|finger|gripper/)) return 'Left Hand';
        }

        return 'Other';
    }

    // ==================== Limb Chain Detection ====================

    computeLimbsData(model) {
        if (this._limbsCache) return this._limbsCache;
        if (!model.joints || !model.links) return [];

        const childrenMap = new Map();
        const parentMap = new Map();

        model.joints.forEach(joint => {
            if (joint.parent && joint.child) {
                if (!childrenMap.has(joint.parent)) childrenMap.set(joint.parent, []);
                childrenMap.get(joint.parent).push({ joint: joint.name, child: joint.child });
                parentMap.set(joint.child, { parent: joint.parent, joint: joint.name });
            }
        });

        const branchPoints = new Set();
        childrenMap.forEach((children, parent) => {
            if (children.length > 1) branchPoints.add(parent);
        });

        const leafLinks = new Set();
        model.links.forEach((link, name) => {
            if (!childrenMap.has(name) || childrenMap.get(name).length === 0) {
                leafLinks.add(name);
            }
        });

        const limbs = [];
        const linkToLimbMap = new Map();

        leafLinks.forEach(leaf => {
            const chain = [];
            const jointNames = [];
            let current = leaf;

            while (current) {
                if (!parentMap.has(current)) {
                    chain.unshift(current);
                    break;
                }
                const { parent, joint } = parentMap.get(current);
                chain.unshift(current);
                jointNames.unshift(joint);

                if (branchPoints.has(parent)) {
                    chain.unshift(parent);
                    break;
                }
                current = parent;
            }

            if (chain.length < 2) return;

            const category = this.classifyLimbCategory(chain, jointNames);

            let totalMass = 0;
            chain.forEach(linkName => {
                const link = model.links.get(linkName);
                if (link && link.inertial) totalMass += link.inertial.mass;
            });

            const firstLink = model.links.get(chain[0]);
            const lastLink = model.links.get(chain[chain.length - 1]);
            let dx = 0, dy = 0, dz = 0, dist = 0;

            if (firstLink?.threeObject && lastLink?.threeObject) {
                const posA = new THREE.Vector3();
                const posB = new THREE.Vector3();
                firstLink.threeObject.updateMatrixWorld(true);
                lastLink.threeObject.updateMatrixWorld(true);
                firstLink.threeObject.getWorldPosition(posA);
                lastLink.threeObject.getWorldPosition(posB);
                dx = posB.x - posA.x;
                dy = posB.y - posA.y;
                dz = posB.z - posA.z;
                dist = Math.sqrt(dx * dx + dy * dy + dz * dz);
            }

            const limbName = `${chain[0]} → ${chain[chain.length - 1]}`;
            const limbEntry = {
                category,
                name: limbName,
                linkCount: chain.length,
                jointCount: jointNames.length,
                mass: totalMass,
                dx: dx * 1000,
                dy: dy * 1000,
                dz: dz * 1000,
                dist: dist * 1000,
                chainLinks: chain
            };
            limbs.push(limbEntry);

            chain.forEach(linkName => {
                linkToLimbMap.set(linkName, category);
            });
        });

        this._limbsCache = limbs;
        this._linkToLimbMap = linkToLimbMap;
        return limbs;
    }

    getLinkToLimbMap() {
        if (!this._linkToLimbMap) this.computeLimbsData(this.currentModel);
        return this._linkToLimbMap || new Map();
    }

    // ==================== Links Table Tab ====================

    renderLinksTable(container) {
        const model = this.currentModel;
        const t = (k) => window.i18n?.t(k) || k;
        const limbMap = this.getLinkToLimbMap();

        const rows = [];
        if (model.links) {
            model.links.forEach((link, name) => {
                rows.push({
                    name,
                    mass: link.inertial ? link.inertial.mass : 0,
                    limb: limbMap.get(name) || '—'
                });
            });
        }

        const table = document.createElement('table');
        table.className = 'measure-table';

        const thead = document.createElement('thead');
        const headerRow = document.createElement('tr');
        const columns = [
            { key: 'name', label: t('measureLinkName') },
            { key: 'mass', label: `${t('mass')} (kg)` },
            { key: 'limb', label: t('measureLimbCategory') }
        ];

        columns.forEach(col => {
            const th = document.createElement('th');
            th.textContent = col.label;
            th.className = 'measure-th-sortable';
            th.addEventListener('click', () => {
                if (this.sortColumn === col.key) {
                    this.sortAscending = !this.sortAscending;
                } else {
                    this.sortColumn = col.key;
                    this.sortAscending = true;
                }
                this.render();
            });
            if (this.sortColumn === col.key) {
                th.textContent += this.sortAscending ? ' ▲' : ' ▼';
            }
            headerRow.appendChild(th);
        });
        thead.appendChild(headerRow);
        table.appendChild(thead);

        let sortedRows = [...rows];
        if (this.sortColumn) {
            sortedRows.sort((a, b) => {
                let va = a[this.sortColumn];
                let vb = b[this.sortColumn];
                if (typeof va === 'number' && typeof vb === 'number') {
                    return this.sortAscending ? va - vb : vb - va;
                }
                va = String(va || '');
                vb = String(vb || '');
                return this.sortAscending ? va.localeCompare(vb) : vb.localeCompare(va);
            });
        }

        const tbody = document.createElement('tbody');
        sortedRows.forEach(row => {
            const tr = document.createElement('tr');
            tr.addEventListener('click', () => this.highlightLink(row.name));
            tr.innerHTML = `
                <td title="${row.name}">${row.name}</td>
                <td>${row.mass.toFixed(3)}</td>
                <td>${row.limb}</td>
            `;
            tbody.appendChild(tr);
        });
        table.appendChild(tbody);

        const wrapper = document.createElement('div');
        wrapper.className = 'measure-table-wrapper';
        wrapper.appendChild(table);
        container.appendChild(wrapper);
    }

    // ==================== Limbs Table Tab ====================

    renderLimbsTable(container) {
        const model = this.currentModel;
        const t = (k) => window.i18n?.t(k) || k;

        const limbs = this.computeLimbsData(model);

        if (limbs.length === 0) {
            container.innerHTML = `<div class="empty-state">${t('measureNoLimbs')}</div>`;
            return;
        }

        const table = document.createElement('table');
        table.className = 'measure-table';

        const thead = document.createElement('thead');
        const headerRow = document.createElement('tr');
        const columns = [
            { key: 'category', label: t('measureLimbCategory') },
            { key: 'name', label: t('measureLimbName') },
            { key: 'linkCount', label: t('measureLimbLinks') },
            { key: 'jointCount', label: t('measureLimbJoints') },
            { key: 'mass', label: `${t('mass')} (kg)` },
            { key: 'dist', label: `${t('measureLimbLength')} (mm)` }
        ];

        columns.forEach(col => {
            const th = document.createElement('th');
            th.textContent = col.label;
            th.className = 'measure-th-sortable';
            th.addEventListener('click', () => {
                if (this.sortColumn === col.key) {
                    this.sortAscending = !this.sortAscending;
                } else {
                    this.sortColumn = col.key;
                    this.sortAscending = true;
                }
                this.render();
            });
            if (this.sortColumn === col.key) {
                th.textContent += this.sortAscending ? ' ▲' : ' ▼';
            }
            headerRow.appendChild(th);
        });
        thead.appendChild(headerRow);
        table.appendChild(thead);

        let sortedLimbs = [...limbs];
        if (this.sortColumn) {
            sortedLimbs.sort((a, b) => {
                let va = a[this.sortColumn];
                let vb = b[this.sortColumn];
                if (typeof va === 'number' && typeof vb === 'number') {
                    return this.sortAscending ? va - vb : vb - va;
                }
                va = String(va || '');
                vb = String(vb || '');
                return this.sortAscending ? va.localeCompare(vb) : vb.localeCompare(va);
            });
        }

        const tbody = document.createElement('tbody');
        sortedLimbs.forEach(limb => {
            const tr = document.createElement('tr');
            tr.innerHTML = `
                <td><span class="measure-category-badge">${limb.category}</span></td>
                <td title="${limb.name}">${limb.name}</td>
                <td>${limb.linkCount}</td>
                <td>${limb.jointCount}</td>
                <td>${limb.mass.toFixed(3)}</td>
                <td>${limb.dist.toFixed(1)}</td>
            `;

            const detailRow = document.createElement('tr');
            detailRow.className = 'measure-limb-detail-row';
            detailRow.innerHTML = `
                <td colspan="6" class="measure-limb-detail">
                    ΔX: ${limb.dx.toFixed(1)} mm &nbsp;
                    ΔY: ${limb.dy.toFixed(1)} mm &nbsp;
                    ΔZ: ${limb.dz.toFixed(1)} mm &nbsp;
                    ‖d‖: ${limb.dist.toFixed(1)} mm
                </td>
            `;
            detailRow.style.display = 'none';

            tr.style.cursor = 'pointer';
            tr.addEventListener('click', () => {
                detailRow.style.display = detailRow.style.display === 'none' ? '' : 'none';
            });

            tbody.appendChild(tr);
            tbody.appendChild(detailRow);
        });
        table.appendChild(tbody);

        const wrapper = document.createElement('div');
        wrapper.className = 'measure-table-wrapper';
        wrapper.appendChild(table);
        container.appendChild(wrapper);
    }

    // ==================== Distance Tab ====================

    renderDistanceTool(container) {
        const model = this.currentModel;
        const t = (k) => window.i18n?.t(k) || k;

        const joints = [];
        if (model.joints) {
            model.joints.forEach((joint, name) => {
                if (joint.type !== 'fixed') joints.push(name);
            });
            model.joints.forEach((joint, name) => {
                if (joint.type === 'fixed') joints.push(name);
            });
        }

        const html = `
            <div class="measure-distance-tool">
                <div class="measure-select-group">
                    <label>${t('measureJoint')} A:</label>
                    <select id="measure-joint-a" class="measure-select">
                        <option value="">— ${t('measureSelectJoint')} —</option>
                        ${joints.map(j => `<option value="${j}" ${this.distanceSelection.first === j ? 'selected' : ''}>${j}</option>`).join('')}
                    </select>
                </div>
                <div class="measure-select-group">
                    <label>${t('measureJoint')} B:</label>
                    <select id="measure-joint-b" class="measure-select">
                        <option value="">— ${t('measureSelectJoint')} —</option>
                        ${joints.map(j => `<option value="${j}" ${this.distanceSelection.second === j ? 'selected' : ''}>${j}</option>`).join('')}
                    </select>
                </div>
                <div class="measure-distance-buttons">
                    <button id="measure-distance-btn" class="measure-action-btn">${t('measureCompute')}</button>
                    <button id="measure-clear-btn" class="measure-action-btn secondary">${t('measureClear')}</button>
                </div>
                <div id="measure-distance-result" class="measure-distance-result"></div>
            </div>
        `;
        container.innerHTML = html;

        const selA = container.querySelector('#measure-joint-a');
        const selB = container.querySelector('#measure-joint-b');
        const btn = container.querySelector('#measure-distance-btn');
        const clearBtn = container.querySelector('#measure-clear-btn');

        if (selA) selA.addEventListener('change', (e) => { this.distanceSelection.first = e.target.value || null; });
        if (selB) selB.addEventListener('change', (e) => { this.distanceSelection.second = e.target.value || null; });

        if (btn) {
            btn.addEventListener('click', () => {
                this.computeAndShowDistance();
            });
        }

        if (clearBtn) {
            clearBtn.addEventListener('click', () => {
                this.clearDistanceMeasurement();
                this.distanceSelection = { first: null, second: null };
                if (selA) selA.value = '';
                if (selB) selB.value = '';
                const resultDiv = container.querySelector('#measure-distance-result');
                if (resultDiv) resultDiv.innerHTML = '';
            });
        }

        if (this.distanceSelection.first && this.distanceSelection.second && this.distanceLine) {
            this.showDistanceResult(container);
        }
    }

    // ==================== Distance Measurement Logic ====================

    getJointWorldPosition(jointName) {
        const model = this.currentModel;
        if (!model || !model.joints) return null;
        const joint = model.joints.get(jointName);
        if (!joint || !joint.threeObject) return null;

        const pos = new THREE.Vector3();
        joint.threeObject.updateMatrixWorld(true);
        joint.threeObject.getWorldPosition(pos);
        return pos;
    }

    computeAndShowDistance() {
        const { first, second } = this.distanceSelection;
        if (!first || !second) return;

        const posA = this.getJointWorldPosition(first);
        const posB = this.getJointWorldPosition(second);
        if (!posA || !posB) return;

        this.clearDistanceLine();

        const lineMat = new THREE.LineDashedMaterial({
            color: 0x0a84ff,
            dashSize: 0.02,
            gapSize: 0.01,
            depthTest: false,
            transparent: true,
            opacity: 0.9
        });

        const lineGeo = new THREE.BufferGeometry().setFromPoints([posA, posB]);
        this.distanceLine = new THREE.Line(lineGeo, lineMat);
        this.distanceLine.computeLineDistances();
        this.distanceLine.name = 'measureDistanceLine';
        this.distanceLine.renderOrder = 999;
        this.sceneManager.scene.add(this.distanceLine);

        const axisPoints = [
            { points: [posA, new THREE.Vector3(posB.x, posA.y, posA.z)], color: 0xff4444 },
            { points: [new THREE.Vector3(posB.x, posA.y, posA.z), new THREE.Vector3(posB.x, posB.y, posA.z)], color: 0x44ff44 },
            { points: [new THREE.Vector3(posB.x, posB.y, posA.z), posB], color: 0x4488ff }
        ];

        axisPoints.forEach(({ points, color }) => {
            const geo = new THREE.BufferGeometry().setFromPoints(points);
            const mat = new THREE.LineDashedMaterial({
                color, dashSize: 0.01, gapSize: 0.008,
                depthTest: false, transparent: true, opacity: 0.5
            });
            const line = new THREE.Line(geo, mat);
            line.computeLineDistances();
            line.renderOrder = 998;
            line.name = 'measureAxisLine';
            this.sceneManager.scene.add(line);
            this.distanceLabels.push(line);
        });

        const sphereGeo = new THREE.SphereGeometry(0.008, 16, 16);
        const sphereMat = new THREE.MeshBasicMaterial({ color: 0x0a84ff, depthTest: false });

        const sphereA = new THREE.Mesh(sphereGeo, sphereMat);
        sphereA.position.copy(posA);
        sphereA.renderOrder = 999;
        this.sceneManager.scene.add(sphereA);
        this.distanceLabels.push(sphereA);

        const sphereB = new THREE.Mesh(sphereGeo.clone(), sphereMat.clone());
        sphereB.position.copy(posB);
        sphereB.renderOrder = 999;
        this.sceneManager.scene.add(sphereB);
        this.distanceLabels.push(sphereB);

        this.sceneManager.redraw();
        this.sceneManager.render();

        this.showDistanceResult();
    }

    showDistanceResult(parentContainer) {
        const { first, second } = this.distanceSelection;
        if (!first || !second) return;

        const posA = this.getJointWorldPosition(first);
        const posB = this.getJointWorldPosition(second);
        if (!posA || !posB) return;

        const t = (k) => window.i18n?.t(k) || k;
        const dx = posB.x - posA.x;
        const dy = posB.y - posA.y;
        const dz = posB.z - posA.z;
        const dist = Math.sqrt(dx * dx + dy * dy + dz * dz);

        const resultDiv = (parentContainer || document).querySelector('#measure-distance-result')
            || document.getElementById('measure-distance-result');
        if (resultDiv) {
            resultDiv.innerHTML = `
                <div class="measure-distance-value">
                    <strong>${t('measureTotalDistance')}:</strong> ${(dist * 1000).toFixed(1)} mm
                </div>
                <div class="measure-stat-grid">
                    <span class="measure-dim-label" style="color:#ff4444">ΔX:</span>
                    <span class="measure-dim-value">${(dx * 1000).toFixed(1)} mm</span>
                    <span class="measure-dim-label" style="color:#44ff44">ΔY:</span>
                    <span class="measure-dim-value">${(dy * 1000).toFixed(1)} mm</span>
                    <span class="measure-dim-label" style="color:#4488ff">ΔZ:</span>
                    <span class="measure-dim-value">${(dz * 1000).toFixed(1)} mm</span>
                </div>
            `;
        }
    }

    clearDistanceLine() {
        if (this.distanceLine) {
            this.sceneManager.scene.remove(this.distanceLine);
            this.distanceLine.geometry.dispose();
            this.distanceLine.material.dispose();
            this.distanceLine = null;
        }
        this.distanceLabels.forEach(obj => {
            this.sceneManager.scene.remove(obj);
            if (obj.geometry) obj.geometry.dispose();
            if (obj.material) obj.material.dispose();
        });
        this.distanceLabels = [];
    }

    clearDistanceMeasurement() {
        this.clearDistanceLine();
        this.sceneManager?.redraw();
        this.sceneManager?.render();
    }

    // ==================== Highlight Link in 3D ====================

    highlightLink(linkName) {
        if (!this.currentModel || !this.sceneManager) return;
        const link = this.currentModel.links.get(linkName);
        if (link && this.sceneManager.highlightManager) {
            this.sceneManager.highlightManager.highlightLink(link, this.currentModel);
            if (link.threeObject) {
                this.sceneManager.focusObject(link.threeObject);
            }
        }
    }
}
