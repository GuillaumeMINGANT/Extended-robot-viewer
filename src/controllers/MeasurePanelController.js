/**
 * MeasurePanelController - Measure panel controller
 * Provides robot inspection: total weight, dimensions, per-link/limb data,
 * and joint-to-joint distance measurement with 3D visualization.
 */
import * as THREE from 'three';
import { CoordinateAxesManager } from '../renderer/CoordinateAxesManager.js';
import { InertialVisualization } from '../renderer/InertialVisualization.js';
import { HumanoidKinematicsAnalyzer } from '../kinematics/HumanoidKinematicsAnalyzer.js';
import {
    loadMeasureUnits,
    formatAngleFromRad,
    formatLinearFromMeters,
    formatVelocityFromRadS,
    formatMeasureDecimal,
    formatTorqueFromNm,
    angleUnitLabel,
    linearUnitLabel,
    velocityUnitLabel,
    torqueUnitLabel
} from '../utils/MeasureUnitSettings.js';

export class MeasurePanelController {
    constructor(sceneManager) {
        this.sceneManager = sceneManager;
        this.currentModel = null;
        /** Display units for all measure tabs (see MeasureUnitSettings.js). */
        this.measureUnits = loadMeasureUnits();
        this.activeTab = 'overview';
        /** Distance tab: two joint names + scene overlays. */
        this.distanceSelection = { first: null, second: null };
        this.distanceLine = null;
        this.distanceLabels = [];
        this.sortColumn = null;
        this.sortAscending = true;

        // Overview tab optional 3D helpers
        this.showGlobalCOM = false;
        this.globalCOMMarker = null;
        this.showBBox = false;
        this.bboxHelper = null;

        /** Cached HumanoidKinematicsAnalyzer result per model load. */
        this._kinematicsCache = null;
    }

    update(model) {
        this.currentModel = model;
        this.clearDistanceMeasurement();
        this.clearOverlays();
        this._kinematicsCache = null;
        this.render();
    }

    clear() {
        this.currentModel = null;
        this.clearDistanceMeasurement();
        this.clearOverlays();
        this._kinematicsCache = null;
        const container = document.getElementById('measure-panel-content');
        if (container) {
            const t = (k) => window.i18n?.t(k) || k;
            container.innerHTML = `<div class="empty-state">${t('noModel')}</div>`;
        }
    }

    /** Rebuild tab bar + active tab content (called on model change and unit change). */
    render() {
        const container = document.getElementById('measure-panel-content');
        if (!container || !this.currentModel) return;

        // Legacy tab id from older builds
        if (this.activeTab === 'limbs') this.activeTab = 'joints';

        const t = (k) => window.i18n?.t(k) || k;
        container.innerHTML = '';

        const tabBar = document.createElement('div');
        tabBar.className = 'measure-tab-bar';

        const tabs = [
            { id: 'overview', label: t('measureOverview') },
            { id: 'links', label: t('measureLinks') },
            { id: 'joints', label: t('measureJointIdentification') },
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
        if (this.activeTab === 'links' || this.activeTab === 'joints') {
            content.classList.add('measure-tab-content--table');
        }

        switch (this.activeTab) {
            case 'overview': this.renderOverview(content); break;
            case 'links': this.renderLinksTable(content); break;
            case 'joints': this.renderJointsTable(content); break;
            case 'distance': this.renderDistanceTool(content); break;
        }
        container.appendChild(content);
    }

    // ==================== Overview Tab ====================

    renderOverview(container) {
        const model = this.currentModel;
        const t = (k) => window.i18n?.t(k) || k;
        const stats = this.computeOverviewStats(model);
        // bbox/com are in meters; format with user's linear unit
        const fmt = (m) => this.formatLinearDisplay(m);

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
                        <span class="measure-dim-value">${fmt(stats.bbox.x)}</span>
                        <span class="measure-dim-label">Y:</span>
                        <span class="measure-dim-value">${fmt(stats.bbox.y)}</span>
                        <span class="measure-dim-label">Z:</span>
                        <span class="measure-dim-value">${fmt(stats.bbox.z)}</span>
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
                        <span class="measure-dim-value">${fmt(stats.com.x)}</span>
                        <span class="measure-dim-label">Y:</span>
                        <span class="measure-dim-value">${fmt(stats.com.y)}</span>
                        <span class="measure-dim-label">Z:</span>
                        <span class="measure-dim-value">${fmt(stats.com.z)}</span>
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

    /** Mass-weighted COM and axis-aligned bbox size (all lengths in meters). */
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

    // ==================== Limb / joint identification ====================

    /** Joint Identification / Links category columns (cached until model changes). */
    getKinematicsAnalysis(model = this.currentModel) {
        if (!model) return { jointRows: [], linkCategories: new Map() };
        if (!this._kinematicsCache) {
            this._kinematicsCache = HumanoidKinematicsAnalyzer.analyze(model);
        }
        return this._kinematicsCache;
    }

    formatCategoryBadges(categories) {
        if (!categories?.length) return '—';
        return categories.map(cat =>
            `<span class="measure-category-badge">${cat}</span>`
        ).join('');
    }

    formatJointAxisLabel(axis, t) {
        if (axis === 'yaw') return t('measureJointAxisYaw');
        if (axis === 'pitch') return t('measureJointAxisPitch');
        if (axis === 'roll') return t('measureJointAxisRoll');
        return '—';
    }

    formatCategoriesPlain(categories) {
        return categories?.length ? categories.join(', ') : '—';
    }

    getJointTableColumns(t) {
        const u = this.measureUnits;
        const ang = angleUnitLabel(u.angle);
        const lin = linearUnitLabel(u.linear);
        const vel = velocityUnitLabel(u.velocity);
        return [
            { key: 'name', label: t('measureJoint') },
            { key: 'type', label: t('type') },
            { key: 'categoryPlain', label: t('measureLimbCategory') },
            { key: 'jointAxisLabel', label: t('measureJointAxis') },
            { key: 'limitMin', label: `${t('measureLimitMin')} (${ang} / ${lin})` },
            { key: 'limitMax', label: `${t('measureLimitMax')} (${ang} / ${lin})` },
            { key: 'torque', label: `${t('measureTorque')} (${torqueUnitLabel(u.torque)})`, sortKey: 'torqueSort' },
            { key: 'velocity', label: `${t('measureUnitVelocity')} (${vel})`, sortKey: 'velocitySort' }
        ];
    }

    compareMeasureRows(a, b, columnKey) {
        const numericKeys = {
            mass: 'mass',
            length: 'length',
            limitMin: 'limitMinSort',
            limitMax: 'limitMaxSort',
            torque: 'torqueSort',
            torqueSort: 'torqueSort',
            velocity: 'velocitySort',
            velocitySort: 'velocitySort'
        };
        const numericField = numericKeys[columnKey];
        if (numericField) {
            const va = a[numericField];
            const vb = b[numericField];
            const aNum = typeof va === 'number' ? va : -Infinity;
            const bNum = typeof vb === 'number' ? vb : -Infinity;
            return this.sortAscending ? aNum - bNum : bNum - aNum;
        }

        let va = a[columnKey];
        let vb = b[columnKey];
        if (typeof va === 'number' && typeof vb === 'number') {
            return this.sortAscending ? va - vb : vb - va;
        }
        va = String(va || '');
        vb = String(vb || '');
        return this.sortAscending ? va.localeCompare(vb) : vb.localeCompare(va);
    }

    /**
     * Link length for the table (returns mm for sorting; display uses formatLinkLength).
     * Prefer parent joint origin offset; fallback to world-space parent→child distance.
     */
    computeLinkLengthMm(model, linkName) {
        if (!model?.links || !model?.joints) return null;

        let parentJoint = null;
        model.joints.forEach(joint => {
            if (joint.child === linkName) parentJoint = joint;
        });

        if (parentJoint?.origin?.xyz) {
            const [x, y, z] = parentJoint.origin.xyz;
            const offset = Math.sqrt(x * x + y * y + z * z);
            if (offset > 1e-9) return offset * 1000;
        }

        const link = model.links.get(linkName);
        const parentLink = parentJoint?.parent ? model.links.get(parentJoint.parent) : null;
        if (link?.threeObject && parentLink?.threeObject) {
            const posA = new THREE.Vector3();
            const posB = new THREE.Vector3();
            parentLink.threeObject.updateMatrixWorld(true);
            link.threeObject.updateMatrixWorld(true);
            parentLink.threeObject.getWorldPosition(posA);
            link.threeObject.getWorldPosition(posB);
            return posA.distanceTo(posB) * 1000;
        }

        return null;
    }

    formatLinkLength(lengthMm) {
        if (lengthMm === null || Number.isNaN(lengthMm)) return '—';
        const formatted = formatLinearFromMeters(lengthMm / 1000, this.measureUnits.linear);
        return formatted ?? '—';
    }

    /** Format a length in meters with the current linear unit (Overview, Distance, etc.). */
    formatLinearDisplay(meters) {
        if (meters === null || meters === undefined || Number.isNaN(meters)) return '—';
        const formatted = formatLinearFromMeters(meters, this.measureUnits.linear);
        if (formatted == null) return '—';
        return `${formatted} ${linearUnitLabel(this.measureUnits.linear)}`;
    }

    /**
     * Joint limits: revolute/continuous use angle unit; prismatic uses linear unit.
     * @param {number} valueRadOrM - URDF lower/upper (rad or m)
     */
    formatLimitValue(joint, valueRadOrM) {
        const dash = '—';
        if (valueRadOrM === null || valueRadOrM === undefined || Number.isNaN(valueRadOrM)) return dash;
        if (joint?.type === 'prismatic') {
            return formatLinearFromMeters(valueRadOrM, this.measureUnits.linear) ?? dash;
        }
        return formatAngleFromRad(valueRadOrM, this.measureUnits.angle) ?? dash;
    }

    /**
     * Torque/velocity from URDF limits; effort in Nm, velocity in rad/s.
     * @param {import('../models/UnifiedRobotModel.js').Joint} joint
     */
    getJointLimitFields(joint) {
        const dash = '—';
        const empty = {
            limitMin: dash,
            limitMax: dash,
            torque: dash,
            velocity: dash,
            limitMinSort: null,
            limitMaxSort: null,
            torqueSort: null,
            velocitySort: null
        };

        if (!joint || joint.type === 'fixed') return empty;

        const u = this.measureUnits;
        const torqueVal = joint.limits?.effort;
        const velVal = joint.limits?.velocity;
        const torqueFormatted = formatTorqueFromNm(torqueVal, u.torque);
        const torque = torqueFormatted != null
            ? `${torqueFormatted} ${torqueUnitLabel(u.torque)}`
            : dash;
        const velocityFormatted = formatVelocityFromRadS(velVal, u.velocity);
        const velocity = velocityFormatted != null ? `${velocityFormatted} ${velocityUnitLabel(u.velocity)}` : dash;

        if (joint.type === 'continuous') {
            return {
                limitMin: '−∞',
                limitMax: '∞',
                torque,
                velocity,
                limitMinSort: null,
                limitMaxSort: null,
                torqueSort: torqueVal ?? null,
                velocitySort: velVal ?? null
            };
        }

        const lim = joint.limits;
        if (!lim || lim.lower === undefined || lim.upper === undefined) {
            return {
                ...empty,
                torque,
                velocity,
                torqueSort: torqueVal ?? null,
                velocitySort: velVal ?? null
            };
        }

        return {
            limitMin: this.formatLimitValue(joint, lim.lower),
            limitMax: this.formatLimitValue(joint, lim.upper),
            torque,
            velocity,
            limitMinSort: lim.lower,
            limitMaxSort: lim.upper,
            torqueSort: torqueVal ?? null,
            velocitySort: velVal ?? null
        };
    }

    enrichJointTableRow(row, model, t) {
        const joint = model?.joints?.get(row.name);
        const limitFields = this.getJointLimitFields(joint);

        return {
            ...row,
            categoryPlain: this.formatCategoriesPlain(row.categories),
            jointAxisLabel: this.formatJointAxisLabel(row.jointAxis, t),
            ...limitFields
        };
    }

    getSourceFileBasename(model = this.currentModel) {
        const raw = model?.sourceFileName || model?.name || 'robot';
        const base = String(raw).split(/[/\\]/).pop() || 'robot';
        const withoutExt = base.replace(/\.[^.]+$/, '');
        return withoutExt || 'robot';
    }

    sanitizeExportFilenamePart(value) {
        return String(value)
            .replace(/[/\\?%*:|"<>]/g, '_')
            .replace(/\s+/g, '_')
            .replace(/_+/g, '_')
            .replace(/^_|_$/g, '') || 'export';
    }

    buildExportFilename(model, tabLabel) {
        const filePart = this.sanitizeExportFilenamePart(this.getSourceFileBasename(model));
        const tabPart = this.sanitizeExportFilenamePart(tabLabel);
        return `${filePart}_${tabPart}.csv`;
    }

    escapeCsvCell(value) {
        const s = String(value ?? '');
        if (/[",\r\n]/.test(s)) return `"${s.replace(/"/g, '""')}"`;
        return s;
    }

    tableToCsv(headers, rows) {
        const lines = [
            headers.map(h => this.escapeCsvCell(h)).join(','),
            ...rows.map(row => row.map(cell => this.escapeCsvCell(cell)).join(','))
        ];
        return lines.join('\r\n');
    }

    getLinksTableDataset(model = this.currentModel) {
        const t = (k) => window.i18n?.t(k) || k;
        const lin = linearUnitLabel(this.measureUnits.linear);
        const { linkCategories } = this.getKinematicsAnalysis(model);
        const headers = [
            t('measureLinkName'),
            `${t('mass')} (kg)`,
            `${t('measureLinkLength')} (${lin})`,
            t('measureLimbCategory')
        ];
        const rows = [];

        if (model?.links) {
            model.links.forEach((link, name) => {
                const categories = linkCategories.get(name) || [];
                const lengthMm = this.computeLinkLengthMm(model, name);
                rows.push([
                    name,
                    formatMeasureDecimal(link.inertial ? link.inertial.mass : 0, 3) ?? '0,000',
                    this.formatLinkLength(lengthMm),
                    this.formatCategoriesPlain(categories)
                ]);
            });
        }

        return {
            headers,
            rows,
            filename: this.buildExportFilename(model, t('measureLinks'))
        };
    }

    getJointsTableDataset(model = this.currentModel) {
        const t = (k) => window.i18n?.t(k) || k;
        const { jointRows } = this.getKinematicsAnalysis(model);
        const headers = this.getJointTableColumns(t).map(c => c.label);
        const rows = jointRows.map(row => {
            const enriched = this.enrichJointTableRow(row, model, t);
            return [
                row.name,
                row.type,
                enriched.categoryPlain,
                enriched.jointAxisLabel,
                enriched.limitMin,
                enriched.limitMax,
                enriched.torque,
                enriched.velocity
            ];
        });

        return {
            headers,
            rows,
            filename: this.buildExportFilename(model, t('measureJointIdentification'))
        };
    }

    downloadTableCsv(dataset) {
        const csv = '\uFEFF' + this.tableToCsv(dataset.headers, dataset.rows);
        const blob = new Blob([csv], { type: 'text/csv;charset=utf-8' });
        const url = URL.createObjectURL(blob);
        const a = document.createElement('a');
        a.href = url;
        a.download = dataset.filename;
        a.click();
        URL.revokeObjectURL(url);
    }

    async copyTableCsv(dataset) {
        const t = (k) => window.i18n?.t(k) || k;
        const csv = this.tableToCsv(dataset.headers, dataset.rows);
        try {
            await navigator.clipboard.writeText(csv);
            this.showMeasureExportToast(t('measureTableCopied'));
        } catch {
            this.showMeasureExportToast(t('measureCopyFailed'));
        }
    }

    showMeasureExportToast(message) {
        const existing = document.querySelector('.measure-export-toast');
        if (existing) existing.remove();

        const toast = document.createElement('div');
        toast.className = 'measure-export-toast';
        toast.textContent = message;
        document.body.appendChild(toast);
        requestAnimationFrame(() => toast.classList.add('visible'));
        setTimeout(() => {
            toast.classList.remove('visible');
            setTimeout(() => toast.remove(), 200);
        }, 2000);
    }

    renderTableExportToolbar(container, dataset) {
        const t = (k) => window.i18n?.t(k) || k;
        const toolbar = document.createElement('div');
        toolbar.className = 'measure-table-toolbar';

        const copyBtn = document.createElement('button');
        copyBtn.type = 'button';
        copyBtn.className = 'measure-export-btn';
        copyBtn.textContent = t('measureCopyTable');
        copyBtn.disabled = dataset.rows.length === 0;
        copyBtn.addEventListener('click', () => this.copyTableCsv(dataset));

        const downloadBtn = document.createElement('button');
        downloadBtn.type = 'button';
        downloadBtn.className = 'measure-export-btn primary';
        downloadBtn.textContent = t('measureDownloadTable');
        downloadBtn.disabled = dataset.rows.length === 0;
        downloadBtn.addEventListener('click', () => this.downloadTableCsv(dataset));

        toolbar.appendChild(copyBtn);
        toolbar.appendChild(downloadBtn);
        container.appendChild(toolbar);
    }

    // ==================== Links Table Tab ====================

    renderLinksTable(container) {
        const model = this.currentModel;
        const t = (k) => window.i18n?.t(k) || k;
        const dataset = this.getLinksTableDataset(model);
        this.renderTableExportToolbar(container, dataset);

        const { linkCategories } = this.getKinematicsAnalysis(model);
        const rows = [];
        if (model.links) {
            model.links.forEach((link, name) => {
                const categories = linkCategories.get(name) || [];
                const lengthMm = this.computeLinkLengthMm(model, name);
                rows.push({
                    name,
                    mass: link.inertial ? link.inertial.mass : 0,
                    length: lengthMm,
                    lengthDisplay: this.formatLinkLength(lengthMm),
                    categories,
                    categoryPlain: this.formatCategoriesPlain(categories)
                });
            });
        }

        const table = document.createElement('table');
        table.className = 'measure-table';

        const thead = document.createElement('thead');
        const headerRow = document.createElement('tr');
        const lin = linearUnitLabel(this.measureUnits.linear);
        const columns = [
            { key: 'name', label: t('measureLinkName') },
            { key: 'mass', label: `${t('mass')} (kg)` },
            { key: 'length', label: `${t('measureLinkLength')} (${lin})` },
            { key: 'categoryPlain', label: t('measureLimbCategory') }
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
            sortedRows.sort((a, b) => this.compareMeasureRows(a, b, this.sortColumn));
        }

        const tbody = document.createElement('tbody');
        sortedRows.forEach(row => {
            const tr = document.createElement('tr');
            tr.addEventListener('click', () => this.highlightLink(row.name));
            tr.innerHTML = `
                <td title="${row.name}">${row.name}</td>
                <td>${formatMeasureDecimal(row.mass, 3) ?? '0,000'}</td>
                <td>${row.lengthDisplay}</td>
                <td><div class="measure-category-badges">${this.formatCategoryBadges(row.categories)}</div></td>
            `;
            tbody.appendChild(tr);
        });
        table.appendChild(tbody);

        const wrapper = document.createElement('div');
        wrapper.className = 'measure-table-wrapper';
        wrapper.appendChild(table);
        container.appendChild(wrapper);
    }

    // ==================== Joint Identification Tab ====================

    renderJointsTable(container) {
        const model = this.currentModel;
        const t = (k) => window.i18n?.t(k) || k;
        const dataset = this.getJointsTableDataset(model);
        this.renderTableExportToolbar(container, dataset);

        const { jointRows } = this.getKinematicsAnalysis(model);

        if (jointRows.length === 0) {
            const empty = document.createElement('div');
            empty.className = 'empty-state';
            empty.textContent = t('measureNoJoints');
            container.appendChild(empty);
            return;
        }

        const table = document.createElement('table');
        table.className = 'measure-table';

        const thead = document.createElement('thead');
        const headerRow = document.createElement('tr');
        const columns = this.getJointTableColumns(t);

        columns.forEach(col => {
            const th = document.createElement('th');
            th.textContent = col.label;
            th.className = 'measure-th-sortable';
            const sortKey = col.sortKey || col.key;
            th.addEventListener('click', () => {
                if (this.sortColumn === sortKey) {
                    this.sortAscending = !this.sortAscending;
                } else {
                    this.sortColumn = sortKey;
                    this.sortAscending = true;
                }
                this.render();
            });
            if (this.sortColumn === sortKey) {
                th.textContent += this.sortAscending ? ' ▲' : ' ▼';
            }
            headerRow.appendChild(th);
        });
        thead.appendChild(headerRow);
        table.appendChild(thead);

        const rows = jointRows.map(row => this.enrichJointTableRow(row, model, t));

        let sortedRows = [...rows];
        if (this.sortColumn) {
            sortedRows.sort((a, b) => this.compareMeasureRows(a, b, this.sortColumn));
        }

        const tbody = document.createElement('tbody');
        sortedRows.forEach(row => {
            const tr = document.createElement('tr');
            tr.style.cursor = 'pointer';
            tr.innerHTML = `
                <td title="${row.name}">${row.name}</td>
                <td>${row.type}</td>
                <td><div class="measure-category-badges">${this.formatCategoryBadges(row.categories)}</div></td>
                <td><span class="measure-axis-badge">${row.jointAxisLabel}</span></td>
                <td>${row.limitMin}</td>
                <td>${row.limitMax}</td>
                <td>${row.torque}</td>
                <td>${row.velocity}</td>
            `;
            tr.addEventListener('click', () => {
                if (row.child) this.highlightLink(row.child);
            });
            tbody.appendChild(tr);
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

        // Keep numeric result when user switches unit preset (re-render only)
        if (this.distanceSelection.first && this.distanceSelection.second && this.distanceLine) {
            this.showDistanceResult(container);
        }
    }

    // ==================== Distance Measurement Logic ====================

    /** Joint frame origin in world space (meters). */
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

        // Orthogonal path A → B for ΔX / ΔY / ΔZ (same values as shown in the panel)
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
        const fmt = (m) => this.formatLinearDisplay(m); // dx, dy, dz, dist in meters

        const resultDiv = (parentContainer || document).querySelector('#measure-distance-result')
            || document.getElementById('measure-distance-result');
        if (resultDiv) {
            resultDiv.innerHTML = `
                <div class="measure-distance-value">
                    <strong>${t('measureTotalDistance')}:</strong> ${fmt(dist)}
                </div>
                <div class="measure-stat-grid">
                    <span class="measure-dim-label" style="color:#ff4444">ΔX:</span>
                    <span class="measure-dim-value">${fmt(dx)}</span>
                    <span class="measure-dim-label" style="color:#44ff44">ΔY:</span>
                    <span class="measure-dim-value">${fmt(dy)}</span>
                    <span class="measure-dim-label" style="color:#4488ff">ΔZ:</span>
                    <span class="measure-dim-value">${fmt(dz)}</span>
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
