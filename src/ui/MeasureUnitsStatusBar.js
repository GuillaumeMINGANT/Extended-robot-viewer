import {
    loadMeasureUnits,
    saveMeasureUnits,
    loadMeasureUnitSystem,
    saveMeasureUnitSystem,
    MEASURE_UNIT_PRESETS,
    MEASURE_UNIT_FIELD_OPTIONS,
    MEASURE_UNIT_SYSTEM_CUSTOM,
    getUnitsStatusLabel
} from '../utils/MeasureUnitSettings.js';

/**
 * SolidWorks-style units control fixed at the bottom-right of the viewport.
 *
 * Shows "Units (Custom)" or "Units (MMGS)" etc. Presets apply a full unit set;
 * "Edit units…" opens per-dimension dropdowns and switches the label to Custom.
 */
export class MeasureUnitsStatusBar {
    constructor() {
        this.root = null;
        this.menu = null;
        this.customPanel = null;
        this.triggerLabel = null;
        /** MeasurePanelController — re-rendered when units change. */
        this.measurePanelController = null;
        this.units = loadMeasureUnits();
        this.menuOpen = false;
        this.customOpen = false;
        this._onDocumentClick = this._onDocumentClick.bind(this);
    }

    /** Build trigger + popup once and attach to document.body. */
    mount() {
        if (this.root) return;

        this.root = document.createElement('div');
        this.root.id = 'measure-units-status';
        this.root.className = 'measure-units-status';

        const trigger = document.createElement('button');
        trigger.type = 'button';
        trigger.className = 'measure-units-trigger';
        trigger.setAttribute('aria-haspopup', 'true');
        trigger.setAttribute('aria-expanded', 'false');

        this.triggerLabel = document.createElement('span');
        this.triggerLabel.className = 'measure-units-label';
        trigger.appendChild(this.triggerLabel);

        const caret = document.createElement('span');
        caret.className = 'measure-units-caret';
        caret.textContent = '▲';
        trigger.appendChild(caret);

        trigger.addEventListener('click', (e) => {
            e.stopPropagation();
            this.toggleMenu();
        });

        this.menu = document.createElement('div');
        this.menu.className = 'measure-units-menu';
        this.menu.hidden = true;

        // MKS / CGS / MMGS / IPS — marker shows active preset from measureUnitSystem
        const presetList = document.createElement('ul');
        presetList.className = 'measure-units-preset-list';
        MEASURE_UNIT_PRESETS.forEach(preset => {
            const li = document.createElement('li');
            li.className = 'measure-units-preset-item';
            li.dataset.presetId = preset.id;

            const marker = document.createElement('span');
            marker.className = 'measure-units-preset-marker';
            li.appendChild(marker);

            const text = document.createElement('span');
            text.className = 'measure-units-preset-label';
            text.dataset.i18n = preset.labelKey;
            li.appendChild(text);

            li.addEventListener('click', (e) => {
                e.stopPropagation();
                this.applyUnits(preset.units, preset.id);
                this.closeMenu();
            });
            presetList.appendChild(li);
        });
        this.menu.appendChild(presetList);

        const divider = document.createElement('div');
        divider.className = 'measure-units-menu-divider';
        this.menu.appendChild(divider);

        const editBtn = document.createElement('button');
        editBtn.type = 'button';
        editBtn.className = 'measure-units-edit-btn';
        editBtn.dataset.i18n = 'measureEditUnits';
        editBtn.addEventListener('click', (e) => {
            e.stopPropagation();
            this.toggleCustomPanel();
        });
        this.menu.appendChild(editBtn);

        // Per-dimension overrides (any change forces "Custom" on the button)
        this.customPanel = document.createElement('div');
        this.customPanel.className = 'measure-units-custom';
        this.customPanel.hidden = true;

        const fields = [
            { key: 'angle', labelKey: 'measureUnitAngle' },
            { key: 'linear', labelKey: 'measureUnitLinear' },
            { key: 'velocity', labelKey: 'measureUnitVelocity' },
            { key: 'torque', labelKey: 'measureUnitTorque' }
        ];

        fields.forEach(({ key, labelKey }) => {
            const options = MEASURE_UNIT_FIELD_OPTIONS[key] || [];
            const row = document.createElement('label');
            row.className = 'measure-units-custom-row';
            const label = document.createElement('span');
            label.dataset.i18n = labelKey;
            row.appendChild(label);

            const select = document.createElement('select');
            select.className = 'measure-units-select';
            select.dataset.unitKey = key;
            options.forEach(([value, text]) => {
                const opt = document.createElement('option');
                opt.value = value;
                opt.textContent = text;
                select.appendChild(opt);
            });
            select.addEventListener('change', () => {
                this.applyUnits({ [key]: select.value }, MEASURE_UNIT_SYSTEM_CUSTOM);
            });
            row.appendChild(select);
            this.customPanel.appendChild(row);
        });

        this.menu.appendChild(this.customPanel);

        this.root.appendChild(trigger);
        this.root.appendChild(this.menu);
        document.body.appendChild(this.root);

        this.refresh();
        window.i18n?.updatePageLanguage();
    }

    /** Share unit state with the measure panel so tables/overview stay in sync. */
    bind(measurePanelController) {
        this.measurePanelController = measurePanelController;
        if (measurePanelController) {
            measurePanelController.measureUnits = this.units;
            measurePanelController.measureUnitsStatusBar = this;
        }
        this.syncCustomSelects();
    }

    /** Update button label, preset marker, and custom dropdown values. */
    refresh() {
        if (!this.root) return;
        this.units = this.measurePanelController?.measureUnits || loadMeasureUnits();
        if (this.triggerLabel) {
            this.triggerLabel.textContent = getUnitsStatusLabel();
            this.triggerLabel.title = this._fullUnitsTitle();
        }
        this._updatePresetMarkers();
        this.syncCustomSelects();
        window.i18n?.updatePageLanguage();
    }

    _fullUnitsTitle() {
        const u = this.units;
        return `${u.angle} · ${u.linear} · ${u.torque} · ${u.velocity}`;
    }

    /** Highlight preset from saved system id, not from matching unit values. */
    _updatePresetMarkers() {
        const activeId = loadMeasureUnitSystem();
        this.menu?.querySelectorAll('.measure-units-preset-item').forEach(li => {
            li.classList.toggle('active', li.dataset.presetId === activeId);
        });
    }

    syncCustomSelects() {
        if (!this.customPanel) return;
        this.customPanel.querySelectorAll('select').forEach(select => {
            const key = select.dataset.unitKey;
            if (key && this.units[key] !== undefined) {
                select.value = this.units[key];
            }
        });
    }

    /**
     * @param {object} partial - Fields to merge into measureUnits
     * @param {string} system - Preset id (mks, mmgs, …) or MEASURE_UNIT_SYSTEM_CUSTOM
     */
    applyUnits(partial, system = MEASURE_UNIT_SYSTEM_CUSTOM) {
        if (system) saveMeasureUnitSystem(system);
        this.units = saveMeasureUnits(partial);
        if (this.measurePanelController) {
            this.measurePanelController.measureUnits = this.units;
            // Rebuild active tab so Overview / Distance / tables pick up new units
            this.measurePanelController.render();
        }
        this.refresh();
    }

    toggleMenu() {
        if (this.menuOpen) this.closeMenu();
        else this.openMenu();
    }

    openMenu() {
        this.menuOpen = true;
        this.menu.hidden = false;
        this.root.classList.add('open');
        this.root.querySelector('.measure-units-trigger')?.setAttribute('aria-expanded', 'true');
        document.addEventListener('click', this._onDocumentClick);
        window.i18n?.updatePageLanguage();
    }

    closeMenu() {
        this.menuOpen = false;
        this.customOpen = false;
        if (this.menu) this.menu.hidden = true;
        if (this.customPanel) this.customPanel.hidden = true;
        this.root?.classList.remove('open', 'custom-open');
        this.root?.querySelector('.measure-units-trigger')?.setAttribute('aria-expanded', 'false');
        document.removeEventListener('click', this._onDocumentClick);
    }

    toggleCustomPanel() {
        this.customOpen = !this.customOpen;
        if (this.customPanel) this.customPanel.hidden = !this.customOpen;
        this.root?.classList.toggle('custom-open', this.customOpen);
        this.syncCustomSelects();
    }

    _onDocumentClick(e) {
        if (this.root && !this.root.contains(e.target)) {
            this.closeMenu();
        }
    }
}
