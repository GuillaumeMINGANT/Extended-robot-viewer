/**
 * Left sidebar UI for browsing and loading robots from a remote JSON catalog.
 */

import {
    fetchCatalogManifest,
    filterModels,
    getCategoryList,
    groupModelsByBrand
} from '../catalog/RobotCatalogClient.js';
import { getBrandLogoUrl } from '../catalog/catalogConfig.js';
const CATEGORY_LABEL_KEYS = {
    arm: 'catalogCategoryArm',
    biped: 'catalogCategoryBiped',
    drone: 'catalogCategoryDrone',
    dual_arm: 'catalogCategoryDualArm',
    hand: 'catalogCategoryHand',
    humanoid: 'catalogCategoryHumanoid',
    mobile: 'catalogCategoryMobile',
    quadruped: 'catalogCategoryQuadruped',
    wheeled: 'catalogCategoryWheeled'
};

export class RobotCatalogPanel {
    /**
     * @param {{ onSelectModel: (entry: object) => Promise<void>|void }} options
     */
    constructor(options = {}) {
        this.onSelectModel = options.onSelectModel ?? (() => {});
        this.overlay = document.getElementById('robot-catalog-overlay');
        this.shell = document.getElementById('robot-catalog-shell');
        this.panel = document.getElementById('robot-catalog-panel');
        this.searchInput = document.getElementById('robot-catalog-search');
        this.chipsContainer = document.getElementById('robot-catalog-chips');
        this.body = document.getElementById('robot-catalog-body');
        this.closeBtn = document.getElementById('robot-catalog-close');
        this.toggleBtn = document.getElementById('robot-catalog-toggle');
        this.statusEl = document.getElementById('robot-catalog-status');

        this.models = [];
        this.activeCategory = null;
        this.view = 'brands';
        this.activeBrand = null;
        this.loadingModelId = null;

        this._bindChrome();
        this._syncToggleState();
    }

    _bindChrome() {
        this.closeBtn?.addEventListener('click', () => this.close());
        this.toggleBtn?.addEventListener('click', () => this.toggle());
        this.overlay?.addEventListener('click', () => this.close());
        this.searchInput?.addEventListener('input', () => {
            if (this.view === 'brands') {
                this.renderBrandGrid();
            } else {
                this.renderRobotList();
            }
        });
        document.addEventListener('keydown', (e) => {
            if (e.key === 'Escape' && this.isOpen()) {
                this.close();
            }
        });
    }

    isOpen() {
        return this.shell && !this.shell.classList.contains('is-collapsed');
    }

    toggle() {
        if (this.isOpen()) {
            this.close();
        } else {
            this.open();
        }
    }

    async open() {
        if (!this.shell || !this.panel) return;
        this.shell.classList.remove('is-collapsed');
        this.overlay?.classList.remove('hidden');
        this.panel.setAttribute('aria-hidden', 'false');
        this.overlay?.setAttribute('aria-hidden', 'false');
        document.body.classList.add('robot-catalog-open');
        this._syncToggleState();

        if (this.models.length === 0) {
            await this._loadManifest();
        } else {
            this._render();
        }

        this.searchInput?.focus();
    }

    close() {
        this.shell?.classList.add('is-collapsed');
        this.overlay?.classList.add('hidden');
        this.panel?.setAttribute('aria-hidden', 'true');
        this.overlay?.setAttribute('aria-hidden', 'true');
        document.body.classList.remove('robot-catalog-open');
        this._syncToggleState();
    }

    _syncToggleState() {
        const open = this.isOpen();
        this.toggleBtn?.setAttribute('aria-expanded', open ? 'true' : 'false');
        const t = window.i18n?.t?.bind(window.i18n) ?? ((k) => k);
        const titleKey = open ? 'catalogToggleClose' : 'catalogToggleOpen';
        const label = t(titleKey);
        if (this.toggleBtn) {
            this.toggleBtn.title = label;
            this.toggleBtn.setAttribute('aria-label', label);
            this.toggleBtn.dataset.i18nTitle = titleKey;
        }
    }

    async _loadManifest() {
        this._setStatus('catalogLoading', true);
        try {
            const manifest = await fetchCatalogManifest();
            this.models = manifest.models;
            this._renderCategoryChips();
            this.view = 'brands';
            this.activeBrand = null;
            this._render();
            this._setStatus('');
        } catch (err) {
            console.error(err);
            this._setStatus('catalogLoadError', false, err.message);
            if (this.body) {
                this.body.innerHTML = '';
            }
        }
    }

    _setStatus(key, loading = false, detail = '') {
        if (!this.statusEl) return;
        if (!key) {
            this.statusEl.classList.add('hidden');
            this.statusEl.textContent = '';
            return;
        }
        this.statusEl.classList.remove('hidden');
        const t = window.i18n?.t?.bind(window.i18n) ?? ((k) => k);
        this.statusEl.textContent = detail || t(key);
        this.statusEl.classList.toggle('loading', loading);
    }

    _filteredModels() {
        return filterModels(this.models, {
            query: this.searchInput?.value ?? '',
            category: this.activeCategory
        });
    }

    _render() {
        if (this.view === 'brands') {
            this.renderBrandGrid();
        } else {
            this.renderRobotList();
        }
    }

    _renderCategoryChips() {
        if (!this.chipsContainer) return;
        this.chipsContainer.innerHTML = '';

        const t = window.i18n?.t?.bind(window.i18n) ?? ((k) => k);
        const categories = getCategoryList(this.models);

        const makeChip = (label, value) => {
            const btn = document.createElement('button');
            btn.type = 'button';
            btn.className = 'robot-catalog-chip';
            if ((value === null && !this.activeCategory) || value === this.activeCategory) {
                btn.classList.add('active');
            }
            btn.textContent = label;
            btn.addEventListener('click', () => {
                this.activeCategory = value;
                this._renderCategoryChips();
                if (this.view === 'brands') {
                    this.renderBrandGrid();
                } else {
                    this.renderRobotList();
                }
            });
            this.chipsContainer.appendChild(btn);
        };

        makeChip(t('catalogCategoryAll'), null);
        for (const cat of categories) {
            const key = CATEGORY_LABEL_KEYS[cat];
            makeChip(key ? t(key) : cat.replace(/_/g, ' '), cat);
        }
    }

    /**
     * Brand logo image with two-letter fallback when the remote asset is missing.
     * @param {string} brand
     */
    _createBrandIcon(brand) {
        const icon = document.createElement('div');
        icon.className = 'robot-catalog-brand-icon';

        const img = document.createElement('img');
        img.className = 'robot-catalog-brand-logo';
        img.alt = '';
        img.loading = 'lazy';
        img.decoding = 'async';
        img.src = getBrandLogoUrl(brand);
        img.onerror = () => {
            img.remove();
            if (icon.querySelector('.robot-catalog-brand-fallback')) return;
            const fallback = document.createElement('span');
            fallback.className = 'robot-catalog-brand-fallback';
            fallback.textContent = brand.slice(0, 2).toUpperCase();
            icon.appendChild(fallback);
        };

        icon.appendChild(img);
        return icon;
    }

    renderBrandGrid() {
        if (!this.body) return;
        this.view = 'brands';
        this.body.innerHTML = '';

        const filtered = this._filteredModels();
        const brandMap = groupModelsByBrand(filtered);
        const brands = [...brandMap.keys()].sort((a, b) => a.localeCompare(b));

        const t = window.i18n?.t?.bind(window.i18n) ?? ((k) => k);

        if (brands.length === 0) {
            const empty = document.createElement('div');
            empty.className = 'robot-catalog-empty';
            empty.textContent = t('catalogNoResults');
            this.body.appendChild(empty);
            return;
        }

        const grid = document.createElement('div');
        grid.className = 'robot-catalog-brand-grid';

        for (const brand of brands) {
            const count = brandMap.get(brand).length;
            const tile = document.createElement('button');
            tile.type = 'button';
            tile.className = 'robot-catalog-brand-tile';

            tile.appendChild(this._createBrandIcon(brand));

            const name = document.createElement('span');
            name.className = 'robot-catalog-brand-name';
            name.textContent = brand;
            tile.appendChild(name);

            const countEl = document.createElement('span');
            countEl.className = 'robot-catalog-brand-count';
            countEl.textContent = count === 1
                ? t('catalogModelCountOne')
                : t('catalogModelCountMany').replace('{n}', String(count));
            tile.appendChild(countEl);

            tile.addEventListener('click', () => {
                this.activeBrand = brand;
                this.renderRobotList();
            });

            grid.appendChild(tile);
        }

        this.body.appendChild(grid);
    }

    renderRobotList() {
        if (!this.body || !this.activeBrand) {
            this.renderBrandGrid();
            return;
        }

        this.view = 'robots';
        this.body.innerHTML = '';

        const t = window.i18n?.t?.bind(window.i18n) ?? ((k) => k);
        const filtered = this._filteredModels().filter((m) => m.brand === this.activeBrand);

        const header = document.createElement('div');
        header.className = 'robot-catalog-list-header';

        const back = document.createElement('button');
        back.type = 'button';
        back.className = 'robot-catalog-back';
        back.textContent = t('catalogBack');
        back.addEventListener('click', () => {
            this.activeBrand = null;
            this.renderBrandGrid();
        });
        header.appendChild(back);

        const title = document.createElement('span');
        title.className = 'robot-catalog-list-title';
        title.textContent = this.activeBrand;
        header.appendChild(title);

        this.body.appendChild(header);

        if (filtered.length === 0) {
            const empty = document.createElement('div');
            empty.className = 'robot-catalog-empty';
            empty.textContent = t('catalogNoResults');
            this.body.appendChild(empty);
            return;
        }

        const list = document.createElement('ul');
        list.className = 'robot-catalog-robot-list';

        for (const entry of filtered) {
            const li = document.createElement('li');
            const btn = document.createElement('button');
            btn.type = 'button';
            btn.className = 'robot-catalog-robot-item';
            if (this.loadingModelId === entry.id) {
                btn.classList.add('loading');
                btn.disabled = true;
            }

            const name = document.createElement('span');
            name.className = 'robot-catalog-robot-name';
            name.textContent = entry.name;
            btn.appendChild(name);

            const meta = document.createElement('span');
            meta.className = 'robot-catalog-robot-meta';
            const parts = [];
            if (entry.dof) parts.push(`${entry.dof} DOF`);
            if (entry.category) parts.push(entry.category.replace(/_/g, ' '));
            meta.textContent = parts.join(' · ');
            btn.appendChild(meta);

            btn.addEventListener('click', () => this._selectModel(entry, btn));
            li.appendChild(btn);
            list.appendChild(li);
        }

        this.body.appendChild(list);
    }

    async _selectModel(entry, buttonEl) {
        this.loadingModelId = entry.id;
        buttonEl?.classList.add('loading');
        this._setStatus('catalogModelLoading', true);

        try {
            await this.onSelectModel(entry);
            this.close();
        } catch (err) {
            console.error(err);
            this._setStatus('catalogModelError', false, err.message || String(err));
        } finally {
            this.loadingModelId = null;
            buttonEl?.classList.remove('loading');
        }
    }

    /** Re-render chips and list when language changes. */
    refreshLocale() {
        this._syncToggleState();
        this._renderCategoryChips();
        this._render();
    }
}
