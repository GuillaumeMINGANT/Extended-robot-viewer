/**
 * ControllerPanelUI — IK controls section in the Controller panel's IK sub-tab.
 *
 * Adds: IK toggle, tip visibility list, Home/Random buttons, solver/space/gizmo controls.
 */

export class ControllerPanelUI {
    /**
     * @param {{
     *   ikController: import('../controllers/IkController.js').IkController,
     *   onRequestInfoModal?: () => void
     * }} options
     */
    constructor(options) {
        this.ikController = options.ikController;
        this.onRequestInfoModal = options.onRequestInfoModal ?? (() => {});
        this._root = null;
        this._tipListEl = null;
    }

    /**
     * Build and mount the IK controls into the IK pane of the Controller panel.
     */
    mount() {
        const ikPane = document.getElementById('controller-ik-pane');
        if (!ikPane) return;

        if (this._root) this._root.remove();

        this._root = document.createElement('div');
        this._root.className = 'ik-controls-section';
        this._root.innerHTML = this._buildHtml();

        ikPane.appendChild(this._root);
        this._bindEvents();
    }

    unmount() {
        this._root?.remove();
        this._root = null;
    }

    /**
     * Refresh the tip list when model changes.
     */
    refresh() {
        if (!this._root) return;
        this._tipListEl = this._root.querySelector('.ik-tip-list');
        if (this._tipListEl) {
            this._tipListEl.innerHTML = this._buildTipListHtml();
            this._bindTipToggles();
        }
        this._syncIkToggle();
        this._syncSolverButtons();
    }

    _buildHtml() {
        const t = (key) => window.i18n?.t(key) ?? key;
        const solverType = this.ikController.solverType;
        return `
            <div class="ik-controls-header">
                <div class="ik-controls-row">
                    <label class="ik-toggle-label">
                        <input type="checkbox" id="ik-enable-toggle" class="ik-checkbox">
                        <span class="ik-toggle-text">${t('ikEnable')}</span>
                        <span class="ik-solver-badge" id="ik-solver-badge"
                              title="${solverType === 'qp' ? 'QP (Task-space)' : 'DLS (Jacobian)'}">${solverType === 'qp' ? 'QP' : 'DLS'}</span>
                    </label>
                    <div class="ik-actions">
                        <button class="ik-btn" id="ik-home-btn" title="${t('ikHome')}">
                            <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
                                <path d="M3 9l9-7 9 7v11a2 2 0 01-2 2H5a2 2 0 01-2-2z"/>
                            </svg>
                        </button>
                        <button class="ik-btn" id="ik-random-btn" title="${t('ikRandom')}">
                            <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
                                <path d="M16 3h5v5M4 20L21 3M21 16v5h-5M15 15l6 6M4 4l5 5"/>
                            </svg>
                        </button>
                        <button class="ik-btn" id="ik-info-btn" title="${t('ikInfo')}">
                            <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
                                <circle cx="12" cy="12" r="10"/><line x1="12" y1="16" x2="12" y2="12"/><line x1="12" y1="8" x2="12.01" y2="8"/>
                            </svg>
                        </button>
                    </div>
                </div>
                <div class="ik-solver-row">
                    <span class="ik-solver-label">${t('ikSolver')}</span>
                    <div class="ik-space-toggle">
                        <button class="ik-space-btn${solverType === 'jacobian' ? ' active' : ''}" id="ik-solver-jacobian"
                                title="${t('ikSolverDLSDesc')}">DLS</button>
                        <button class="ik-space-btn${solverType === 'qp' ? ' active' : ''}"
                                id="ik-solver-qp"
                                title="${t('ikSolverQPDesc')}">QP</button>
                    </div>
                </div>
                <div class="ik-mode-row">
                    <div class="ik-space-toggle">
                        <button class="ik-space-btn active" id="ik-space-local" title="Local frame (L)">Local</button>
                        <button class="ik-space-btn" id="ik-space-world" title="World frame (L)">World</button>
                    </div>
                    <div class="ik-gizmo-toggles">
                        <button class="ik-gizmo-toggle-btn active" id="ik-translate-toggle" title="Toggle position control (T)">
                            <svg width="12" height="12" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.5">
                                <path d="M5 9l-3 3 3 3M9 5l3-3 3 3M15 19l-3 3-3-3M19 9l3 3-3 3M2 12h20M12 2v20"/>
                            </svg>
                        </button>
                        <button class="ik-gizmo-toggle-btn active" id="ik-rotate-toggle" title="Toggle rotation control (R)">
                            <svg width="12" height="12" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.5">
                                <path d="M21 12a9 9 0 11-6.2-8.6"/>
                                <path d="M21 3v5h-5"/>
                            </svg>
                        </button>
                        <button class="ik-gizmo-toggle-btn" id="ik-lock-links-toggle" title="Lock links (prevent direct joint articulation)">
                            <svg width="12" height="12" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.5">
                                <rect x="3" y="11" width="18" height="11" rx="2" ry="2"/>
                                <path d="M7 11V7a5 5 0 0110 0v4"/>
                            </svg>
                        </button>
                    </div>
                </div>
            </div>
            <div class="ik-tip-list"></div>
        `;
    }

    _buildTipListHtml() {
        const tips = this.ikController.tipLinks;
        if (tips.length === 0) return '';

        return tips.map((tip, idx) => {
            const label = this.ikController.getTipLabel(tip);
            const color = this.ikController.getTipColor(tip);
            const visible = this.ikController.isTipVisible(tip);
            const hexColor = '#' + color.toString(16).padStart(6, '0');

            return `
                <label class="ik-tip-row" data-tip="${tip}">
                    <span class="ik-tip-dot" style="background:${hexColor}"></span>
                    <input type="checkbox" class="ik-tip-checkbox" data-tip="${tip}" ${visible ? 'checked' : ''}>
                    <span class="ik-tip-label">${label}</span>
                </label>
            `;
        }).join('');
    }

    _bindEvents() {
        const enableToggle = this._root.querySelector('#ik-enable-toggle');
        enableToggle?.addEventListener('change', (e) => {
            if (e.target.checked) {
                this.ikController.enable();
            } else {
                this.ikController.disable();
            }
            this._updateActionButtons();
        });

        this._root.querySelector('#ik-home-btn')?.addEventListener('click', () => {
            this.ikController.goHome();
        });

        this._root.querySelector('#ik-random-btn')?.addEventListener('click', () => {
            this.ikController.goRandom();
        });

        this._root.querySelector('#ik-info-btn')?.addEventListener('click', () => {
            this.onRequestInfoModal();
        });

        // Solver toggle (DLS / QP)
        this._root.querySelector('#ik-solver-jacobian')?.addEventListener('click', () => {
            this.ikController.setSolverType('jacobian');
            this._syncSolverButtons();
        });
        this._root.querySelector('#ik-solver-qp')?.addEventListener('click', () => {
            this.ikController.setSolverType('qp');
            this._syncSolverButtons();
        });

        // Space toggle (local / world)
        this._root.querySelector('#ik-space-local')?.addEventListener('click', () => {
            this.ikController.setGizmoSpace('local');
            this._syncSpaceButtons();
        });
        this._root.querySelector('#ik-space-world')?.addEventListener('click', () => {
            this.ikController.setGizmoSpace('world');
            this._syncSpaceButtons();
        });

        // Gizmo mode toggles
        this._root.querySelector('#ik-translate-toggle')?.addEventListener('click', (e) => {
            const btn = e.currentTarget;
            const nowActive = !btn.classList.contains('active');
            btn.classList.toggle('active', nowActive);
            this.ikController.setTranslateEnabled(nowActive);
        });
        this._root.querySelector('#ik-rotate-toggle')?.addEventListener('click', (e) => {
            const btn = e.currentTarget;
            const nowActive = !btn.classList.contains('active');
            btn.classList.toggle('active', nowActive);
            this.ikController.setRotateEnabled(nowActive);
        });
        this._root.querySelector('#ik-lock-links-toggle')?.addEventListener('click', (e) => {
            const btn = e.currentTarget;
            const nowActive = !btn.classList.contains('active');
            btn.classList.toggle('active', nowActive);
            this.ikController.setLockLinks(nowActive);
        });

        this._tipListEl = this._root.querySelector('.ik-tip-list');
        this._bindTipToggles();
    }

    _bindTipToggles() {
        if (!this._tipListEl) return;
        const checkboxes = this._tipListEl.querySelectorAll('.ik-tip-checkbox');
        checkboxes.forEach(cb => {
            cb.addEventListener('change', () => {
                this.ikController.setTipVisible(cb.dataset.tip, cb.checked);
            });
        });
    }

    _syncIkToggle() {
        const toggle = this._root?.querySelector('#ik-enable-toggle');
        if (toggle) toggle.checked = this.ikController.enabled;
        this._updateActionButtons();
    }

    _updateActionButtons() {
        const enabled = this.ikController.enabled;
        const btns = this._root?.querySelectorAll('.ik-actions .ik-btn');
        btns?.forEach(btn => {
            if (btn.id !== 'ik-info-btn') {
                btn.disabled = !this.ikController.model;
            }
        });
    }

    _syncSpaceButtons() {
        const space = this.ikController.gizmoSpace;
        this._root?.querySelector('#ik-space-local')?.classList.toggle('active', space === 'local');
        this._root?.querySelector('#ik-space-world')?.classList.toggle('active', space === 'world');
    }

    _syncSolverButtons() {
        const type = this.ikController.solverType;

        this._root?.querySelector('#ik-solver-jacobian')?.classList.toggle('active', type === 'jacobian');
        this._root?.querySelector('#ik-solver-qp')?.classList.toggle('active', type === 'qp');

        const badge = this._root?.querySelector('#ik-solver-badge');
        if (badge) {
            badge.textContent = type === 'qp' ? 'QP' : 'DLS';
            badge.title = type === 'qp' ? 'QP (Task-space)' : 'DLS (Jacobian)';
            badge.classList.toggle('ik-solver-badge--qp', type === 'qp');
        }
    }
}
