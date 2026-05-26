/**
 * Right sidebar for application settings (appearance).
 */

const FLOOR_GRID_STORAGE_KEY = 'settings-floor-grid';

export class SettingsPanel {
    /**
     * @param {{
     *   onThemeChanged?: (theme: string) => void,
     *   onLanguageChanged?: (lang: string) => void,
     *   onFloorGridChanged?: (visible: boolean) => void,
     *   performanceMonitor?: import('./PerformanceMonitor.js').PerformanceMonitor
     * }} options
     */
    constructor(options = {}) {
        this.onThemeChanged = options.onThemeChanged ?? (() => {});
        this.onLanguageChanged = options.onLanguageChanged ?? (() => {});
        this.onFloorGridChanged = options.onFloorGridChanged ?? (() => {});
        this.performanceMonitor = options.performanceMonitor ?? null;

        this.overlay = document.getElementById('settings-overlay');
        this.shell = document.getElementById('settings-shell');
        this.panel = document.getElementById('settings-panel');
        this.openBtn = document.getElementById('settings-open-btn');
        this.closeBtn = document.getElementById('settings-close');
        this.toggleBtn = document.getElementById('settings-toggle');

        this.themeDarkBtn = document.getElementById('settings-theme-dark');
        this.themeLightBtn = document.getElementById('settings-theme-light');
        this.langEnBtn = document.getElementById('settings-lang-en');
        this.langZhBtn = document.getElementById('settings-lang-zh');
        this.floorGridSwitch = document.getElementById('settings-floor-grid');
        this.performanceSwitch = document.getElementById('settings-performance');

        this._applyStoredFloorGrid();
        this._bindChrome();
        this._syncToggleState();
        this._syncPerformanceSwitch();
    }

    _bindChrome() {
        this.openBtn?.addEventListener('click', () => this.toggle());
        this.closeBtn?.addEventListener('click', () => this.close());
        this.toggleBtn?.addEventListener('click', () => this.toggle());
        this.overlay?.addEventListener('click', () => this.close());

        this.themeDarkBtn?.addEventListener('click', () => this._applyTheme('dark'));
        this.themeLightBtn?.addEventListener('click', () => this._applyTheme('light'));
        this.langEnBtn?.addEventListener('click', () => this._applyLanguage('en-US'));
        this.langZhBtn?.addEventListener('click', () => this._applyLanguage('zh-CN'));

        this.floorGridSwitch?.addEventListener('click', () => {
            this._toggleSwitch(this.floorGridSwitch, (on) => {
                localStorage.setItem(FLOOR_GRID_STORAGE_KEY, on ? 'true' : 'false');
                this.onFloorGridChanged(on);
            });
        });

        this.performanceSwitch?.addEventListener('click', () => {
            this._toggleSwitch(this.performanceSwitch, (on) => {
                this.performanceMonitor?.setEnabled(on);
            });
        });

        document.addEventListener('keydown', (e) => {
            if (e.key === 'Escape' && this.isOpen()) {
                this.close();
            }
        });
    }

    _applyStoredFloorGrid() {
        const stored = localStorage.getItem(FLOOR_GRID_STORAGE_KEY);
        const visible = stored === null ? true : stored === 'true';
        this._setSwitch(this.floorGridSwitch, visible);
        this.onFloorGridChanged(visible);
    }

    _toggleSwitch(button, callback) {
        if (!button) return;
        const next = button.getAttribute('data-checked') !== 'true';
        this._setSwitch(button, next);
        callback(next);
    }

    _setSwitch(button, on) {
        if (!button) return;
        button.classList.toggle('active', on);
        button.setAttribute('data-checked', on ? 'true' : 'false');
        button.setAttribute('aria-checked', on ? 'true' : 'false');
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

    open() {
        if (!this.shell || !this.panel) return;
        this.shell.classList.remove('is-collapsed');
        this.overlay?.classList.remove('hidden');
        this.panel.setAttribute('aria-hidden', 'false');
        this.overlay?.setAttribute('aria-hidden', 'false');
        document.body.classList.add('settings-open');
        this._syncFromAppState();
        this._syncToggleState();
    }

    close() {
        this.shell?.classList.add('is-collapsed');
        this.overlay?.classList.add('hidden');
        this.panel?.setAttribute('aria-hidden', 'true');
        this.overlay?.setAttribute('aria-hidden', 'true');
        document.body.classList.remove('settings-open');
        this._syncToggleState();
    }

    _syncToggleState() {
        const open = this.isOpen();
        this.toggleBtn?.setAttribute('aria-expanded', open ? 'true' : 'false');
        this.openBtn?.setAttribute('aria-expanded', open ? 'true' : 'false');
        const t = window.i18n?.t?.bind(window.i18n) ?? ((k) => k);
        const titleKey = open ? 'settingsToggleClose' : 'settingsToggleOpen';
        const label = t(titleKey);
        if (this.toggleBtn) {
            this.toggleBtn.title = label;
            this.toggleBtn.setAttribute('aria-label', label);
            this.toggleBtn.dataset.i18nTitle = titleKey;
        }
    }

    _syncFromAppState() {
        const theme = document.documentElement.getAttribute('data-theme') || 'dark';
        this._setThemeButtons(theme);

        const lang = window.i18n?.getCurrentLanguage?.() || 'zh-CN';
        this._setLanguageButtons(lang);

        const gridVisible = window.sceneManager?.environmentManager?.isGridVisible?.()
            ?? window.sceneManager?.referenceGrid?.visible
            ?? true;
        this._setSwitch(this.floorGridSwitch, gridVisible);

        this._syncPerformanceSwitch();
    }

    _syncPerformanceSwitch() {
        const on = this.performanceMonitor?.isEnabled() ?? false;
        this._setSwitch(this.performanceSwitch, on);
    }

    _applyTheme(theme) {
        document.documentElement.setAttribute('data-theme', theme);
        localStorage.setItem('theme', theme);
        this._setThemeButtons(theme);
        this._updateToolbarThemeIcon(theme);
        window.sceneManager?.updateBackgroundColor?.();
        this.onThemeChanged(theme);
    }

    _applyLanguage(lang) {
        this._setLanguageButtons(lang);
        this.onLanguageChanged(lang);
    }

    _setThemeButtons(theme) {
        this.themeDarkBtn?.classList.toggle('active', theme === 'dark');
        this.themeLightBtn?.classList.toggle('active', theme === 'light');
    }

    _setLanguageButtons(lang) {
        this.langEnBtn?.classList.toggle('active', lang === 'en-US');
        this.langZhBtn?.classList.toggle('active', lang === 'zh-CN');
    }

    _updateToolbarThemeIcon(theme) {
        const icon = document.querySelector('#theme-toggle .tool-button-icon');
        if (icon) {
            icon.textContent = theme === 'dark' ? '🌙' : '☀️';
        }
    }

    refreshLocale() {
        this._syncToggleState();
    }
}
