/**
 * Draggable three-panel performance HUD (FPS, MS, MB) — stats.js style.
 */

const STORAGE_KEY = 'settings-performance';
const HISTORY_LEN = 70;
const PANEL_WIDTH = 56;
const PANEL_HEIGHT = 30;
const DEFAULT_INSET = 20;
const GEAR_CLEARANCE = 48;

export class PerformanceMonitor {
    constructor() {
        this.enabled = localStorage.getItem(STORAGE_KEY) === 'true';
        this._frameTimestamps = [];
        this._fpsHistory = [];
        this._msHistory = [];
        this._mbHistory = [];
        this._fpsMin = Infinity;
        this._fpsMax = 0;
        this._msMin = Infinity;
        this._msMax = 0;
        this._mbMin = Infinity;
        this._mbMax = 0;
        this._buildDom();
        localStorage.removeItem('performance-monitor-position');
        this._resetToDefaultPosition();
        this._setupDrag();
        this.setEnabled(this.enabled, { persist: false });
    }

    isEnabled() {
        return this.enabled;
    }

    /**
     * @param {boolean} enabled
     * @param {{ persist?: boolean }} [options]
     */
    setEnabled(enabled, options = {}) {
        const { persist = true } = options;
        this.enabled = enabled;
        if (persist) {
            localStorage.setItem(STORAGE_KEY, enabled ? 'true' : 'false');
        }
        this.root?.classList.toggle('hidden', !enabled);
        this.root?.setAttribute('aria-hidden', enabled ? 'false' : 'true');
        if (!enabled) {
            this._resetHistory();
        }
    }

    /**
     * @param {number} ms — elapsed time for one animation frame
     */
    recordFrame(ms) {
        if (!this.enabled) {
            return;
        }

        const now = performance.now();

        if (!this._rangeResetTime || now - this._rangeResetTime >= 1000) {
            this._rangeResetTime = now;
            this._fpsMin = Infinity;
            this._fpsMax = 0;
            this._msMin = Infinity;
            this._msMax = 0;
            this._mbMin = Infinity;
            this._mbMax = 0;
        }

        this._frameTimestamps.push(now);
        const cutoff = now - 1000;
        while (this._frameTimestamps.length > 0 && this._frameTimestamps[0] < cutoff) {
            this._frameTimestamps.shift();
        }

        const fps = this._frameTimestamps.length;
        this._pushHistory(this._fpsHistory, fps);
        this._fpsMin = Math.min(this._fpsMin, fps);
        this._fpsMax = Math.max(this._fpsMax, fps);

        this._pushHistory(this._msHistory, ms);
        this._msMin = Math.min(this._msMin, ms);
        this._msMax = Math.max(this._msMax, ms);

        const mb = this._readMemoryMb();
        if (mb !== null) {
            this._pushHistory(this._mbHistory, mb);
            this._mbMin = Math.min(this._mbMin, mb);
            this._mbMax = Math.max(this._mbMax, mb);
        }

        this._updatePanel(this.fpsPanel, fps, 'FPS', this._fpsMin, this._fpsMax, (ctx, samples) => {
            this._drawBarGraph(ctx, samples, PANEL_WIDTH, PANEL_HEIGHT, '#22d3ee', 62);
        }, this._fpsHistory);

        this._updatePanel(this.msPanel, Math.round(ms), 'MS', this._msMin, this._msMax, (ctx, samples) => {
            this._drawLineGraph(ctx, samples, PANEL_WIDTH, PANEL_HEIGHT, '#4ade80', 33);
        }, this._msHistory);

        if (this.mbPanel && mb !== null) {
            this._updatePanel(this.mbPanel, Math.round(mb), 'MB', this._mbMin, this._mbMax, (ctx, samples) => {
                this._drawBarGraph(ctx, samples, PANEL_WIDTH, PANEL_HEIGHT, '#e879f9', 200);
            }, this._mbHistory);
        } else if (this.mbPanel) {
            this.mbPanel.valueEl.textContent = '— MB';
        }
    }

    _readMemoryMb() {
        if (performance.memory?.usedJSHeapSize) {
            return performance.memory.usedJSHeapSize / 1048576;
        }
        return null;
    }

    _pushHistory(arr, value) {
        arr.push(value);
        if (arr.length > HISTORY_LEN) {
            arr.shift();
        }
    }

    _resetHistory() {
        this._frameTimestamps = [];
        this._fpsHistory = [];
        this._msHistory = [];
        this._mbHistory = [];
        this._fpsMin = Infinity;
        this._fpsMax = 0;
        this._msMin = Infinity;
        this._msMax = 0;
        this._mbMin = Infinity;
        this._mbMax = 0;
    }

    _formatValue(value, unit) {
        return `${Math.round(value)} ${unit}`;
    }

    _formatRangeTitle(value, min, max, unit) {
        const lo = Number.isFinite(min) && min !== Infinity ? Math.round(min) : Math.round(value);
        const hi = Number.isFinite(max) && max > 0 ? Math.round(max) : Math.round(value);
        return `${Math.round(value)} ${unit} (${lo}-${hi})`;
    }

    _updatePanel(panel, value, unit, min, max, drawFn, history) {
        if (!panel) return;
        panel.valueEl.textContent = this._formatValue(value, unit);
        panel.valueEl.title = this._formatRangeTitle(value, min, max, unit);
        if (panel.ctx && history.length > 1) {
            drawFn(panel.ctx, history);
        }
    }

    _buildDom() {
        const root = document.createElement('div');
        root.id = 'performance-monitor';
        root.className = 'performance-monitor hidden';
        root.setAttribute('aria-hidden', 'true');

        const handle = document.createElement('div');
        handle.className = 'performance-monitor-handle';
        handle.setAttribute('title', 'Drag to move');
        handle.innerHTML = '<span class="performance-monitor-handle-label" data-i18n="settingsPerformance">performance</span>';

        const body = document.createElement('div');
        body.className = 'performance-monitor-body';

        this.fpsPanel = this._createPanel('fps', 'FPS');
        this.msPanel = this._createPanel('ms', 'MS');
        this.mbPanel = this._createPanel('mb', 'MB');

        body.appendChild(this.fpsPanel.el);
        body.appendChild(this.msPanel.el);
        body.appendChild(this.mbPanel.el);

        root.appendChild(handle);
        root.appendChild(body);

        const host = document.getElementById('canvas-container') || document.body;
        host.appendChild(root);

        this.root = root;
        this.handle = handle;
        this.body = body;
    }

    _createPanel(id, label) {
        const el = document.createElement('div');
        el.className = `performance-panel performance-panel--${id}`;
        el.innerHTML = `
            <div class="performance-panel-value"></div>
            <canvas class="performance-panel-graph" width="${PANEL_WIDTH}" height="${PANEL_HEIGHT}" aria-hidden="true"></canvas>
        `;
        const canvas = el.querySelector('canvas');
        const valueEl = el.querySelector('.performance-panel-value');
        valueEl.textContent = `0 ${label}`;
        return {
            el,
            canvas,
            ctx: canvas.getContext('2d'),
            valueEl
        };
    }

    _getDefaultPosition() {
        const width = PANEL_WIDTH * 3;
        const height = PANEL_HEIGHT + 16;
        return {
            left: Math.max(DEFAULT_INSET, window.innerWidth - width - DEFAULT_INSET - GEAR_CLEARANCE),
            top: DEFAULT_INSET
        };
    }

    _applyPosition(left, top) {
        if (!this.root) return;
        const clamped = this._clampPosition(left, top);
        this.root.style.left = `${clamped.left}px`;
        this.root.style.top = `${clamped.top}px`;
        this.root.style.right = 'auto';
    }

    _clampPosition(left, top) {
        const rect = this.root.getBoundingClientRect();
        const w = rect.width || PANEL_WIDTH * 3;
        const h = rect.height || PANEL_HEIGHT + 16;
        const maxLeft = Math.max(0, window.innerWidth - w);
        const maxTop = Math.max(0, window.innerHeight - h);
        return {
            left: Math.min(maxLeft, Math.max(0, left)),
            top: Math.min(maxTop, Math.max(0, top))
        };
    }

    _resetToDefaultPosition() {
        const def = this._getDefaultPosition();
        this._applyPosition(def.left, def.top);
    }

    _setupDrag() {
        if (!this.handle || !this.root) return;

        let dragging = false;
        let offsetX = 0;
        let offsetY = 0;

        const onPointerDown = (e) => {
            if (e.button !== 0) return;
            e.preventDefault();
            dragging = true;
            this.root.classList.add('is-dragging');

            const rect = this.root.getBoundingClientRect();
            this._applyPosition(rect.left, rect.top);
            offsetX = e.clientX - rect.left;
            offsetY = e.clientY - rect.top;

            this.handle.setPointerCapture(e.pointerId);
        };

        const onPointerMove = (e) => {
            if (!dragging) return;
            this._applyPosition(e.clientX - offsetX, e.clientY - offsetY);
        };

        const onPointerUp = (e) => {
            if (!dragging) return;
            dragging = false;
            this.root.classList.remove('is-dragging');
            this.handle.releasePointerCapture(e.pointerId);
        };

        this.handle.addEventListener('pointerdown', onPointerDown);
        this.handle.addEventListener('pointermove', onPointerMove);
        this.handle.addEventListener('pointerup', onPointerUp);
        this.handle.addEventListener('pointercancel', onPointerUp);

        window.addEventListener('resize', () => {
            const rect = this.root.getBoundingClientRect();
            const clamped = this._clampPosition(rect.left, rect.top);
            if (clamped.left !== rect.left || clamped.top !== rect.top) {
                this._applyPosition(clamped.left, clamped.top);
            }
        });
    }

    _drawBarGraph(ctx, samples, width, height, color, cap) {
        ctx.clearRect(0, 0, width, height);
        const max = Math.max(cap * 0.25, ...samples);
        const barW = width / HISTORY_LEN;

        for (let i = 0; i < samples.length; i++) {
            const h = (samples[i] / max) * (height - 2);
            ctx.fillStyle = color;
            ctx.fillRect(i * barW, height - h, Math.max(1, barW - 1), h);
        }
    }

    _drawLineGraph(ctx, samples, width, height, color, cap) {
        ctx.clearRect(0, 0, width, height);
        const max = Math.max(cap * 0.25, ...samples);
        const step = width / (HISTORY_LEN - 1);
        const offset = HISTORY_LEN - samples.length;

        ctx.strokeStyle = color;
        ctx.lineWidth = 1;
        ctx.beginPath();

        for (let i = 0; i < samples.length; i++) {
            const x = (offset + i) * step;
            const y = height - (samples[i] / max) * (height - 4) - 2;
            if (i === 0) {
                ctx.moveTo(x, y);
            } else {
                ctx.lineTo(x, y);
            }
        }
        ctx.stroke();
    }
}
