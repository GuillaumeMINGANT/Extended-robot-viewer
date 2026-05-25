/**
 * Measure panel unit preferences and formatting.
 *
 * URDF values are always SI internally (m, rad, rad/s, Nm). This module converts
 * them for display and stores the user's choices in localStorage.
 *
 * Two keys are persisted:
 * - measureUnits: per-dimension display units (angle, linear, velocity, torque)
 * - measureUnitSystem: status-bar label (custom | mks | cgs | mmgs | ips)
 */
const STORAGE_KEY = 'measureUnits';
/** Which preset name appears on the button, independent of numeric values. */
const SYSTEM_STORAGE_KEY = 'measureUnitSystem';

export const MEASURE_UNIT_DEFAULTS = {
    angle: 'deg',
    linear: 'mm',
    velocity: 'rpm',
    torque: 'Nm'
};

/** Allowed values per dimension (also used to populate the custom editor). */
export const MEASURE_UNIT_ALLOWED = {
    angle: ['deg', 'rad'],
    linear: ['mm', 'cm', 'm', 'in'],
    velocity: ['rpm', 'rad/s'],
    torque: ['Nm', 'lbf-ft']
};

/** Display labels for custom-editor dropdowns. */
export const MEASURE_UNIT_FIELD_OPTIONS = {
    angle: [['deg', 'Deg'], ['rad', 'rad']],
    linear: [['mm', 'mm'], ['cm', 'cm'], ['m', 'm'], ['in', 'in']],
    velocity: [['rpm', 'rpm'], ['rad/s', 'rad/s']],
    torque: [['Nm', 'Nm'], ['lbf-ft', 'lbf·ft']]
};

// Conversion constants (URDF → display)
const METERS_PER_INCH = 0.0254;
const METERS_PER_CM = 0.01;
/** 1 lbf·ft ≈ 1.35582 Nm */
const NM_PER_LBF_FT = 1 / 0.737562;

/** Default status label: Custom (user settings above), not a named preset. */
export const MEASURE_UNIT_SYSTEM_CUSTOM = 'custom';

/**
 * Standard unit systems (SolidWorks naming).
 * Measure display uses the closest supported units per dimension.
 */
export const MEASURE_UNIT_PRESETS = [
    {
        id: 'mks',
        short: 'MKS',
        labelKey: 'measurePresetMks',
        units: { angle: 'rad', linear: 'm', velocity: 'rad/s', torque: 'Nm' }
    },
    {
        id: 'cgs',
        short: 'CGS',
        labelKey: 'measurePresetCgs',
        units: { angle: 'rad', linear: 'cm', velocity: 'rad/s', torque: 'Nm' }
    },
    {
        id: 'mmgs',
        short: 'MMGS',
        labelKey: 'measurePresetMmgs',
        units: { angle: 'deg', linear: 'mm', velocity: 'rpm', torque: 'Nm' }
    },
    {
        id: 'ips',
        short: 'IPS',
        labelKey: 'measurePresetIps',
        units: { angle: 'deg', linear: 'in', velocity: 'rpm', torque: 'lbf-ft' }
    }
];

export function unitsMatch(a, b) {
    return a.angle === b.angle
        && a.linear === b.linear
        && a.velocity === b.velocity
        && a.torque === b.torque;
}

export function findPresetById(id) {
    return MEASURE_UNIT_PRESETS.find(p => p.id === id) || null;
}

/**
 * @returns {'custom'|'mks'|'cgs'|'mmgs'|'ips'}
 */
export function loadMeasureUnitSystem() {
    try {
        const raw = localStorage.getItem(SYSTEM_STORAGE_KEY);
        if (raw === MEASURE_UNIT_SYSTEM_CUSTOM) return MEASURE_UNIT_SYSTEM_CUSTOM;
        if (findPresetById(raw)) return raw;
    } catch { /* ignore */ }
    return MEASURE_UNIT_SYSTEM_CUSTOM;
}

/**
 * @param {'custom'|'mks'|'cgs'|'mmgs'|'ips'} system
 */
export function saveMeasureUnitSystem(system) {
    localStorage.setItem(SYSTEM_STORAGE_KEY, system);
}

/**
 * Status bar label, e.g. "Units (Custom)" or "Units (MMGS)".
 */
export function getUnitsStatusLabel() {
    const t = (k) => window.i18n?.t(k) || k;
    const system = loadMeasureUnitSystem();
    const typeName = system === MEASURE_UNIT_SYSTEM_CUSTOM
        ? t('measureUnitSystemCustom')
        : (findPresetById(system)?.short || system.toUpperCase());
    return `${t('measureUnits')} (${typeName})`;
}

/**
 * @returns {{ angle: string, linear: string, velocity: string, torque: string }}
 */
/** Drop unknown unit strings (e.g. after an upgrade) so localStorage stays valid. */
function sanitizeMeasureUnits(parsed) {
    const merged = { ...MEASURE_UNIT_DEFAULTS, ...parsed };
    for (const [key, allowed] of Object.entries(MEASURE_UNIT_ALLOWED)) {
        if (!allowed.includes(merged[key])) merged[key] = MEASURE_UNIT_DEFAULTS[key];
    }
    return merged;
}

export function loadMeasureUnits() {
    try {
        const raw = localStorage.getItem(STORAGE_KEY);
        if (!raw) return { ...MEASURE_UNIT_DEFAULTS };
        return sanitizeMeasureUnits(JSON.parse(raw));
    } catch {
        return { ...MEASURE_UNIT_DEFAULTS };
    }
}

/**
 * @param {{ angle?: string, linear?: string, velocity?: string, torque?: string }} units
 */
export function saveMeasureUnits(units) {
    const merged = sanitizeMeasureUnits({ ...loadMeasureUnits(), ...units });
    localStorage.setItem(STORAGE_KEY, JSON.stringify(merged));
    return merged;
}

/**
 * Format a number for display in the Measure panel (comma decimal separator).
 * Trailing zeros are omitted so 5.5 displays as "5,5" not "5,500".
 * @param {number} value
 * @param {number} [decimals=3]
 * @returns {string|null}
 */
export function formatMeasureDecimal(value, decimals = 3) {
    if (value === null || value === undefined || Number.isNaN(value)) return null;
    return Number(value).toLocaleString('de-DE', {
        minimumFractionDigits: 0,
        maximumFractionDigits: decimals
    });
}

/**
 * @param {number} radians
 * @param {'deg'|'rad'} unit
 */
export function formatAngleFromRad(radians, unit) {
    if (radians === null || radians === undefined || Number.isNaN(radians)) return null;
    if (unit === 'deg') return formatMeasureDecimal(radians * 180 / Math.PI, 2);
    return formatMeasureDecimal(radians, 3);
}

/**
 * @param {number} meters
 * @param {'mm'|'cm'|'m'|'in'} unit
 */
export function formatLinearFromMeters(meters, unit) {
    if (meters === null || meters === undefined || Number.isNaN(meters)) return null;
    if (unit === 'mm') return formatMeasureDecimal(meters * 1000, 1);
    if (unit === 'cm') return formatMeasureDecimal(meters / METERS_PER_CM, 2);
    if (unit === 'in') return formatMeasureDecimal(meters / METERS_PER_INCH, 3);
    return formatMeasureDecimal(meters, 4);
}

/**
 * URDF velocity limit is rad/s.
 * @param {number} radPerS
 * @param {'rpm'|'rad/s'} unit
 */
export function formatVelocityFromRadS(radPerS, unit) {
    if (radPerS === null || radPerS === undefined || Number.isNaN(radPerS)) return null;
    if (unit === 'rpm') return formatMeasureDecimal(radPerS * 60 / (2 * Math.PI), 2);
    return formatMeasureDecimal(radPerS, 3);
}

/**
 * URDF effort is newton-meters.
 * @param {number} newtonMeters
 * @param {'Nm'|'lbf-ft'} unit
 */
export function formatTorqueFromNm(newtonMeters, unit) {
    if (newtonMeters === null || newtonMeters === undefined || Number.isNaN(newtonMeters)) return null;
    if (unit === 'lbf-ft') return formatMeasureDecimal(newtonMeters / NM_PER_LBF_FT, 3);
    return formatMeasureDecimal(newtonMeters, 3);
}

/**
 * @param {'deg'|'rad'} unit
 */
export function angleUnitLabel(unit) {
    return unit === 'deg' ? 'Deg' : 'rad';
}

/**
 * @param {'mm'|'cm'|'m'|'in'} unit
 */
export function linearUnitLabel(unit) {
    return unit;
}

/**
 * @param {'rpm'|'rad/s'} unit
 */
export function velocityUnitLabel(unit) {
    return unit;
}

/**
 * @param {'Nm'|'lbf-ft'} unit
 */
export function torqueUnitLabel(unit) {
    return unit === 'lbf-ft' ? 'lbf·ft' : unit;
}
