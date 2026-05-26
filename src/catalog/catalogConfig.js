/**
 * Optional remote robot catalog (manifest + URDF assets).
 * Override at build time: VITE_ROBOT_CATALOG_BASE_URL=https://your-cdn.example/dist/
 *
 * Default points at a public GitHub raw tree (robot-explorer-models). This is a
 * runtime data URL only — no viewer code from third-party repos is bundled here.
 */

const DEFAULT_CATALOG_BASE_URL =
    'https://raw.githubusercontent.com/ferrolho/robot-explorer-models/dist/';

export function getCatalogBaseUrl() {
    const fromEnv = import.meta.env?.VITE_ROBOT_CATALOG_BASE_URL;
    const base = (fromEnv && String(fromEnv).trim()) || DEFAULT_CATALOG_BASE_URL;
    return base.endsWith('/') ? base : `${base}/`;
}

export function getCatalogManifestUrl() {
    return `${getCatalogBaseUrl()}manifest.json`;
}

/** Default: brand PNGs from robot-explorer `public/images/logos/` (runtime asset URLs only). */
const DEFAULT_LOGOS_BASE_URL =
    'https://raw.githubusercontent.com/ferrolho/robot-explorer/master/public/images/logos/';

export function getCatalogLogosBaseUrl() {
    const fromEnv = import.meta.env?.VITE_ROBOT_CATALOG_LOGOS_BASE_URL;
    const base = (fromEnv && String(fromEnv).trim()) || DEFAULT_LOGOS_BASE_URL;
    return base.endsWith('/') ? base : `${base}/`;
}

/**
 * @param {string} brand - Display brand name from manifest
 */
export function brandNameToLogoSlug(brand) {
    return brand.replace(/\s+/g, '-').toLowerCase();
}

/**
 * @param {string} brand
 */
export function getBrandLogoUrl(brand) {
    return `${getCatalogLogosBaseUrl()}${brandNameToLogoSlug(brand)}.png`;
}
