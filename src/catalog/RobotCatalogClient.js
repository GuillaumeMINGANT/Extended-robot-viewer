/**
 * Fetches and indexes a JSON robot catalog manifest (URDF entries + metadata).
 */

import { getCatalogManifestUrl, getCatalogBaseUrl } from './catalogConfig.js';

/** @typedef {{ id: string, brand: string, name: string, category: string, dof?: number, urdf: string, reach?: number, payload?: number }} CatalogModel */

let cachedManifest = null;

/**
 * @returns {Promise<{ version: number, models: CatalogModel[] }>}
 */
export async function fetchCatalogManifest() {
    if (cachedManifest) {
        return cachedManifest;
    }

    const url = getCatalogManifestUrl();
    const res = await fetch(url, { cache: 'default' });
    if (!res.ok) {
        throw new Error(`Catalog manifest unavailable (${res.status})`);
    }

    const data = await res.json();
    if (!data?.models || !Array.isArray(data.models)) {
        throw new Error('Invalid catalog manifest format');
    }

    cachedManifest = data;
    return data;
}

/**
 * @param {string} baseUrl
 * @param {string} urdfPath
 */
export function buildUrdfUrl(baseUrl, urdfPath) {
    const base = baseUrl.endsWith('/') ? baseUrl : `${baseUrl}/`;
    const path = urdfPath.startsWith('/') ? urdfPath.slice(1) : urdfPath;
    return new URL(path, base).href;
}

/**
 * @param {CatalogModel[]} models
 * @returns {string[]}
 */
export function getCategoryList(models) {
    const set = new Set(models.map((m) => m.category).filter(Boolean));
    return [...set].sort((a, b) => a.localeCompare(b));
}

/**
 * @param {CatalogModel[]} models
 * @returns {Map<string, CatalogModel[]>}
 */
export function groupModelsByBrand(models) {
    const map = new Map();
    for (const model of models) {
        const list = map.get(model.brand) ?? [];
        list.push(model);
        map.set(model.brand, list);
    }
    for (const list of map.values()) {
        list.sort((a, b) => a.name.localeCompare(b.name));
    }
    return map;
}

/**
 * @param {CatalogModel[]} models
 * @param {{ query?: string, category?: string|null }} filters
 */
export function filterModels(models, filters = {}) {
    const q = (filters.query || '').trim().toLowerCase();
    const category = filters.category;

    return models.filter((m) => {
        if (category && m.category !== category) return false;
        if (!q) return true;
        return (
            m.name.toLowerCase().includes(q) ||
            m.brand.toLowerCase().includes(q) ||
            m.id.toLowerCase().includes(q)
        );
    });
}

export function getCatalogBaseUrlForLoad() {
    return getCatalogBaseUrl();
}
