/**
 * HumanoidKinematicsAnalyzer — topology + naming heuristics for limb/joint labeling.
 * Supports multiple category tags per joint/link and joint-axis semantics
 * (yaw/rotation, pitch/abduction, roll/flexion).
 */

/** Keyword → anatomical part; first matches win unless merged later. */
const PART_RULES = [
    { tag: 'Toe', keywords: ['toe', 'phalange'] },
    { tag: 'Foot', keywords: ['foot', 'sole'] },
    { tag: 'Ankle', keywords: ['ankle'] },
    { tag: 'Knee', keywords: ['knee', 'shin', 'calf', 'tibia', 'tibias'] },
    { tag: 'Hip', keywords: ['hip', 'thigh', 'tigh', 'femur'] },
    { tag: 'Finger', keywords: ['finger', 'thumb', 'pinky', 'index', 'ring', 'middle', 'dip', 'pip', 'mcp', 'knuckle', 'knucle'] },
    { tag: 'Gripper', keywords: ['gripper', 'grasp', 'end_effector', 'ee_link', 'tool0', 'tool_tip', 'claw'] },
    { tag: 'Hand', keywords: ['hand', 'palm'] },
    { tag: 'Wrist', keywords: ['wrist'] },
    { tag: 'Elbow', keywords: ['elbow', 'forearm'] },
    { tag: 'Shoulder', keywords: ['shoulder', 'upper_arm', 'upperarm', 'humerus'] },
    { tag: 'Neck', keywords: ['neck', 'cervical'] },
    { tag: 'Head', keywords: ['head', 'skull', 'gaze'] },
    { tag: 'Waist', keywords: ['waist', 'lumbar', 'abdomen', 'pelvis'] },
    { tag: 'Torso', keywords: ['torso', 'trunk', 'chest', 'spine', 'sternum', 'subassembly'] },
    { tag: 'Arm', keywords: ['arm', 'manipulator'] },
    { tag: 'Leg', keywords: ['leg'] },
    { tag: 'Base', keywords: ['base_link', 'imu'] }
];

const ARM_PARTS = new Set(['Shoulder', 'Elbow', 'Wrist', 'Hand', 'Finger', 'Gripper', 'Arm']);
const LEG_PARTS = new Set(['Hip', 'Knee', 'Ankle', 'Foot', 'Toe', 'Leg']);
const TRUNK_PARTS = new Set(['Waist', 'Torso', 'Neck', 'Head', 'Base']);
const LIMB_REGION_TAGS = new Set([
    'Left Arm', 'Right Arm', 'Left Leg', 'Right Leg', 'Left Hand', 'Right Hand'
]);

/** Display order for region tags (Left Arm, Right Leg, …). */
const REGION_ORDER = [
    'Left Arm', 'Right Arm', 'Left Leg', 'Right Leg', 'Left Hand', 'Right Hand'
];
const PART_ORDER = [
    'Shoulder', 'Elbow', 'Wrist', 'Hand', 'Gripper', 'Finger',
    'Hip', 'Knee', 'Ankle', 'Foot', 'Toe',
    'Neck', 'Head', 'Waist', 'Torso', 'Arm', 'Leg', 'Base', 'Other'
];

function normalizeText(...parts) {
    return parts.filter(Boolean).join(' ').toLowerCase();
}

/**
 * Primary numeric limb band from CLOVIS / ASTRO style names (R10_, Link30_, base_link_R0_).
 * @param {string} name
 * @returns {number|null}
 */
export function detectPrimaryNumericId(name) {
    if (!name) return null;
    const stripped = name.replace(/^base_link_/i, '');

    let m = stripped.match(/^[Rr](\d{1,2})(?:_|$)/);
    if (m) return parseInt(m[1], 10);

    m = stripped.match(/^Link(\d{1,2})_/i);
    if (m) return parseInt(m[1], 10);

    m = stripped.match(/^link(\d{1,2})_/i);
    if (m) return parseInt(m[1], 10);

    m = name.match(/_[Rr](\d{1,2})(?:_|$)/);
    if (m) return parseInt(m[1], 10);

    return null;
}

/**
 * @param {number} id
 * @returns {'left'|'right'|null}
 */
function sideFromNumericId(id) {
    if (id >= 10 && id <= 19) return 'right';
    if (id >= 20 && id <= 29) return 'left';
    if (id >= 30 && id <= 39) return 'right';
    if (id >= 40 && id <= 49) return 'left';
    if (id >= 50 && id <= 59) return 'right';
    if (id >= 60 && id <= 69) return 'left';
    if (id >= 70 && id <= 79) return 'right';
    if (id >= 80 && id <= 89) return 'left';
    if (id >= 0 && id <= 9) return null;
    return null;
}

/**
 * Normalize separator characters (space, hyphen, slash) to underscores
 * for uniform pattern matching.
 * @param {string} text
 * @returns {string}
 */
function normalizeSeparators(text) {
    return text.replace(/[\s\-\/]/g, '_');
}

const BODY_PART_PREFIXES = '(?:arm|leg|hand|foot|hip|knee|ankle|shoulder|elbow|wrist|palm|finger|thumb|toe|gripper)';

/**
 * @param {string} text
 * @returns {'left'|'right'|null}
 */
function detectSide(text) {
    const t = normalizeSeparators(text);

    if (isLeftToken(t)) return 'left';
    if (isRightToken(t)) return 'right';

    const primaryId = detectPrimaryNumericId(text);
    if (primaryId !== null) {
        return sideFromNumericId(primaryId);
    }

    return null;
}

/** @param {string} t  separator-normalized lowercase text */
function isLeftToken(t) {
    return new RegExp(
        '(?:^|_)left(?:_|$)'                       // left as token: left_hip, hip_left
        + '|(?:^|_)l_'                              // l_ prefix/infix: l_hip, arm_l_joint
        + '|_l$'                                    // _l suffix: hip_l
        + '|(?:^|_)(?:fl|rl|lf|lh|lr)_'            // quadruped / JAXON: FL_, RL_, LF_, LH_
        + '|(?:^|_)l' + BODY_PART_PREFIXES          // compact: larm, lleg, lhip, leftarm, leftpalm
        + '|(?:^|_)left' + BODY_PART_PREFIXES
        + '|_sym(?:_|$)'                            // symmetry marker convention
    ).test(t);
}

/** @param {string} t  separator-normalized lowercase text */
function isRightToken(t) {
    return new RegExp(
        '(?:^|_)right(?:_|$)'                      // right as token
        + '|(?:^|_)r_'                              // r_ prefix/infix
        + '|_r$'                                    // _r suffix
        + '|(?:^|_)(?:fr|rr|rf|rh)_'               // quadruped / JAXON: FR_, RR_, RF_, RH_
        + '|(?:^|_)r' + BODY_PART_PREFIXES          // compact: rarm, rleg, rhip, rightarm
        + '|(?:^|_)right' + BODY_PART_PREFIXES
    ).test(t);
}

/**
 * @param {string} text
 * @returns {boolean}
 */
function isMechanismName(text) {
    return /linkage|_motor_|pelvis_imu/i.test(text)
        || (/\bclaw\b/.test(text) && !/finger|hand|gripper/.test(text));
}

/**
 * @param {string} text
 * @returns {string[]}
 */
function detectParts(text) {
    if (isMechanismName(text)) return ['Other'];

    const found = [];
    for (const rule of PART_RULES) {
        if (rule.tag === 'Torso' && text.includes('subassembly') && /hip|thigh|tigh|femur|knee|ankle|foot/.test(text)) {
            continue;
        }
        if (rule.keywords.some(kw => text.includes(kw))) {
            found.push(rule.tag);
        }
    }
    return found;
}

/**
 * @param {string[]} tags
 * @returns {string[]}
 */
function sortTags(tags) {
    const set = new Set(tags);
    const ordered = [];
    REGION_ORDER.forEach(r => { if (set.has(r)) ordered.push(r); });
    PART_ORDER.forEach(p => { if (set.has(p) && !ordered.includes(p)) ordered.push(p); });
    set.forEach(t => { if (!ordered.includes(t)) ordered.push(t); });
    return ordered;
}

/**
 * Strip erroneous limb regions from trunk links (numeric id 0–9).
 * @param {string[]} categories
 * @param {number|null} numericId
 */
function applyTrunkRegionFilter(categories, numericId) {
    if (numericId === null || numericId > 9) return categories;
    return categories.filter(c => !LIMB_REGION_TAGS.has(c));
}

/**
 * Multi-label categories for a joint or link from names.
 * @param {string} primaryName
 * @param {string|null} childName
 * @param {string|null} parentName
 * @returns {string[]}
 */
export function classifyCategories(primaryName, childName = null, parentName = null) {
    const text = normalizeText(primaryName, childName, parentName);
    const numericId = detectPrimaryNumericId(primaryName) ?? detectPrimaryNumericId(childName);
    const side = detectSide(text);
    const parts = detectParts(text);

    if (parts.length === 1 && parts[0] === 'Other') {
        return ['Other'];
    }

    const tags = new Set();
    const hasArmPart = parts.some(p => ARM_PARTS.has(p));
    const hasLegPart = parts.some(p => LEG_PARTS.has(p));

    if (side === 'left' && hasArmPart) tags.add('Left Arm');
    if (side === 'right' && hasArmPart) tags.add('Right Arm');
    if (side === 'left' && hasLegPart) tags.add('Left Leg');
    if (side === 'right' && hasLegPart) tags.add('Right Leg');

    if (side === 'left' && (parts.includes('Hand') || parts.includes('Finger') || parts.includes('Gripper'))) {
        tags.add('Left Hand');
    }
    if (side === 'right' && (parts.includes('Hand') || parts.includes('Finger') || parts.includes('Gripper'))) {
        tags.add('Right Hand');
    }

    if (parts.includes('Gripper')) {
        if (side === 'left') tags.add('Left Arm');
        if (side === 'right') tags.add('Right Arm');
    }

    if (parts.includes('Finger')) {
        if (side === 'left') {
            tags.add('Left Arm');
            tags.add('Left Hand');
        }
        if (side === 'right') {
            tags.add('Right Arm');
            tags.add('Right Hand');
        }
    }

    parts.forEach(p => tags.add(p));

    if (tags.size === 0) {
        tags.add('Other');
    }

    return applyTrunkRegionFilter(sortTags([...tags]), numericId);
}

/**
 * @param {string} jointName
 * @param {{ axis?: { xyz: number[] }, type?: string }} joint
 * @returns {'yaw'|'pitch'|'roll'|null}
 */
export function classifyJointAxis(jointName, joint) {
    const text = normalizeSeparators(jointName.toLowerCase());

    if (/(?:^|_)abduction(?:_|$)|(?:^|_)adduction(?:_|$)/.test(text)) return 'pitch';
    if (/(?:^|_)flexion(?:_|$)/.test(text)) return 'roll';
    if (/(?:^|_)rotation(?:_|$)|(?:^|_)rotate(?:_|$)/.test(text)) return 'yaw';
    if (/(?:^|_)roll(?:_|$)/.test(text)) return 'roll';
    if (/(?:^|_)pitch(?:_|$)/.test(text)) return 'pitch';
    if (/(?:^|_)yaw(?:_|$)/.test(text)) return 'yaw';

    if (/axisrz|(?:^|_)axisrz(?:_|$)|_rz(?:_|$)/.test(text)) return 'yaw';
    if (/axisry|(?:^|_)axisry(?:_|$)|_ry(?:_|$)/.test(text)) return 'roll';
    if (/axisrx|(?:^|_)axisrx(?:_|$)|_rx(?:_|$)/.test(text)) return 'pitch';

    if (/(?:^|_)hipz(?:_|$)|anklez|neck.*(?:rz|yaw)/.test(text)) return 'yaw';
    if (/(?:^|_)hipy(?:_|$)|ankley|neck.*(?:ry|pitch)/.test(text)) return 'pitch';
    if (/(?:^|_)hipx(?:_|$)|neck.*(?:rx|roll)/.test(text)) return 'roll';

    if (/wrist[_\s]?1|wrist1/.test(text)) return 'yaw';
    if (/wrist[_\s]?2|wrist2/.test(text)) return 'pitch';
    if (/wrist[_\s]?3|wrist3/.test(text)) return 'roll';

    if (joint?.type === 'fixed' || joint?.type === 'floating') return null;

    const xyz = joint?.axis?.xyz;
    if (xyz && xyz.length >= 3) {
        const ax = Math.abs(xyz[0]);
        const ay = Math.abs(xyz[1]);
        const az = Math.abs(xyz[2]);
        const max = Math.max(ax, ay, az);
        if (max < 1e-6) return null;
        if (az >= ay && az >= ax) return 'yaw';
        if (ay >= ax && ay >= az) return 'pitch';
        if (ax >= ay && ax >= az) return 'roll';
    }

    return null;
}

/**
 * Apply joint classification to its child link only (avoid polluting trunk parents).
 */
function assignChildLinkCategories(linkCategories, joint) {
    if (!joint.child || joint.type === 'fixed') return;

    const jointCats = classifyCategories(joint.name, joint.child, joint.parent);
    const linkId = detectPrimaryNumericId(joint.child);
    const jointId = detectPrimaryNumericId(joint.name);
    const existing = linkCategories.get(joint.child);

    if (linkId !== null && jointId !== null && Math.floor(linkId / 10) === Math.floor(jointId / 10)) {
        linkCategories.set(joint.child, jointCats);
        return;
    }

    if (linkId !== null && linkId <= 9) return;

    if (!existing) {
        linkCategories.set(joint.child, jointCats);
        return;
    }

    linkCategories.set(joint.child, sortTags([...new Set([...existing, ...jointCats])]));
}

export class HumanoidKinematicsAnalyzer {
    /**
     * @param {import('../models/UnifiedRobotModel.js').UnifiedRobotModel} model
     */
    static analyze(model) {
        if (!model?.joints || !model?.links) {
            return { jointRows: [], linkCategories: new Map(), rootLink: null };
        }

        const graph = HumanoidKinematicsAnalyzer.buildGraph(model);
        const rootLink = model.rootLink || HumanoidKinematicsAnalyzer.findRootLink(model, graph);
        const branchPoints = HumanoidKinematicsAnalyzer.findBranchPoints(graph.childrenMap);

        const jointRows = [];
        const linkCategories = new Map();

        model.links.forEach((link, name) => {
            linkCategories.set(name, classifyCategories(name));
        });

        model.joints.forEach((joint, name) => {
            if (joint.type === 'fixed') return;

            const categories = classifyCategories(name, joint.child, joint.parent);
            const jointAxis = classifyJointAxis(name, joint);
            const limbChain = HumanoidKinematicsAnalyzer.buildLimbChainForJoint(
                joint, graph, branchPoints, rootLink
            );

            jointRows.push({
                name,
                categories,
                jointAxis,
                limbChain,
                type: joint.type,
                parent: joint.parent,
                child: joint.child
            });

            assignChildLinkCategories(linkCategories, joint);
        });

        return { jointRows, linkCategories, rootLink, graph };
    }

    static buildGraph(model) {
        const childrenMap = new Map();
        const parentMap = new Map();

        model.joints.forEach(joint => {
            if (!joint.parent || !joint.child) return;
            if (!childrenMap.has(joint.parent)) childrenMap.set(joint.parent, []);
            childrenMap.get(joint.parent).push({
                joint: joint.name,
                child: joint.child
            });
            parentMap.set(joint.child, { parent: joint.parent, joint: joint.name });
        });

        return { childrenMap, parentMap };
    }

    static findRootLink(model, graph) {
        const childLinks = new Set(
            [...model.joints.values()].map(j => j.child).filter(Boolean)
        );
        const roots = [...model.links.keys()].filter(name => !childLinks.has(name));
        if (roots.length === 1) return roots[0];
        if (model.rootLink && model.links.has(model.rootLink)) return model.rootLink;
        return roots[0] || null;
    }

    static findBranchPoints(childrenMap) {
        const branchPoints = new Set();
        childrenMap.forEach((children, parent) => {
            if (children.length > 1) branchPoints.add(parent);
        });
        return branchPoints;
    }

    static buildLimbChainForJoint(joint, graph, branchPoints, rootLink) {
        const { parentMap } = graph;
        const chain = [];
        let current = joint.child;

        while (current) {
            chain.unshift(current);
            if (!parentMap.has(current)) break;
            const { parent, joint: parentJointName } = parentMap.get(current);
            if (branchPoints.has(parent) || parent === rootLink) {
                chain.unshift(parent);
                break;
            }
            current = parent;
        }

        if (chain.length === 0 && joint.parent) {
            return joint.parent;
        }
        return chain.join(' → ');
    }
}
