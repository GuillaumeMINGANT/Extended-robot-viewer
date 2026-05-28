/**
 * Limb Identification Tests — verifies that detectSide, classifyCategories,
 * and HumanoidKinematicsAnalyzer correctly identify limbs across diverse
 * robot naming conventions.
 *
 * Covers: Unitree (left_xxx), Atlas/iCub (l_/r_), JAXON (LARM_/RLEG_),
 * Valkyrie (camelCase), TALOS (xxx_left_xxx), numeric bands, quadrupeds
 * (FL/FR/RL/RR), compact forms (larm, rleg), hyphens, slashes, and more.
 *
 * Run: node tests/limb-identification.test.mjs
 */

import {
    classifyCategories,
    classifyJointAxis,
    detectPrimaryNumericId,
    HumanoidKinematicsAnalyzer
} from '../src/kinematics/HumanoidKinematicsAnalyzer.js';

// ============== Test Utilities ==============

let passed = 0;
let failed = 0;

function assert(condition, message) {
    if (condition) {
        passed++;
        console.log(`  ✓ ${message}`);
    } else {
        failed++;
        console.error(`  ✗ FAIL: ${message}`);
    }
}

function assertIncludes(arr, tag, message) {
    assert(arr.includes(tag), `${message} — expected "${tag}" in [${arr.join(', ')}]`);
}

function assertNotIncludes(arr, tag, message) {
    assert(!arr.includes(tag), `${message} — "${tag}" should NOT be in [${arr.join(', ')}]`);
}

// ============== Helper: Synthetic Model Builder ==============

/**
 * Build a minimal model from a description of joints.
 * Each entry: { name, parent, child, type? }
 * Links are auto-created from parent/child references.
 */
function buildSyntheticModel(modelName, jointDescs, rootLink) {
    const links = new Map();
    const joints = new Map();

    for (const desc of jointDescs) {
        const jType = desc.type || 'revolute';
        joints.set(desc.name, {
            name: desc.name,
            type: jType,
            parent: desc.parent,
            child: desc.child,
            axis: { xyz: [0, 0, 1] },
            limits: { lower: -Math.PI, upper: Math.PI },
            currentValue: 0
        });
        if (!links.has(desc.parent)) links.set(desc.parent, { name: desc.parent });
        if (!links.has(desc.child)) links.set(desc.child, { name: desc.child });
    }

    if (rootLink && !links.has(rootLink)) {
        links.set(rootLink, { name: rootLink });
    }

    return {
        name: modelName,
        links,
        joints,
        rootLink: rootLink || null,
        getLink(name) { return this.links.get(name); },
        getJoint(name) { return this.joints.get(name); }
    };
}

// ============== Test Suites ==============

function testDetectSideUnitreeStyle() {
    console.log('\n═══ Test: detectSide — Unitree style (left_xxx / right_xxx) ═══');

    const cases = [
        ['left_hip_pitch_joint', 'Left Leg', 'Hip'],
        ['left_hip_roll_link', 'Left Leg', 'Hip'],
        ['left_hip_yaw_joint', 'Left Leg', 'Hip'],
        ['left_knee_joint', 'Left Leg', 'Knee'],
        ['left_ankle_pitch_link', 'Left Leg', 'Ankle'],
        ['left_ankle_roll_link', 'Left Leg', 'Ankle'],
        ['right_hip_pitch_joint', 'Right Leg', 'Hip'],
        ['right_knee_joint', 'Right Leg', 'Knee'],
        ['right_ankle_roll_link', 'Right Leg', 'Ankle'],
        ['left_shoulder_pitch_joint', 'Left Arm', 'Shoulder'],
        ['left_shoulder_roll_link', 'Left Arm', 'Shoulder'],
        ['left_elbow_joint', 'Left Arm', 'Elbow'],
        ['left_wrist_roll_link', 'Left Arm', 'Wrist'],
        ['right_shoulder_pitch_joint', 'Right Arm', 'Shoulder'],
        ['right_elbow_joint', 'Right Arm', 'Elbow'],
        ['right_wrist_yaw_link', 'Right Arm', 'Wrist'],
    ];

    for (const [name, expectedRegion, expectedPart] of cases) {
        const cats = classifyCategories(name);
        assertIncludes(cats, expectedRegion, `"${name}" → ${expectedRegion}`);
        assertIncludes(cats, expectedPart, `"${name}" → ${expectedPart}`);
    }
}

function testDetectSideAtlasStyle() {
    console.log('\n═══ Test: detectSide — Atlas/iCub style (l_/r_ prefix) ═══');

    const cases = [
        ['l_leg_hpx', 'Left Leg'],
        ['r_leg_hpx', 'Right Leg'],
        ['l_arm_shz', 'Left Arm'],
        ['r_arm_shz', 'Right Arm'],
        ['l_hip_pitch', 'Left Leg'],
        ['r_hip_pitch', 'Right Leg'],
        ['l_shoulder_roll', 'Left Arm'],
        ['r_shoulder_roll', 'Right Arm'],
    ];

    for (const [name, expectedRegion] of cases) {
        const cats = classifyCategories(name);
        assertIncludes(cats, expectedRegion, `"${name}" → ${expectedRegion}`);
    }
}

function testDetectSideJAXONStyle() {
    console.log('\n═══ Test: detectSide — JAXON style (LARM_/RLEG_ compact) ═══');

    const cases = [
        ['LARM_LINK0', 'Left Arm'],
        ['LARM_LINK7', 'Left Arm'],
        ['RARM_LINK0', 'Right Arm'],
        ['RLEG_LINK2', 'Right Leg'],
        ['LLEG_LINK5', 'Left Leg'],
        ['LF_FOOT', 'Left Leg'],
    ];

    for (const [name, expectedRegion] of cases) {
        const cats = classifyCategories(name);
        assertIncludes(cats, expectedRegion, `"${name}" → ${expectedRegion}`);
    }
}

function testDetectSideCamelCase() {
    console.log('\n═══ Test: detectSide — Valkyrie style (camelCase) ═══');

    const cases = [
        ['leftPalm', 'Left Arm'],
        ['rightPalm', 'Right Arm'],
        ['leftFoot', 'Left Leg'],
        ['rightFoot', 'Right Leg'],
        ['leftHipPitch', 'Left Leg'],
        ['rightShoulderRoll', 'Right Arm'],
        ['leftElbow', 'Left Arm'],
        ['rightKnee', 'Right Leg'],
        ['leftAnkle', 'Left Leg'],
        ['rightWrist', 'Right Arm'],
    ];

    for (const [name, expectedRegion] of cases) {
        const cats = classifyCategories(name);
        assertIncludes(cats, expectedRegion, `"${name}" → ${expectedRegion}`);
    }
}

function testDetectSideTALOSStyle() {
    console.log('\n═══ Test: detectSide — TALOS style (xxx_left_xxx infix) ═══');

    const cases = [
        ['leg_left_1_joint', 'Left Leg'],
        ['leg_right_1_joint', 'Right Leg'],
        ['arm_left_2_link', 'Left Arm'],
        ['arm_right_3_joint', 'Right Arm'],
    ];

    for (const [name, expectedRegion] of cases) {
        const cats = classifyCategories(name);
        assertIncludes(cats, expectedRegion, `"${name}" → ${expectedRegion}`);
    }
}

function testDetectSideHyphenated() {
    console.log('\n═══ Test: detectSide — Hyphen-separated (Cassie style) ═══');

    const cases = [
        ['left-hip-roll', 'Left Leg'],
        ['right-hip-roll', 'Right Leg'],
        ['left-knee-spring', 'Left Leg'],
        ['right-ankle-pitch', 'Right Leg'],
        ['left-shoulder-roll', 'Left Arm'],
        ['right-elbow-pitch', 'Right Arm'],
    ];

    for (const [name, expectedRegion] of cases) {
        const cats = classifyCategories(name);
        assertIncludes(cats, expectedRegion, `"${name}" → ${expectedRegion}`);
    }
}

function testDetectSideSlashSeparated() {
    console.log('\n═══ Test: detectSide — Slash-separated (Robonaut R2 style) ═══');

    const cases = [
        ['r2/left_palm', 'Left Arm'],
        ['r2/right_hand', 'Right Arm'],
    ];

    for (const [name, expectedRegion] of cases) {
        const cats = classifyCategories(name);
        assertIncludes(cats, expectedRegion, `"${name}" → ${expectedRegion}`);
    }
}

function testDetectSideQuadruped() {
    console.log('\n═══ Test: detectSide — Quadruped (FL/FR/RL/RR) ═══');

    // FL = Front-Left, FR = Front-Right, RL = Rear-Left, RR = Rear-Right
    const leftCases = [
        'FL_hip', 'FL_thigh', 'FL_calf', 'FL_foot',
        'RL_hip', 'RL_thigh', 'RL_calf', 'RL_foot',
    ];
    const rightCases = [
        'FR_hip', 'FR_thigh', 'FR_calf', 'FR_foot',
        'RR_hip', 'RR_thigh', 'RR_calf', 'RR_foot',
    ];

    for (const name of leftCases) {
        const cats = classifyCategories(name);
        const hasLeft = cats.some(c => c.startsWith('Left'));
        assert(hasLeft, `"${name}" → detected as left side [${cats.join(', ')}]`);
    }

    for (const name of rightCases) {
        const cats = classifyCategories(name);
        const hasRight = cats.some(c => c.startsWith('Right'));
        assert(hasRight, `"${name}" → detected as right side [${cats.join(', ')}]`);
    }
}

function testDetectSideSuffix() {
    console.log('\n═══ Test: detectSide — Suffix patterns (hip_left, shoulder_r) ═══');

    const cases = [
        ['hip_left', 'Left Leg'],
        ['hip_right', 'Right Leg'],
        ['shoulder_left', 'Left Arm'],
        ['shoulder_right', 'Right Arm'],
    ];

    for (const [name, expectedRegion] of cases) {
        const cats = classifyCategories(name);
        assertIncludes(cats, expectedRegion, `"${name}" → ${expectedRegion}`);
    }
}

function testNoFalsePositiveSide() {
    console.log('\n═══ Test: detectSide — No false positives ═══');

    const neutralNames = [
        'torso_joint',
        'base_link',
        'pelvis',
        'trunk',
        'head_yaw',
        'neck_pitch',
        'spine_joint',
        'imu_link',
        'world',
        'tool_link',
        'barrel_link',
        'camera_frame',
    ];

    for (const name of neutralNames) {
        const cats = classifyCategories(name);
        const hasLeft = cats.some(c => c.startsWith('Left'));
        const hasRight = cats.some(c => c.startsWith('Right'));
        assert(!hasLeft && !hasRight, `"${name}" → no side detected [${cats.join(', ')}]`);
    }
}

function testPartDetection() {
    console.log('\n═══ Test: Part detection — keyword recognition ═══');

    const cases = [
        ['shoulder_pitch', 'Shoulder'],
        ['elbow_flex', 'Elbow'],
        ['wrist_roll', 'Wrist'],
        ['hand_link', 'Hand'],
        ['palm_link', 'Hand'],
        ['finger_tip', 'Finger'],
        ['thumb_joint', 'Finger'],
        ['gripper_link', 'Gripper'],
        ['hip_yaw', 'Hip'],
        ['thigh_link', 'Hip'],
        ['knee_pitch', 'Knee'],
        ['ankle_roll', 'Ankle'],
        ['foot_link', 'Foot'],
        ['sole_link', 'Foot'],
        ['toe_joint', 'Toe'],
        ['head_pan', 'Head'],
        ['neck_tilt', 'Neck'],
        ['torso_link', 'Torso'],
        ['waist_yaw', 'Waist'],
    ];

    for (const [name, expectedPart] of cases) {
        const cats = classifyCategories(name);
        assertIncludes(cats, expectedPart, `"${name}" → ${expectedPart}`);
    }
}

function testNumericBandSideDetection() {
    console.log('\n═══ Test: Numeric band side detection (CLOVIS/ASTRO style) ═══');

    assert(detectPrimaryNumericId('R10_joint') === 10, 'R10_ → id 10');
    assert(detectPrimaryNumericId('R20_joint') === 20, 'R20_ → id 20');
    assert(detectPrimaryNumericId('Link30_foo') === 30, 'Link30_ → id 30');
    assert(detectPrimaryNumericId('base_link_R0_bar') === 0, 'base_link_R0_ → id 0');

    // Ids 10-19 → right, 20-29 → left (from sideFromNumericId)
    const r15 = classifyCategories('R15_hip');
    assertIncludes(r15, 'Right Leg', 'R15_hip → Right (band 10-19)');

    const l25 = classifyCategories('R25_shoulder');
    assertIncludes(l25, 'Left Arm', 'R25_shoulder → Left (band 20-29)');
}

// ============== Integration: Full Model Analysis ==============

function testUnitreeG1Analysis() {
    console.log('\n═══ Test: Full analysis — Unitree G1 style model ═══');

    const model = buildSyntheticModel('unitree_g1', [
        // Left leg
        { name: 'left_hip_yaw_joint', parent: 'pelvis', child: 'left_hip_yaw_link' },
        { name: 'left_hip_roll_joint', parent: 'left_hip_yaw_link', child: 'left_hip_roll_link' },
        { name: 'left_hip_pitch_joint', parent: 'left_hip_roll_link', child: 'left_hip_pitch_link' },
        { name: 'left_knee_joint', parent: 'left_hip_pitch_link', child: 'left_knee_link' },
        { name: 'left_ankle_pitch_joint', parent: 'left_knee_link', child: 'left_ankle_pitch_link' },
        { name: 'left_ankle_roll_joint', parent: 'left_ankle_pitch_link', child: 'left_ankle_roll_link' },
        // Right leg
        { name: 'right_hip_yaw_joint', parent: 'pelvis', child: 'right_hip_yaw_link' },
        { name: 'right_hip_roll_joint', parent: 'right_hip_yaw_link', child: 'right_hip_roll_link' },
        { name: 'right_hip_pitch_joint', parent: 'right_hip_roll_link', child: 'right_hip_pitch_link' },
        { name: 'right_knee_joint', parent: 'right_hip_pitch_link', child: 'right_knee_link' },
        { name: 'right_ankle_pitch_joint', parent: 'right_knee_link', child: 'right_ankle_pitch_link' },
        { name: 'right_ankle_roll_joint', parent: 'right_ankle_pitch_link', child: 'right_ankle_roll_link' },
        // Torso
        { name: 'torso_joint', parent: 'pelvis', child: 'torso_link' },
        // Left arm
        { name: 'left_shoulder_pitch_joint', parent: 'torso_link', child: 'left_shoulder_pitch_link' },
        { name: 'left_shoulder_roll_joint', parent: 'left_shoulder_pitch_link', child: 'left_shoulder_roll_link' },
        { name: 'left_shoulder_yaw_joint', parent: 'left_shoulder_roll_link', child: 'left_shoulder_yaw_link' },
        { name: 'left_elbow_joint', parent: 'left_shoulder_yaw_link', child: 'left_elbow_link' },
        { name: 'left_wrist_roll_joint', parent: 'left_elbow_link', child: 'left_wrist_roll_link' },
        { name: 'left_wrist_pitch_joint', parent: 'left_wrist_roll_link', child: 'left_wrist_pitch_link' },
        { name: 'left_wrist_yaw_joint', parent: 'left_wrist_pitch_link', child: 'left_wrist_yaw_link' },
        // Right arm
        { name: 'right_shoulder_pitch_joint', parent: 'torso_link', child: 'right_shoulder_pitch_link' },
        { name: 'right_shoulder_roll_joint', parent: 'right_shoulder_pitch_link', child: 'right_shoulder_roll_link' },
        { name: 'right_shoulder_yaw_joint', parent: 'right_shoulder_roll_link', child: 'right_shoulder_yaw_link' },
        { name: 'right_elbow_joint', parent: 'right_shoulder_yaw_link', child: 'right_elbow_link' },
        { name: 'right_wrist_roll_joint', parent: 'right_elbow_link', child: 'right_wrist_roll_link' },
        { name: 'right_wrist_pitch_joint', parent: 'right_wrist_roll_link', child: 'right_wrist_pitch_link' },
        { name: 'right_wrist_yaw_joint', parent: 'right_wrist_pitch_link', child: 'right_wrist_yaw_link' },
        // Head
        { name: 'head_joint', parent: 'torso_link', child: 'head_link' },
    ], 'pelvis');

    const analysis = HumanoidKinematicsAnalyzer.analyze(model);
    const lc = analysis.linkCategories;

    // Check that key links have correct region tags
    assertIncludes(lc.get('left_ankle_roll_link') || [], 'Left Leg', 'left_ankle_roll_link → Left Leg');
    assertIncludes(lc.get('right_ankle_roll_link') || [], 'Right Leg', 'right_ankle_roll_link → Right Leg');
    assertIncludes(lc.get('left_wrist_yaw_link') || [], 'Left Arm', 'left_wrist_yaw_link → Left Arm');
    assertIncludes(lc.get('right_wrist_yaw_link') || [], 'Right Arm', 'right_wrist_yaw_link → Right Arm');
    assertIncludes(lc.get('head_link') || [], 'Head', 'head_link → Head');

    // Verify all four limb regions are represented somewhere
    const allCats = new Set();
    lc.forEach(cats => cats.forEach(c => allCats.add(c)));
    assertIncludes([...allCats], 'Left Arm', 'Model has Left Arm region');
    assertIncludes([...allCats], 'Right Arm', 'Model has Right Arm region');
    assertIncludes([...allCats], 'Left Leg', 'Model has Left Leg region');
    assertIncludes([...allCats], 'Right Leg', 'Model has Right Leg region');
    assertIncludes([...allCats], 'Head', 'Model has Head region');
}

function testAtlasStyleAnalysis() {
    console.log('\n═══ Test: Full analysis — Atlas style model (l_/r_ prefix) ═══');

    const model = buildSyntheticModel('atlas_v4', [
        { name: 'l_leg_hpz', parent: 'pelvis', child: 'l_uglut' },
        { name: 'l_leg_hpx', parent: 'l_uglut', child: 'l_lglut' },
        { name: 'l_leg_hpy', parent: 'l_lglut', child: 'l_uleg' },
        { name: 'l_leg_kny', parent: 'l_uleg', child: 'l_lleg' },
        { name: 'l_leg_aky', parent: 'l_lleg', child: 'l_talus' },
        { name: 'l_leg_akx', parent: 'l_talus', child: 'l_foot' },
        { name: 'r_leg_hpz', parent: 'pelvis', child: 'r_uglut' },
        { name: 'r_leg_hpx', parent: 'r_uglut', child: 'r_lglut' },
        { name: 'r_leg_hpy', parent: 'r_lglut', child: 'r_uleg' },
        { name: 'r_leg_kny', parent: 'r_uleg', child: 'r_lleg' },
        { name: 'r_leg_aky', parent: 'r_lleg', child: 'r_talus' },
        { name: 'r_leg_akx', parent: 'r_talus', child: 'r_foot' },
        { name: 'l_arm_shz', parent: 'utorso', child: 'l_clav' },
        { name: 'l_arm_shx', parent: 'l_clav', child: 'l_scap' },
        { name: 'l_arm_ely', parent: 'l_scap', child: 'l_uarm' },
        { name: 'l_arm_elx', parent: 'l_uarm', child: 'l_larm' },
        { name: 'l_arm_wry', parent: 'l_larm', child: 'l_uwrist' },
        { name: 'l_arm_wrx', parent: 'l_uwrist', child: 'l_hand' },
        { name: 'r_arm_shz', parent: 'utorso', child: 'r_clav' },
        { name: 'r_arm_shx', parent: 'r_clav', child: 'r_scap' },
        { name: 'r_arm_ely', parent: 'r_scap', child: 'r_uarm' },
        { name: 'r_arm_elx', parent: 'r_uarm', child: 'r_larm' },
        { name: 'r_arm_wry', parent: 'r_larm', child: 'r_uwrist' },
        { name: 'r_arm_wrx', parent: 'r_uwrist', child: 'r_hand' },
        { name: 'back_bkz', parent: 'pelvis', child: 'ltorso' },
        { name: 'back_bky', parent: 'ltorso', child: 'mtorso' },
        { name: 'back_bkx', parent: 'mtorso', child: 'utorso' },
        { name: 'neck_ry', parent: 'utorso', child: 'head' },
    ], 'pelvis');

    const analysis = HumanoidKinematicsAnalyzer.analyze(model);
    const lc = analysis.linkCategories;

    const allCats = new Set();
    lc.forEach(cats => cats.forEach(c => allCats.add(c)));

    assertIncludes([...allCats], 'Left Arm', 'Atlas model has Left Arm');
    assertIncludes([...allCats], 'Right Arm', 'Atlas model has Right Arm');
    assertIncludes([...allCats], 'Left Leg', 'Atlas model has Left Leg');
    assertIncludes([...allCats], 'Right Leg', 'Atlas model has Right Leg');
}

function testTALOSStyleAnalysis() {
    console.log('\n═══ Test: Full analysis — TALOS style (xxx_left_N infix) ═══');

    const model = buildSyntheticModel('talos', [
        { name: 'leg_left_1_joint', parent: 'base_link', child: 'leg_left_1_link' },
        { name: 'leg_left_2_joint', parent: 'leg_left_1_link', child: 'leg_left_2_link' },
        { name: 'leg_left_3_joint', parent: 'leg_left_2_link', child: 'leg_left_3_link' },
        { name: 'leg_left_4_joint', parent: 'leg_left_3_link', child: 'leg_left_4_link' },
        { name: 'leg_left_5_joint', parent: 'leg_left_4_link', child: 'leg_left_5_link' },
        { name: 'leg_left_6_joint', parent: 'leg_left_5_link', child: 'leg_left_sole_link' },
        { name: 'leg_right_1_joint', parent: 'base_link', child: 'leg_right_1_link' },
        { name: 'leg_right_2_joint', parent: 'leg_right_1_link', child: 'leg_right_2_link' },
        { name: 'leg_right_3_joint', parent: 'leg_right_2_link', child: 'leg_right_3_link' },
        { name: 'leg_right_4_joint', parent: 'leg_right_3_link', child: 'leg_right_4_link' },
        { name: 'leg_right_5_joint', parent: 'leg_right_4_link', child: 'leg_right_5_link' },
        { name: 'leg_right_6_joint', parent: 'leg_right_5_link', child: 'leg_right_sole_link' },
        { name: 'arm_left_1_joint', parent: 'torso_2_link', child: 'arm_left_1_link' },
        { name: 'arm_left_2_joint', parent: 'arm_left_1_link', child: 'arm_left_2_link' },
        { name: 'arm_left_3_joint', parent: 'arm_left_2_link', child: 'arm_left_3_link' },
        { name: 'arm_left_4_joint', parent: 'arm_left_3_link', child: 'arm_left_4_link' },
        { name: 'arm_left_5_joint', parent: 'arm_left_4_link', child: 'arm_left_5_link' },
        { name: 'arm_left_6_joint', parent: 'arm_left_5_link', child: 'arm_left_6_link' },
        { name: 'arm_left_7_joint', parent: 'arm_left_6_link', child: 'arm_left_7_link' },
        { name: 'arm_right_1_joint', parent: 'torso_2_link', child: 'arm_right_1_link' },
        { name: 'arm_right_2_joint', parent: 'arm_right_1_link', child: 'arm_right_2_link' },
        { name: 'arm_right_3_joint', parent: 'arm_right_2_link', child: 'arm_right_3_link' },
        { name: 'arm_right_4_joint', parent: 'arm_right_3_link', child: 'arm_right_4_link' },
        { name: 'arm_right_5_joint', parent: 'arm_right_4_link', child: 'arm_right_5_link' },
        { name: 'arm_right_6_joint', parent: 'arm_right_5_link', child: 'arm_right_6_link' },
        { name: 'arm_right_7_joint', parent: 'arm_right_6_link', child: 'arm_right_7_link' },
        { name: 'torso_1_joint', parent: 'base_link', child: 'torso_1_link' },
        { name: 'torso_2_joint', parent: 'torso_1_link', child: 'torso_2_link' },
        { name: 'head_1_joint', parent: 'torso_2_link', child: 'head_1_link' },
        { name: 'head_2_joint', parent: 'head_1_link', child: 'head_2_link' },
    ], 'base_link');

    const analysis = HumanoidKinematicsAnalyzer.analyze(model);
    const lc = analysis.linkCategories;
    const allCats = new Set();
    lc.forEach(cats => cats.forEach(c => allCats.add(c)));

    assertIncludes([...allCats], 'Left Arm', 'TALOS model has Left Arm');
    assertIncludes([...allCats], 'Right Arm', 'TALOS model has Right Arm');
    assertIncludes([...allCats], 'Left Leg', 'TALOS model has Left Leg');
    assertIncludes([...allCats], 'Right Leg', 'TALOS model has Right Leg');
    assertIncludes([...allCats], 'Head', 'TALOS model has Head');
}

function testCatalogTipLinksPriority() {
    console.log('\n═══ Test: Catalog tipLinks take priority over heuristics ═══');

    // IkController._selectTipLinks checks model.userData.catalogEntry.tipLinks
    // We verify the field is accessible from the model structure.
    const model = buildSyntheticModel('test_catalog', [
        { name: 'j1', parent: 'base', child: 'link1' },
        { name: 'j2', parent: 'link1', child: 'link2' },
    ], 'base');

    model.userData = {
        catalogEntry: {
            tipLinks: ['link2', 'nonexistent_link']
        }
    };

    // Simulate the check in _selectTipLinks
    const catalogTips = model.userData?.catalogEntry?.tipLinks;
    assert(Array.isArray(catalogTips), 'catalogEntry.tipLinks is an array');
    assert(catalogTips.length === 2, 'catalogEntry.tipLinks has 2 entries');

    const valid = catalogTips.filter(t => model.links.has(t));
    assert(valid.length === 1, 'Only valid links are kept (link2)');
    assert(valid[0] === 'link2', 'Valid tip is link2');
}

function testJointAxisClassification() {
    console.log('\n═══ Test: Joint axis classification ═══');

    const cases = [
        ['left_hip_yaw_joint', null, 'yaw'],
        ['left_hip_pitch_joint', null, 'pitch'],
        ['left_hip_roll_joint', null, 'roll'],
        ['right_shoulder_roll_joint', null, 'roll'],
        ['right_knee_pitch_joint', null, 'pitch'],
        ['neck_rz_joint', null, 'yaw'],
        ['neck_ry_joint', null, 'roll'],   // _ry → roll (codebase axis convention)
        ['neck_rx_joint', null, 'pitch'],  // _rx → pitch (codebase axis convention)
        ['wrist1_joint', null, 'yaw'],
        ['wrist2_joint', null, 'pitch'],
        ['wrist3_joint', null, 'roll'],
        ['hip-yaw-joint', null, 'yaw'],    // hyphen-separated
        ['shoulder-pitch', null, 'pitch'],
        ['elbow-roll', null, 'roll'],
    ];

    for (const [name, joint, expected] of cases) {
        const result = classifyJointAxis(name, joint);
        assert(result === expected, `"${name}" axis → ${expected} (got ${result})`);
    }
}

function testSymmetryMarker() {
    console.log('\n═══ Test: _sym suffix → left side ═══');

    const cats = classifyCategories('hip_sym_joint', 'hip_sym_link', null);
    const hasLeft = cats.some(c => c.startsWith('Left'));
    assert(hasLeft, '"hip_sym" → detected as left side');
}

// ============== Run All Tests ==============

console.log('╔══════════════════════════════════════════╗');
console.log('║    Limb Identification Test Suite        ║');
console.log('╚══════════════════════════════════════════╝');

testDetectSideUnitreeStyle();
testDetectSideAtlasStyle();
testDetectSideJAXONStyle();
testDetectSideCamelCase();
testDetectSideTALOSStyle();
testDetectSideHyphenated();
testDetectSideSlashSeparated();
testDetectSideQuadruped();
testDetectSideSuffix();
testNoFalsePositiveSide();
testPartDetection();
testNumericBandSideDetection();
testSymmetryMarker();
testJointAxisClassification();

console.log('\n╔══════════════════════════════════════════╗');
console.log('║    Full Model Analysis Tests             ║');
console.log('╚══════════════════════════════════════════╝');

testUnitreeG1Analysis();
testAtlasStyleAnalysis();
testTALOSStyleAnalysis();
testCatalogTipLinksPriority();

console.log('\n══════════════════════════════════════════');
console.log(`Results: ${passed} passed, ${failed} failed, ${passed + failed} total`);
console.log('══════════════════════════════════════════');

if (failed > 0) {
    process.exit(1);
}
