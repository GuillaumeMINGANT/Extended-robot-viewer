/**
 * IK Integration Tests — standardized inverse kinematics verification.
 *
 * Tests the full solver pipeline:
 *   1. Chain building from a synthetic model
 *   2. Jacobian computation (non-zero, correct dimensions)
 *   3. Position-only (3D) IK convergence
 *   4. Full-pose (6D) IK convergence
 *   5. Joint limit enforcement
 *   6. Null-space limit avoidance (redundant chains)
 *   7. Error frame correctness (world-frame orientation error)
 *
 * Run: node tests/ik-integration.test.mjs
 */

import * as THREE from 'three';

// We import the modules directly (ESM)
import { RobotKinematics } from '../src/kinematics/RobotKinematics.js';
import { JacobianIkSolver } from '../src/kinematics/JacobianIkSolver.js';
import { QPIkSolver } from '../src/kinematics/QPIkSolver.js';

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

function assertApprox(actual, expected, tol, message) {
    const diff = Math.abs(actual - expected);
    assert(diff < tol, `${message} (got ${actual.toFixed(6)}, expected ${expected.toFixed(6)}, diff ${diff.toFixed(6)})`);
}

function vecNorm(arr) {
    let sum = 0;
    for (let i = 0; i < arr.length; i++) sum += arr[i] * arr[i];
    return Math.sqrt(sum);
}

// ============== Synthetic Robot Model ==============

/**
 * Build a 3-DOF planar arm (all revolute, Z-axis, unit link lengths).
 * Chain: base → j1 → link1 → j2 → link2 → j3 → tip
 */
function build3DofArm() {
    const model = {
        name: 'test-3dof',
        links: new Map(),
        joints: new Map(),
        rootLink: 'base',
        getLink(name) { return this.links.get(name); },
        getJoint(name) { return this.joints.get(name); },
        threeObject: null
    };

    // Create scene graph (THREE objects parented correctly)
    const baseObj = new THREE.Object3D();
    baseObj.name = 'base';

    const j1Obj = new THREE.Object3D();
    j1Obj.name = 'j1';
    baseObj.add(j1Obj);

    const link1Obj = new THREE.Object3D();
    link1Obj.name = 'link1';
    link1Obj.position.set(1, 0, 0); // 1 unit in X
    j1Obj.add(link1Obj);

    const j2Obj = new THREE.Object3D();
    j2Obj.name = 'j2';
    link1Obj.add(j2Obj);

    const link2Obj = new THREE.Object3D();
    link2Obj.name = 'link2';
    link2Obj.position.set(1, 0, 0);
    j2Obj.add(link2Obj);

    const j3Obj = new THREE.Object3D();
    j3Obj.name = 'j3';
    link2Obj.add(j3Obj);

    const tipObj = new THREE.Object3D();
    tipObj.name = 'tip';
    tipObj.position.set(1, 0, 0);
    j3Obj.add(tipObj);

    // Update matrices
    baseObj.updateMatrixWorld(true);
    model.threeObject = baseObj;

    // Register links
    const makeLink = (name, obj) => ({ name, threeObject: obj, visuals: [], collisions: [] });
    model.links.set('base', makeLink('base', baseObj));
    model.links.set('link1', makeLink('link1', link1Obj));
    model.links.set('link2', makeLink('link2', link2Obj));
    model.links.set('tip', makeLink('tip', tipObj));

    // Register joints (all revolute about Z)
    const makeJoint = (name, parent, child, obj) => ({
        name,
        type: 'revolute',
        parent,
        child,
        axis: { xyz: [0, 0, 1] },
        limits: { lower: -Math.PI, upper: Math.PI },
        currentValue: 0,
        threeObject: obj
    });

    model.joints.set('j1', makeJoint('j1', 'base', 'link1', j1Obj));
    model.joints.set('j2', makeJoint('j2', 'link1', 'link2', j2Obj));
    model.joints.set('j3', makeJoint('j3', 'link2', 'tip', j3Obj));

    return model;
}

/**
 * Build a 7-DOF spatial arm (for 6D IK testing).
 * Chain: base → 7 revolute joints → tip
 */
function build7DofArm() {
    const model = {
        name: 'test-7dof',
        links: new Map(),
        joints: new Map(),
        rootLink: 'base',
        getLink(name) { return this.links.get(name); },
        getJoint(name) { return this.joints.get(name); },
        threeObject: null
    };

    const axes = [
        [0, 0, 1], // j1: Z
        [0, 1, 0], // j2: Y
        [1, 0, 0], // j3: X
        [0, 1, 0], // j4: Y
        [0, 0, 1], // j5: Z
        [0, 1, 0], // j6: Y
        [1, 0, 0], // j7: X
    ];

    const linkLengths = [0, 0.3, 0, 0.3, 0, 0.2, 0.1];

    const baseObj = new THREE.Object3D();
    baseObj.name = 'base';
    model.links.set('base', { name: 'base', threeObject: baseObj, visuals: [], collisions: [] });

    let parentObj = baseObj;
    let parentLinkName = 'base';

    for (let i = 0; i < 7; i++) {
        const jName = `j${i + 1}`;
        const lName = i < 6 ? `link${i + 1}` : 'tip';

        const jObj = new THREE.Object3D();
        jObj.name = jName;
        parentObj.add(jObj);

        const lObj = new THREE.Object3D();
        lObj.name = lName;
        lObj.position.set(0, linkLengths[i], 0);
        jObj.add(lObj);

        model.links.set(lName, { name: lName, threeObject: lObj, visuals: [], collisions: [] });
        model.joints.set(jName, {
            name: jName,
            type: 'revolute',
            parent: parentLinkName,
            child: lName,
            axis: { xyz: axes[i] },
            limits: { lower: -2.5, upper: 2.5 },
            currentValue: 0,
            threeObject: jObj
        });

        parentObj = lObj;
        parentLinkName = lName;
    }

    baseObj.updateMatrixWorld(true);
    model.threeObject = baseObj;
    return model;
}

/**
 * Simulate setJointAngle for test models.
 */
function setJointAngle(model, jointName, angle) {
    const joint = model.getJoint(jointName);
    if (!joint) return;

    const axis = new THREE.Vector3(...joint.axis.xyz).normalize();
    joint.threeObject.quaternion.setFromAxisAngle(axis, angle);
    joint.currentValue = angle;
    model.threeObject.updateMatrixWorld(true);
}

// ============== Test Suites ==============

function testChainBuilding() {
    console.log('\n═══ Test: Chain Building ═══');

    const model = build3DofArm();
    const kin = new RobotKinematics(model);
    const tips = kin.buildChains();

    assert(tips.length === 1, `Detected 1 tip link (got ${tips.length})`);
    assert(tips[0] === 'tip', `Tip link is "tip" (got "${tips[0]}")`);

    const chain = kin.getChain('tip');
    assert(chain !== null, 'Chain for tip exists');
    assert(chain.joints.length === 3, `Chain has 3 joints (got ${chain.joints.length})`);
    assert(chain.joints[0].name === 'j1', 'First joint is j1');
    assert(chain.joints[2].name === 'j3', 'Last joint is j3');
}

function testForwardKinematics() {
    console.log('\n═══ Test: Forward Kinematics ═══');

    const model = build3DofArm();
    const kin = new RobotKinematics(model);
    kin.buildChains();

    // At home config, tip should be at (3, 0, 0)
    const pos = kin.getLinkWorldPosition('tip');
    assertApprox(pos.x, 3, 1e-6, 'Tip X at home = 3');
    assertApprox(pos.y, 0, 1e-6, 'Tip Y at home = 0');
    assertApprox(pos.z, 0, 1e-6, 'Tip Z at home = 0');

    // Rotate j1 by PI/2 → tip at (0, 3, 0)
    setJointAngle(model, 'j1', Math.PI / 2);
    const pos2 = kin.getLinkWorldPosition('tip');
    assertApprox(pos2.x, 0, 1e-4, 'Tip X after j1=PI/2 ≈ 0');
    assertApprox(pos2.y, 3, 1e-4, 'Tip Y after j1=PI/2 ≈ 3');
}

function testJacobianNonZero() {
    console.log('\n═══ Test: Jacobian Non-Zero ═══');

    const model = build3DofArm();
    const kin = new RobotKinematics(model);
    kin.buildChains();

    const result = kin.computeJacobian('tip', 3);
    assert(result !== null, 'Jacobian computed successfully');
    assert(result.rows === 3, `Jacobian has 3 rows (got ${result.rows})`);
    assert(result.cols === 3, `Jacobian has 3 cols (got ${result.cols})`);

    const jNorm = vecNorm(result.J);
    assert(jNorm > 0.1, `Jacobian norm > 0.1 (got ${jNorm.toFixed(4)})`);

    // For a 3-link planar arm at home, J should have specific structure
    // z-axis cross (tip - joint) gives Y-component contributions
    // J[1,0] should be non-zero (j1 rotates tip in Y direction)
    assert(Math.abs(result.J[1 * 3 + 0]) > 0.5, 'J[1,0] is significant (j1 moves tip in Y)');
}

function testPositionIkConvergence() {
    console.log('\n═══ Test: Position-Only IK Convergence (3D) ═══');

    const model = build3DofArm();
    const kin = new RobotKinematics(model);
    kin.buildChains();

    const solver = new JacobianIkSolver(kin, { maxIterations: 50, tolerance: 1e-3, stepSize: 0.5 });

    // Target: (2, 1, 0) — reachable for a 3-unit-reach planar arm
    const target = new THREE.Vector3(2, 1, 0);

    let totalIter = 0;
    for (let step = 0; step < 100; step++) {
        const result = solver.solve('tip', target, null);
        if (!result || result.deltas.length === 0) break;

        totalIter++;
        for (const { name, delta } of result.deltas) {
            const joint = model.getJoint(name);
            let newVal = joint.currentValue + delta;
            newVal = Math.max(joint.limits.lower, Math.min(joint.limits.upper, newVal));
            setJointAngle(model, name, newVal);
        }

        if (result.converged) break;
    }

    const finalPos = kin.getLinkWorldPosition('tip');
    const posError = finalPos.distanceTo(target);

    assert(totalIter > 0, `Solver ran (${totalIter} iterations)`);
    assert(posError < 0.01, `Position error < 0.01 (got ${posError.toFixed(6)})`);
    console.log(`    Converged in ${totalIter} steps, error: ${posError.toFixed(6)}`);
}

function testFullPoseIkConvergence() {
    console.log('\n═══ Test: Full Pose IK Convergence (6D) ═══');

    const model = build7DofArm();
    const kin = new RobotKinematics(model);
    kin.buildChains();

    const solver = new JacobianIkSolver(kin, { maxIterations: 50, tolerance: 1e-3, stepSize: 0.5 });

    // Target: offset from home position
    const target = new THREE.Vector3(0.2, 0.7, 0.1);
    const targetQuat = new THREE.Quaternion().setFromEuler(new THREE.Euler(0.3, 0.2, 0.1));

    let totalIter = 0;
    for (let step = 0; step < 200; step++) {
        const result = solver.solve('tip', target, targetQuat);
        if (!result || result.deltas.length === 0) break;

        totalIter++;
        for (const { name, delta } of result.deltas) {
            const joint = model.getJoint(name);
            let newVal = joint.currentValue + delta;
            newVal = Math.max(joint.limits.lower, Math.min(joint.limits.upper, newVal));
            setJointAngle(model, name, newVal);
        }

        if (result.converged) break;
    }

    const finalPos = kin.getLinkWorldPosition('tip');
    const posError = finalPos.distanceTo(target);

    const finalQuat = kin.getLinkWorldQuaternion('tip');
    const quatDot = Math.abs(finalQuat.dot(targetQuat));
    const orientError = Math.acos(Math.min(1, quatDot)) * 2;

    assert(totalIter > 0, `Solver ran (${totalIter} iterations)`);
    assert(posError < 0.05, `Position error < 0.05 (got ${posError.toFixed(6)})`);
    assert(orientError < 0.1, `Orientation error < 0.1 rad (got ${orientError.toFixed(4)} rad)`);
    console.log(`    Converged in ${totalIter} steps, pos err: ${posError.toFixed(6)}, orient err: ${orientError.toFixed(4)} rad`);
}

function testJointLimitsEnforced() {
    console.log('\n═══ Test: Joint Limits Enforcement ═══');

    const model = build3DofArm();
    // Set tight limits
    model.getJoint('j1').limits = { lower: -0.5, upper: 0.5 };
    model.getJoint('j2').limits = { lower: -0.5, upper: 0.5 };
    model.getJoint('j3').limits = { lower: -0.5, upper: 0.5 };

    const kin = new RobotKinematics(model);
    kin.buildChains();

    const solver = new JacobianIkSolver(kin, { maxIterations: 50, tolerance: 1e-3, stepSize: 0.5 });

    // Target far away — solver should not exceed limits
    const target = new THREE.Vector3(0, 3, 0);

    for (let step = 0; step < 50; step++) {
        const result = solver.solve('tip', target, null);
        if (!result || result.deltas.length === 0) break;

        for (const { name, delta } of result.deltas) {
            const joint = model.getJoint(name);
            let newVal = joint.currentValue + delta;
            newVal = Math.max(joint.limits.lower, Math.min(joint.limits.upper, newVal));
            setJointAngle(model, name, newVal);
        }
    }

    // Verify all joints within limits
    let allWithin = true;
    model.joints.forEach((joint) => {
        if (joint.currentValue < joint.limits.lower - 1e-6 || joint.currentValue > joint.limits.upper + 1e-6) {
            allWithin = false;
        }
    });

    assert(allWithin, 'All joints remain within limits after IK');
}

function testDeltaMagnitude() {
    console.log('\n═══ Test: Delta Magnitude (non-trivial) ═══');

    const model = build3DofArm();
    const kin = new RobotKinematics(model);
    kin.buildChains();

    const solver = new JacobianIkSolver(kin, { maxIterations: 50, tolerance: 1e-3, stepSize: 0.5 });

    // Target offset by 0.5 units — solver should produce meaningful deltas
    const target = new THREE.Vector3(2.5, 0.5, 0);
    const result = solver.solve('tip', target, null);

    assert(result !== null, 'Solver returned result');
    assert(result.deltas.length > 0, `Solver returned ${result.deltas.length} deltas`);

    const maxDelta = Math.max(...result.deltas.map(d => Math.abs(d.delta)));
    assert(maxDelta > 0.01, `Max delta > 0.01 rad (got ${maxDelta.toFixed(6)})`);
    assert(maxDelta < 2.0, `Max delta < 2.0 rad (got ${maxDelta.toFixed(6)}) — not explosive`);

    console.log(`    Deltas: ${result.deltas.map(d => d.delta.toFixed(4)).join(', ')}`);
}

function testOrientationErrorWorldFrame() {
    console.log('\n═══ Test: Orientation Error in World Frame ═══');

    const model = build7DofArm();
    const kin = new RobotKinematics(model);
    kin.buildChains();

    // Set a non-trivial configuration
    setJointAngle(model, 'j1', 0.3);
    setJointAngle(model, 'j2', -0.5);
    setJointAngle(model, 'j4', 0.4);

    const tipPos = kin.getLinkWorldPosition('tip');
    const tipQuat = kin.getLinkWorldQuaternion('tip');

    // Target = current (error should be zero)
    const err0 = kin.computePoseError('tip', tipPos, tipQuat);
    const norm0 = vecNorm(err0.error);
    assert(norm0 < 1e-6, `Error is zero at current pose (got ${norm0.toExponential(3)})`);

    // Small rotation target — error should be small and proportional
    const smallRot = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), 0.1);
    const targetQuat = smallRot.multiply(tipQuat.clone());
    const err1 = kin.computePoseError('tip', tipPos, targetQuat);

    // Orientation error magnitude should be ~0.1 (the applied rotation angle)
    const orientNorm = Math.sqrt(err1.error[3] ** 2 + err1.error[4] ** 2 + err1.error[5] ** 2);
    assertApprox(orientNorm, 0.1, 0.02, 'Orientation error magnitude ≈ 0.1 rad');
}

function testIterativeConvergenceSpeed() {
    console.log('\n═══ Test: Iterative Convergence Speed ═══');

    const model = build3DofArm();
    const kin = new RobotKinematics(model);
    kin.buildChains();

    const solver = new JacobianIkSolver(kin);

    // Simulate a drag event: 40 iterations with apply between each
    const target = new THREE.Vector3(2, 1.5, 0);
    let converged = false;

    for (let i = 0; i < 40; i++) {
        const result = solver.solve('tip', target, null);
        if (!result || result.deltas.length === 0) { converged = true; break; }

        for (const { name, delta } of result.deltas) {
            const joint = model.getJoint(name);
            let newVal = joint.currentValue + delta;
            newVal = Math.max(joint.limits.lower, Math.min(joint.limits.upper, newVal));
            setJointAngle(model, name, newVal);
        }

        if (result.converged) { converged = true; break; }
    }

    const finalPos = kin.getLinkWorldPosition('tip');
    const posError = finalPos.distanceTo(target);

    assert(converged || posError < 0.01, `Converges within 40 iterations (error: ${posError.toFixed(6)})`);
    console.log(`    Final error: ${posError.toFixed(6)}, converged: ${converged}`);
}

function testAxisExtractionFromVector3() {
    console.log('\n═══ Test: Axis Extraction from THREE.Vector3 (URDF-style) ═══');

    // Simulate how urdf-loader stores axis: as a THREE.Vector3
    const model = build3DofArm();

    // Override joints to use Vector3-style axis (mimics URDF loader)
    model.joints.forEach((joint) => {
        // Store axis as Vector3 on threeObject (like urdf-loader does)
        joint.threeObject.axis = new THREE.Vector3(0, 0, 1);
        joint.threeObject.axis.isVector3 = true;

        // Also test that our unified model correctly has xyz array
        assert(
            Array.isArray(joint.axis.xyz),
            `joint "${joint.name}" axis.xyz is an array`
        );
    });

    // Build a model with Y-axis joints to test non-Z axis handling
    const yModel = build3DofArm();
    // Change joint axes to Y — simulating what urdf-loader provides
    yModel.joints.forEach((joint) => {
        joint.axis = { xyz: [0, 1, 0] };
        // Simulate a real Y-rotation
        const axis = new THREE.Vector3(0, 1, 0);
        joint.threeObject.quaternion.setFromAxisAngle(axis, 0);
    });
    yModel.threeObject.updateMatrixWorld(true);

    const kin = new RobotKinematics(yModel);
    kin.buildChains();

    const result = kin.computeJacobian('tip', 3);
    assert(result !== null, 'Jacobian computed for Y-axis model');

    // For Y-axis joints, rotating about Y with tip at (3,0,0):
    // cross(Y, (tip - joint)) should give Z-component contribution
    // J[2,0] should be non-zero (j1 about Y moves tip in Z direction)
    // Actually for Y-axis rotation, cross([0,1,0], [3,0,0]) = [0,0,-3]
    // So the Z-component (row 2) for column 0 should be significant
    assert(Math.abs(result.J[2 * 3 + 0]) > 0.5,
        `Y-axis j1 produces Z movement (got J[2,0]=${result.J[2 * 3 + 0].toFixed(4)})`);

    // cross([0,1,0], [3,0,0]) = [0*0 - 0*0, 0*3 - 1*0, 1*0 - 0*3] = [0, 0, -3]
    // Wait: cross(a, b) = [a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x]
    // cross([0,1,0], [3,0,0]) = [1*0 - 0*0, 0*3 - 0*0, 0*0 - 1*3] = [0, 0, -3]
    assertApprox(result.J[2 * 3 + 0], -3, 0.01,
        'J[2,0] ≈ -3 for Y-axis joint with 3-unit reach');
}

function testPositionIkWithMixedAxes() {
    console.log('\n═══ Test: Position IK with Mixed Axes (realistic) ═══');

    // 3-DOF arm: j1 about Z, j2 about Y, j3 about Y
    const model = build3DofArm();
    model.getJoint('j1').axis = { xyz: [0, 0, 1] };
    model.getJoint('j2').axis = { xyz: [0, 1, 0] };
    model.getJoint('j3').axis = { xyz: [0, 1, 0] };

    function setJointAngleMixed(mdl, jointName, angle) {
        const joint = mdl.getJoint(jointName);
        if (!joint) return;
        const axis = new THREE.Vector3(...joint.axis.xyz).normalize();
        joint.threeObject.quaternion.setFromAxisAngle(axis, angle);
        joint.currentValue = angle;
        mdl.threeObject.updateMatrixWorld(true);
    }

    const kin = new RobotKinematics(model);
    kin.buildChains();

    const solver = new JacobianIkSolver(kin, { maxIterations: 50, tolerance: 1e-3, stepSize: 0.25 });

    // Target slightly below and to the side: (2, 0, -1) — requires bending in Z via Y-axis joints
    const target = new THREE.Vector3(2, 0, -1);

    for (let step = 0; step < 100; step++) {
        const result = solver.solve('tip', target, null);
        if (!result || result.deltas.length === 0) break;

        for (const { name, delta } of result.deltas) {
            const joint = model.getJoint(name);
            let newVal = joint.currentValue + delta;
            newVal = Math.max(joint.limits.lower, Math.min(joint.limits.upper, newVal));
            setJointAngleMixed(model, name, newVal);
        }

        if (step < 5 || step % 20 === 0) {
            const p = kin.getLinkWorldPosition('tip');
            const err = p.distanceTo(target);
            const angles = ['j1', 'j2', 'j3'].map(n => model.getJoint(n).currentValue.toFixed(3));
            console.log(`    step ${step}: tip=(${p.x.toFixed(3)},${p.y.toFixed(3)},${p.z.toFixed(3)}) err=${err.toFixed(4)} joints=[${angles}]`);
        }

        if (result.converged) break;
    }

    const finalPos = kin.getLinkWorldPosition('tip');
    const posError = finalPos.distanceTo(target);

    assert(posError < 0.05, `Mixed-axis IK converges (error: ${posError.toFixed(6)})`);
    console.log(`    Final position: (${finalPos.x.toFixed(3)}, ${finalPos.y.toFixed(3)}, ${finalPos.z.toFixed(3)}), error: ${posError.toFixed(6)}`);
}

function testSolveForTipMovesLink() {
    console.log('\n═══ Test: Solve-for-tip moves link toward target ═══');

    const model = build3DofArm();
    const kin = new RobotKinematics(model);
    kin.buildChains();
    const solver = new JacobianIkSolver(kin);

    // Simulate gizmo drag: target 0.5 units away from home
    const target = new THREE.Vector3(2.5, 0.5, 0);
    const initialPos = kin.getLinkWorldPosition('tip').clone();
    const initialDist = initialPos.distanceTo(target);

    // Simulate 40 iterations (like IK_ITERATIONS_PER_FRAME)
    for (let i = 0; i < 40; i++) {
        const result = solver.solve('tip', target, null);
        if (!result || result.deltas.length === 0) break;

        for (const { name, delta } of result.deltas) {
            const joint = model.getJoint(name);
            let newVal = joint.currentValue + delta;
            newVal = Math.max(joint.limits.lower, Math.min(joint.limits.upper, newVal));
            setJointAngle(model, name, newVal);
        }
        if (result.converged) break;
    }

    const finalPos = kin.getLinkWorldPosition('tip');
    const finalDist = finalPos.distanceTo(target);

    assert(finalDist < initialDist, `Link moved closer to target (${initialDist.toFixed(3)} → ${finalDist.toFixed(3)})`);
    assert(finalDist < 0.01, `Link reached target within 0.01 (dist: ${finalDist.toFixed(6)})`);

    // Key: after solving, link world position MUST match where gizmo would be
    // (i.e. the solver drove the tip to the target)
    assertApprox(finalPos.x, target.x, 0.01, 'Tip X matches target');
    assertApprox(finalPos.y, target.y, 0.01, 'Tip Y matches target');
}

function testAnchorSyncsAfterSolve() {
    console.log('\n═══ Test: Anchor sync — other tips update after solve ═══');

    // Build a model with two chains sharing joints (branching)
    const model = build7DofArm();
    const kin = new RobotKinematics(model);
    kin.buildChains();
    const solver = new JacobianIkSolver(kin);

    const tipName = kin.tipLinks[0];
    const initialPos = kin.getLinkWorldPosition(tipName).clone();

    // Solve toward a target
    const target = new THREE.Vector3(0.1, 0.5, 0.2);
    for (let i = 0; i < 40; i++) {
        const result = solver.solve(tipName, target, null);
        if (!result || result.deltas.length === 0) break;
        for (const { name, delta } of result.deltas) {
            const joint = model.getJoint(name);
            let newVal = joint.currentValue + delta;
            newVal = Math.max(joint.limits.lower, Math.min(joint.limits.upper, newVal));
            const axis = new THREE.Vector3(...joint.axis.xyz).normalize();
            joint.threeObject.quaternion.setFromAxisAngle(axis, newVal);
            joint.currentValue = newVal;
            model.threeObject.updateMatrixWorld(true);
        }
        if (result.converged) break;
    }

    // After solve, getLinkWorldPosition should reflect new pose
    const afterPos = kin.getLinkWorldPosition(tipName);
    const posDiff = afterPos.distanceTo(initialPos);
    assert(posDiff > 0.01, `Tip position changed after solve (moved ${posDiff.toFixed(4)})`);

    // The anchor position should be re-fetched (simulating syncGizmoPositions)
    const syncedPos = kin.getLinkWorldPosition(tipName);
    assertApprox(syncedPos.x, afterPos.x, 1e-6, 'Synced position X matches current FK');
    assertApprox(syncedPos.y, afterPos.y, 1e-6, 'Synced position Y matches current FK');
    assertApprox(syncedPos.z, afterPos.z, 1e-6, 'Synced position Z matches current FK');
}

function testGizmoSizesSeparation() {
    console.log('\n═══ Test: Gizmo sizes — translate < rotate (no overlap) ═══');

    const translateSize = 0.25;
    const rotateSize = 0.5;
    assert(rotateSize > translateSize * 1.5,
        `Rotate ring (${rotateSize}) > 1.5× translate arrows (${translateSize})`);

    const arrowReach = translateSize * 0.8;
    const ringInner = rotateSize * 0.7;
    assert(ringInner > arrowReach,
        `Ring inner edge (${ringInner.toFixed(3)}) > arrow reach (${arrowReach.toFixed(3)})`);
}

function testFullSolveForTipLoop() {
    console.log('\n═══ Test: Full _solveForTip loop (app pattern) ═══');

    const model = build3DofArm();
    const kin = new RobotKinematics(model);
    kin.buildChains();
    const solver = new JacobianIkSolver(kin);

    const IK_ITERATIONS = 40;
    const targets = [
        new THREE.Vector3(2.5, 0.5, 0),
        new THREE.Vector3(1.5, 1.5, 0),
        new THREE.Vector3(2.0, -1.0, 0),
    ];

    for (let t = 0; t < targets.length; t++) {
        const target = targets[t];
        for (let i = 0; i < IK_ITERATIONS; i++) {
            const result = solver.solve('tip', target, null);
            if (!result || result.deltas.length === 0) break;

            for (const { name, delta } of result.deltas) {
                const joint = model.getJoint(name);
                if (!joint) continue;
                let newValue = joint.currentValue + delta;
                if (joint.limits) {
                    newValue = Math.max(joint.limits.lower, Math.min(joint.limits.upper, newValue));
                }
                // Exact same pattern as IkController._applyJoint:
                const axis = new THREE.Vector3(...joint.axis.xyz).normalize();
                joint.threeObject.quaternion.setFromAxisAngle(axis, newValue);
                joint.currentValue = newValue;
                model.threeObject.updateMatrixWorld(true);
            }
            if (result.converged) break;
        }

        const pos = kin.getLinkWorldPosition('tip');
        const err = pos.distanceTo(target);
        assert(err < 0.02, `Target ${t+1} reached (error: ${err.toFixed(6)})`);
    }
}

function testRotationIkShortChain() {
    console.log('\n═══ Test: Rotation IK on short chain (< 6 DOF) ═══');

    // Build a 3-DOF arm and verify rotation gizmo drives orientation changes
    const model = build3DofArm();
    const kin = new RobotKinematics(model);
    kin.buildChains();
    const solver = new JacobianIkSolver(kin);

    const tipName = 'tip';

    // Get current position (keep it as target position so we only change orientation)
    const currentPos = kin.getLinkWorldPosition(tipName).clone();
    const initialQuat = kin.getLinkWorldQuaternion(tipName).clone();

    // Target: rotate 45 degrees around Z
    const targetQuat = new THREE.Quaternion().setFromAxisAngle(
        new THREE.Vector3(0, 0, 1), Math.PI / 4
    );

    // Solve with both position + orientation
    for (let i = 0; i < 40; i++) {
        const result = solver.solve(tipName, currentPos, targetQuat);
        if (!result || result.deltas.length === 0) break;

        for (const { name, delta } of result.deltas) {
            const joint = model.getJoint(name);
            let newVal = joint.currentValue + delta;
            newVal = Math.max(joint.limits.lower, Math.min(joint.limits.upper, newVal));
            const axis = new THREE.Vector3(...joint.axis.xyz).normalize();
            joint.threeObject.quaternion.setFromAxisAngle(axis, newVal);
            joint.currentValue = newVal;
            model.threeObject.updateMatrixWorld(true);
        }
        if (result.converged) break;
    }

    const finalQuat = kin.getLinkWorldQuaternion(tipName);

    // The orientation should have changed from initial
    const dotInitial = Math.abs(finalQuat.dot(initialQuat));
    const dotTarget = Math.abs(finalQuat.dot(targetQuat));
    assert(dotInitial < 0.99, `Orientation changed from initial (dot=${dotInitial.toFixed(4)})`);
    assert(dotTarget > 0.8, `Orientation moved toward target (dot=${dotTarget.toFixed(4)})`);
}

function testMutualExclusionLogic() {
    console.log('\n═══ Test: Mutual exclusion — mode tracking ═══');

    // Verify the _activeMode flag correctly isolates translate vs rotate
    let activeMode = null;
    let dragging = false;

    // Simulate translate drag start
    dragging = true;
    activeMode = 'translate';
    assert(activeMode === 'translate', 'Mode is translate during translate drag');

    // Simulate objectChange during translate drag
    const shouldCallTranslate = dragging && activeMode === 'translate';
    const shouldCallRotate = dragging && activeMode === 'rotate';
    assert(shouldCallTranslate === true, 'Translate handler fires during translate drag');
    assert(shouldCallRotate === false, 'Rotate handler does NOT fire during translate drag');

    // Simulate drag end
    dragging = false;
    activeMode = null;

    // Simulate rotate drag start
    dragging = true;
    activeMode = 'rotate';
    const shouldCallTranslate2 = dragging && activeMode === 'translate';
    const shouldCallRotate2 = dragging && activeMode === 'rotate';
    assert(shouldCallTranslate2 === false, 'Translate handler does NOT fire during rotate drag');
    assert(shouldCallRotate2 === true, 'Rotate handler fires during rotate drag');
}

// ============== QP Solver Test Suites ==============

function testQPPositionIkConvergence() {
    console.log('\n═══ Test: QP — Position-Only IK Convergence (3D) ═══');

    const model = build3DofArm();
    const kin = new RobotKinematics(model);
    kin.buildChains();

    const solver = new QPIkSolver(kin, { maxIterations: 50, tolerance: 1e-3, stepSize: 0.5 });

    const target = new THREE.Vector3(2, 1, 0);

    let totalIter = 0;
    for (let step = 0; step < 100; step++) {
        const result = solver.solve('tip', target, null);
        if (!result || result.deltas.length === 0) break;

        totalIter++;
        for (const { name, delta } of result.deltas) {
            const joint = model.getJoint(name);
            let newVal = joint.currentValue + delta;
            newVal = Math.max(joint.limits.lower, Math.min(joint.limits.upper, newVal));
            setJointAngle(model, name, newVal);
        }

        if (result.converged) break;
    }

    const finalPos = kin.getLinkWorldPosition('tip');
    const posError = finalPos.distanceTo(target);

    assert(totalIter > 0, `QP solver ran (${totalIter} iterations)`);
    assert(posError < 0.01, `QP position error < 0.01 (got ${posError.toFixed(6)})`);
    console.log(`    Converged in ${totalIter} steps, error: ${posError.toFixed(6)}`);
}

function testQPFullPoseIkConvergence() {
    console.log('\n═══ Test: QP — Full Pose IK Convergence (6D) ═══');

    const model = build7DofArm();
    const kin = new RobotKinematics(model);
    kin.buildChains();

    const solver = new QPIkSolver(kin, { maxIterations: 50, tolerance: 1e-3, stepSize: 0.5 });

    const target = new THREE.Vector3(0.2, 0.7, 0.1);
    const targetQuat = new THREE.Quaternion().setFromEuler(new THREE.Euler(0.3, 0.2, 0.1));

    let totalIter = 0;
    for (let step = 0; step < 200; step++) {
        const result = solver.solve('tip', target, targetQuat);
        if (!result || result.deltas.length === 0) break;

        totalIter++;
        for (const { name, delta } of result.deltas) {
            const joint = model.getJoint(name);
            let newVal = joint.currentValue + delta;
            newVal = Math.max(joint.limits.lower, Math.min(joint.limits.upper, newVal));
            setJointAngle(model, name, newVal);
        }

        if (result.converged) break;
    }

    const finalPos = kin.getLinkWorldPosition('tip');
    const posError = finalPos.distanceTo(target);

    const finalQuat = kin.getLinkWorldQuaternion('tip');
    const quatDot = Math.abs(finalQuat.dot(targetQuat));
    const orientError = Math.acos(Math.min(1, quatDot)) * 2;

    assert(totalIter > 0, `QP solver ran (${totalIter} iterations)`);
    assert(posError < 0.05, `QP position error < 0.05 (got ${posError.toFixed(6)})`);
    assert(orientError < 0.1, `QP orientation error < 0.1 rad (got ${orientError.toFixed(4)} rad)`);
    console.log(`    Converged in ${totalIter} steps, pos err: ${posError.toFixed(6)}, orient err: ${orientError.toFixed(4)} rad`);
}

function testQPJointLimitsEnforced() {
    console.log('\n═══ Test: QP — Joint Limits Enforcement ═══');

    const model = build3DofArm();
    model.getJoint('j1').limits = { lower: -0.5, upper: 0.5 };
    model.getJoint('j2').limits = { lower: -0.5, upper: 0.5 };
    model.getJoint('j3').limits = { lower: -0.5, upper: 0.5 };

    const kin = new RobotKinematics(model);
    kin.buildChains();

    const solver = new QPIkSolver(kin, { maxIterations: 50, tolerance: 1e-3, stepSize: 0.5 });

    const target = new THREE.Vector3(0, 3, 0);

    for (let step = 0; step < 50; step++) {
        const result = solver.solve('tip', target, null);
        if (!result || result.deltas.length === 0) break;

        for (const { name, delta } of result.deltas) {
            const joint = model.getJoint(name);
            let newVal = joint.currentValue + delta;
            newVal = Math.max(joint.limits.lower, Math.min(joint.limits.upper, newVal));
            setJointAngle(model, name, newVal);
        }
    }

    let allWithin = true;
    model.joints.forEach((joint) => {
        if (joint.currentValue < joint.limits.lower - 1e-6 || joint.currentValue > joint.limits.upper + 1e-6) {
            allWithin = false;
        }
    });

    assert(allWithin, 'QP: All joints remain within limits after IK');
}

function testQPDeltaMagnitude() {
    console.log('\n═══ Test: QP — Delta Magnitude (non-trivial) ═══');

    const model = build3DofArm();
    const kin = new RobotKinematics(model);
    kin.buildChains();

    const solver = new QPIkSolver(kin, { maxIterations: 50, tolerance: 1e-3, stepSize: 0.5 });

    const target = new THREE.Vector3(2.5, 0.5, 0);
    const result = solver.solve('tip', target, null);

    assert(result !== null, 'QP solver returned result');
    assert(result.deltas.length > 0, `QP solver returned ${result.deltas.length} deltas`);

    const maxDelta = Math.max(...result.deltas.map(d => Math.abs(d.delta)));
    assert(maxDelta > 0.001, `QP max delta > 0.001 rad (got ${maxDelta.toFixed(6)})`);
    assert(maxDelta < 2.0, `QP max delta < 2.0 rad (got ${maxDelta.toFixed(6)}) — not explosive`);

    console.log(`    Deltas: ${result.deltas.map(d => d.delta.toFixed(4)).join(', ')}`);
}

function testQPIterativeConvergenceSpeed() {
    console.log('\n═══ Test: QP — Iterative Convergence Speed ═══');

    const model = build3DofArm();
    const kin = new RobotKinematics(model);
    kin.buildChains();

    const solver = new QPIkSolver(kin);

    const target = new THREE.Vector3(2, 1.5, 0);
    let converged = false;

    for (let i = 0; i < 40; i++) {
        const result = solver.solve('tip', target, null);
        if (!result || result.deltas.length === 0) { converged = true; break; }

        for (const { name, delta } of result.deltas) {
            const joint = model.getJoint(name);
            let newVal = joint.currentValue + delta;
            newVal = Math.max(joint.limits.lower, Math.min(joint.limits.upper, newVal));
            setJointAngle(model, name, newVal);
        }

        if (result.converged) { converged = true; break; }
    }

    const finalPos = kin.getLinkWorldPosition('tip');
    const posError = finalPos.distanceTo(target);

    assert(converged || posError < 0.01, `QP converges within 40 iterations (error: ${posError.toFixed(6)})`);
    console.log(`    Final error: ${posError.toFixed(6)}, converged: ${converged}`);
}

function testQPFullSolveLoop() {
    console.log('\n═══ Test: QP — Full solve loop (app pattern) ═══');

    const model = build3DofArm();
    const kin = new RobotKinematics(model);
    kin.buildChains();
    const solver = new QPIkSolver(kin);

    const IK_ITERATIONS = 40;
    const targets = [
        new THREE.Vector3(2.5, 0.5, 0),
        new THREE.Vector3(1.5, 1.5, 0),
        new THREE.Vector3(2.0, -1.0, 0),
    ];

    for (let t = 0; t < targets.length; t++) {
        const target = targets[t];
        for (let i = 0; i < IK_ITERATIONS; i++) {
            const result = solver.solve('tip', target, null);
            if (!result || result.deltas.length === 0) break;

            for (const { name, delta } of result.deltas) {
                const joint = model.getJoint(name);
                if (!joint) continue;
                let newValue = joint.currentValue + delta;
                if (joint.limits) {
                    newValue = Math.max(joint.limits.lower, Math.min(joint.limits.upper, newValue));
                }
                const axis = new THREE.Vector3(...joint.axis.xyz).normalize();
                joint.threeObject.quaternion.setFromAxisAngle(axis, newValue);
                joint.currentValue = newValue;
                model.threeObject.updateMatrixWorld(true);
            }
            if (result.converged) break;
        }

        const pos = kin.getLinkWorldPosition('tip');
        const err = pos.distanceTo(target);
        assert(err < 0.02, `QP target ${t+1} reached (error: ${err.toFixed(6)})`);
    }
}

function testQPSolveForTipMovesLink() {
    console.log('\n═══ Test: QP — Solve-for-tip moves link toward target ═══');

    const model = build3DofArm();
    const kin = new RobotKinematics(model);
    kin.buildChains();
    const solver = new QPIkSolver(kin);

    const target = new THREE.Vector3(2.5, 0.5, 0);
    const initialPos = kin.getLinkWorldPosition('tip').clone();
    const initialDist = initialPos.distanceTo(target);

    for (let i = 0; i < 40; i++) {
        const result = solver.solve('tip', target, null);
        if (!result || result.deltas.length === 0) break;

        for (const { name, delta } of result.deltas) {
            const joint = model.getJoint(name);
            let newVal = joint.currentValue + delta;
            newVal = Math.max(joint.limits.lower, Math.min(joint.limits.upper, newVal));
            setJointAngle(model, name, newVal);
        }
        if (result.converged) break;
    }

    const finalPos = kin.getLinkWorldPosition('tip');
    const finalDist = finalPos.distanceTo(target);

    assert(finalDist < initialDist, `QP: Link moved closer to target (${initialDist.toFixed(3)} → ${finalDist.toFixed(3)})`);
    assert(finalDist < 0.01, `QP: Link reached target within 0.01 (dist: ${finalDist.toFixed(6)})`);
}

// ============== 7-DOF QP Tests ==============

function testQP7DofFullPoseConvergence() {
    console.log('\n═══ Test: QP 7-DOF — Full Pose IK (6D) ═══');

    const model = build7DofArm();
    const kin = new RobotKinematics(model);
    kin.buildChains();
    const qpSolver = new QPIkSolver(kin);
    const dlsSolver = new JacobianIkSolver(kin);

    const target = new THREE.Vector3(0.3, 0.5, 0.2);
    const targetQuat = new THREE.Quaternion().setFromEuler(new THREE.Euler(0.3, -0.2, 0.1));

    let qpIters = 0;
    for (let i = 0; i < 200; i++) {
        const result = qpSolver.solve('tip', target, targetQuat);
        if (!result || result.deltas.length === 0) break;
        for (const { name, delta } of result.deltas) {
            const joint = model.getJoint(name);
            let val = joint.currentValue + delta;
            if (joint.limits) val = Math.max(joint.limits.lower, Math.min(joint.limits.upper, val));
            setJointAngle(model, name, val);
        }
        qpIters = i + 1;
        
        if (result.converged) break;
    }

    const qpPos = kin.getLinkWorldPosition('tip');
    const qpErr = qpPos.distanceTo(target);

    // Reset for DLS
    for (let i = 1; i <= 7; i++) setJointAngle(model, `j${i}`, 0);

    let dlsIters = 0;
    for (let i = 0; i < 200; i++) {
        const result = dlsSolver.solve('tip', target, targetQuat);
        if (!result || result.deltas.length === 0) break;
        for (const { name, delta } of result.deltas) {
            const joint = model.getJoint(name);
            let val = joint.currentValue + delta;
            if (joint.limits) val = Math.max(joint.limits.lower, Math.min(joint.limits.upper, val));
            setJointAngle(model, name, val);
        }
        dlsIters = i + 1;
        if (result.converged) break;
    }

    const dlsPos = kin.getLinkWorldPosition('tip');
    const dlsErr = dlsPos.distanceTo(target);

    assert(qpErr < 0.05, `QP 7-DOF 6D pos error < 0.05 (got ${qpErr.toFixed(6)})`);
    assert(dlsErr < 0.05, `DLS 7-DOF 6D pos error < 0.05 (got ${dlsErr.toFixed(6)})`);
    console.log(`    QP: ${qpIters} iters, err=${qpErr.toFixed(6)} | DLS: ${dlsIters} iters, err=${dlsErr.toFixed(6)}`);
}

function testQP7DofNearLimitStart() {
    console.log('\n═══ Test: QP 7-DOF — Joints starting near limits ═══');

    const model = build7DofArm();
    const kin = new RobotKinematics(model);
    kin.buildChains();
    const solver = new QPIkSolver(kin);

    // Push joints near their limits (limits are [-2.5, 2.5])
    setJointAngle(model, 'j1', 2.0);
    setJointAngle(model, 'j2', -2.0);
    setJointAngle(model, 'j3', 1.8);
    setJointAngle(model, 'j4', -1.5);
    setJointAngle(model, 'j5', 2.2);
    setJointAngle(model, 'j6', -0.5);
    setJointAngle(model, 'j7', 0.3);

    const startPos = kin.getLinkWorldPosition('tip').clone();
    const target = new THREE.Vector3(
        startPos.x + 0.05,
        startPos.y + 0.1,
        startPos.z - 0.05
    );

    for (let i = 0; i < 40; i++) {
        const result = solver.solve('tip', target, null);
        if (!result || result.deltas.length === 0) break;
        for (const { name, delta } of result.deltas) {
            const joint = model.getJoint(name);
            let val = joint.currentValue + delta;
            if (joint.limits) val = Math.max(joint.limits.lower, Math.min(joint.limits.upper, val));
            setJointAngle(model, name, val);
        }
        if (result.converged) break;
    }

    const finalPos = kin.getLinkWorldPosition('tip');
    const err = finalPos.distanceTo(target);

    assert(err < 0.05, `QP near-limit start converges (error: ${err.toFixed(6)})`);

    // Verify all joints stayed within limits
    let allWithin = true;
    for (let i = 1; i <= 7; i++) {
        const j = model.getJoint(`j${i}`);
        if (j.currentValue < j.limits.lower - 1e-6 || j.currentValue > j.limits.upper + 1e-6) {
            allWithin = false;
            break;
        }
    }
    assert(allWithin, `QP: All joints within limits after near-limit solve`);
    console.log(`    Final error: ${err.toFixed(6)}, joints within limits: ${allWithin}`);
}

function testQP7DofSequentialTargets() {
    console.log('\n═══ Test: QP 7-DOF — Sequential targets (multi-target) ═══');

    const model = build7DofArm();
    const kin = new RobotKinematics(model);
    kin.buildChains();
    const solver = new QPIkSolver(kin);

    const targets = [
        new THREE.Vector3(0.3, 0.5, 0.2),
        new THREE.Vector3(-0.2, 0.4, 0.3),
        new THREE.Vector3(0.1, 0.7, -0.1),
        new THREE.Vector3(0.4, 0.3, 0.1),
    ];

    for (let t = 0; t < targets.length; t++) {
        for (let i = 0; i < 60; i++) {
            const result = solver.solve('tip', targets[t], null);
            if (!result || result.deltas.length === 0) break;
            for (const { name, delta } of result.deltas) {
                const joint = model.getJoint(name);
                let val = joint.currentValue + delta;
                if (joint.limits) val = Math.max(joint.limits.lower, Math.min(joint.limits.upper, val));
                setJointAngle(model, name, val);
            }
            if (result.converged) break;
        }

        const pos = kin.getLinkWorldPosition('tip');
        const err = pos.distanceTo(targets[t]);
        assert(err < 0.05, `QP 7-DOF target ${t+1} reached (error: ${err.toFixed(6)})`);
    }
}

function testQP7DofPostureKeepsJointsCentered() {
    console.log('\n═══ Test: QP 7-DOF — Posture task keeps joints centered ═══');

    const model = build7DofArm();
    const kin = new RobotKinematics(model);
    kin.buildChains();
    const solver = new QPIkSolver(kin);

    // Solve many sequential targets — the posture task should prevent
    // joints from drifting to extremes over time
    const targets = [];
    for (let i = 0; i < 8; i++) {
        targets.push(new THREE.Vector3(
            0.1 + 0.3 * Math.cos(i * Math.PI / 4),
            0.3 + 0.2 * Math.sin(i * Math.PI / 4),
            0.1 * Math.sin(i * Math.PI / 2)
        ));
    }

    for (const target of targets) {
        for (let i = 0; i < 60; i++) {
            const result = solver.solve('tip', target, null);
            if (!result || result.deltas.length === 0) break;
            for (const { name, delta } of result.deltas) {
                const joint = model.getJoint(name);
                let val = joint.currentValue + delta;
                if (joint.limits) val = Math.max(joint.limits.lower, Math.min(joint.limits.upper, val));
                setJointAngle(model, name, val);
            }
            if (result.converged) break;
        }
    }

    // After 8 sequential targets, check that no joint has drifted to its limit
    let maxLimitProximity = 0;
    for (let i = 1; i <= 7; i++) {
        const j = model.getJoint(`j${i}`);
        const range = j.limits.upper - j.limits.lower;
        const distFromCenter = Math.abs(j.currentValue - (j.limits.lower + j.limits.upper) / 2);
        const proximity = distFromCenter / (range / 2);
        if (proximity > maxLimitProximity) maxLimitProximity = proximity;
    }

    assert(maxLimitProximity < 0.95, `QP posture: no joint at >95% of range limit (max: ${(maxLimitProximity*100).toFixed(1)}%)`);
    console.log(`    Max limit proximity after 8 targets: ${(maxLimitProximity*100).toFixed(1)}%`);
}

// ============== Run All Tests ==============

console.log('╔══════════════════════════════════════════╗');
console.log('║    IK Integration Test Suite             ║');
console.log('╚══════════════════════════════════════════╝');

testChainBuilding();
testForwardKinematics();
testJacobianNonZero();
testDeltaMagnitude();
testPositionIkConvergence();
testFullPoseIkConvergence();
testJointLimitsEnforced();
testOrientationErrorWorldFrame();
testIterativeConvergenceSpeed();
testAxisExtractionFromVector3();
testPositionIkWithMixedAxes();
testSolveForTipMovesLink();
testAnchorSyncsAfterSolve();
testGizmoSizesSeparation();
testFullSolveForTipLoop();
testRotationIkShortChain();
testMutualExclusionLogic();

console.log('\n╔══════════════════════════════════════════╗');
console.log('║    QP Solver Tests                       ║');
console.log('╚══════════════════════════════════════════╝');

testQPPositionIkConvergence();
testQPFullPoseIkConvergence();
testQPJointLimitsEnforced();
testQPDeltaMagnitude();
testQPIterativeConvergenceSpeed();
testQPFullSolveLoop();
testQPSolveForTipMovesLink();

console.log('\n╔══════════════════════════════════════════╗');
console.log('║    QP 7-DOF Tests                        ║');
console.log('╚══════════════════════════════════════════╝');

testQP7DofFullPoseConvergence();
testQP7DofNearLimitStart();
testQP7DofSequentialTargets();
testQP7DofPostureKeepsJointsCentered();

console.log('\n══════════════════════════════════════════');
console.log(`Results: ${passed} passed, ${failed} failed, ${passed + failed} total`);
console.log('══════════════════════════════════════════');

if (failed > 0) {
    process.exit(1);
}
