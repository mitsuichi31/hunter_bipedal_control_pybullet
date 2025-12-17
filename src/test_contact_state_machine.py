"""
Unit tests for Contact State Machine

Tests:
1. State transitions occur at correct times
2. Contact flags match expected phase
3. Phase timing is consistent
4. Contact detection works with PyBullet
5. Swing phase progress calculation
"""

import os
import sys
import numpy as np
import pybullet as p
import time

from robot_constants import BASE_HEIGHT, standing_config_copy
from contact_state_machine import ContactStateMachine, ContactStateParams, ContactPhase
from simulation_env import HunterSimulation


def test_state_machine_timing():
    """Test that state transitions occur at correct times"""
    print("\n" + "="*60)
    print("TEST 1: State Machine Timing")
    print("="*60)

    params = ContactStateParams(
        step_period=1.0,
        double_support_ratio=0.3
    )
    fsm = ContactStateMachine(params)

    # Expected durations
    ds_duration = 0.3
    swing_duration = 0.7
    full_cycle = 2.0  # DS → LS → DS → RS → DS

    print(f"Expected DS duration: {ds_duration:.3f}s")
    print(f"Expected Swing duration: {swing_duration:.3f}s")
    print(f"Expected Full cycle: {full_cycle:.3f}s")

    # Simulate and record transitions
    dt = 0.001
    t = 0.0
    transitions = []
    last_phase = fsm.phase

    while t < full_cycle + 0.5:
        phase = fsm.update(dt)

        if phase != last_phase:
            transitions.append((t, phase))
            last_phase = phase

        t += dt

    print(f"\nRecorded {len(transitions)} transitions:")
    for trans_time, phase in transitions:
        print(f"  t={trans_time:.3f}s → {phase.name}")

    # Validate timing
    assert len(transitions) >= 4, f"Expected at least 4 transitions, got {len(transitions)}"

    # Check first transition (DS → LS) occurs near 0.3s
    assert abs(transitions[0][0] - 0.3) < 0.01, f"First transition at {transitions[0][0]:.3f}s, expected 0.3s"
    assert transitions[0][1] == ContactPhase.LEFT_SWING

    # Check second transition (LS → DS) occurs near 1.0s
    assert abs(transitions[1][0] - 1.0) < 0.01, f"Second transition at {transitions[1][0]:.3f}s, expected 1.0s"
    assert transitions[1][1] == ContactPhase.DOUBLE_SUPPORT

    print("\n✓ State machine timing is correct!")


def test_contact_flags():
    """Test that contact flags match expected phase"""
    print("\n" + "="*60)
    print("TEST 2: Contact Flags")
    print("="*60)

    params = ContactStateParams(step_period=1.0, double_support_ratio=0.3)
    fsm = ContactStateMachine(params)

    test_cases = [
        (ContactPhase.DOUBLE_SUPPORT, (True, True)),
        (ContactPhase.LEFT_SWING, (False, True)),
        (ContactPhase.RIGHT_SWING, (True, False)),
    ]

    for phase, expected_contacts in test_cases:
        fsm.phase = phase
        left, right = fsm.get_contact_state()
        print(f"Phase: {phase.name:20s} → L:{left}, R:{right}")
        assert (left, right) == expected_contacts, f"Contact mismatch for {phase.name}"

    print("\n✓ Contact flags are correct!")


def test_swing_phase_progress():
    """Test swing phase progress calculation"""
    print("\n" + "="*60)
    print("TEST 3: Swing Phase Progress")
    print("="*60)

    params = ContactStateParams(step_period=1.0, double_support_ratio=0.3)
    fsm = ContactStateMachine(params)

    # Move to left swing phase
    fsm.phase = ContactPhase.LEFT_SWING
    fsm.phase_time = 0.0

    # Test progress at different times
    test_times = [0.0, 0.175, 0.35, 0.525, 0.7]
    expected_progress = [0.0, 0.25, 0.5, 0.75, 1.0]

    for t, expected in zip(test_times, expected_progress):
        fsm.phase_time = t
        progress = fsm.get_swing_phase_progress()
        print(f"Time: {t:.3f}s → Progress: {progress:.2f} (expected: {expected:.2f})")
        assert abs(progress - expected) < 0.01, f"Progress mismatch at t={t}"

    print("\n✓ Swing phase progress is correct!")


def test_contact_detection_with_pybullet():
    """Test contact detection with actual PyBullet simulation"""
    print("\n" + "="*60)
    print("TEST 4: Contact Detection with PyBullet")
    print("="*60)

    # Get URDF path
    script_dir = os.path.dirname(os.path.abspath(__file__))
    urdf_path = os.path.join(script_dir, "../models/urdf/hunter.urdf")

    # Create simulation (headless)
    sim = HunterSimulation(urdf_path=urdf_path, dt=0.001, use_gui=False)
    sim.connect()
    sim.load_robot(start_position=[0, 0, BASE_HEIGHT])

    # Set robot to standing configuration
    standing_config = standing_config_copy()
    sim.reset_robot(position=[0, 0, BASE_HEIGHT], joint_positions=standing_config)

    # Initialize contact state machine
    params = ContactStateParams(
        step_period=1.0,
        double_support_ratio=0.3,
        contact_force_threshold=5.0
    )
    fsm = ContactStateMachine(params)
    fsm.initialize(sim.robot_id, sim.joint_dict)

    print(f"Left foot link: {fsm.left_foot_link}")
    print(f"Right foot link: {fsm.right_foot_link}")

    # Let robot settle (enable contact stabilization)
    for _ in range(500):
        p.stepSimulation()

    # Check contact detection with robot standing
    left_contact, left_force = fsm.detect_ground_contact(sim.robot_id, fsm.left_foot_link)
    right_contact, right_force = fsm.detect_ground_contact(sim.robot_id, fsm.right_foot_link)

    print(f"\nStanding contact state:")
    print(f"  Left foot:  contact={left_contact}, force={left_force:.2f}N")
    print(f"  Right foot: contact={right_contact}, force={right_force:.2f}N")

    # Both feet should be in contact when standing
    assert left_contact, "Left foot should be in contact"
    assert right_contact, "Right foot should be in contact"
    assert left_force > 10.0, f"Left foot force too low: {left_force:.2f}N"
    assert right_force > 10.0, f"Right foot force too low: {right_force:.2f}N"

    # Test get_actual_contact_state()
    actual_left, actual_right = fsm.get_actual_contact_state()
    print(f"\nActual contact state: L:{actual_left}, R:{actual_right}")
    assert actual_left and actual_right, "Both feet should be detected in contact"

    sim.disconnect()
    print("\n✓ Contact detection works with PyBullet!")


def test_reset():
    """Test state machine reset"""
    print("\n" + "="*60)
    print("TEST 5: Reset Functionality")
    print("="*60)

    params = ContactStateParams(step_period=1.0, double_support_ratio=0.3)
    fsm = ContactStateMachine(params)

    # Advance state machine
    for _ in range(100):
        fsm.update(0.01)

    print(f"Before reset: phase={fsm.phase.name}, time={fsm.phase_time:.3f}s, steps={fsm.step_count}")

    # Reset
    fsm.reset()

    print(f"After reset:  phase={fsm.phase.name}, time={fsm.phase_time:.3f}s, steps={fsm.step_count}")

    # Verify reset state
    assert fsm.phase == ContactPhase.DOUBLE_SUPPORT, "Phase should be DOUBLE_SUPPORT"
    assert fsm.phase_time == 0.0, "Phase time should be 0"
    assert fsm.cycle_time == 0.0, "Cycle time should be 0"
    assert fsm.step_count == 0, "Step count should be 0"

    print("\n✓ Reset works correctly!")


def test_transition_detection():
    """Test is_in_transition() method"""
    print("\n" + "="*60)
    print("TEST 6: Transition Detection")
    print("="*60)

    params = ContactStateParams(step_period=1.0, double_support_ratio=0.3)
    fsm = ContactStateMachine(params)

    transition_duration = 0.05

    # Test at start of phase (not in transition)
    fsm.phase = ContactPhase.DOUBLE_SUPPORT
    fsm.phase_time = 0.0
    in_trans = fsm.is_in_transition(transition_duration)
    print(f"Start of DS phase: in_transition={in_trans} (expected: False)")
    assert not in_trans, "Should not be in transition at start"

    # Test near end of phase (in transition)
    fsm.phase_time = fsm.ds_duration - 0.03  # 0.03s before end
    in_trans = fsm.is_in_transition(transition_duration)
    print(f"Near end of DS phase: in_transition={in_trans} (expected: True)")
    assert in_trans, "Should be in transition near end"

    print("\n✓ Transition detection works!")


def run_all_tests():
    """Run all tests"""
    print("\n" + "="*70)
    print("CONTACT STATE MACHINE UNIT TESTS")
    print("="*70)

    tests = [
        ("State Machine Timing", test_state_machine_timing),
        ("Contact Flags", test_contact_flags),
        ("Swing Phase Progress", test_swing_phase_progress),
        ("Contact Detection with PyBullet", test_contact_detection_with_pybullet),
        ("Reset Functionality", test_reset),
        ("Transition Detection", test_transition_detection),
    ]

    passed = 0
    failed = 0

    for test_name, test_func in tests:
        try:
            if test_func():
                passed += 1
        except AssertionError as e:
            print(f"\n✗ TEST FAILED: {test_name}")
            print(f"  Error: {e}")
            failed += 1
        except Exception as e:
            print(f"\n✗ TEST ERROR: {test_name}")
            print(f"  Exception: {e}")
            failed += 1

    print("\n" + "="*70)
    print(f"TEST SUMMARY: {passed} passed, {failed} failed out of {len(tests)} tests")
    print("="*70)

    if failed == 0:
        print("\n✓ ALL TESTS PASSED!")
        return True
    else:
        print(f"\n✗ {failed} TEST(S) FAILED!")
        return False


if __name__ == "__main__":
    success = run_all_tests()
    sys.exit(0 if success else 1)
