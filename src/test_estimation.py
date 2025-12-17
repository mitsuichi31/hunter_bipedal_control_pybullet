"""
Unit tests for estimation components (StateFilter and ContactEstimator).
"""

import numpy as np

from estimation.state_filter import StateFilter
from estimation.contact_estimator import ContactEstimator, ContactEstimatorParams


def test_state_filter_base_lowpass():
    """Base position/velocity should low-pass toward inputs."""
    filt = StateFilter(base_alpha=0.5, joint_alpha=0.5)
    pos0 = np.array([0.0, 0.0, 0.0])
    vel0 = np.array([0.0, 0.0, 0.0])
    pos_f, vel_f = filt.filter_base(pos0, vel0)
    assert np.allclose(pos_f, pos0)
    assert np.allclose(vel_f, vel0)

    pos1 = np.array([1.0, 0.0, 0.0])
    vel1 = np.array([0.2, 0.0, 0.0])
    pos_f, vel_f = filt.filter_base(pos1, vel1)
    # With alpha=0.5, filtered should be midpoint of prev and new
    assert np.allclose(pos_f, 0.5 * (pos0 + pos1))
    assert np.allclose(vel_f, 0.5 * (vel0 + vel1))


def test_state_filter_joint_lowpass():
    """Joint filters should initialize on first value and smooth subsequent readings."""
    filt = StateFilter(base_alpha=0.2, joint_alpha=0.5)
    joints = {"leg_l1_joint": (0.0, 0.0)}
    first = filt.filter_joints(joints)
    assert np.allclose(first["leg_l1_joint"], (0.0, 0.0))

    joints2 = {"leg_l1_joint": (1.0, 2.0)}
    second = filt.filter_joints(joints2)
    # alpha=0.5 -> average of old and new
    assert np.allclose(second["leg_l1_joint"][0], 0.5)
    assert np.allclose(second["leg_l1_joint"][1], 1.0)


def test_contact_estimator_hysteresis():
    """Contact estimator should respect threshold and hysteresis."""
    params = ContactEstimatorParams(contact_threshold=10.0, release_threshold=5.0)
    est = ContactEstimator(params)
    est.reset(num_feet=2)

    # Below threshold: no contact
    state = est.update([0.0, 0.0])
    assert state == [False, False]

    # Exceed contact threshold
    state = est.update([12.0, 0.0])
    assert state == [True, False]

    # Drop below release threshold for foot 0: should release
    state = est.update([4.0, 0.0])
    assert state == [False, False]

    # Keep contact on second foot after crossing threshold
    state = est.update([0.0, 15.0])
    assert state == [False, True]

    # Stay in contact until below release threshold
    state = est.update([0.0, 6.0])
    assert state == [False, True]
    state = est.update([0.0, 4.0])
    assert state == [False, False]
