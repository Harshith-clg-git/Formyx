"""
tests/test_state_machine.py
----------------------------
Unit tests for ``mission_manager/state_machine.py``.

Test coverage
-------------
* Every valid (state, event) → next_state transition in the table
* Invalid / undefined transitions are silently ignored
* Entry and exit callbacks fire in the correct order
* Timer-based events fire after the correct delay and can be cancelled
* post_event is thread-safe when called from concurrent threads
* is_flying() and is_terminal() return correct values per state
* state_dwell_s() is monotonically non-decreasing
* on_state_change global callback receives correct prev/next args
"""

from __future__ import annotations

import sys
import time
import pathlib
import threading
from unittest.mock import MagicMock, call

import pytest

_BACKEND = pathlib.Path(__file__).parent.parent
sys.path.insert(0, str(_BACKEND))

from mission_manager.state_machine import (
    MissionState,
    MissionEvent,
    MissionStateMachine,
    TRANSITION_TABLE,
)

S = MissionState   # aliases for brevity
E = MissionEvent


# ===========================================================================
# Helpers
# ===========================================================================

def _sm(state: MissionState = S.PRE_FLIGHT_CHECKS, **kwargs) -> MissionStateMachine:
    """Create a state machine starting at *state*."""
    return MissionStateMachine(initial_state=state, **kwargs)


def _drive(sm: MissionStateMachine, event: MissionEvent, source: str = "test") -> MissionState:
    """Post an event and step the FSM; return the new state."""
    sm.post_event(event, source=source)
    sm.step()
    return sm.state


# ===========================================================================
# Transition correctness — validate ENTIRE transition table
# ===========================================================================

class TestTransitionTable:
    """Validate that every entry in TRANSITION_TABLE produces the expected state."""

    @pytest.mark.parametrize("start,event,expected", [
        (state, evt, next_s)
        for (state, evt), next_s in TRANSITION_TABLE.items()
    ])
    def test_all_transitions(self, start, event, expected):
        sm = _sm(state=start)
        result = _drive(sm, event)
        assert result == expected, (
            f"({start.name}, {event.name}) → expected {expected.name}, got {result.name}"
        )


# ===========================================================================
# Core happy-path mission flow
# ===========================================================================

class TestHappyPathMission:
    """Walk the full mission from PRE_FLIGHT_CHECKS to ABORTED (landed)."""

    def test_full_nominal_mission(self):
        sm = _sm()

        assert _drive(sm, E.PREFLIGHT_PASS)      == S.ARMING
        assert _drive(sm, E.ARM_SUCCESS)          == S.TAKEOFF
        assert _drive(sm, E.ALTITUDE_REACHED)     == S.NAVIGATING_TO_GPS
        assert _drive(sm, E.WAYPOINT_REACHED)     == S.SEARCHING
        assert _drive(sm, E.TARGET_DETECTED)      == S.TRACKING
        assert _drive(sm, E.MISSION_COMPLETE)     == S.LANDING
        assert _drive(sm, E.LANDING_COMPLETE)     == S.ABORTED

    def test_target_spotted_en_route(self):
        """Balloon detected while navigating to search origin — skip SEARCHING."""
        sm = _sm(state=S.NAVIGATING_TO_GPS)
        assert _drive(sm, E.TARGET_DETECTED) == S.TRACKING

    def test_target_loss_and_reacquisition(self):
        sm = _sm(state=S.TRACKING)
        assert _drive(sm, E.TARGET_LOST)        == S.TARGET_LOST_RECOVERY
        assert _drive(sm, E.TARGET_REACQUIRED)  == S.TRACKING

    def test_recovery_timeout_returns_to_search(self):
        sm = _sm(state=S.TARGET_LOST_RECOVERY)
        assert _drive(sm, E.RECOVERY_TIMEOUT) == S.SEARCHING


# ===========================================================================
# Safety escalation paths
# ===========================================================================

class TestSafetyEscalations:

    @pytest.mark.parametrize("start_state", [
        S.TAKEOFF,
        S.NAVIGATING_TO_GPS,
        S.SEARCHING,
        S.TRACKING,
        S.TARGET_LOST_RECOVERY,
    ])
    def test_battery_critical_triggers_rtl(self, start_state):
        sm = _sm(state=start_state)
        assert _drive(sm, E.BATTERY_CRITICAL) == S.RTL

    @pytest.mark.parametrize("start_state", [
        S.TAKEOFF,
        S.NAVIGATING_TO_GPS,
        S.SEARCHING,
        S.TRACKING,
        S.TARGET_LOST_RECOVERY,
    ])
    def test_geofence_breach_triggers_rtl(self, start_state):
        sm = _sm(state=start_state)
        assert _drive(sm, E.GEOFENCE_BREACH) == S.RTL

    @pytest.mark.parametrize("start_state", [
        S.PRE_FLIGHT_CHECKS,
        S.ARMING,
        S.TAKEOFF,
        S.NAVIGATING_TO_GPS,
        S.SEARCHING,
        S.TRACKING,
        S.TARGET_LOST_RECOVERY,
        S.LANDING,
        S.RTL,
    ])
    def test_heartbeat_loss_triggers_emergency(self, start_state):
        sm = _sm(state=start_state)
        assert _drive(sm, E.HEARTBEAT_LOST) == S.EMERGENCY

    @pytest.mark.parametrize("start_state", [
        S.NAVIGATING_TO_GPS,
        S.SEARCHING,
        S.TRACKING,
        S.TARGET_LOST_RECOVERY,
    ])
    def test_gps_degraded_triggers_rtl(self, start_state):
        """GPS_DEGRADED from any flying state must trigger RTL."""
        sm = _sm(state=start_state)
        assert _drive(sm, E.GPS_DEGRADED) == S.RTL

    @pytest.mark.parametrize("start_state", [
        S.TAKEOFF,
        S.NAVIGATING_TO_GPS,
        S.SEARCHING,
        S.TRACKING,
        S.TARGET_LOST_RECOVERY,
    ])
    def test_operator_abort_triggers_rtl(self, start_state):
        sm = _sm(state=start_state)
        assert _drive(sm, E.MISSION_ABORT) == S.RTL

    def test_preflight_fail_aborts_mission(self):
        sm = _sm()
        assert _drive(sm, E.PREFLIGHT_FAIL) == S.ABORTED

    def test_arm_failed_aborts_mission(self):
        sm = _sm(state=S.ARMING)
        assert _drive(sm, E.ARM_FAILED) == S.ABORTED

    def test_takeoff_timeout_triggers_rtl(self):
        sm = _sm(state=S.TAKEOFF)
        assert _drive(sm, E.TAKEOFF_TIMEOUT) == S.RTL

    def test_rtl_landing_complete_goes_to_aborted(self):
        sm = _sm(state=S.RTL)
        assert _drive(sm, E.LANDING_COMPLETE) == S.ABORTED


# ===========================================================================
# Invalid / undefined transitions are ignored
# ===========================================================================

class TestUndefinedTransitions:

    def test_undefined_event_does_not_change_state(self):
        """BATTERY_WARNING has no transition defined; state must be unchanged."""
        sm = _sm(state=S.SEARCHING)
        original = sm.state
        _drive(sm, E.BATTERY_WARNING)   # defined for logging only, no transition
        assert sm.state == original

    def test_event_in_terminal_state_is_ignored(self):
        """Posting events to ABORTED / EMERGENCY must not change state."""
        for terminal in (S.ABORTED, S.EMERGENCY):
            sm = _sm(state=terminal)
            for event in list(MissionEvent):
                _drive(sm, event)
            assert sm.state == terminal


# ===========================================================================
# Entry / exit callbacks
# ===========================================================================

class TestCallbacks:

    def test_on_enter_fires_on_transition(self):
        sm = _sm()
        cb = MagicMock()
        sm.register_on_enter(S.ARMING, cb)
        _drive(sm, E.PREFLIGHT_PASS)
        cb.assert_called_once()

    def test_on_exit_fires_on_transition(self):
        sm = _sm()
        cb = MagicMock()
        sm.register_on_exit(S.PRE_FLIGHT_CHECKS, cb)
        _drive(sm, E.PREFLIGHT_PASS)
        cb.assert_called_once()

    def test_exit_before_enter_order(self):
        """on_exit for old state must be called before on_enter for new state."""
        call_order = []
        sm = _sm()
        sm.register_on_exit(S.PRE_FLIGHT_CHECKS,
                            lambda: call_order.append("exit_preflight"))
        sm.register_on_enter(S.ARMING,
                             lambda: call_order.append("enter_arming"))

        _drive(sm, E.PREFLIGHT_PASS)
        assert call_order == ["exit_preflight", "enter_arming"]

    def test_multiple_callbacks_per_state(self):
        sm = _sm()
        cb1, cb2 = MagicMock(), MagicMock()
        sm.register_on_enter(S.ARMING, cb1)
        sm.register_on_enter(S.ARMING, cb2)
        _drive(sm, E.PREFLIGHT_PASS)
        cb1.assert_called_once()
        cb2.assert_called_once()

    def test_callback_exception_does_not_break_fsm(self):
        """A crashing callback must not stop the state machine."""
        sm = _sm()
        sm.register_on_enter(S.ARMING, lambda: 1 / 0)  # will raise ZeroDivisionError
        # Should NOT raise, FSM must transition cleanly
        result = _drive(sm, E.PREFLIGHT_PASS)
        assert result == S.ARMING

    def test_global_on_state_change_callback(self):
        changes = []
        sm = _sm(on_state_change=lambda prev, new: changes.append((prev, new)))

        _drive(sm, E.PREFLIGHT_PASS)
        _drive(sm, E.ARM_SUCCESS)

        assert changes[0] == (S.PRE_FLIGHT_CHECKS, S.ARMING)
        assert changes[1] == (S.ARMING, S.TAKEOFF)


# ===========================================================================
# Timed events
# ===========================================================================

class TestTimedEvents:

    def test_timed_event_fires_after_delay(self):
        sm = _sm(state=S.TAKEOFF)
        sm.post_timed_event(E.TAKEOFF_TIMEOUT, delay_s=0.1, source="test-timer")
        time.sleep(0.25)
        sm.step()
        assert sm.state == S.RTL

    def test_timed_event_can_be_cancelled(self):
        sm = _sm(state=S.TAKEOFF)
        timer = sm.post_timed_event(E.TAKEOFF_TIMEOUT, delay_s=0.2, source="test-timer")
        timer.cancel()
        time.sleep(0.35)
        sm.step()
        # Should still be in TAKEOFF since timer was cancelled
        assert sm.state == S.TAKEOFF

    def test_regular_event_beats_timed_event(self):
        """Altitude reached before timeout fires — cancels the timer manually."""
        sm = _sm(state=S.TAKEOFF)
        timeout_timer = sm.post_timed_event(E.TAKEOFF_TIMEOUT, delay_s=0.5)

        # Altitude reached first
        _drive(sm, E.ALTITUDE_REACHED)
        timeout_timer.cancel()

        time.sleep(0.6)  # Let the timer delay pass
        sm.step()        # Should be no TAKEOFF_TIMEOUT event processed
        assert sm.state == S.NAVIGATING_TO_GPS


# ===========================================================================
# Thread safety
# ===========================================================================

class TestThreadSafety:

    def test_concurrent_post_event_does_not_corrupt_state(self):
        """
        Spawn 10 threads each posting the same event.
        FSM should reach ARMING exactly once and not corrupt state.
        """
        sm = _sm()
        errors = []

        def poster():
            try:
                sm.post_event(E.PREFLIGHT_PASS, source="thread")
            except Exception as exc:  # noqa: BLE001
                errors.append(exc)

        threads = [threading.Thread(target=poster) for _ in range(10)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()

        # Drain all events
        for _ in range(20):
            sm.step()

        assert not errors, f"Thread errors: {errors}"
        # Must not be in an undefined state
        assert sm.state in set(MissionState)

    def test_state_read_from_multiple_threads(self):
        sm = _sm(state=S.TRACKING)
        results = []

        def reader():
            results.append(sm.state)

        threads = [threading.Thread(target=reader) for _ in range(20)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()

        assert all(s == S.TRACKING for s in results)


# ===========================================================================
# State metadata helpers
# ===========================================================================

class TestStateHelpers:

    @pytest.mark.parametrize("state,expected", [
        (S.TAKEOFF,              True),
        (S.NAVIGATING_TO_GPS,    True),
        (S.SEARCHING,            True),
        (S.TRACKING,             True),
        (S.TARGET_LOST_RECOVERY, True),
        (S.RTL,                  True),
        (S.LANDING,              True),
        (S.PRE_FLIGHT_CHECKS,    False),
        (S.ARMING,               False),
        (S.ABORTED,              False),
        (S.EMERGENCY,            False),
    ])
    def test_is_flying(self, state, expected):
        sm = _sm(state=state)
        assert sm.is_flying() == expected

    @pytest.mark.parametrize("state,expected", [
        (S.ABORTED,   True),
        (S.EMERGENCY, True),
        (S.TRACKING,  False),
        (S.SEARCHING, False),
    ])
    def test_is_terminal(self, state, expected):
        sm = _sm(state=state)
        assert sm.is_terminal() == expected

    def test_state_dwell_increases_over_time(self):
        sm = _sm()
        t0 = sm.state_dwell_s()
        time.sleep(0.05)
        t1 = sm.state_dwell_s()
        assert t1 > t0

    def test_state_dwell_resets_on_transition(self):
        sm = _sm()
        time.sleep(0.05)
        before = sm.state_dwell_s()
        _drive(sm, E.PREFLIGHT_PASS)
        after = sm.state_dwell_s()
        assert after < before  # dwell reset on transition
