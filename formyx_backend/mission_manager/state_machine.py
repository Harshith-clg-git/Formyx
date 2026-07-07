"""
formyx_backend/mission_manager/state_machine.py
------------------------------------------------
Deterministic Finite State Machine (FSM) that governs the entire
autonomous balloon-tracking mission lifecycle.

States
------
PRE_FLIGHT_CHECKS  → Validates GPS, battery, and MAVLink health.
ARMING             → Arms the vehicle motors.
TAKEOFF            → Commands and awaits the target altitude AGL.
NAVIGATING_TO_GPS  → Flies to an optional pre-programmed search origin.
SEARCHING          → Executes a search pattern until target is acquired.
TRACKING           → Actively follows the detected balloon target.
TARGET_LOST_RECOVERY → Predicted-path / visual-sweep reacquisition.
LANDING            → Lands at current position (mission complete).
RTL                → Returns to launch (safety-triggered or low battery).
EMERGENCY          → Motors off or hard failsafe (unrecoverable fault).
ABORTED            → Mission halted by operator.

Design decisions
----------------
* **Event-driven**: State transitions are triggered by discrete events
  (see ``MissionEvent``), not polled conditions.  This makes unit
  testing trivial and keeps the loop deterministic.
* **Transition table**: All valid (state, event) → next_state mappings
  live in one dict.  Any unlisted pair is a no-op (ignored with a
  warning), preventing accidental state corruption.
* **Entry / exit actions**: Each state can register ``on_enter`` and
  ``on_exit`` callbacks, keeping side-effects decoupled from logic.
* **Thread-safe**: ``post_event()`` can be called from any thread;
  events are queued and consumed by ``step()``.
* **Timer escalation**: Timer-based state escalations (e.g. taking
  too long to reach altitude) are handled via ``post_timed_event()``.
"""

from __future__ import annotations

import logging
import queue
import threading
import time
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Callable, Optional

log = logging.getLogger(__name__)


# ===========================================================================
# State & Event enumerations
# ===========================================================================

class MissionState(Enum):
    PRE_FLIGHT_CHECKS     = auto()
    ARMING                = auto()
    TAKEOFF               = auto()
    NAVIGATING_TO_GPS     = auto()
    SEARCHING             = auto()
    TRACKING              = auto()
    TARGET_LOST_RECOVERY  = auto()
    LANDING               = auto()
    RTL                   = auto()
    EMERGENCY             = auto()
    ABORTED               = auto()


class MissionEvent(Enum):
    # Pre-flight
    PREFLIGHT_PASS        = auto()   # All checks passed → arm
    PREFLIGHT_FAIL        = auto()   # Critical check failed → abort

    # Arming
    ARM_SUCCESS           = auto()   # Motors armed → takeoff
    ARM_FAILED            = auto()   # Arm rejected → abort

    # Takeoff
    ALTITUDE_REACHED      = auto()   # At target AGL → navigate / search
    TAKEOFF_TIMEOUT       = auto()   # Altitude not reached in time → RTL

    # Navigation
    WAYPOINT_REACHED      = auto()   # Arrived at search origin → search

    # Perception
    TARGET_DETECTED       = auto()   # Balloon seen → track
    TARGET_LOST           = auto()   # Balloon dropped from view → recovery
    TARGET_REACQUIRED     = auto()   # Balloon seen again → track
    RECOVERY_TIMEOUT      = auto()   # Recovery timed out → search

    # Safety
    BATTERY_WARNING       = auto()   # Low battery → log but continue
    BATTERY_CRITICAL      = auto()   # Critical battery → RTL
    GPS_DEGRADED          = auto()   # Satellite count dropped → loiter
    HEARTBEAT_LOST        = auto()   # MAVLink link dropped → emergency
    GEOFENCE_BREACH       = auto()   # Outside boundary → RTL

    # Termination
    LANDING_COMPLETE      = auto()   # Vehicle on ground → done
    MISSION_ABORT         = auto()   # Operator abort → RTL
    MISSION_COMPLETE      = auto()   # Operator declared mission done → land


# ===========================================================================
# Transition table
# (current_state, event) → next_state
# Any pair NOT in this table is silently ignored (logged at DEBUG level).
# ===========================================================================

TRANSITION_TABLE: dict[tuple[MissionState, MissionEvent], MissionState] = {

    # --- Pre-flight ---
    (MissionState.PRE_FLIGHT_CHECKS, MissionEvent.PREFLIGHT_PASS):
        MissionState.ARMING,
    (MissionState.PRE_FLIGHT_CHECKS, MissionEvent.PREFLIGHT_FAIL):
        MissionState.ABORTED,

    # --- Arming ---
    (MissionState.ARMING, MissionEvent.ARM_SUCCESS):
        MissionState.TAKEOFF,
    (MissionState.ARMING, MissionEvent.ARM_FAILED):
        MissionState.ABORTED,

    # --- Takeoff ---
    (MissionState.TAKEOFF, MissionEvent.ALTITUDE_REACHED):
        MissionState.NAVIGATING_TO_GPS,
    (MissionState.TAKEOFF, MissionEvent.TAKEOFF_TIMEOUT):
        MissionState.RTL,

    # --- Navigating to search origin ---
    (MissionState.NAVIGATING_TO_GPS, MissionEvent.WAYPOINT_REACHED):
        MissionState.SEARCHING,
    (MissionState.NAVIGATING_TO_GPS, MissionEvent.TARGET_DETECTED):
        MissionState.TRACKING,   # Opportunistic: spotted en-route

    # --- Searching ---
    (MissionState.SEARCHING, MissionEvent.TARGET_DETECTED):
        MissionState.TRACKING,

    # --- Tracking ---
    (MissionState.TRACKING, MissionEvent.TARGET_LOST):
        MissionState.TARGET_LOST_RECOVERY,
    (MissionState.TRACKING, MissionEvent.MISSION_COMPLETE):
        MissionState.LANDING,

    # --- Target lost recovery ---
    (MissionState.TARGET_LOST_RECOVERY, MissionEvent.TARGET_REACQUIRED):
        MissionState.TRACKING,
    (MissionState.TARGET_LOST_RECOVERY, MissionEvent.RECOVERY_TIMEOUT):
        MissionState.SEARCHING,   # Expand search if recovery fails

    # --- Safety escalations (from any flying state) ---
    (MissionState.NAVIGATING_TO_GPS, MissionEvent.BATTERY_CRITICAL):
        MissionState.RTL,
    (MissionState.SEARCHING,         MissionEvent.BATTERY_CRITICAL):
        MissionState.RTL,
    (MissionState.TRACKING,          MissionEvent.BATTERY_CRITICAL):
        MissionState.RTL,
    (MissionState.TARGET_LOST_RECOVERY, MissionEvent.BATTERY_CRITICAL):
        MissionState.RTL,

    (MissionState.NAVIGATING_TO_GPS, MissionEvent.GEOFENCE_BREACH):
        MissionState.RTL,
    (MissionState.SEARCHING,         MissionEvent.GEOFENCE_BREACH):
        MissionState.RTL,
    (MissionState.TRACKING,          MissionEvent.GEOFENCE_BREACH):
        MissionState.RTL,

    # Heartbeat loss → emergency from ANY flying state
    (MissionState.PRE_FLIGHT_CHECKS, MissionEvent.HEARTBEAT_LOST):
        MissionState.EMERGENCY,
    (MissionState.ARMING,            MissionEvent.HEARTBEAT_LOST):
        MissionState.EMERGENCY,
    (MissionState.TAKEOFF,           MissionEvent.HEARTBEAT_LOST):
        MissionState.EMERGENCY,
    (MissionState.NAVIGATING_TO_GPS, MissionEvent.HEARTBEAT_LOST):
        MissionState.EMERGENCY,
    (MissionState.SEARCHING,         MissionEvent.HEARTBEAT_LOST):
        MissionState.EMERGENCY,
    (MissionState.TRACKING,          MissionEvent.HEARTBEAT_LOST):
        MissionState.EMERGENCY,
    (MissionState.TARGET_LOST_RECOVERY, MissionEvent.HEARTBEAT_LOST):
        MissionState.EMERGENCY,

    # Operator abort from any active flying state
    (MissionState.TAKEOFF,           MissionEvent.MISSION_ABORT):
        MissionState.RTL,
    (MissionState.NAVIGATING_TO_GPS, MissionEvent.MISSION_ABORT):
        MissionState.RTL,
    (MissionState.SEARCHING,         MissionEvent.MISSION_ABORT):
        MissionState.RTL,
    (MissionState.TRACKING,          MissionEvent.MISSION_ABORT):
        MissionState.RTL,
    (MissionState.TARGET_LOST_RECOVERY, MissionEvent.MISSION_ABORT):
        MissionState.RTL,

    # --- RTL / Landing completion ---
    (MissionState.RTL,     MissionEvent.LANDING_COMPLETE):
        MissionState.ABORTED,
    (MissionState.LANDING, MissionEvent.LANDING_COMPLETE):
        MissionState.ABORTED,
}


# ===========================================================================
# Queued event
# ===========================================================================

@dataclass(order=True)
class _QueuedEvent:
    """Wraps a MissionEvent with an optional delivery delay."""
    deliver_at: float                           # monotonic timestamp
    event: MissionEvent = field(compare=False)
    source: str         = field(compare=False, default="unknown")


# ===========================================================================
# State Machine
# ===========================================================================

class MissionStateMachine:
    """
    Event-driven mission state machine.

    Parameters
    ----------
    initial_state :
        Starting state (default ``PRE_FLIGHT_CHECKS``).  Override in
        tests to start from a specific mid-mission state.
    on_state_change :
        Optional callback invoked *after* every successful state
        transition.  Signature: ``(prev: MissionState, new: MissionState) → None``.

    Thread Safety
    -------------
    ``post_event()`` and ``post_timed_event()`` are safe to call from
    any thread.  ``step()`` must be called from the main mission loop.
    """

    def __init__(
        self,
        initial_state: MissionState = MissionState.PRE_FLIGHT_CHECKS,
        on_state_change: Optional[Callable[[MissionState, MissionState], None]] = None,
    ) -> None:
        self._state = initial_state
        self._on_state_change = on_state_change
        self._event_queue: queue.PriorityQueue[_QueuedEvent] = queue.PriorityQueue()
        self._lock = threading.Lock()

        # Per-state entry/exit callbacks registry
        self._on_enter: dict[MissionState, list[Callable]] = {s: [] for s in MissionState}
        self._on_exit:  dict[MissionState, list[Callable]] = {s: [] for s in MissionState}

        # State entry timestamps (for dwell-time diagnostics)
        self._state_entered_at: float = time.monotonic()

        log.info("MissionStateMachine initialised in state: %s", self._state.name)

    # ------------------------------------------------------------------
    # Public API — state inspection
    # ------------------------------------------------------------------

    @property
    def state(self) -> MissionState:
        """Current mission state (thread-safe read)."""
        with self._lock:
            return self._state

    def is_flying(self) -> bool:
        """Return True if the drone is expected to be airborne."""
        flying_states = {
            MissionState.TAKEOFF,
            MissionState.NAVIGATING_TO_GPS,
            MissionState.SEARCHING,
            MissionState.TRACKING,
            MissionState.TARGET_LOST_RECOVERY,
            MissionState.RTL,
            MissionState.LANDING,
        }
        return self.state in flying_states

    def is_terminal(self) -> bool:
        """Return True if the FSM has reached a terminal (non-recoverable) state."""
        return self.state in {MissionState.ABORTED, MissionState.EMERGENCY}

    def state_dwell_s(self) -> float:
        """Seconds spent in the current state."""
        return time.monotonic() - self._state_entered_at

    # ------------------------------------------------------------------
    # Public API — event posting
    # ------------------------------------------------------------------

    def post_event(self, event: MissionEvent, source: str = "unknown") -> None:
        """
        Post an event for immediate processing on the next ``step()`` call.

        Parameters
        ----------
        event  : The event to post.
        source : Human-readable label for logging (e.g. ``"safety_monitor"``).
        """
        queued = _QueuedEvent(
            deliver_at=time.monotonic(),
            event=event,
            source=source,
        )
        self._event_queue.put(queued)
        log.debug("Event posted: %s (from %s)", event.name, source)

    def post_timed_event(
        self,
        event: MissionEvent,
        delay_s: float,
        source: str = "timer",
    ) -> threading.Timer:
        """
        Schedule an event to be posted after *delay_s* seconds.

        Returns the ``threading.Timer`` so callers can cancel it if the
        condition is resolved before the timer fires.

        Example
        -------
        >>> timer = sm.post_timed_event(MissionEvent.TAKEOFF_TIMEOUT, 30.0)
        >>> # Later, if altitude reached first:
        >>> timer.cancel()
        """
        def _fire():
            self.post_event(event, source=source)

        timer = threading.Timer(delay_s, _fire)
        timer.daemon = True
        timer.start()
        log.debug(
            "Timed event %s scheduled in %.1fs (from %s)",
            event.name, delay_s, source,
        )
        return timer

    # ------------------------------------------------------------------
    # Public API — step (call from main loop)
    # ------------------------------------------------------------------

    def step(self) -> Optional[MissionState]:
        """
        Process all pending events in the queue.

        Call this method once per main-loop iteration.  Returns the
        new state if a transition occurred, otherwise ``None``.

        Returns
        -------
        MissionState or None
            The *new* state after the last transition this tick, or
            ``None`` if no transitions happened.
        """
        last_new_state: Optional[MissionState] = None
        now = time.monotonic()

        while not self._event_queue.empty():
            try:
                queued = self._event_queue.get_nowait()
            except queue.Empty:
                break

            # Respect delivery delay
            if queued.deliver_at > now:
                self._event_queue.put(queued)  # put back, not ready yet
                break

            new_state = self._process_event(queued.event, queued.source)
            if new_state is not None:
                last_new_state = new_state

        return last_new_state

    # ------------------------------------------------------------------
    # Callback registration
    # ------------------------------------------------------------------

    def register_on_enter(
        self,
        state: MissionState,
        callback: Callable[[], None],
    ) -> None:
        """Register a callback invoked when entering *state*."""
        self._on_enter[state].append(callback)

    def register_on_exit(
        self,
        state: MissionState,
        callback: Callable[[], None],
    ) -> None:
        """Register a callback invoked when leaving *state*."""
        self._on_exit[state].append(callback)

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _process_event(
        self,
        event: MissionEvent,
        source: str,
    ) -> Optional[MissionState]:
        """
        Look up the transition, fire exit/enter callbacks, and update state.

        Returns the new state on transition, or ``None`` if the event is
        ignored for the current state.
        """
        with self._lock:
            current = self._state
            key = (current, event)
            next_state = TRANSITION_TABLE.get(key)

        if next_state is None:
            log.debug(
                "Ignored event %s in state %s (no transition defined).",
                event.name, current.name,
            )
            return None

        log.info(
            "TRANSITION: %s —[%s]→ %s  (source=%s, dwell=%.1fs)",
            current.name,
            event.name,
            next_state.name,
            source,
            self.state_dwell_s(),
        )

        # Fire exit callbacks for the current state
        for cb in self._on_exit[current]:
            try:
                cb()
            except Exception as exc:  # noqa: BLE001
                log.error("on_exit callback error for %s: %s", current.name, exc)

        # Update state
        with self._lock:
            self._state = next_state
            self._state_entered_at = time.monotonic()

        # Fire enter callbacks for the new state
        for cb in self._on_enter[next_state]:
            try:
                cb()
            except Exception as exc:  # noqa: BLE001
                log.error("on_enter callback error for %s: %s", next_state.name, exc)

        # Invoke global state-change listener
        if self._on_state_change:
            try:
                self._on_state_change(current, next_state)
            except Exception as exc:  # noqa: BLE001
                log.error("on_state_change callback error: %s", exc)

        return next_state

    def __repr__(self) -> str:
        return (
            f"MissionStateMachine(state={self._state.name}, "
            f"dwell={self.state_dwell_s():.1f}s)"
        )
