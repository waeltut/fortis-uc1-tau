"""Bounded downstream action lifecycle; independent of ROS for testing."""
import time


class MotionError(RuntimeError):
    def __init__(self, code, message, moveit_code=0):
        super().__init__(message)
        self.code = code
        self.moveit_code = moveit_code


class Canceled(MotionError):
    def __init__(self):
        super().__init__(6, 'Canceled; no further motion steps will start')


def wait_future(future, timeout, canceled=lambda: False):
    deadline = time.monotonic() + timeout
    while not future.done():
        if canceled():
            raise Canceled()
        if time.monotonic() >= deadline:
            raise MotionError(7, 'Response timed out')
        time.sleep(0.02)
    return future.result()


def run_action(client, goal, timeout, cancel_timeout, canceled, fault):
    """Never relinquish motion ownership without a downstream terminal result.

    If the endpoint disappears, latch a fault before releasing the public goal.
    Even a late goal acceptance is canceled, and new goals remain rejected.
    """
    send = client.send_goal_async(goal)
    try:
        # Do not abandon the acceptance future when cancellation arrives.
        handle = wait_future(send, cancel_timeout)
    except Exception:
        fault('Downstream goal acceptance unknown; inspect execution and restart')
        def cancel_late(future):
            try:
                late = future.result()
                if late and late.accepted:
                    late.cancel_goal_async()
            except Exception:
                pass
        send.add_done_callback(cancel_late)
        raise
    if not handle.accepted:
        raise MotionError(2, 'MoveIt rejected the downstream goal')
    try:
        result = handle.get_result_async()
    except Exception:
        fault('Could not obtain downstream result handle; execution state unknown')
        raise
    deadline = time.monotonic() + timeout
    cancel_sent = False
    timed_out = False
    stop_deadline = None
    while not result.done():
        now = time.monotonic()
        if not cancel_sent and (canceled() or now >= deadline):
            timed_out = now >= deadline
            try:
                handle.cancel_goal_async()
            except Exception:
                fault('Could not request cancellation; downstream state unknown')
                raise
            cancel_sent = True
            stop_deadline = now + cancel_timeout
        if cancel_sent and now >= stop_deadline:
            fault('No terminal result after cancellation; inspect execution and restart')
            raise MotionError(7, 'Cancellation not confirmed; commander locked')
        time.sleep(0.02)
    try:
        response = result.result()
    except Exception:
        fault('Downstream result unavailable; inspect execution and restart')
        raise
    if timed_out:
        raise MotionError(7, 'Motion timed out; downstream action has terminated')
    # Return successful completion even during a cancel race so per-arm result
    # flags can truthfully record completed execution. Parent still ends canceled.
    if canceled() and response.status != 4:
        raise Canceled()
    return response
