"""Track a motion goal until terminal; never silently abandon late acceptance."""
import time


def run_action(client, goal, acceptance_timeout, result_timeout, cancel_timeout,
               stopped, fault, feedback=None):
    sent = client.send_goal_async(goal, feedback_callback=feedback)
    deadline = time.monotonic() + acceptance_timeout
    try:
        while not sent.done():
            if time.monotonic() >= deadline:
                raise TimeoutError('Action goal response timed out')
            time.sleep(0.02)
        handle = sent.result()
    except BaseException:
        fault('Goal acceptance unknown; inspect motion and restart mur_test')
        def cancel_late(future):
            try:
                handle = future.result()
                if handle is not None and handle.accepted:
                    handle.cancel_goal_async()
            except Exception as exc:
                fault('Late goal cancellation failed: %s' % exc)
        sent.add_done_callback(cancel_late)
        raise
    if handle is None or not handle.accepted:
        raise RuntimeError('Action goal rejected; check server readiness/busy state')
    try:
        result = handle.get_result_async()
    except Exception:
        handle.cancel_goal_async()
        fault('Action result unavailable; inspect motion and restart mur_test')
        raise
    deadline = time.monotonic() + result_timeout
    reason = None
    interrupt = None
    while not result.done():
        try:
            if reason is None:
                reason = stopped()
                if reason is None and time.monotonic() >= deadline:
                    reason = 'Motion timed out'
                if reason is not None:
                    handle.cancel_goal_async()
                    deadline = time.monotonic() + cancel_timeout
            elif time.monotonic() >= deadline:
                fault('No terminal result after cancellation; inspect motion and restart mur_test')
                raise TimeoutError('Cancellation not confirmed')
            time.sleep(0.02)
        except KeyboardInterrupt as exc:
            interrupt = exc
            if reason is None:
                reason = 'Interrupted'
                handle.cancel_goal_async()
                deadline = time.monotonic() + cancel_timeout
        except Exception:
            fault('Motion tracking failed; inspect motion and restart mur_test')
            raise
    try:
        response = result.result()
    except Exception:
        fault('Terminal result unavailable; inspect motion and restart mur_test')
        raise
    if interrupt is not None:
        raise interrupt
    if reason is not None:
        raise RuntimeError(reason + '; action terminated')
    return response
