from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace
import sys
import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from dual_arm_commander.lifecycle import run_action, MotionError, Canceled


def done(value):
    future = Future(); future.set_result(value); return future


class Handle:
    accepted = True
    def __init__(self, finished=False, cancel_finishes=True):
        self.result = Future()
        self.cancel_finishes = cancel_finishes
        self.cancels = 0
        if finished: self.result.set_result(SimpleNamespace(status=4))
    def get_result_async(self): return self.result
    def cancel_goal_async(self):
        self.cancels += 1
        if self.cancel_finishes and not self.result.done():
            self.result.set_result(SimpleNamespace(status=5))
        return done(None)


class Client:
    def __init__(self, handle): self.sent = done(handle)
    def send_goal_async(self, goal): return self.sent


def run(handle, canceled=lambda: False, faults=None):
    return run_action(Client(handle), None, 0.03, 0.03, canceled,
                      (faults if faults is not None else []).append)


def test_success(): assert run(Handle(finished=True)).status == 4


def test_cancel_waits_for_terminal():
    handle = Handle()
    with pytest.raises(Canceled): run(handle, lambda: True)
    assert handle.cancels == 1 and handle.result.done()


def test_timeout_cancels():
    handle = Handle(); faults = []
    with pytest.raises(MotionError) as error: run(handle, faults=faults)
    assert error.value.code == 7 and handle.cancels == 1 and not faults


def test_no_terminal_latches_fault():
    faults = []
    with pytest.raises(MotionError): run(Handle(cancel_finishes=False), lambda: True, faults)
    assert faults


def test_rejected_goal():
    handle = Handle(); handle.accepted = False
    with pytest.raises(MotionError) as error: run(handle)
    assert error.value.code == 2


def test_success_cancel_race_preserves_completion():
    assert run(Handle(finished=True), lambda: True).status == 4


def test_late_acceptance_is_canceled_and_faulted():
    client = Client(None); client.sent = Future(); faults = []
    with pytest.raises(MotionError):
        run_action(client, None, 0.03, 0.03, lambda: False, faults.append)
    handle = Handle()
    client.sent.set_result(handle)
    assert faults and handle.cancels == 1


def test_broken_result_faults():
    handle = Handle(); handle.result.set_exception(RuntimeError('lost result')); faults = []
    with pytest.raises(RuntimeError): run(handle, faults=faults)
    assert faults
