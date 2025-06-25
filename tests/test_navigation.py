import pytest
from seer_forklift.tasks import PathNavigationTask

class DummyNavClient:
    def __init__(self):
        self.calls = []
    def send_request(self, req_id, msg_type, payload=None):
        self.calls.append((req_id, msg_type, payload))
        # nav_api always returns empty dict
        return {}

class DummyStatusClient:
    def __init__(self):
        self.calls = []
    def send_request(self, req_id, msg_type, payload=None):
        self.calls.append((req_id, msg_type, payload))
        # return completed on any check
        return {"task_status": 4}


def test_nav_sequence():
    nav = DummyNavClient()
    stat = DummyStatusClient()
    task = PathNavigationTask(
        nav_client=nav,
        status_client=stat,
        source_id="LM1",
        target_id="LM2",
        task_id="T123",
        nav_api=3051,
        status_api=1020,
    )
    status = task.run()
    assert status["task_status"] == 4
    assert any(c[1] == 3051 for c in nav.calls)  # nav command
    assert any(c[1] == 1020 for c in stat.calls) # status check