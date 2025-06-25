import pytest
from seer_forklift.tasks import PalletLiftTask

class DummyClient:
    def __init__(self):
        self.calls = []

    def send_request(self, req_id, msg_type, payload=None):
        self.calls.append((req_id, msg_type, payload))
        if msg_type == PalletLiftTask.MSG_TYPE_START:
            return {"state": "started"}
        return {"state": "done"}


def test_pallet_lift_sequence():
    client = DummyClient()
    task = PalletLiftTask(client, pallet_id=26)
    status = task.run()
    assert status["state"] == "done"
    assert any(c[1] == PalletLiftTask.MSG_TYPE_START for c in client.calls)
    assert any(c[1] == PalletLiftTask.MSG_TYPE_STATUS for c in client.calls)