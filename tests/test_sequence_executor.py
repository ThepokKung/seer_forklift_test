import pytest
import sequences.pallet_and_nav as seqmod

class DummyCM:
    """DummyClient that supports 'with' and always returns task_status=4."""
    def __init__(self):
        self.calls = []

    def send_request(self, req_id, msg_type, payload=None):
        self.calls.append((req_id, msg_type, payload))
        return {"task_status": 4}

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        return False

@pytest.fixture(autouse=True)
def patch_seerclient(monkeypatch):
    # ทุกครั้งที่ SequenceExecutor สร้าง SeerClient จะได้ DummyCM แทน
    monkeypatch.setattr(seqmod, "SeerClient", lambda ip, port, timeout: DummyCM())
    yield

def test_sequence_executor_one_nav(tmp_path):
    # เขียน settings.yaml ชั่วคราว
    cfg = tmp_path / "settings.yaml"
    cfg.write_text(
        '''seer:
  status_ip: "127.0.0.1"
  status_port: 19204
  status_api: 1020
  nav_ip: "127.0.0.1"
  nav_port: 19206
  nav_api: 3051
  timeout_sec: 1

logging:
  level: "DEBUG"
  filepath: "logs/run.log"
'''
    )

    executor = seqmod.SequenceExecutor(str(cfg))
    result = executor.run_navigation_sequence("TASK123", [("LM49", "LM51")])
    # ควรได้ list หนึ่งสมาชิก และ task_status == 4
    assert isinstance(result, list)
    assert result and result[0]["task_status"] == 4
