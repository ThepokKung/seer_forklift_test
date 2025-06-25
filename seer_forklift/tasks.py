from abc import ABC, abstractmethod
import time
import logging
from typing import Optional, Dict, Any

class Task(ABC):
    def __init__(self):
        self.logger = logging.getLogger(self.__class__.__name__)

    @abstractmethod
    def run(self):
        pass

class PalletLiftTask(Task):
    MSG_TYPE_START = 1110
    MSG_TYPE_STATUS = 1111

    def __init__(self, client, pallet_id: int):
        super().__init__()
        self.client = client
        self.pallet_id = pallet_id

    def run(self):
        payload: Dict[str, Any] = {"task_ids": [f"PAL{self.pallet_id:05d}"]}
        self.logger.debug(f"Start lifting pallet {self.pallet_id}")
        self.client.send_request(1, self.MSG_TYPE_START, payload)
        while True:
            status = self.client.send_request(2, self.MSG_TYPE_STATUS, payload)
            self.logger.debug(f"Pallet status: {status}")
            if status.get("state") == "done":
                break
            time.sleep(0.2)
        return status

class PathNavigationTask(Task):
    """
    Navigate one step from source_id to target_id,
    using separate clients for nav-command and status-check.
    """
    def __init__(
        self,
        nav_client,
        status_client,
        source_id: str,
        target_id: str,
        task_id: str,
        nav_api: int,
        status_api: int,
    ):
        super().__init__()
        self.nav_client = nav_client
        self.status_client = status_client
        self.source_id = source_id
        self.target_id = target_id
        self.task_id = task_id
        self.nav_api = nav_api
        self.status_api = status_api

    def run(self):
        payload: Dict[str, Any] = {
            "source_id": self.source_id,
            "id": self.target_id,
            "task_id": self.task_id,
        }
        self.logger.debug(f"Starting navigation {self.source_id} → {self.target_id}")
        self.nav_client.send_request(1, self.nav_api, payload)
        while True:
            status = self.status_client.send_request(2, self.status_api, {"simple": True})
            ts = status.get("task_status")
            self.logger.debug(f"Nav status={ts}, finished_path={status.get('finished_path')}")
            if ts == 4:
                break
            time.sleep(0.2)
        return status

class ControlTask(Task):
    """
    Send Fork control commands (ForkHeight, ForkLoad, etc.)
    then poll status until completed.
    """
    def __init__(
        self,
        nav_client,
        status_client,
        source_id: str,
        target_id: str,
        task_id: str,
        operation: Optional[str] = None,
        end_height: Optional[float] = None,
        nav_api: int = 0,
        status_api: int = 0,
    ):
        super().__init__()
        self.nav_client = nav_client
        self.status_client = status_client
        self.source_id = source_id
        self.target_id = target_id
        self.task_id = task_id
        self.operation = operation or ""
        self.end_height = end_height if end_height is not None else 0.0
        self.nav_api = nav_api
        self.status_api = status_api

    def run(self):
        payload: Dict[str, Any] = {
            "source_id": self.source_id,
            "id": self.target_id,
            "task_id": self.task_id,
        }
        if self.operation:
            payload["operation"] = self.operation
        if self.end_height is not None:
            payload["end_height"] = self.end_height

        self.logger.debug(f"Control: {payload}")
        # send control command
        self.nav_client.send_request(1, self.nav_api, payload)

        # poll status until completed
        while True:
            stat = self.status_client.send_request(2, self.status_api, {"simple": True})
            ts = stat.get("task_status")
            self.logger.debug(f"Control status: {ts}")
            if ts == 4:
                break
            time.sleep(0.2)
        return stat
