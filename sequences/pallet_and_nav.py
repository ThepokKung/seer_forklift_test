import yaml
import logging
import os
import time
from seer_forklift.api import SeerClient
from seer_forklift.tasks import PalletLiftTask, PathNavigationTask, ControlTask

class SequenceExecutor:
    def __init__(self, cfg_path="config/settings.yaml", nav_ip=None, nav_port=None):
        # Load configuration
        cfg = yaml.safe_load(open(cfg_path))
        self.status_cfg = cfg["seer"]
        self.nav_cfg = cfg["seer"]
        self.timeout = cfg["seer"]["timeout_sec"]

        # Override nav IP/Port if provided
        if nav_ip:
            self.nav_cfg["nav_ip"] = nav_ip
        if nav_port:
            self.nav_cfg["nav_port"] = nav_port

        # Ensure log directory exists
        log_path = cfg["logging"]["filepath"]
        log_dir = os.path.dirname(log_path)
        if log_dir and not os.path.exists(log_dir):
            os.makedirs(log_dir, exist_ok=True)
        logging.basicConfig(
            filename=log_path,
            level=cfg["logging"]["level"]
        )

    def run_pallet_then_nav(self, pallet_id: int, steps: list):
        # Execute PalletLift
        with SeerClient(
            self.status_cfg["status_ip"], self.status_cfg["status_port"], self.timeout
        ) as client:
            lift_task = PalletLiftTask(client, pallet_id)
            lift_task.run()
        # Then navigation
        return self.run_navigation_sequence(f"TASK{pallet_id}", steps)

    def run_navigation_sequence(self, task_id: str, steps: list):
        results = []
        # Open nav & status clients
        with SeerClient(
            self.nav_cfg["nav_ip"], self.nav_cfg["nav_port"], self.timeout
        ) as nav_client, SeerClient(
            self.status_cfg["status_ip"], self.status_cfg["status_port"], self.timeout
        ) as status_client:
            for src, dst in steps:
                task = PathNavigationTask(
                    nav_client,
                    status_client,
                    source_id=src,
                    target_id=dst,
                    task_id=task_id,
                    nav_api=self.nav_cfg["nav_api"],
                    status_api=self.status_cfg["status_api"],
                )
                res = task.run()
                results.append(res)
        return results

    def run_mixed_sequence(self, task_id: str, steps: list):
        """
        Execute mixed navigation and control steps.
        steps: list of dicts, each dict with keys:
          type: 'nav' or 'ctrl'
          source, target, (operation, end_height)
        """
        results = []
        with SeerClient(
            self.nav_cfg["nav_ip"], self.nav_cfg["nav_port"], self.timeout
        ) as nav_client, SeerClient(
            self.status_cfg["status_ip"], self.status_cfg["status_port"], self.timeout
        ) as status_client:
            for s in steps:
                if s.get("type") == "nav":
                    task = PathNavigationTask(
                        nav_client,
                        status_client,
                        source_id=s["source"],
                        target_id=s["target"],
                        task_id=task_id,
                        nav_api=self.nav_cfg["nav_api"],
                        status_api=self.status_cfg["status_api"],
                    )
                else:
                    task = ControlTask(
                        nav_client,
                        status_client,
                        source_id=s["source"],
                        target_id=s["target"],
                        task_id=task_id,
                        operation=s.get("operation"),
                        end_height=s.get("end_height"),
                        nav_api=self.nav_cfg["nav_api"],
                        status_api=self.status_cfg["status_api"],
                    )
                res = task.run()
                results.append(res)
        return results

    def run_batch_navigation(self, task_id: str, steps: list):
        """
        Send a batch move_task_list to API 3066 on nav port.
        steps: list of tuples (source_id, target_id)
        """
        # Build move_task_list payload
        move_list = [
            { "source_id": src, "id": dst, "task_id": task_id }
            for src, dst in steps
        ]
        payload = { "move_task_list": move_list }

        # Send batch nav and poll
        with SeerClient(
            self.nav_cfg["nav_ip"], self.nav_cfg["nav_port"], self.timeout
        ) as nav_client, SeerClient(
            self.status_cfg["status_ip"], self.status_cfg["status_port"], self.timeout
        ) as status_client:
            nav_client.send_request(1, 3066, payload)
            while True:
                stat = status_client.send_request(2, self.status_cfg["status_api"], {"simple": True})
                if stat.get("task_status") == 4:
                    break
                time.sleep(0.5)
        return stat