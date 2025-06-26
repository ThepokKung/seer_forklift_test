#!/usr/bin/env python
import argparse, yaml
from sequences.pallet_and_nav import SequenceExecutor

def main():
    p = argparse.ArgumentParser()
    p.add_argument("--preset", required=True,
                   help="ชื่อ preset ใน config/sequences.yaml")
    args = p.parse_args()

    presets = yaml.safe_load(open("config/sequences.yaml"))
    if args.preset not in presets:
        print(f"ERROR: preset '{args.preset}' ไม่พบ")
        exit(1)

    cfg = presets[args.preset]
    task_id  = cfg["task_id"]
    steps    = cfg["steps"]
    use_batch = cfg.get("batch_nav", False)

    executor = SequenceExecutor("config/settings.yaml")

    if use_batch:
        # สำหรับ batch navigation
        # เตรียม list เฉพาะ nav ก้าว
        nav_steps = [(s["source"], s["target"])
                     for s in steps if s["type"] == "nav"]
        result = executor.run_batch_navigation(task_id, nav_steps)
    else:
        # สำหรับ mixed (fork + nav ทีละก้าว)
        result = executor.run_mixed_sequence(task_id, steps)

    print("Result:", result)

if __name__ == "__main__":
    main()
