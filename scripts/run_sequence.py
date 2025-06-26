#!/usr/bin/env python
import argparse
import yaml
from sequences.pallet_and_nav import SequenceExecutor

def main():
    p = argparse.ArgumentParser(description="Run predefined sequences")
    p.add_argument(
        "--preset",
        required=True,
        help="Name of the preset from config/sequences.yaml (e.g., pick_pallet_23)"
    )
    args = p.parse_args()

    # Load presets from YAML
    with open("config/sequences.yaml") as f:
        presets = yaml.safe_load(f)

    if args.preset not in presets:
        print(f"Error: preset '{args.preset}' can not be found in config/sequences.yaml")
        exit(1)

    cfg = presets[args.preset]
    task_id = cfg["task_id"]
    steps   = cfg["steps"]

    # Run sequence
    seq = SequenceExecutor("config/settings.yaml")
    results = seq.run_mixed_sequence(task_id, steps)
    print("Results:", results)

if __name__ == "__main__":
    main()
