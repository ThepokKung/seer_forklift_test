import argparse
from sequences.pallet_and_nav import SequenceExecutor

if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--mode", choices=["pallet","nav","all","one_nav"], default="all")
    p.add_argument("--pallet_id", type=int, default=26)
    p.add_argument("--source", type=str, help="source_id สำหรับ one_nav")
    p.add_argument("--target", type=str, help="target_id สำหรับ one_nav")
    args = p.parse_args()

    seq = SequenceExecutor("config/settings.yaml")
    steps = [("LM49","LM51"), ("LM51","AP50"), ("AP50","LM51"), ("LM51","LM49")]

    if args.mode == "pallet":
        # ยก Pallet เท่านั้น
        results = seq.run_pallet_then_nav(args.pallet_id, [])
    elif args.mode == "nav":
        # Navigation sequence เต็ม
        results = seq.run_navigation_sequence("TASK123", steps)
    elif args.mode == "one_nav":
        # navigation ก้าวเดียว
        if not (args.source and args.target):
            p.error("--mode one_nav ต้องระบุ --source และ --target")
        results = seq.run_navigation_sequence("TASK123", [(args.source, args.target)])
    else:
        # Default: รันทั้ง Pallet + Navigation
        results = seq.run_pallet_then_nav(args.pallet_id, steps)

    print(results)