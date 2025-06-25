#!/usr/bin/env python
from sequences.pallet_and_nav import SequenceExecutor

if __name__=="__main__":
    seq = SequenceExecutor("config/settings.yaml")
    steps = [("LM49","LM51"), ("LM51","LM49")]
    res = seq.run_navigation_sequence("TASK_SAFE", steps)
    print(res)