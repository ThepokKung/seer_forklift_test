from sequences.pallet_and_nav import SequenceExecutor

seq = SequenceExecutor("config/settings.yaml")

steps = [
    { "type":"ctrl", "source":"LM49","target":"LM51","operation":"ForkHeight","end_height":1.565 },
    { "type":"ctrl", "source":"LM51","target":"AP50","operation":"ForkUnload","end_height":1.5 },
    { "type":"ctrl", "source":"AP50","target":"LM51","operation":"ForkHeight","end_height":0.085 },
    { "type":"nav",  "source":"LM51","target":"LM49" },
]

results = seq.run_mixed_sequence("TASK123", steps)
print(results)
