# setup.py
from setuptools import setup, find_packages

setup(
    name="seer_forklift_test",
    version="0.1",
    packages=find_packages(),        # จะรวมโฟลเดอร์ seer_forklift, tests
    install_requires=[
        "pyyaml",
    ],
)
