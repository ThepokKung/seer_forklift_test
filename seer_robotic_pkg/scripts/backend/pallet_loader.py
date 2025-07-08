import yaml
import sys
import os

class PalletLoader:
    def __init__(self, yaml_file):
        self.yaml_file = yaml_file
        self.pallet_data = self.load_pallet_data()

    def load_pallet_data(self):
        # Get the directory of the current script
        current_dir = os.path.dirname(os.path.abspath(__file__))
        # Navigate to the package root and then to the data directory
        yaml_path = os.path.join(current_dir, '..', '..', 'data', 'pallet.yaml')
        
        with open(yaml_path, 'r') as file:
            return yaml.safe_load(file)

    def get_pallet_info(self, pallet_id):
        for pallet in self.pallet_data.get('pallet', []):
            if pallet.get('pallet_id') == pallet_id:
                return pallet
        return None

    def select_pallet(self, pallet_id):
        """Select and return pallet information by ID"""
        pallet_info = self.get_pallet_info(pallet_id)
        if pallet_info:
            print(f"Selected pallet ID: {pallet_id}")
            return pallet_info
        else:
            print(f"Pallet ID {pallet_id} not found")
            return None