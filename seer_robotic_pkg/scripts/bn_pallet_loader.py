import yaml
import sys
import os

class PalletLoader:
    def __init__(self, yaml_file):
        self.yaml_file = yaml_file
        self.pallet_data = self.load_pallet_data()

    def load_pallet_data(self):
        # Try multiple possible locations for the yaml file
        current_dir = os.path.dirname(os.path.abspath(__file__))
        
        # Possible paths to check
        possible_paths = [
            # Absolute path if provided
            self.yaml_file if os.path.isabs(self.yaml_file) else None,
            # Relative to current directory
            os.path.join(current_dir, self.yaml_file),
            # Standard package data directory (source)
            os.path.join(current_dir, '..', '..', 'data', self.yaml_file),
            # Standard package data directory (install)
            os.path.join(current_dir, '..', '..', '..', 'share', 'seer_robotic_pkg', 'data', self.yaml_file),
            # Fallback locations
            os.path.join(current_dir, '..', 'data', self.yaml_file),
            os.path.join(current_dir, 'data', self.yaml_file),
        ]
        
        # Remove None values
        possible_paths = [path for path in possible_paths if path is not None]
        
        # Try each path until we find the file
        for yaml_path in possible_paths:
            if os.path.exists(yaml_path):
                print(f"Loading pallet data from: {yaml_path}")
                try:
                    with open(yaml_path, 'r') as file:
                        return yaml.safe_load(file)
                except Exception as e:
                    print(f"Error loading {yaml_path}: {e}")
                    continue
        return None
        
    def get_pallet_info(self, pallet_id):
        """Get pallet information by ID, handling different yaml formats"""
        if self.pallet_data is None:
            print(f"Warning: No pallet data loaded, cannot find pallet {pallet_id}")
            return None
            
        # Convert pallet_id to string and int for comparison
        pallet_id_str = str(pallet_id)
        try:
            pallet_id_int = int(pallet_id)
        except (ValueError, TypeError):
            pallet_id_int = None
        
        # Search in pallet data
        for pallet in self.pallet_data.get('pallet', []):
            # Check different possible ID formats
            stored_id = pallet.get('pallet_id')
            if stored_id is not None:
                stored_id_str = str(stored_id)
                # Try exact match (string), numeric match, or ID format match
                if (stored_id == pallet_id or 
                    stored_id_str == pallet_id_str or
                    (pallet_id_int is not None and stored_id == pallet_id_int)):
                    return pallet
        return None

    def get_pallet_level_info(self, level):
        if self.pallet_data is None:
            print(f"Warning: No pallet data loaded, cannot find pallet level {level}")
            return None
            
        for pallet_level in self.pallet_data.get('pallet_level', []):
            if pallet_level.get('pallet_level') == level:  # Fixed: use 'pallet_level' instead of 'level'
                return pallet_level
        return None

    def is_data_loaded(self):
        """Check if pallet data was successfully loaded"""
        return self.pallet_data is not None

    def get_all_pallets(self):
        """Get all pallet information"""
        if self.pallet_data is None:
            print("Warning: No pallet data loaded")
            return []
        return self.pallet_data.get('pallet', [])

    def get_all_pallet_levels(self):
        """Get all pallet level information"""
        if self.pallet_data is None:
            print("Warning: No pallet data loaded")
            return []
        return self.pallet_data.get('pallet_level', [])

    def get_pick_height(self, level):
        """Get pick height for a specific pallet level"""
        level_info = self.get_pallet_level_info(level)
        if level_info:
            return level_info.get('pick_height', 0.0)
        return 0.0

    def get_place_height(self, level):
        """Get place height for a specific pallet level"""
        level_info = self.get_pallet_level_info(level)
        if level_info:
            return level_info.get('place_height', 0.0)
        return 0.0

    def get_default_height(self, level):
        """Get default height for a specific pallet level"""
        level_info = self.get_pallet_level_info(level)
        if level_info:
            return level_info.get('default_height', 0.0)
        return 0.0

    def get_pallet_heights(self, pallet_id):
        """Get all heights for a specific pallet based on its level"""
        pallet_info = self.get_pallet_info(pallet_id)
        if not pallet_info:
            return None
        
        pallet_level = pallet_info.get('pallet_level')
        if pallet_level is None:
            return None
        
        level_info = self.get_pallet_level_info(pallet_level)
        if level_info:
            return {
                'pallet_id': pallet_id,
                'pallet_level': pallet_level,
                'pick_height': level_info.get('pick_height', 0.0),
                'place_height': level_info.get('place_height', 0.0),
                'default_height': level_info.get('default_height', 0.0)
            }
        return None