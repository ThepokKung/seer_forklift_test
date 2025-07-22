import json

class JsonCommandBuilder:
    def __init__(self):
        self.data = {
            "pallet_id": None,
            "station_id": None,
            "pre_station_id": None,
            "pallet_level": None,
            "pick_height": None,
            "place_height": None,
            "default_height": None
        }

    def pallet_pick_init_command(self,current_station_id,pallet_data,task_id):
        command = [
                ### Step 1 :Move to Pre Station
                {
                    "id": pallet_data["pre_station_id"],
                    "source_id": current_station_id,
                    "task_id": task_id,
                    "operation": "ForkHeight",
                    "end_height": pallet_data["default_height"]
                },
                ### Step 2 : Move to Pallet Station
                {
                    "id": pallet_data["station_id"],
                    "source_id": pallet_data["pre_station_id"],
                    "task_id": task_id,
                    "operation": "ForkLoad",
                    "end_height": pallet_data["pick_height"]
                },
                ### Step 3 : Move to Pre Station
                {
                    "id": pallet_data["pre_station_id"],
                    "source_id": pallet_data["station_id"],
                    "task_id": task_id,
                    "operation": "ForkHeight",
                    "end_height": 0.125
                }
        ]
        return command
    
    def pallet_place_init_command(self,current_station_id,pallet_data,task_id):
        command = [
                ### Step 1 : Move to Pre Station
                {
                    "id": pallet_data["pre_station_id"],
                    "source_id": current_station_id,
                    "task_id": task_id,
                    "operation": "ForkHeight",
                    "end_height": pallet_data["pick_height"]
                },
                ### Step 2 : Move to Pallet Station
                {
                    "id": pallet_data["station_id"],
                    "source_id": pallet_data["pre_station_id"],
                    "task_id": task_id,
                    "operation": "ForkUnload",
                    "end_height": pallet_data["place_height"]
                },
                ### Step 3 : Move to Pre Station
                {
                    "id": pallet_data["pre_station_id"],
                    "source_id": pallet_data["station_id"],
                    "task_id": task_id,
                    "operation": "ForkHeight",
                    "end_height": 0.125
                }
            ]
        
        return command
    
    def pallet_pick_to_manipulator_command(self, current_station_id, pallet_data, task_id):
        command = [
                ### Step 1 : Move to Pre Station
                {
                    "id": pallet_data["pre_station_id"],
                    "source_id": current_station_id,
                    "task_id": task_id,
                    "operation": "ForkHeight",
                    "end_height": pallet_data["default_height"]
                },
                ### Step 2 : Move to Pallet Station
                {
                    "id": pallet_data["station_id"],
                    "source_id": pallet_data["pre_station_id"],
                    "task_id": task_id,
                    "operation": "ForkLoad",
                    "end_height": pallet_data["pick_height"]
                }
        ]
        
        return command

    def test_command(self, current_station_id, station_2go, task_id):
        command = [
                ### Step 1 :Move to Pr  e Station
                {
                    "source_id": current_station_id,
                    "id": station_2go,
                    "task_id": task_id,
                },
                ### Step 2 : Move to Pallet Station
                {
                    "source_id": station_2go,
                    "id": current_station_id,
                    "task_id": task_id,
                },
                ### Step 3 : Move to Pre Station
                {
                    "source_id": current_station_id,
                    "id": station_2go,
                    "task_id": task_id,
                },
                ### Step 4 : Move to Pallet Station
                {
                    "source_id": station_2go,
                    "id": current_station_id,
                    "task_id": task_id,
                }
        ]
        return command