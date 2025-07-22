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
        command = {
            "move_task_list": [
                ### Step 1 :Move to Pre Station
                {
                    "id": pallet_data["pre_station_id"],
                    "source_id": current_station_id,
                    "task_id": task_id,
                    "operation": "JackHeight",
                    "end_height": pallet_data["default_height"]
                },
                ### Step 2 : Move to Pallet Station
                {
                    "id": pallet_data["station_id"],
                    "source_id": pallet_data["pre_station_id"],
                    "task_id": task_id,
                    "operation": "JackLoad",
                    "end_height": pallet_data["pick_height"]
                },
                ### Step 3 : Move to Pre Station
                {
                    "id": pallet_data["pre_station_id"],
                    "source_id": pallet_data["station_id"],
                    "task_id": task_id,
                    "operation": "JackHeight",
                    "end_height": 0.125
                }
            ]
        }
        return str(command)
    
    def pallet_place_init_command(self,current_station_id,pallet_data,task_id):
        command = {
            "move_task_list": [
                ### Step 1 : Move to Pre Station
                {
                    "id": pallet_data["pre_station_id"],
                    "source_id": current_station_id,
                    "task_id": task_id,
                    "operation": "JackHeight",
                    "end_height": pallet_data["pick_height"]
                },
                ### Step 2 : Move to Pallet Station
                {
                    "id": pallet_data["station_id"],
                    "source_id": pallet_data["pre_station_id"],
                    "task_id": task_id,
                    "operation": "JackUnload",
                    "end_height": pallet_data["place_height"]
                },
                ### Step 3 : Move to Pre Station
                {
                    "id": pallet_data["pre_station_id"],
                    "source_id": pallet_data["station_id"],
                    "task_id": task_id,
                    "operation": "JackHeight",
                    "end_height": 0.125
                }
            ]
        }
        return str(command)

    def test_command(self, current_station_id, station_2go, task_id):
        command = {
            "move_task_list": [
                {
                    "id": station_2go,
                    "source_id": current_station_id,
                    "task_id": task_id
                }
            ]
        }
        return str(command)