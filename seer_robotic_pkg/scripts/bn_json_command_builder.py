class JsonCommandBuilder:
    def __init__(self):
        self.data = {
            "pallet_id": None,
            "station_id": None,
            "pre_station_id": None,
            "pallet_level": None,
            "pick_height": None,
            "place_height": None,
            "default_height": None,
            "running_height": None,
            "down_height": None
        }

    #####################################################
    ###                Pick Place Init                ###
    #####################################################

    def pallet_pick_init_command(self,current_station_id,pallet_data,task_id):
        command = [
                ### Step 1 : Move robot from current station to Pre pallet Station
                {
                    "source_id": current_station_id,
                    "id": pallet_data["pre_station_id"],
                    "task_id": task_id,
                    "operation": "ForkHeight",
                    "end_height": pallet_data["default_height"]
                },
                ### Step 2 : Move robot from Pre pallet Station to Pallet Station
                {
                    "source_id": pallet_data["pre_station_id"],
                    "id": pallet_data["station_id"],
                    "task_id": task_id,
                    "operation": "ForkLoad",
                    "end_height": pallet_data["pick_height"]
                },
                ### Step 3 :Move robot from Pallet Station to Pre pallet Station (For register pallet)
                {
                    "source_id": pallet_data["station_id"],
                    "id": pallet_data["pre_station_id"],
                    "task_id": task_id,
                    "operation": "ForkHeight",
                    "end_height": pallet_data["down_height"] # Down pallet to ground
                }
        ]
        return command
    
    def pallet_place_init_command(self,current_station_id,pallet_data,task_id):
        command = [
                ### Step 1 : Rise Fork to pick height from current station
                {
                    "id": "SELF_POSITION",
                    "source_id": "SELF_POSITION",
                    "task_id": task_id,
                    "operation": "ForkHeight",
                    "end_height": pallet_data["pick_height"]
                },
                ### Step 2 : Move robot from pre pallet station to Pallet Station
                {
                    "id": pallet_data["station_id"],
                    "source_id": pallet_data["pre_station_id"],
                    "task_id": task_id,
                    "operation": "ForkUnload",
                    "end_height": pallet_data["place_height"]
                },
                ### Step 3 : Move robot from Pallet Station to Pre pallet Station
                {
                    "id": pallet_data["pre_station_id"],
                    "source_id": pallet_data["station_id"],
                    "task_id": task_id,
                    "operation": "ForkHeight",
                    "end_height": pallet_data["running_height"]
                }
            ]
        return command
    
    #####################################################
    ###              Pick Place Manipulator           ###
    #####################################################
    
    def pallet_pick_to_manipulator_command(self, current_station_id, pallet_data, task_id):
        command = [
                ### Step 1 :Move to Pre Station
                {
                    "source_id": current_station_id,
                    "id": pallet_data["pre_station_id"],
                    "task_id": task_id,
                    "operation": "ForkHeight",
                    "end_height": pallet_data["default_height"]
                },
                ### Step 2 : Move to Pallet Station
                {
                    "source_id": pallet_data["pre_station_id"],
                    "id": pallet_data["station_id"],
                    "task_id": task_id,
                    "operation": "ForkLoad",
                    "end_height": pallet_data["pick_height"]
                },
                ### Step 3 : Move to Pre Station and down height to running height
                {
                    "source_id": pallet_data["station_id"],
                    "id": pallet_data["pre_station_id"],
                    "task_id": task_id,
                    "operation": "ForkHeight",
                    "end_height": pallet_data["running_height"] # Test
                },
                ### Step 4 : Move to Pre Manipulator Station
                {
                    "source_id": pallet_data["pre_station_id"],
                    "id": "LM22", #Pre Manipulator Station
                    "task_id": task_id
                    ### It's still in fork running height
                    # "operation": "ForkHeight",
                    # "end_height": pallet_data["default_height"]
                },
                ### Step 5 : Move to Manipulator Station
                {
                    "source_id": "LM22", #Pre Manipulator Station
                    "id": "AP1", #Pre Manipulator Station
                    "task_id": task_id,
                    "operation": "ForkUnload",
                    "end_height": pallet_data["down_height"] # Down pallet to ground
                },
                ### Step 6 : Move to Pre Manipulator Station
                {
                    "source_id": "AP1", #Pre Manipulator Station
                    "id": "LM22", #Pre Manipulator Station
                    "task_id": task_id,
                    "operation": "ForkHeight",
                    "end_height": pallet_data["running_height"] # Fork back to default height
                }
        ]
        
        return command
    
    def pallet_pick_from_manipulator_command(self, current_station_id, pallet_data, task_id):
        command = [
                ### Step 1 :Move robot from Pre manipulation station to mainipulator station
                {
                    "source_id": "LM22", # Pre Manipulation Station
                    "id": "AP1", # Manipulator Station
                    "task_id": task_id,
                    "operation": "ForkLoad",
                    "end_height": pallet_data["running_height"]
                },
                ### Step 2 : Move robot from manipulator station to pre manipulation station
                {
                    "source_id": "AP1", # Manipulator Station
                    "id": "LM22", # Pre Manipulation Station
                    "task_id": task_id,
                    ### it's still in fork running height
                    # "operation": "ForkLoad",
                    # "end_height": pallet_data["pick_height"]
                },
                ### Step 3 : Move to Pre manipulator Station to pre pallet station
                {
                    "source_id": "LM22", # Pre Manipulation Station
                    "id": pallet_data["pre_station_id"], # Pre Pallet Station
                    "task_id": task_id,
                    "operation": "ForkHeight",
                    "end_height": pallet_data["pick_height"]
                },
                ### Step 4 : Move robot from Pre Pallet Station to Pallet Station
                {
                    "source_id": pallet_data["pre_station_id"], # Pre Pallet Station
                    "id": pallet_data["station_id"], # Pallet Station
                    "task_id": task_id,
                    "operation": "ForkUnload",
                    "end_height": pallet_data["place_height"]
                },
                ### Step 5 : Move robot from Pallet Station to Pre Pallet Station
                {
                    "source_id": pallet_data["station_id"],
                    "id": pallet_data["pre_station_id"],
                    "task_id": task_id,
                    "operation": "ForkHeight",
                    "end_height": pallet_data["running_height"] # Down pallet to ground
                }
        ]
        return command

    def test_command(self, current_station_id, station_2go, task_id):
        command = [
                ### Step 1 :Move to Pre Station
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