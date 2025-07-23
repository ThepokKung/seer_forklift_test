from bn_json_command_builder import JsonCommandBuilder
from bn_pallet_loader import PalletLoader

from dotenv import load_dotenv
import os
load_dotenv()

import json

def main():
    # Initialize the JsonCommandBuilder
    json_command_builder = JsonCommandBuilder()
    print("=" * 70)

    db_host = os.getenv('DB_HOST', 'localhost')
    db_port = os.getenv('DB_PORT', '5432')
    db_name = os.getenv('DB_NAME', 'seer_db')
    db_user = os.getenv('DB_USER', 'seer_user')
    db_pass = os.getenv('DB_PASS', 'seer_pass')

    pallet_loader = PalletLoader(
        db_host=db_host,
        db_port=db_port,
        db_name=db_name,
        db_user=db_user,
        db_pass=db_pass
    )
    
    try:
        # Connect to the database
        engine = pallet_loader.connect_db()
        if engine:
            print("✅ Database connection established successfully.")
            all_pallets = pallet_loader.get_all_pallet_data()
            if all_pallets is not None:
                print(f"Loaded {len(all_pallets)} pallets from the database.")
            else:
                print("❌ Failed to load pallets from the database.")
        else:
            print("❌ Failed to connect to the database.")
    except Exception as e:
        print(f"❌ An error occurred: {e}")
    finally:
        print("Exiting the Pallet Loader script.")
        print("=" * 70)

    
    # try:
    #     print("Pallet Pick Command Example")
    #     # Example usage of the pallet_pick_init_command method
    #     current_station_id = "LM44"
    #     # pallet_data = {
    #     #     "pre_station_id": "LM43",
    #     #     "station_id": "AP9",
    #     #     "pick_height": 0.5,
    #     #     "default_height": 0.25
    #     # }
    #     pallet_data = pallet_loader.get_pallet_data_id(99)
    #     task_id = "task_123"

    #     command_list = json_command_builder.pallet_pick_init_command(current_station_id, pallet_data, task_id)
    #     # Now iterate through each step command
    #     for step_num, step_command in enumerate(command_list, 1):
    #         step_json = json.dumps(step_command)
    #         print(f"Step {step_num}: {step_json}")
            
    # except Exception as e:
    #     print(f"An error occurred: {e}")
    # finally:
    #     print("=" * 70)

    # try:
    #     print("Pallet Place Command Example")
    #     # Example usage of the pallet_place_init_command method
    #     current_station_id = "46"
    #     # pallet_data = {
    #     #     "pre_station_id": "LM43",
    #     #     "station_id": "AP9",
    #     #     "pick_height": 0.5,
    #     #     "default_height": 0.25
    #     # }
    #     pallet_data = pallet_loader.get_pallet_data_id(99)
    #     task_id = "task_123"

    #     command_list = json_command_builder.pallet_place_init_command(current_station_id, pallet_data, task_id)
    #     # Now iterate through each step command
    #     for step_num, step_command in enumerate(command_list, 1):
    #         step_json = json.dumps(step_command)
    #         print(f"Step {step_num}: {step_json}")
            
    # except Exception as e:
    #     print(f"An error occurred: {e}")
    # finally:
    #     print("=" * 70)

    try:
        print("Pallet to manipulation Command Example")
        # Example usage of the pallet_place_init_command method
        current_station_id = "LM22"
        # pallet_data = {
        #     "pre_station_id": "LM43",
        #     "station_id": "AP9",
        #     "pick_height": 0.5,
        #     "default_height": 0.25
        # }
        pallet_data = pallet_loader.get_pallet_data_id(8)
        task_id = "task_123"

        command_list = json_command_builder.pallet_pick_to_manipulator_command(current_station_id, pallet_data, task_id)
        # Now iterate through each step command
        for step_num, step_command in enumerate(command_list, 1):
            step_json = json.dumps(step_command)
            print(f"Step {step_num}: {step_json}")
            
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        print("=" * 70)

    try:
        print("Pallet from manipulation Command Example")
        # Example usage of the pallet_place_init_command method
        current_station_id = "LM22"
        # pallet_data = {
        #     "pre_station_id": "LM43",
        #     "station_id": "AP9",
        #     "pick_height": 0.5,
        #     "default_height": 0.25
        # }
        pallet_data = pallet_loader.get_pallet_data_id(8)
        task_id = "task_123"

        command_list = json_command_builder.pallet_pick_from_manipulator_command(current_station_id, pallet_data, task_id)
        # Now iterate through each step command
        for step_num, step_command in enumerate(command_list, 1):
            step_json = json.dumps(step_command)
            print(f"Step {step_num}: {step_json}")
            
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        print("=" * 70)

if __name__ == "__main__":
    main()