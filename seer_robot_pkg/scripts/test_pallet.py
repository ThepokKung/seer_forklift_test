from seer_robot_pkg.pallet_loader import PalletLoader
from dotenv import load_dotenv
import os

load_dotenv()  # Load environment variables from .env file

pallet_loader = PalletLoader(
            db_host=os.getenv('DB_HOST'),
            db_port=os.getenv('DB_PORT'),
            db_name=os.getenv('DB_NAME'),
            db_user=os.getenv('DB_USER'),
            db_pass=os.getenv('DB_PASS')
        )


pallet_data = pallet_loader.get_pallet_data_id(5)

print(f'Pallet data for ID 5: {pallet_data}')