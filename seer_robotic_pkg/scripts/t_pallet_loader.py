from bn_pallet_loader import PalletLoader
from dotenv import load_dotenv
import os

load_dotenv()

def main():
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

if __name__ == "__main__":
    main()