import pandas as pd
from sqlalchemy import create_engine, text
from sqlalchemy.exc import OperationalError, SQLAlchemyError
from sqlalchemy.orm import sessionmaker

class PalletLoader:
    def __init__(self, db_host, db_port, db_name, db_user, db_pass):
        self.db_host = db_host
        self.db_port = db_port
        self.db_name = db_name
        self.db_user = db_user
        self.db_pass = db_pass
        self.engine = None

    #####################################################
    ###                 Public Class                  ###
    #####################################################

    def connect_db(self):
        try:
            conn_str = f"postgresql://{self.db_user}:{self.db_pass}@{self.db_host}:{self.db_port}/{self.db_name}"
            self.engine = create_engine(conn_str)
            self.Session = sessionmaker(bind=self.engine)
            # ทดสอบ connection
            with self.engine.connect() as conn:
                conn.execute(text("SELECT 1"))
            return self.engine
        except OperationalError as e:
            # print(f"❌ DB connection error: {e}")
            return False
        
    def get_all_pallet_data(self):
        self._ensure_connection()
        query = text("""
            SELECT
                    pallet_id, pallet_level, station_id, pre_station_id
              FROM 
                    pallet
        """)
        try:
            df = pd.read_sql_query(query, self.engine) # type: ignore
            # print(f"✅ Loaded {len(df)} pallets")
            return df
        except SQLAlchemyError as e:
            # print(f"❌ Read error: {e}")
            return None
        
    def get_all_pallet_levels(self):
        self._ensure_connection()
        query = text("SELECT * FROM pallet_level;")
        try:
            df = pd.read_sql_query(query, self.engine) # type: ignore
            # print(f"✅ Loaded {len(df)} levels")
            return df
        except SQLAlchemyError as e:
            # print(f"❌ Read error: {e}")
            return None
        
    def get_pallet_data_id(self, pallet_id):
       temp_body = self._get_pallet_by_id(pallet_id)
       if temp_body is None:
           return None
       temp_level = self._get_pallets_level(temp_body['pallet_level']) if temp_body else None

       if temp_body is None or temp_level is None:
           return None

       return {
           "pallet_id": temp_body['pallet_id'],
           "station_id": temp_body['station_id'],
           "pre_station_id": temp_body['pre_station_id'],
           "pallet_level": temp_body['pallet_level'],
           "pick_height": temp_level['pick_height'],
           "place_height": temp_level['place_height'],
           "default_height": temp_level['default_height'],
           "running_height": temp_level['running_height'],
           "down_height": temp_level['down_height']
       }

    #####################################################
    ###                Private Class                  ###
    #####################################################

    def _ensure_connection(self):
        if self.engine is None and not self.connect_db():
            raise RuntimeError("❌ Unable to connect to DB")

    def _get_pallet_by_id(self, pallet_id):
        self._ensure_connection()
        pallet_id = int(pallet_id)
        query = text("""
            SELECT 
                    pallet_id, pallet_level, station_id, pre_station_id
              FROM 
                    pallet
             WHERE 
                    pallet_id = :pallet_id
        """)
        try:
            df = pd.read_sql_query(query, self.engine, params={"pallet_id": pallet_id}) # type: ignore
            if df.empty:
                # print(f"⚠️ No pallet with ID={pallet_id}")
                return None
            return df.iloc[0].to_dict()
        except SQLAlchemyError as e:
            # print(f"❌ Read error: {e}")
            return None

    def _get_pallets_level(self, level):
        self._ensure_connection()
        level = int(level)
        query = text("""
            SELECT
                pick_height, place_height, default_height, running_height, down_height
            FROM
                pallet_level
            WHERE
                pallet_level = :level
        """)
        try:
            df = pd.read_sql_query(query, self.engine, params={"level": level}) # type: ignore
            # print(f"✅ Found {len(df)} records at level {level}")
            return df.iloc[0].to_dict()
        except SQLAlchemyError as e:
            # print(f"❌ Read error: {e}")
            return None
