import pandas as pd
from sqlalchemy import create_engine, text
from sqlalchemy.exc import SQLAlchemyError, OperationalError
from sqlalchemy.orm import sessionmaker

"""
PostgreSQL Pallet Loader with SQLAlchemy
Connects to PostgreSQL database and loads pallet data using SQLAlchemy ORM

Connection parameters:
    host=DB_HOST,
    port=DB_PORT,
    dbname=DB_NAME,
    user=DB_USER,
    password=DB_PASS
"""

class PalletLoader:
    def __init__(self, db_host, db_port, db_name, db_user, db_pass):
        self.db_host = db_host
        self.db_port = db_port
        self.db_name = db_name
        self.db_user = db_user
        self.db_pass = db_pass
        self.engine = None
        self.Session = None

    def connect_db(self):
        """Establish connection to PostgreSQL database using SQLAlchemy"""
        try:
            connection_string = f"postgresql://{self.db_user}:{self.db_pass}@{self.db_host}:{self.db_port}/{self.db_name}"
            self.engine = create_engine(connection_string)
            self.Session = sessionmaker(bind=self.engine)
            
            # Test connection
            with self.engine.connect() as conn:
                conn.execute(text("SELECT 1"))
            
            return self.engine
        except OperationalError as e:
            print(f"❌ Database connection error: {e}")
            return None
        except Exception as e:
            print(f"❌ Unexpected error connecting to database: {e}")
            return None

    def disconnect_db(self):
        """Close database connection"""
        if self.engine:
            self.engine.dispose()
            self.engine = None
            self.Session = None
            print("🔌 Database connection closed")
    
    def get_all_pallet_data(self):
        """Load pallet data from PostgreSQL database"""
        query = "SELECT pallet_id,pallet_level,pallet_level,pre_station_id FROM pallets;"
        
        try:
            if self.engine is None:
                if self.connect_db() is None:
                    return None
            
            print("📊 Executing query: SELECT * FROM pallets;")
            df = pd.read_sql_query(query, self.engine) # type: ignore
            
            if df.empty:
                print("⚠️  No data found in pallets table")
            else:
                print(f"✅ Successfully loaded {len(df)} records from pallets table")
                
            return df
            
        except SQLAlchemyError as e:
            print(f"❌ SQLAlchemy error: {e}")
            return None
        except pd.errors.DatabaseError as e:
            print(f"❌ Pandas database error: {e}")
            return None
        except Exception as e:
            print(f"❌ Unexpected error loading pallet data: {e}")
            return None

    def get_all_pallet_levels(self):
        """Load pallet levels from PostgreSQL database"""
        query = "SELECT * FROM pallet_levels;"
        
        try:
            if self.engine is None:
                if self.connect_db() is None:
                    return None
            
            print("📊 Executing query: SELECT * FROM pallet_levels;")
            df = pd.read_sql_query(query, self.engine) # type: ignore
            
            if df.empty:
                print("⚠️  No data found in pallet_levels table")
            else:
                print(f"✅ Successfully loaded {len(df)} records from pallet_levels table")
            return df
            
        except SQLAlchemyError as e:
            print(f"❌ SQLAlchemy error: {e}")
            return None
        except pd.errors.DatabaseError as e:
            print(f"❌ Pandas database error: {e}")
            return None
        except Exception as e:
            print(f"❌ Unexpected error loading pallet data: {e}")
            return None

    def get_pallet_by_id(self, pallet_id):
        """Get a specific pallet by its ID"""
        query = f"SELECT pallet_id,pallet_level,pallet_level,pre_station_id FROM pallets WHERE pallet_id = {pallet_id};"
        
        try:
            if self.engine is None:
                if self.connect_db() is None:
                    return None
            
            # Convert to int to handle numpy.int64 issues
            pallet_id = int(pallet_id)
            
            print(f"📊 Executing query: SELECT * FROM pallets WHERE pallet_id = {pallet_id};")
            df = pd.read_sql_query(query, self.engine, params={"pallet_id": pallet_id}) # type: ignore
            
            if df.empty:
                print(f"⚠️  No pallet found with ID: {pallet_id}")
                return None
            else:
                print(f"✅ Found pallet with ID: {pallet_id}")
                return df.iloc[0].to_dict()
                
        except Exception as e:
            print(f"❌ Error getting pallet by ID: {e}")
            return None


    def get_pallets_level(self, pallet_level):
        """Get all pallets at a specific level"""
        query = f"SELECT * FROM pallet_levels WHERE pallet_level = {pallet_level};"
        
        try:
            if self.engine is None:
                if self.connect_db() is None:
                    return None
            
            # Convert to int to handle numpy types
            pallet_level = int(pallet_level)

            df = pd.read_sql_query(query, self.engine, params={"pallet_levels": pallet_level}) # type: ignore

            if df.empty:
                print(f"⚠️  No pallets found at level: {pallet_level}")
                return None
            else:
                print(f"✅ Found {len(df)} pallets at level: {pallet_level}")
                return df
                
        except Exception as e:
            print(f"❌ Error getting pallets by level: {e}")
            return None
