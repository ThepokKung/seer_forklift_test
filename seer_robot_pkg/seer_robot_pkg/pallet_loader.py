import pandas as pd
import psycopg2
from psycopg2.extras import RealDictCursor
from typing import Optional, Dict, Any


class PalletLoader:
    def __init__(self, db_host: str, db_port: int, db_name: str, db_user: str, db_pass: str):
        self.db_host = db_host
        self.db_port = db_port
        self.db_name = db_name
        self.db_user = db_user
        self.db_pass = db_pass

        self.conn: Optional[psycopg2.extensions.connection] = None
        self.cursor: Optional[psycopg2.extensions.cursor] = None

    #####################################################
    ###                 Public Class                  ###
    #####################################################

    def connect_db(self) -> bool:
        if self.conn and getattr(self.conn, "closed", 1) == 0:
            return True
        try:
            self.conn = psycopg2.connect(
                host=self.db_host,
                port=self.db_port,
                user=self.db_user,
                password=self.db_pass,
                dbname=self.db_name,
            )
            self.conn.autocommit = True
            self.cursor = self.conn.cursor(cursor_factory=RealDictCursor)
            return True
        except psycopg2.Error:
            self.conn = None
            self.cursor = None
            return False

    def close(self) -> None:
        """Close cursor and connection if open."""
        try:
            if self.cursor and not self.cursor.closed:
                self.cursor.close()
        finally:
            self.cursor = None
            if self.conn and getattr(self.conn, "closed", 1) == 0:
                self.conn.close()
            self.conn = None

    def get_all_pallet_data(self):
        self._ensure_connection()
        sql = "SELECT * FROM pallet_data;"
        self.cursor.execute(sql) # type: ignore 
        rows = self.cursor.fetchall() # type: ignore
        self.close()
        return pd.DataFrame(rows)
        # return self._fetch_all_df(sql)

    def get_all_pallet_levels(self):
        self._ensure_connection()
        sql = "SELECT * FROM pallet_level;"
        self.cursor.execute(sql) # type: ignore
        rows = self.cursor.fetchall() # type: ignore
        self.close()
        return pd.DataFrame(rows)
        # return self._fetch_all_df(sql)

    def get_pallet_data_id(self, pallet_id):
        temp_body = self._get_pallet_by_id(pallet_id)
        if temp_body is None:
            return None
        temp_level = self._get_pallets_level(temp_body['pallet_level']) # type: ignore

        if temp_body is None or temp_level is None:
            return None
        self.close()
        return {
            "pallet_id": temp_body["pallet_id"], # type: ignore
            "station_id": temp_body["station_id"], # type: ignore
            "pre_station_id": temp_body["pre_station_id"], # type: ignore
            "pallet_level": temp_body["pallet_level"], # type: ignore
            "pick_height": temp_level["pick_height"], # type: ignore
            "place_height": temp_level["place_height"], # type: ignore
            "default_height": temp_level["default_height"], # type: ignore
            "running_height": temp_level["running_height"], # type: ignore
            "down_height": temp_level["down_height"], # type: ignore
        }
    
    #####################################################
    ###                Private Class                  ###
    #####################################################

    # Check for connection
    def _ensure_connection(self):
        if not self.connect_db() or self.cursor is None:
            raise RuntimeError("Unable to connect to DB or cursor is not initialized")

    def _get_pallet_by_id(self, pallet_id):
        self._ensure_connection()
        pallet_id = int(pallet_id)
        sql = (
            """
            SELECT
                pallet_id, pallet_level, station_id, pre_station_id
            FROM
                pallet_data
            WHERE
                pallet_id = %s
            """
        )
        self.cursor.execute(sql, (pallet_id,)) # type: ignore
        row = self.cursor.fetchone() # type: ignore
        # self.close()  # <-- REMOVE THIS LINE
        if row is None:
            return None
        return row
        

    def _get_pallets_level(self, level):
        self._ensure_connection()
        level = int(level)
        sql = (
            """
            SELECT
                pick_height, place_height, default_height, running_height, down_height
            FROM
                pallet_level
            WHERE
                pallet_level = %s
            """
        )
        self.cursor.execute(sql, (level,)) # type: ignore
        row = self.cursor.fetchone() # type: ignore
        # self.close()  # <-- REMOVE THIS LINE
        if row is None:
            return None
        return row