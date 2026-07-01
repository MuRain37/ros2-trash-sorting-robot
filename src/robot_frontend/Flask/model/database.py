import json
import re
from datetime import datetime

try:
    import pymysql
    from pymysql.cursors import DictCursor
except ImportError as exc:
    pymysql = None
    DictCursor = None
    PYMYSQL_IMPORT_ERROR = exc
else:
    PYMYSQL_IMPORT_ERROR = None

from config.config import DB_CONFIG


def _json_default(value):
    if isinstance(value, datetime):
        return value.strftime("%Y-%m-%d %H:%M:%S")
    return str(value)


def _quote_identifier(identifier):
    if not re.match(r"^[A-Za-z0-9_]+$", identifier or ""):
        raise ValueError(f"非法数据库标识符: {identifier}")
    return f"`{identifier}`"


class Database:
    """MySQL 连接与操作日志封装。"""

    last_error = None
    initialized = False

    @staticmethod
    def available():
        return pymysql is not None and Database.initialized

    @staticmethod
    def driver_available():
        return pymysql is not None

    @staticmethod
    def get_connection(use_database=True):
        if pymysql is None:
            Database.last_error = f"缺少 PyMySQL: {PYMYSQL_IMPORT_ERROR}"
            print(Database.last_error)
            return None

        config = dict(DB_CONFIG)
        if not use_database:
            config.pop("database", None)
        config["cursorclass"] = DictCursor

        try:
            return pymysql.connect(**config)
        except pymysql.Error as exc:
            Database.last_error = f"数据库连接失败: {exc}"
            print(Database.last_error)
            return None

    @staticmethod
    def ensure_database_exists():
        database = DB_CONFIG.get("database")
        if not database:
            Database.last_error = "DB_CONFIG 缺少 database"
            return False

        connection = Database.get_connection(use_database=False)
        if not connection:
            return False

        try:
            with connection.cursor() as cursor:
                cursor.execute(
                    f"CREATE DATABASE IF NOT EXISTS {_quote_identifier(database)} "
                    "DEFAULT CHARACTER SET utf8mb4 COLLATE utf8mb4_unicode_ci"
                )
            connection.commit()
            return True
        except (pymysql.Error, ValueError) as exc:
            Database.last_error = f"创建数据库失败: {exc}"
            print(Database.last_error)
            connection.rollback()
            return False
        finally:
            connection.close()

    @staticmethod
    def execute_query(query, params=None, fetch_one=False, fetch_all=False):
        connection = Database.get_connection()
        if not connection:
            return None

        try:
            with connection.cursor() as cursor:
                cursor.execute(query, params or ())
                if fetch_one:
                    result = cursor.fetchone()
                elif fetch_all:
                    result = cursor.fetchall()
                else:
                    result = cursor.lastrowid
                connection.commit()
                return result
        except pymysql.Error as exc:
            Database.last_error = f"SQL执行错误: {exc}"
            print(Database.last_error)
            connection.rollback()
            return None
        finally:
            connection.close()

    @staticmethod
    def execute_many(query, params_list):
        connection = Database.get_connection()
        if not connection:
            return False

        try:
            with connection.cursor() as cursor:
                cursor.executemany(query, params_list)
            connection.commit()
            return True
        except pymysql.Error as exc:
            Database.last_error = f"批量SQL执行错误: {exc}"
            print(Database.last_error)
            connection.rollback()
            return False
        finally:
            connection.close()

    @staticmethod
    def _column_exists(cursor, table, column):
        cursor.execute(
            """
            SELECT COUNT(*) AS count
            FROM INFORMATION_SCHEMA.COLUMNS
            WHERE TABLE_SCHEMA = %s AND TABLE_NAME = %s AND COLUMN_NAME = %s
            """,
            (DB_CONFIG["database"], table, column),
        )
        row = cursor.fetchone() or {}
        return bool(row.get("count"))

    @staticmethod
    def _ensure_column(cursor, table, column, ddl):
        if not Database._column_exists(cursor, table, column):
            cursor.execute(f"ALTER TABLE {_quote_identifier(table)} ADD COLUMN {ddl}")

    @staticmethod
    def log_operation(
        action_type,
        action_details,
        user_id="web_user",
        ip_address=None,
        status="完成",
        success=True,
        metadata=None,
    ):
        details = (
            json.dumps(action_details, ensure_ascii=False, default=_json_default)
            if isinstance(action_details, (dict, list))
            else str(action_details)
        )
        metadata_text = (
            json.dumps(metadata, ensure_ascii=False, default=_json_default)
            if metadata is not None
            else None
        )
        query = """
            INSERT INTO operation_logs
            (user_id, action_type, action_details, status, success, ip_address, metadata)
            VALUES (%s, %s, %s, %s, %s, %s, %s)
        """
        params = (
            user_id,
            action_type,
            details,
            status,
            1 if success else 0,
            ip_address,
            metadata_text,
        )
        return Database.execute_query(query, params) is not None

    @staticmethod
    def fetch_operation_logs(page=1, page_size=20, keyword=None, action_type=None):
        page = max(1, int(page or 1))
        page_size = min(100, max(1, int(page_size or 20)))
        offset = (page - 1) * page_size

        where = []
        params = []
        if action_type:
            where.append("action_type = %s")
            params.append(action_type)
        if keyword:
            like = f"%{keyword}%"
            where.append(
                "(action_type LIKE %s OR action_details LIKE %s "
                "OR user_id LIKE %s OR status LIKE %s)"
            )
            params.extend([like, like, like, like])

        where_sql = f"WHERE {' AND '.join(where)}" if where else ""
        count_row = Database.execute_query(
            f"SELECT COUNT(*) AS total FROM operation_logs {where_sql}",
            params,
            fetch_one=True,
        )
        if count_row is None:
            return None

        rows = Database.execute_query(
            f"""
            SELECT id, timestamp, user_id, action_type, action_details,
                   status, success, ip_address, metadata
            FROM operation_logs
            {where_sql}
            ORDER BY timestamp DESC, id DESC
            LIMIT %s OFFSET %s
            """,
            params + [page_size, offset],
            fetch_all=True,
        )
        if rows is None:
            return None

        for row in rows:
            timestamp = row.get("timestamp")
            if isinstance(timestamp, datetime):
                row["timestamp"] = timestamp.strftime("%Y-%m-%d %H:%M:%S")
            row["success"] = bool(row.get("success"))

        total = int(count_row.get("total") or 0)
        return {
            "items": rows,
            "total": total,
            "page": page,
            "page_size": page_size,
            "pages": max(1, (total + page_size - 1) // page_size),
        }

    @staticmethod
    def delete_operation_log(log_id):
        affected = Database.execute_query(
            "DELETE FROM operation_logs WHERE id = %s",
            (int(log_id),),
        )
        return affected is not None

    @staticmethod
    def clear_operation_logs():
        return Database.execute_query("DELETE FROM operation_logs") is not None


def init_database():
    """初始化 MySQL 数据库和表结构。"""
    if pymysql is None:
        Database.last_error = f"缺少 PyMySQL: {PYMYSQL_IMPORT_ERROR}"
        print(Database.last_error)
        return False

    if not Database.ensure_database_exists():
        return False

    connection = Database.get_connection()
    if not connection:
        return False

    try:
        with connection.cursor() as cursor:
            cursor.execute(
                """
                CREATE TABLE IF NOT EXISTS robot_status (
                    id BIGINT AUTO_INCREMENT PRIMARY KEY,
                    timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
                    battery_level FLOAT,
                    temperature FLOAT,
                    humidity FLOAT,
                    position_x FLOAT,
                    position_y FLOAT,
                    orientation FLOAT,
                    speed FLOAT,
                    status VARCHAR(50),
                    error_code INT,
                    error_message TEXT,
                    INDEX idx_robot_status_timestamp (timestamp)
                ) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4
                """
            )
            cursor.execute(
                """
                CREATE TABLE IF NOT EXISTS sensor_data (
                    id BIGINT AUTO_INCREMENT PRIMARY KEY,
                    timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
                    sensor_type VARCHAR(50),
                    sensor_id VARCHAR(50),
                    value FLOAT,
                    unit VARCHAR(20),
                    INDEX idx_sensor_timestamp (timestamp),
                    INDEX idx_sensor_type (sensor_type)
                ) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4
                """
            )
            cursor.execute(
                """
                CREATE TABLE IF NOT EXISTS map_data (
                    id BIGINT AUTO_INCREMENT PRIMARY KEY,
                    timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
                    grid_x INT,
                    grid_y INT,
                    value FLOAT,
                    data_type VARCHAR(20),
                    INDEX idx_map_data_type (data_type)
                ) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4
                """
            )
            cursor.execute(
                """
                CREATE TABLE IF NOT EXISTS map_regions (
                    id BIGINT AUTO_INCREMENT PRIMARY KEY,
                    region_id INT,
                    region_name VARCHAR(100),
                    vertices TEXT,
                    color VARCHAR(20),
                    created_at DATETIME DEFAULT CURRENT_TIMESTAMP
                ) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4
                """
            )
            cursor.execute(
                """
                CREATE TABLE IF NOT EXISTS voice_commands (
                    id BIGINT AUTO_INCREMENT PRIMARY KEY,
                    timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
                    command_text TEXT,
                    recognized_text TEXT,
                    confidence FLOAT,
                    status VARCHAR(20),
                    response TEXT
                ) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4
                """
            )
            cursor.execute(
                """
                CREATE TABLE IF NOT EXISTS text_commands (
                    id BIGINT AUTO_INCREMENT PRIMARY KEY,
                    timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
                    command_text TEXT,
                    user_id VARCHAR(50),
                    status VARCHAR(20),
                    response TEXT
                ) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4
                """
            )
            cursor.execute(
                """
                CREATE TABLE IF NOT EXISTS operation_logs (
                    id BIGINT AUTO_INCREMENT PRIMARY KEY,
                    timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
                    user_id VARCHAR(64),
                    action_type VARCHAR(80),
                    action_details TEXT,
                    status VARCHAR(32) DEFAULT '完成',
                    success TINYINT(1) DEFAULT 1,
                    ip_address VARCHAR(64),
                    metadata JSON NULL,
                    INDEX idx_operation_timestamp (timestamp),
                    INDEX idx_operation_action (action_type)
                ) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4
                """
            )
            cursor.execute(
                """
                CREATE TABLE IF NOT EXISTS camera_frames (
                    id BIGINT AUTO_INCREMENT PRIMARY KEY,
                    timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
                    camera_id VARCHAR(50),
                    frame_data LONGBLOB,
                    frame_type VARCHAR(20)
                ) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4
                """
            )

            Database._ensure_column(
                cursor,
                "operation_logs",
                "status",
                "status VARCHAR(32) DEFAULT '完成'",
            )
            Database._ensure_column(
                cursor,
                "operation_logs",
                "success",
                "success TINYINT(1) DEFAULT 1",
            )
            Database._ensure_column(
                cursor,
                "operation_logs",
                "metadata",
                "metadata JSON NULL",
            )

        connection.commit()
        Database.initialized = True
        Database.last_error = None
        print("数据库表初始化完成")
        return True
    except (pymysql.Error, ValueError) as exc:
        Database.initialized = False
        Database.last_error = f"数据库初始化失败: {exc}"
        print(Database.last_error)
        connection.rollback()
        return False
    finally:
        connection.close()


if __name__ == "__main__":
    init_database()
