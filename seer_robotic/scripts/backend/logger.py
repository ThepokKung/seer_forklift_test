#!/usr/bin/env python3

"""
Logging utility for robot API communication.
Provides structured logging with timestamps and different log levels.
"""

import logging
import os
import sys
from datetime import datetime
from pathlib import Path

class RobotLogger:
    """
    Logger for robot API communication with file and console output.
    """
    
    def __init__(self, name="robot_api", log_level=logging.INFO):
        """
        Initialize the robot logger.
        
        Args:
            name (str): Logger name
            log_level: Logging level (DEBUG, INFO, WARNING, ERROR, CRITICAL)
        """
        self.logger = logging.getLogger(name)
        self.logger.setLevel(log_level)
        
        # Prevent adding handlers multiple times
        if not self.logger.handlers:
            self._setup_logger()
    
    def _setup_logger(self):
        """Setup logger with file and console handlers."""
        # Create logs directory
        logs_dir = self._get_logs_directory()
        logs_dir.mkdir(parents=True, exist_ok=True)
        
        # Create formatter with timestamp
        formatter = logging.Formatter(
            '%(asctime)s - %(name)s - %(levelname)s - %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        )
        
        # File handler with rotating logs
        log_file = logs_dir / f"robot_api_{datetime.now().strftime('%Y%m%d')}.log"
        file_handler = logging.FileHandler(log_file, mode='a')
        file_handler.setLevel(logging.DEBUG)
        file_handler.setFormatter(formatter)
        
        # Console handler (optional - can be disabled)
        console_handler = logging.StreamHandler(sys.stdout)
        console_handler.setLevel(logging.WARNING)  # Only warnings and errors to console
        console_handler.setFormatter(formatter)
        
        # Add handlers
        self.logger.addHandler(file_handler)
        self.logger.addHandler(console_handler)
    
    def _get_logs_directory(self):
        """Get the logs directory path."""
        # Get the directory of the current script
        current_dir = Path(__file__).parent
        # Go up to the scripts directory and create logs folder
        logs_dir = current_dir.parent / "logs"
        return logs_dir
    
    def debug(self, message, *args, **kwargs):
        """Log debug message."""
        self.logger.debug(message, *args, **kwargs)
    
    def info(self, message, *args, **kwargs):
        """Log info message."""
        self.logger.info(message, *args, **kwargs)
    
    def warning(self, message, *args, **kwargs):
        """Log warning message."""
        self.logger.warning(message, *args, **kwargs)
    
    def error(self, message, *args, **kwargs):
        """Log error message."""
        self.logger.error(message, *args, **kwargs)
    
    def critical(self, message, *args, **kwargs):
        """Log critical message."""
        self.logger.critical(message, *args, **kwargs)
    
    def log_hex_data(self, data, message="Hex data", level=logging.DEBUG):
        """
        Log binary data in hex format.
        
        Args:
            data (bytes): Binary data to log
            message (str): Descriptive message
            level: Log level
        """
        hex_str = ' '.join('{:02X}'.format(x) for x in data)
        self.logger.log(level, f"{message}: {hex_str}")
    
    def log_json_data(self, data, message="JSON data", level=logging.DEBUG):
        """
        Log JSON data with pretty formatting.
        
        Args:
            data (dict): JSON data to log
            message (str): Descriptive message
            level: Log level
        """
        import json
        json_str = json.dumps(data, indent=2)
        self.logger.log(level, f"{message}:\n{json_str}")
    
    def log_connection_info(self, ip, port, status="connected"):
        """
        Log connection information.
        
        Args:
            ip (str): IP address
            port (int): Port number
            status (str): Connection status
        """
        self.info(f"Connection {status}: {ip}:{port}")
    
    def log_api_request(self, api_number, req_id, payload=None):
        """
        Log API request information.
        
        Args:
            api_number (int): API number
            req_id (int): Request ID
            payload (dict): Request payload
        """
        self.info(f"API Request - Number: {api_number}, ReqID: {req_id}")
        if payload:
            self.debug(f"Request payload: {payload}")
    
    def log_api_response(self, header, data=None):
        """
        Log API response information.
        
        Args:
            header (dict): Response header
            data (dict): Response data
        """
        self.info(f"API Response - ReqID: {header.get('req_id', 'N/A')}, "
                  f"Type: {header.get('msg_type', 'N/A')}, "
                  f"Length: {header.get('msg_length', 'N/A')}")
        if data:
            self.debug(f"Response data: {data}")
    
    def log_error(self, error, context=""):
        """
        Log error with context.
        
        Args:
            error (Exception): Exception object
            context (str): Additional context
        """
        if context:
            self.error(f"{context}: {str(error)}")
        else:
            self.error(f"Error: {str(error)}")

# Global logger instances
_loggers = {}

def get_logger(name="robot_api", log_level=logging.INFO):
    """
    Get or create a logger instance.
    
    Args:
        name (str): Logger name
        log_level: Logging level
        
    Returns:
        RobotLogger: Logger instance
    """
    if name not in _loggers:
        _loggers[name] = RobotLogger(name, log_level)
    return _loggers[name]

# Convenience function for default logger
def get_default_logger():
    """Get the default robot API logger."""
    return get_logger("robot_api")