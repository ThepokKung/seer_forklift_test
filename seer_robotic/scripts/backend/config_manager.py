#!/usr/bin/env python3

"""
Configuration manager for robot parameters and API settings.
Loads configuration from YAML files and provides easy access to robot and API settings.
"""

import os
import yaml
from typing import Dict, Any, Optional
from .logger import get_logger

class ConfigManager:
    """
    Manager for robot and API configuration settings.
    """
    
    def __init__(self, config_dir: Optional[str] = None):
        """
        Initialize the configuration manager.
        
        Args:
            config_dir (str): Path to configuration directory. If None, uses default.
        """
        if config_dir is None:
            # Default to config directory relative to this script
            current_dir = os.path.dirname(os.path.abspath(__file__))
            config_dir = os.path.join(os.path.dirname(current_dir), 'config')
        
        self.config_dir = config_dir
        self.robot_params = {}
        self.api_ports = {}
        self.logger = get_logger("config_manager")
        
        # Load configuration files
        self._load_robot_params()
        self._load_api_ports()
    
    def _load_robot_params(self):
        """Load robot parameters from YAML file."""
        robot_params_file = os.path.join(self.config_dir, 'robot_params.yaml')
        
        try:
            with open(robot_params_file, 'r') as f:
                self.robot_params = yaml.safe_load(f)
                self.logger.info(f"Robot parameters loaded from {robot_params_file}")
        except FileNotFoundError:
            self.logger.warning(f"Robot params file not found at {robot_params_file}")
            self.robot_params = self._get_default_robot_params()
        except yaml.YAMLError as e:
            self.logger.error(f"Error loading robot params: {e}")
            self.robot_params = self._get_default_robot_params()
    
    def _load_api_ports(self):
        """Load API port configuration from YAML file."""
        api_ports_file = os.path.join(self.config_dir, 'api_ports.yaml')
        
        try:
            with open(api_ports_file, 'r') as f:
                self.api_ports = yaml.safe_load(f)
                self.logger.info(f"API ports loaded from {api_ports_file}")
        except FileNotFoundError:
            self.logger.warning(f"API ports file not found at {api_ports_file}")
            self.api_ports = self._get_default_api_ports()
        except yaml.YAMLError as e:
            self.logger.error(f"Error loading API ports: {e}")
            self.api_ports = self._get_default_api_ports()
    
    def _get_default_robot_params(self) -> Dict[str, Any]:
        """Get default robot parameters if config file is not available."""
        return {
            'robots': {
                'robot_01': {
                    'robot_name': 'SEER_Robot_01',
                    'robot_id': 1,
                    'robot_ip': '192.168.0.180'
                },
                'robot_02': {
                    'robot_name': 'SEER_Robot_02',
                    'robot_id': 2,
                    'robot_ip': '192.168.0.181'
                }
            },
            'default_robot': 'robot_01',
            'network': {
                'timeout': 5,
                'retry_attempts': 3,
                'retry_delay': 1.0
            }
        }
    
    def _get_default_api_ports(self) -> Dict[str, Any]:
        """Get default API port configuration if config file is not available."""
        return {
            'api_ports': {
                'robot_status_api': {'port': 19204, 'max_connections': 10},
                'robot_control_api': {'port': 19205, 'max_connections': 5},
                'robot_navigation_api': {'port': 19206, 'max_connections': 5},
                'robot_configuration_api': {'port': 19207, 'max_connections': 5},
                'other_api': {'port': 19210, 'max_connections': 5},
                'robot_push_api': {'port': 19301, 'max_connections': 10}
            },
            'default_api': 'robot_status_api'
        }
    
    def get_robot_info(self, robot_key: Optional[str] = None) -> Dict[str, Any]:
        """
        Get robot information by key.
        
        Args:
            robot_key (str): Robot key (e.g., 'robot_01'). If None, uses default.
            
        Returns:
            Dict containing robot information
        """
        if robot_key is None:
            robot_key = self.robot_params.get('default_robot', 'robot_01')
        
        robots = self.robot_params.get('robots', {})
        if robot_key not in robots:
            raise ValueError(f"Robot '{robot_key}' not found in configuration")
        
        return robots[robot_key]
    
    def get_robot_ip(self, robot_key: Optional[str] = None) -> str:
        """
        Get robot IP address.
        
        Args:
            robot_key (str): Robot key. If None, uses default.
            
        Returns:
            str: Robot IP address
        """
        robot_info = self.get_robot_info(robot_key)
        return robot_info.get('robot_ip', '192.168.0.180')
    
    def get_robot_id(self, robot_key: Optional[str] = None) -> int:
        """
        Get robot ID.
        
        Args:
            robot_key (str): Robot key. If None, uses default.
            
        Returns:
            int: Robot ID
        """
        robot_info = self.get_robot_info(robot_key)
        return robot_info.get('robot_id', 1)
    
    def get_robot_name(self, robot_key: Optional[str] = None) -> str:
        """
        Get robot name.
        
        Args:
            robot_key (str): Robot key. If None, uses default.
            
        Returns:
            str: Robot name
        """
        robot_info = self.get_robot_info(robot_key)
        return robot_info.get('robot_name', 'SEER_Robot')
    
    def get_api_port(self, api_name: Optional[str] = None) -> int:
        """
        Get API port number.
        
        Args:
            api_name (str): API name (e.g., 'robot_status_api'). If None, uses default.
            
        Returns:
            int: Port number
        """
        if api_name is None:
            api_name = self.api_ports.get('default_api', 'robot_status_api')
        
        api_ports = self.api_ports.get('api_ports', {})
        if api_name not in api_ports:
            raise ValueError(f"API '{api_name}' not found in configuration")
        
        return api_ports[api_name].get('port', 19204)
    
    def get_api_info(self, api_name: Optional[str] = None) -> Dict[str, Any]:
        """
        Get complete API information.
        
        Args:
            api_name (str): API name. If None, uses default.
            
        Returns:
            Dict containing API information
        """
        if api_name is None:
            api_name = self.api_ports.get('default_api', 'robot_status_api')
        
        api_ports = self.api_ports.get('api_ports', {})
        if api_name not in api_ports:
            raise ValueError(f"API '{api_name}' not found in configuration")
        
        return api_ports[api_name]
    
    def get_api_by_category(self, category: str) -> str:
        """
        Get API name by category.
        
        Args:
            category (str): API category (e.g., 'status', 'control')
            
        Returns:
            str: API name
        """
        api_categories = self.api_ports.get('api_categories', {})
        if category not in api_categories:
            raise ValueError(f"API category '{category}' not found")
        
        return api_categories[category]
    
    def get_network_timeout(self) -> int:
        """Get network timeout setting."""
        return self.robot_params.get('network', {}).get('timeout', 5)
    
    def get_retry_attempts(self) -> int:
        """Get retry attempts setting."""
        return self.robot_params.get('network', {}).get('retry_attempts', 3)
    
    def get_message_format_settings(self) -> Dict[str, Any]:
        """Get message format settings."""
        return self.api_ports.get('message_format', {
            'start_byte': 0x5A,
            'version': 0x01,
            'pack_format': '!BBHLH6s'
        })
    
    def list_robots(self) -> list:
        """Get list of available robots."""
        return list(self.robot_params.get('robots', {}).keys())
    
    def list_apis(self) -> list:
        """Get list of available APIs."""
        return list(self.api_ports.get('api_ports', {}).keys())
    
    def log_config_summary(self, log_level=None):
        """Log a summary of the loaded configuration."""
        if log_level is None:
            log_level = self.logger.info
        
        log_level("Configuration Summary")
        log_level("=" * 50)
        
        log_level("Robots:")
        for robot_key in self.list_robots():
            robot_info = self.get_robot_info(robot_key)
            log_level(f"  {robot_key}: {robot_info['robot_name']} (ID: {robot_info['robot_id']}, IP: {robot_info['robot_ip']})")
        
        log_level(f"Default Robot: {self.robot_params.get('default_robot', 'N/A')}")
        
        log_level("API Ports:")
        for api_name in self.list_apis():
            api_info = self.get_api_info(api_name)
            log_level(f"  {api_name}: Port {api_info['port']} (Max Connections: {api_info.get('max_connections', 'N/A')})")
        
        log_level(f"Default API: {self.api_ports.get('default_api', 'N/A')}")
        log_level(f"Network Timeout: {self.get_network_timeout()}s")
    
    def print_config_summary(self):
        """Print a summary of the loaded configuration (for backward compatibility)."""
        print("Configuration Summary")
        print("=" * 50)
        
        print("\nRobots:")
        for robot_key in self.list_robots():
            robot_info = self.get_robot_info(robot_key)
            print(f"  {robot_key}: {robot_info['robot_name']} (ID: {robot_info['robot_id']}, IP: {robot_info['robot_ip']})")
        
        print(f"\nDefault Robot: {self.robot_params.get('default_robot', 'N/A')}")
        
        print("\nAPI Ports:")
        for api_name in self.list_apis():
            api_info = self.get_api_info(api_name)
            print(f"  {api_name}: Port {api_info['port']} (Max Connections: {api_info.get('max_connections', 'N/A')})")
        
        print(f"\nDefault API: {self.api_ports.get('default_api', 'N/A')}")
        print(f"Network Timeout: {self.get_network_timeout()}s")

# Example usage and testing
if __name__ == "__main__":
    # Create configuration manager
    config = ConfigManager()
    
    # Print configuration summary (for console output)
    config.print_config_summary()
    
    # Log configuration summary (to log files)
    config.log_config_summary()
    
    # Test robot information retrieval
    logger = config.logger
    logger.info("=" * 50)
    logger.info("Testing Robot Information:")
    
    for robot_key in config.list_robots():
        logger.info(f"{robot_key}:")
        logger.info(f"  Name: {config.get_robot_name(robot_key)}")
        logger.info(f"  ID: {config.get_robot_id(robot_key)}")
        logger.info(f"  IP: {config.get_robot_ip(robot_key)}")
    
    # Test API information retrieval
    logger.info("=" * 50)
    logger.info("Testing API Information:")
    
    for api_name in config.list_apis():
        logger.info(f"{api_name}:")
        logger.info(f"  Port: {config.get_api_port(api_name)}")
        api_info = config.get_api_info(api_name)
        logger.info(f"  Max Connections: {api_info.get('max_connections', 'N/A')}")
        logger.info(f"  Category: {api_info.get('category', 'N/A')}")
