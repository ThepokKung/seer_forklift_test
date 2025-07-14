#!/usr/bin/python3

import rclpy
from rclpy.node import Node

from std_srvs.srv import Trigger

from bn_robot_navigation_api import RobotNavigationAPI
from bn_robot_status_api import RobotStatusAPI
from bn_pallet_loader import PalletLoader

import os
from dotenv import load_dotenv
load_dotenv()  # Load environment variables from .env file

class TestPickPlace(Node):
    def __init__(self):
        super().__init__('test_pick_place')
        self.get_logger().info('Test Pick Place node started')

        # Declare parameters
        # robot 1
        # self.declare_parameter('robot_id', 'robot_01')
        # self.declare_parameter('robot_name', 'SEER_Robot_01')
        # self.declare_parameter('robot_ip', '192.168.0.180')
        # robot 2
        self.declare_parameter('robot_id', 'robot_02')
        self.declare_parameter('robot_name', 'SEER_Robot_02')
        self.declare_parameter('robot_ip', '192.168.0.181')

        # Parameters
        self.robot_id = self.get_parameter('robot_id').get_parameter_value().string_value
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        self.robot_ip = self.get_parameter('robot_ip').get_parameter_value().string_value

        # API call
        self.navigation_api = RobotNavigationAPI(self.robot_ip)
        self.status_api = RobotStatusAPI(self.robot_ip)
        # self.pallet_loader = PalletLoader()
        db_host = os.getenv("DB_HOST")
        db_port = os.getenv("DB_PORT")
        db_name = os.getenv("DB_NAME")
        db_user = os.getenv("DB_USER")
        db_pass = os.getenv("DB_PASS")

        self.get_logger().info(f'Connecting to database {db_name} at {db_host}:{db_port} as user {db_user}')

        # Create an instance of PalletLoader
        print("🔧 Initializing PalletLoader...")
        # self.pallet_loader = PalletLoader(db_host, db_port, db_name, db_user, db_pass)
        self.pallet_loader = PalletLoader(db_host, db_port, db_name, db_user, db_pass)

        # Service server
        self.pick_service = self.create_service(Trigger, 'pick', self.pick_callback)
        self.place_service = self.create_service(Trigger, 'place', self.place_callback)
        self.test_db_service = self.create_service(Trigger, 'test_db_connection', self.test_db_callback)
        self.test_connection_service = self.create_service(Trigger, 'test_connection', self.test_connection_callback)

    def pick_callback(self, request, response):
        """Execute complete pick operation with 4 steps and navigation status checking"""
        self.get_logger().info('=== Starting Pick Operation ===')
        
        try:
            # Generate unique task ID
            import time
            task_id = f"PICK_{int(time.time())}"
            self.get_logger().info(f'Task ID: {task_id}')
            
            # Execute pick steps
            result = self.execute_pick_steps(task_id)
            
            if result['success']:
                response.success = True
                response.message = f"Pick operation completed successfully. Steps: {result['steps_completed']}/{result['total_steps']}"
                self.get_logger().info('=== Pick Operation Completed Successfully ===')
            else:
                response.success = False
                response.message = f"Pick operation failed: {result['error']}. Steps completed: {result['steps_completed']}/{result['total_steps']}"
                self.get_logger().error(f"Pick operation failed: {result['error']}")
                
        except Exception as e:
            response.success = False
            response.message = f"Pick operation error: {str(e)}"
            self.get_logger().error(f'Pick operation error: {e}')
            
        return response
    
    def place_callback(self, request, response):
        """Execute complete place operation with 4 steps and navigation status checking"""
        self.get_logger().info('=== Starting Place Operation ===')
        
        try:
            # Generate unique task ID
            import time
            task_id = f"PLACE_{int(time.time())}"
            self.get_logger().info(f'Task ID: {task_id}')
            
            # Execute place steps
            result = self.execute_place_steps(task_id)
            
            if result['success']:
                response.success = True
                response.message = f"Place operation completed successfully. Steps: {result['steps_completed']}/{result['total_steps']}"
                self.get_logger().info('=== Place Operation Completed Successfully ===')
            else:
                response.success = False
                response.message = f"Place operation failed: {result['error']}. Steps completed: {result['steps_completed']}/{result['total_steps']}"
                self.get_logger().error(f"Place operation failed: {result['error']}")
                
        except Exception as e:
            response.success = False
            response.message = f"Place operation error: {str(e)}"
            self.get_logger().error(f'Place operation error: {e}')
            
        return response
    
    def test_connection_callback(self, request, response):
        """Test connection to robot and return success/failure"""
        self.get_logger().info('=== Testing Connection ===')
        
        try:
            # Test navigation API connection
            if not self.navigation_api.connect():
                response.success = False
                response.message = "Navigation API connection failed"
                self.get_logger().error('Navigation API connection failed')
                return response
            
            self.get_logger().info('Navigation API connected successfully')
            
            # Test status API connection
            if not self.status_api.connect():
                response.success = False
                response.message = "Status API connection failed"
                self.get_logger().error('Status API connection failed')
                return response
            
            self.get_logger().info('Status API connected successfully')
            
            response.success = True
            response.message = "Connection test successful"
            self.get_logger().info('=== Connection Test Successful ===')
            
        except Exception as e:
            response.success = False
            response.message = f"Connection test error: {str(e)}"
            self.get_logger().error(f'Connection test error: {e}')
            
        return response
    
    def check_navigation_status(self):
        self.get_logger().info('Checking navigation status')
        if self.status_api.connect():
            status = self.status_api.get_navigation_status()
            self.get_logger().info(f'Navigation Status: {status}')
            self.status_api.disconnect()
        else:
            self.get_logger().error('Failed to connect to status API')

    def execute_pick_steps(self, task_id):
        """Execute 4-step pick operation: ขาไปยกของ - ONE STEP AT A TIME"""
        steps_completed = 0
        total_steps = 4
        
        try:
            self.get_logger().info("=== STARTING SEQUENTIAL PICK OPERATION ===")
            self.get_logger().info("Each step will wait for navigation_status = 4 before proceeding")
            
            # Step 1: ไป LM47 --> LM32 (ForkHeight)
            self.get_logger().info('🔄 Step 1/4: ไป LM47 --> LM32 (Set fork height)')
            step1_payload = {
                "source_id": "LM47",
                "id": "LM32",
                "task_id": task_id,
                "operation": "ForkHeight",
                "end_height": 0.085
            }
            
            if not self.send_command_and_wait(step1_payload, "Step 1"):
                self.get_logger().error("❌ Step 1 failed - stopping pick operation")
                return {"success": False, "error": "Step 1 failed", "steps_completed": steps_completed, "total_steps": total_steps}
            
            steps_completed = 1
            self.get_logger().info("✅ Step 1 COMPLETED - proceeding to Step 2")
            
            # Step 2: เข้าไปยก LM32 --> AP8 (ForkLoad)
            self.get_logger().info('🔄 Step 2/4: เข้าไปยก LM32 --> AP8 (Pick pallet)')
            step2_payload = {
                "source_id": "LM32",
                "id": "AP8",
                "task_id": task_id,
                "operation": "ForkLoad",
                "end_height": 0.155
            }
            
            if not self.send_command_and_wait(step2_payload, "Step 2"):
                self.get_logger().error("❌ Step 2 failed - stopping pick operation")
                return {"success": False, "error": "Step 2 failed", "steps_completed": steps_completed, "total_steps": total_steps}
                
            steps_completed = 2
            self.get_logger().info("✅ Step 2 COMPLETED - proceeding to Step 3")
            
            # Step 3: ถอยมา AP8 --> LM32 (Retreat)
            self.get_logger().info('🔄 Step 3/4: ถอยมา AP8 --> LM32 (Retreat with pallet)')
            step3_payload = {
                "source_id": "AP8",
                "id": "LM32",
                "task_id": task_id
            }
            
            if not self.send_command_and_wait(step3_payload, "Step 3"):
                self.get_logger().error("❌ Step 3 failed - stopping pick operation")
                return {"success": False, "error": "Step 3 failed", "steps_completed": steps_completed, "total_steps": total_steps}
                
            steps_completed = 3
            self.get_logger().info("✅ Step 3 COMPLETED - proceeding to Step 4")
            
            # Step 4: กลับ LM32 --> LM47 (Return)
            self.get_logger().info('🔄 Step 4/4: กลับ LM32 --> LM47 (Return to home)')
            step4_payload = {
                "source_id": "LM32",
                "id": "LM47",
                "task_id": task_id
            }
            
            if not self.send_command_and_wait(step4_payload, "Step 4"):
                self.get_logger().error("❌ Step 4 failed - pick operation incomplete")
                return {"success": False, "error": "Step 4 failed", "steps_completed": steps_completed, "total_steps": total_steps}
                
            steps_completed = 4
            self.get_logger().info("✅ Step 4 COMPLETED")
            self.get_logger().info("🎉 ALL PICK STEPS COMPLETED SUCCESSFULLY!")
            
            return {"success": True, "steps_completed": steps_completed, "total_steps": total_steps}
            
        except Exception as e:
            self.get_logger().error(f"💥 Pick operation error: {e}")
            return {"success": False, "error": str(e), "steps_completed": steps_completed, "total_steps": total_steps}

    def execute_place_steps(self, task_id):
        """Execute 4-step place operation: ขากลับจากยกของ"""
        steps_completed = 0
        total_steps = 4
        
        try:
            # Step 1: ไป LM47 --> LM32 
            self.get_logger().info('Step 1/4: ไป LM47 --> LM32 (Go to preparation)')
            step1_payload = {
                "source_id": "LM47",
                "id": "LM32",
                "task_id": task_id
            }
            
            if not self.send_command_and_wait(step1_payload, "Step 1"):
                return {"success": False, "error": "Step 1 failed", "steps_completed": steps_completed, "total_steps": total_steps}
            steps_completed = 1
            
            # Step 2: เข้าไปวาง LM32 --> AP8 (ForkUnload)
            self.get_logger().info('Step 2/4: เข้าไปวาง LM32 --> AP8 (Place pallet)')
            step2_payload = {
                "source_id": "LM32",
                "id": "AP8",
                "task_id": task_id,
                "operation": "ForkUnload",
                "end_height": 0.085
            }
            
            if not self.send_command_and_wait(step2_payload, "Step 2"):
                return {"success": False, "error": "Step 2 failed", "steps_completed": steps_completed, "total_steps": total_steps}
            steps_completed = 2
            
            # Step 3: ถอยมา AP8 --> LM32 (Retreat)
            self.get_logger().info('Step 3/4: ถอยมา AP8 --> LM32 (Retreat after placing)')
            step3_payload = {
                "source_id": "AP8",
                "id": "LM32",
                "task_id": task_id
            }
            
            if not self.send_command_and_wait(step3_payload, "Step 3"):
                return {"success": False, "error": "Step 3 failed", "steps_completed": steps_completed, "total_steps": total_steps}
            steps_completed = 3
            
            # Step 4: กลับ LM32 --> LM47 (Return)
            self.get_logger().info('Step 4/4: กลับ LM32 --> LM47 (Return to home)')
            step4_payload = {
                "source_id": "LM32",
                "id": "LM47",
                "task_id": task_id
            }
            
            if not self.send_command_and_wait(step4_payload, "Step 4"):
                return {"success": False, "error": "Step 4 failed", "steps_completed": steps_completed, "total_steps": total_steps}
            steps_completed = 4
            
            return {"success": True, "steps_completed": steps_completed, "total_steps": total_steps}
            
        except Exception as e:
            return {"success": False, "error": str(e), "steps_completed": steps_completed, "total_steps": total_steps}

    def send_command_and_wait(self, payload, step_name):
        """Send ONE navigation command and wait for navigation_status = 4 (completed)"""
        try:
            # Log the command being sent
            self.get_logger().info(f'{step_name} - Sending command:')
            self.get_logger().info(f'Payload: {payload}')
            
            # Connect to navigation API
            self.get_logger().info(f'{step_name} - Connecting to navigation API at {self.robot_ip}')
            if not self.navigation_api.connect():
                self.get_logger().error(f'{step_name} - Failed to connect to navigation API at {self.robot_ip}')
                return False
            
            self.get_logger().info(f'{step_name} - Connected successfully, sending navigation command')
            
            # Send ONLY this one command
            result = self.navigation_api.navigation_to_goal(
                id=payload["id"],
                source_id=payload["source_id"],
                task_id=payload["task_id"],
                operation=payload.get("operation"),
                end_height=payload.get("end_height")
            )
            
            self.get_logger().info(f'{step_name} - Navigation API result: {result}')
            self.navigation_api.disconnect()
            
            if not result:
                self.get_logger().error(f'{step_name} - Navigation API returned None')
                return False
                
            # Handle different result formats
            if isinstance(result, dict):
                # Check for ret_code = 0 (success) or success = True
                ret_code = result.get('ret_code')
                success = result.get('success')
                
                if ret_code == 0:  # ret_code = 0 means success
                    self.get_logger().info(f'{step_name} - Command accepted (ret_code=0)')
                elif success is True:
                    self.get_logger().info(f'{step_name} - Command accepted (success=True)')
                elif success is False:
                    self.get_logger().error(f'{step_name} - Navigation command failed: {result}')
                    return False
                else:
                    # If no clear success/failure indicator, log and continue
                    self.get_logger().info(f'{step_name} - Command sent, result: {result}')
            elif not result:
                self.get_logger().error(f'{step_name} - Navigation command returned False')
                return False
            
            self.get_logger().info(f'{step_name} - Command sent successfully')
            self.get_logger().info(f'{step_name} - Now waiting for THIS STEP to complete (navigation_status = 4)...')
            
            # Wait ONLY for THIS step to complete before returning
            if self.wait_for_navigation_completion(step_name):
                self.get_logger().info(f'{step_name} - ✅ STEP COMPLETED (navigation_status = 4)')
                self.get_logger().info(f'{step_name} - Ready to proceed to next step')
                return True
            else:
                self.get_logger().error(f'{step_name} - ❌ STEP FAILED or timed out')
                return False
                
        except Exception as e:
            self.get_logger().error(f'{step_name} - Error: {e}')
            return False

    def wait_for_navigation_completion(self, step_name, timeout=30):
        """Wait for navigation_status to become 4 (completed) - SHORTER TIMEOUT"""
        import time
        start_time = time.time()
        check_interval = 1.0  # Check every 1 second for faster response
        
        # Status code meanings for reference
        status_meanings = {
            0: "NONE (Not started)",
            1: "WAITING (Queued)",
            2: "RUNNING (In progress)",
            3: "SUSPENDED (Paused)",
            4: "COMPLETED (Success)",
            5: "FAILED (Error)",
            6: "CANCELED (Aborted)"
        }
        
        self.get_logger().info(f'{step_name} - ⏳ Waiting for navigation_status = 4 (COMPLETED) - timeout: {timeout}s')
        self.get_logger().info(f'{step_name} - Status codes: 0=NONE, 1=WAITING, 2=RUNNING, 3=SUSPENDED, 4=COMPLETED, 5=FAILED, 6=CANCELED')
        
        while (time.time() - start_time) < timeout:
            try:
                # Connect to status API
                if self.status_api.connect():
                    status = self.status_api.get_navigation_status()
                    self.status_api.disconnect()
                    
                    if status is not None:
                        # Handle different status formats
                        if isinstance(status, dict):
                            task_status = status.get('task_status', status.get('status', 0))
                        else:
                            task_status = status
                        
                        # Get status meaning
                        if isinstance(task_status, int):
                            status_meaning = status_meanings.get(task_status, "UNKNOWN")
                        else:
                            status_meaning = "INVALID_TYPE"
                            task_status = -1  # Default for non-integer status
                        elapsed = time.time() - start_time
                        
                        # EXPLICIT STATUS LOGGING
                        self.get_logger().info(f'{step_name} - 📊 CURRENT navigation_status = {task_status} ({status_meaning}) - elapsed: {elapsed:.1f}s')
                        
                        if task_status == 4:  # Completed successfully
                            self.get_logger().info(f'{step_name} - ✅ Navigation COMPLETED successfully! (status=4)')
                            return True
                        elif task_status == 5:  # Failed
                            self.get_logger().error(f'{step_name} - ❌ Navigation FAILED! (status=5)')
                            return False
                        elif task_status == 6:  # Canceled
                            self.get_logger().error(f'{step_name} - ❌ Navigation CANCELED! (status=6)')
                            return False
                        elif task_status == 0:  # None - not started yet
                            self.get_logger().info(f'{step_name} - 🔄 Not started yet (status=0-NONE)')
                        elif task_status == 1:  # Waiting - queued
                            self.get_logger().info(f'{step_name} - 🔄 Waiting in queue (status=1-WAITING)')
                        elif task_status == 2:  # Running - in progress
                            self.get_logger().info(f'{step_name} - 🔄 Currently running (status=2-RUNNING)')
                        elif task_status == 3:  # Suspended - paused
                            self.get_logger().warning(f'{step_name} - ⏸️  Navigation suspended (status=3-SUSPENDED)')
                        else:
                            self.get_logger().warning(f'{step_name} - ⚠️  Unknown status: {task_status}')
                    else:
                        self.get_logger().warning(f'{step_name} - ⚠️  No status returned from API')
                else:
                    self.get_logger().warning(f'{step_name} - ⚠️  Failed to connect to status API')
                
                # Wait before next check
                time.sleep(check_interval)
                
            except Exception as e:
                self.get_logger().error(f'{step_name} - Error checking status: {e}')
                time.sleep(check_interval)
        
        # Timeout reached
        elapsed_time = time.time() - start_time
        self.get_logger().error(f'{step_name} - ⏰ TIMEOUT after {elapsed_time:.1f}s - never reached status=4 (COMPLETED)')
        return False
    
    #####################################################
    ###                    Test                       ###
    #####################################################

    def test_db_callback(self, request, response):
        """Test database connection and return success/failure"""
        self.get_logger().info('=== Testing Database Connection ===')
        
        try:
            if self.pallet_loader.connect_db():
                self.get_logger().info('Database connection test successful')
                pallet_data = self.pallet_loader.get_all_pallet_data()
                response.success = True
                response.message = "Database connection test successful"
                self.get_logger().info('Database connection test successful')
            else:
                response.success = False
                response.message = "Database connection test failed"
                self.get_logger().error('Database connection test failed')
                
        except Exception as e:
            response.success = False
            response.message = f"Database connection error: {str(e)}"
            self.get_logger().error(f'Database connection error: {e}')
            
        return response

def main(args=None):
    rclpy.init(args=args)
    node = TestPickPlace()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()