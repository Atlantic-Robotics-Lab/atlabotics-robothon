import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image as RosImage
from cv_bridge import CvBridge
import cv2
from PIL import Image
import moondream as md
import google.generativeai as genai
from std_msgs.msg import String
import threading
import yaml 
import os

# --- YAML Custom Dumper for clean formatting ---
class SmartDumper(yaml.SafeDumper):
    def increase_indent(self, flow=False, indentless=False):
        return super().increase_indent(flow, False)

def smart_list_representer(dumper, data):
    # Represent short lists of numbers inline (e.g., [0.0, 0.0, 0.1])
    if all(isinstance(x, (int, float)) for x in data) and len(data) <= 5:
        return dumper.represent_sequence('tag:yaml.org,2002:seq', data, flow_style=True)
    return dumper.represent_sequence('tag:yaml.org,2002:seq', data, flow_style=False)

SmartDumper.add_representer(list, smart_list_representer)

class PipelineServiceNode(Node):
    def __init__(self):
        super().__init__('pipeline_service_node_byod')
        self.get_logger().info("Simple Pipeline Service Node is starting up...")

        # --- State Variables ---
        self.generated_task_name = None
        self.publishing_timer = None

        # Using os.path.expanduser to correctly handle '~' for the home directory
        self.yaml_file_path = os.path.expanduser('/home/atu-2/robothon/src/moveit_interface/config/move_group_params_byod.yaml')
        self.get_logger().info(f"Target YAML file is: {self.yaml_file_path}")


        if not os.path.exists(self.yaml_file_path):
          self.get_logger().info(f"YAML file not found. Creating a new empty file at {self.yaml_file_path}")
          with open(self.yaml_file_path, 'w') as f:
            pass # 'pass' is used to do nothing, just create the empty file.
        
        
        # # --- Configuration ---
        # self.yaml_file_path = os.path.expanduser('~/task_planner.yaml')
        # self.get_logger().info(f"Target YAML file for tasks is: {self.yaml_file_path}")


        moondream_key = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJrZXlfaWQiOiI3NjcxODQ4YS05YjAxLTQ5NzgtYmVjNi0yYTlhNzQ1ZWJhMDciLCJvcmdfaWQiOiJSZnVkSjZnQkRUaVYwQ1pxT1QwR0NUdTFPSXFEMXNYSSIsImlhdCI6MTc0Nzk1ODUwNiwidmVyIjoxfQ.ybo_dyly-WmrTDf79IN9SW9ZO0j7akChNvqoMnVA2o4"
        google_key = "AIzaSyAXlvpGgLo-Ckon4VMNcEa4654NCTpMMtw"

        
        try:
            self.moondream_model = md.vl(api_key=moondream_key)
            self.get_logger().info("Moondream model loaded.")


            genai.configure(api_key=google_key)
            self.gemini_model = genai.GenerativeModel('gemini-1.5-flash')
            self.get_logger().info("Gemini model configured.")
        except Exception as e:
            self.get_logger().fatal(f"Failed to initialize AI models: {e}")
            return
            
        # ---  Gemini Prompt  ---
        yaml_string = """
move_group:
  planner_id: "TRRT"
  planning_time: 10.0
  num_planning_attempts: 10  
  max_velocity_scaling_factor: 0.10
  max_acceleration_scaling_factor: 0.10
  end_effector_link: "tool0"
  pose_reference_frame: "base_link"
 
execute: false
planning : true
 
 
tasks:
  speed_test:
    stages:
      - name: move_home
        type: move_to
        planner: interpolation
        target: home_camera3_vertical
 
      - name: move_blue_pre_button
        type: move_to
        planner: interpolation
        target: blue_button
        offset: [0.0, 0.0, 0.1]
 
      - name: press_blue
        type: move_relative
        planner: cartesian
        target: blue_button
        direction: [0.0, 0.0, -1.0]
 
      - name: retract_blue
        type: move_relative
        planner: cartesian
        target: blue_button
        direction: [0.0, 0.0, 1.0]
      
      - name: blue_to_pre_red
        type: move_to
        planner: interpolation
        target: red_button
        offset: [0.0, 0.0, 0.1]
 
      - name: press_red
        type: move_relative
        planner: cartesian
        target: red_button
        direction: [0.0, 0.0, -1.0]
 
      - name: retract_red
        type: move_relative
        planner: cartesian
        target: red_button
        direction: [0.0, 0.0, 1.0]
 
      - name: move_home
        type: move_to
        planner: interpolation
        target: home_camera3
 
  press_blue_button:
    stages:
      - name: move_home
        type: move_to
        planner: interpolation
        target: home
 
      - name: move_blue_pre_button
        type: move_to
        planner: interpolation
        target: blue_button
        offset: [0.0, 0.0, 0.1]
 
      - name: press_blue
        type: move_relative
        planner: cartesian
        target: blue_button
        direction: [0.0, 0.0, -1.0]  
 
      - name: retract_blue
        type: move_relative
        planner: cartesian
        target: blue_button
        direction: [0.0, 0.0, 1.0]
 
      - name: move_home
        type: move_tot os
        planner: interpolation
        target: home
 
 
  press_red_button:
    stages:
      - name: move_home
        type: move_to
        planner: interpolation
        target: home
 
      - name: press_red
        type: move_relative
        planner: cartesian
        target: red_button
        offset: [0.0, 0.0, -1.0]
 
      - name: retract_red
        type: move_relative
        planner: cartesian
        target: red_button
        direction: [0.0, 0.0, 1.0]
 
      - name: move_home
        type: move_to
        planner: interpolation
        target: home
 
  pickup_pen:
    stages:
      - name: move_home
        type: move_to
        planner: interpolation
        target: home
 
      - name: approach_pen
        type: move_to
        planner: interpolation
        target: pre_pen
 
      - name: grasp
        type: move_relative
        planner: cartesian
        direction: [0.0, 0.0, -0.08]  # move down to pick
 
      - name: lift
        type: move_relative
        planner: cartesian
        direction: [0.0, 0.0, 0.1]


  Push_Blue_Object:
    stages:
      - name: move_home_cam
        type: move_to
        planner: interpolation
        target: home_camera_vertical
        vel_acc: [0.2, 0.2]

      - name: move_blue_pre_object
        type: move_to
        planner: interpolation
        target: blue_object
        offset: [0.0, 0.0, 0.1]

      - name: push_blue_object
        type: move_relative
        planner: cartesian
        target: blue_object
        direction: [0.0, 0.0, -1.0]
        minmax_dist: [0.101, 0.101]

      - name: retract_blue_object
        type: move_relative
        planner: cartesian
        target: blue_object
        direction: [0.0, 0.0, 1.0]

      - name: move_home
        type: move_to
        planner: interpolation
        target: home_camera
"""
        self.prompt_template = (
            "Given the following YAML configuration for robot tasks:\n"
            f"{yaml_string}\n"
            "Your task is to either modify an existing task or create a new one based on my instructions. "
            "When generating the output, adhere to these rules:\n"
            "1.  **YAML Only**: The output must be only the YAML definition for the specified task. It should start by the targeted task name like speed_test or pickup_pen , enclosed in ```yaml ... ```.\n"
            "2.  **Reuse Stages**: If a stage from an existing task is applicable, reuse its name and logic. For instance, to press the red button again, use the `press_red` stage definition.\n"
            "3. if a new task to be added and is similar to an exisiting task use the same naming conveition for the task and stagess\n"
            "4. if you need to do a task again like press blue then press red then press blue again, use the same name of stages for press blue because it is the same. Do that for all applicable"
            "5.  **Strict Relevance**: If the input is not a request to create or modify a task, output `false`.\n\n"
            "Now, based on my request to \"{task}\", generate the corresponding YAML."
        )

        
        self.bridge = CvBridge()
        self.image_lock = threading.Lock()
        self.latest_image_msg = None
        self.image_received = False
	
	
        image_topic = '/camera/camera/color/image_raw'
        self.image_subscriber = self.create_subscription(RosImage, image_topic, self.image_callback, 10)
        self.trigger_service = self.create_service(Trigger, '/trigger_pipeline', self.pipeline_callback)
        
        self.command_publisher_ = self.create_publisher(String, '/structured_command', 10)

        self.get_logger().info("Node is ready.")

    def image_callback(self, msg):
        with self.image_lock:
            self.latest_image_msg = msg
            if not self.image_received:
                #self.get_logger().info("First image received from camera.")
                self.image_received = True
    def publish_task_name_callback(self):
        """Timer callback to continuously publish the stored task name."""
        if self.generated_task_name:
            msg = String()
            msg.data = self.generated_task_name
            self.command_publisher_.publish(msg)
            self.get_logger().info(f"Publishing task name: '{self.generated_task_name}'")


    def _update_yaml_file(self, new_task_block: str) -> (bool, str):
        """
        Updates the task in the primary YAML file using the new task block.
        Returns a tuple of (success_boolean, message_string).
        """
        try:
            with open(self.yaml_file_path, 'r') as file:
                original_yaml = yaml.safe_load(file)

            new_task_data = yaml.safe_load(new_task_block)

            if not isinstance(new_task_data, dict) or not new_task_data:
                raise ValueError("Parsed task block is not a valid dictionary.")
            
            task_name = list(new_task_data.keys())[0]

            if "tasks" not in original_yaml:
                original_yaml["tasks"] = {}
            
            original_yaml["tasks"][task_name] = new_task_data[task_name]

            with open(self.yaml_file_path, 'w') as file:
                yaml.dump(
                    original_yaml, file, Dumper=SmartDumper,
                    default_flow_style=False, allow_unicode=True, sort_keys=False
                )
            self.get_logger().info(f"Successfully updated task '{task_name}' in {self.yaml_file_path}")
            return True, ""

        except FileNotFoundError:
            msg = f"YAML file not found at: {self.yaml_file_path}"
            self.get_logger().error(msg)
            return False, f"Error: {msg}"
        except Exception as e:
            msg = f"Failed to update YAML file: {e}"
            self.get_logger().error(msg)
            return False, f"Error: {msg}"


    def pipeline_callback(self, request, response):
        self.get_logger().info("Service triggered: Generating new task...")

        # --- Stop any previous continuous publishing ---
        if self.publishing_timer is not None:
            self.publishing_timer.cancel()
            self.get_logger().info("Stopped previous publishing timer.")

        if not self.image_received:
            self.get_logger().error("Service called but no image received.")
            response.success = False
            response.message = "Error: No image received from camera."
            return response

        # MOONDREAM
        task_text = ""
        try:
            with self.image_lock:
                local_image_msg = self.latest_image_msg
            cv_image = self.bridge.imgmsg_to_cv2(local_image_msg, "bgr8")
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            pil_image = Image.fromarray(rgb_image)
            prompt = "print the text written in the image. Don't return anything if you couldn't detect the text. if the text was only \"deafult task\" then return false"
            answer = self.moondream_model.query(pil_image, prompt)["answer"]
            task_text = answer.strip().split('\n')[-1] # to get only the last line which is the task
            self.get_logger().info(f"Moondream detected task: '{task_text}'")

            if not task_text or task_text.isspace():
                self.get_logger().info("Moondream did not find any text in the image.")
                response.success = False
                #response.message = "Error: Vision model found no text."
                return response
        except Exception as e:
            self.get_logger().error(f"Error during Moondream inference: {e}")
            response.success = False; response.message = f"Error in vision model: {e}"
            return response

        # GEMINI
        try:
            full_prompt = self.prompt_template.format(task=task_text)
            api_response = self.gemini_model.generate_content(full_prompt)
            
            structured_command = api_response.text.strip()
            if structured_command.startswith("```yaml"):
                structured_command = structured_command[7:]
            if structured_command.endswith("```"):
                structured_command = structured_command[:-3]
            structured_command = structured_command.strip()

            if structured_command.lower() == 'false':
                self.get_logger().info("Gemini deemed the request not a valid task.")
                response.success = False
                response.message = "Language model found no meaningful task."
                return response
            
            self.get_logger().info(f"Gemini generated task:\n---\n{structured_command}\n---")
            
            # --- Extract Task Name and Update YAML ---
            task_name = structured_command.split('\n')[0].rstrip(':').strip()
            self.get_logger().info(f"Gemini generated task: '{task_name}'")
            success, message = self._update_yaml_file(structured_command)
            if not success:
                response.success = False
                response.message = message
                return response
            
             # --- Store the name and start the continuous publisher ---
            self.generated_task_name = task_name
            self.publishing_timer = self.create_timer(1.0, self.publish_task_name_callback)
            self.get_logger().info(f"Starting continuous publishing of task name '{self.generated_task_name}' at 1 Hz.")

            

            # msg = String()
            # msg.data = structured_command
            # self.command_publisher_.publish(msg)
            # self.get_logger().info(f"Published command to /structured_command topic.")

            response.success = True
            response.message = f"Successfully generated task and started continuous publishing of name: {self.generated_task_name}"
        except Exception as e:
            self.get_logger().error(f"Error during Gemini processing: {e}")
            response.success = False
            response.message = f"Error in language model: {e}"

        return response

def main(args=None):
    rclpy.init(args=args)
    node = PipelineServiceNode()
    if rclpy.ok():
        rclpy.spin(node)
        node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
