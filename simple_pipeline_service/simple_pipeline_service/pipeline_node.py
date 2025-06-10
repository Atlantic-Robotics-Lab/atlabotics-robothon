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

class PipelineServiceNode(Node):
    def __init__(self):
        super().__init__('pipeline_service_node')
        self.get_logger().info("Simple Pipeline Service Node is starting up...")

       
        
        moondream_key = "" #To be added after registration from Moondream
        google_key = ""

        
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
        self.prompt_template = (
            "I have a 5x5 grid with rows labeled a through e (in small letters) and columns labeled 1 through 5. Seven elements are positioned on this grid at the following coordinates:" 
            "A: (d, 2)"
            "B: (d, 4)"
            "Background: (b, 3)"
            "sq_up: (a, 3)"
            "sq_down: (c, 3)"
            "sq_left: (b, 2)"
            "sq_right: (b, 4)"
            "Your task is to determine the minimal trajectory (shortest path) between a given start and end element. You only need to provide the starting and ending points for each requested path."
            "For tasks like tap or similar (like press), add at the start the number of taps or -1 if long tap. For other actions like swipe keep 0"
            "if the text that you recive is meaningless or doesn't represnt a relatiev task return just the word false. Also if you"
            "only reacive deafult task without the task description return false."
            "remeber that you only deal with the elements that we defined, for any new element return also false."
            "Here are some example of input output relation:"
            "in: task is go left, out: 0, sq_left, sq_right"
            "in: task is swipe down, out: 0, sq_up, sq_down"
            "in: task is move from A to B and back, out: 0, A, B, A"
            "in: task is long press B, out: -1, B"
            "in: task is double tap Background, out: 2, Background"
            "Now, if the task is \"{task}\", give me the output."
        )

        
        self.bridge = CvBridge()
        self.image_lock = threading.Lock()
        self.latest_image_msg = None
        self.image_received = False

        image_topic = '/camera/camera/color/image_raw'
        self.image_subscriber = self.create_subscription(RosImage, image_topic, self.image_callback, 10)
        self.trigger_service = self.create_service(Trigger, '/trigger_pipeline', self.pipeline_callback)

        self.command_publisher_ = self.create_publisher(String, 'structured_command', 10)

        self.get_logger().info("Node is ready.")

    def image_callback(self, msg):
        with self.image_lock:
            self.latest_image_msg = msg
            if not self.image_received:
                #self.get_logger().info("First image received from camera.")
                self.image_received = True

    def pipeline_callback(self, request, response):
        self.get_logger().info("service call trigger")
        
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
            prompt = "print the text written in the image. Don't return anything if you couldn't detect the text."
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
            structured_command = api_response.text.strip().replace('\n', '')
            cleaned_command = api_response.text.lower().strip().strip("'\"")
            #self.get_logger().info(f"Taks is: '{structured_command}'")
            if cleaned_command == 'false':
                self.get_logger().info("heerere.")
                response.success = True
                response.message = "Error: language model found no meaningful task."
                msg = String()
                msg.data = "0, A, B"
                self.command_publisher_.publish(msg)
                self.get_logger().info(f"Published command to /structured_command topic.")
                
                return response
            
            self.get_logger().info(f"Taks is: '{structured_command}'")

            msg = String()
            msg.data = structured_command
            self.command_publisher_.publish(msg)
            self.get_logger().info(f"Published command to /structured_command topic.")

            response.success = True
            response.message = structured_command
        except Exception as e:
            self.get_logger().error(f"Error during Gemini inference: {e}")
            response.success = False; response.message = f"Error in language model: {e}"

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
