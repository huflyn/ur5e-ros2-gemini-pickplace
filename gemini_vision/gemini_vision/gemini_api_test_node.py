import rclpy
from rclpy.node import Node
import os
from google import genai
from google.genai import types

class GeminiApiTestNode(Node):
    '''A ROS2 node that demonstrates how to use the Gemini API to send a prompt and print the response including token usage information.'''
    def __init__(self):
        super().__init__('gemini_api_test_node')
        
        # Get the API key from environment variable
        GOOGLE_API_KEY  = os.environ.get('GEMINI_API_KEY')

        if not GOOGLE_API_KEY :
            self.get_logger().error("NO API KEY FOUND! Please set 'export GEMINI_API_KEY=...' in .bashrc or .zshrc.")
            return

        self.get_logger().info("Initializing Gemini Client...")

        try:
            # Create the Gemini client with the API key
            client = genai.Client(api_key=GOOGLE_API_KEY)
            self.get_logger().info("Gemini Client initialized.")

            # Define model and send prompt
            MODEL_ID = "gemini-2.5-flash" # Example model, adjust based on availability
            self.get_logger().info(f"Model: {MODEL_ID}")

            PROMPT = "Are you there?"
            self.get_logger().info("------------------------------------------------")
            self.get_logger().info("Sending Prompt:")
            self.get_logger().info(f"{PROMPT}")
            
            # Get response from the model
            response = client.models.generate_content(
                model=MODEL_ID, 
                contents=PROMPT
            )

            # Print the response
            self.get_logger().info("------------------------------------------------")
            self.get_logger().info("Response:")
            self.get_logger().info(f"{response.text}")
            self.get_logger().info("------------------------------------------------")

            # Token usage information
            if response.usage_metadata:
                input_tokens = response.usage_metadata.prompt_token_count
                output_tokens = response.usage_metadata.candidates_token_count
                total_tokens = response.usage_metadata.total_token_count
                
                self.get_logger().info(f"Input Tokens:  {input_tokens}")
                self.get_logger().info(f"Output Tokens: {output_tokens}")
                self.get_logger().info(f"Total Tokens:  {total_tokens}")


        except Exception as e:
            self.get_logger().error(f"Error during request: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = GeminiTestNode()
    
    # Keep the node alive to allow time for the API response to be printed before shutting down after 10 seconds.
    try:
        rclpy.spin_once(node, timeout_sec=10)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()