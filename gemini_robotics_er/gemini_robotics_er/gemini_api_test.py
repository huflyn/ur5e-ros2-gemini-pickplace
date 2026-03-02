import rclpy
from rclpy.node import Node
import os
from google import genai
from google.genai import types

class GeminiApiTestNode(Node):
    def __init__(self):
        super().__init__('gemini_api_test_node')
        
        # API Key holen (Sicherheitshalber prüfen)
        GOOGLE_API_KEY  = os.environ.get('GEMINI_API_KEY')

        if not GOOGLE_API_KEY :
            self.get_logger().error("NO API KEY FOUND! Please set 'export GEMINI_API_KEY=...' in .bashrc or .zshrc.")
            return

        self.get_logger().info("Initializing Gemini Client...")

        try:
            # Client erstellen
            client = genai.Client(api_key=GOOGLE_API_KEY)
            self.get_logger().info("Gemini Client initialized.")

            # Model definieren und Prompt senden
            MODEL_ID = "gemini-robotics-er-1.5-preview"
            self.get_logger().info(f"Model: {MODEL_ID}")

            PROMPT = "Are you there?"
            self.get_logger().info("------------------------------------------------")
            self.get_logger().info("Sending Prompt:")
            self.get_logger().info(f"{PROMPT}")
            
            # Response vom Modell holen
            response = client.models.generate_content(
                model=MODEL_ID, 
                contents=PROMPT
            )

            # Response ausgeben
            self.get_logger().info("------------------------------------------------")
            self.get_logger().info("Response:")
            self.get_logger().info(f"{response.text}")
            self.get_logger().info("------------------------------------------------")

            # Token Usage auslesen
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
    
    # Node nach 10 Sekunden beenden, da es nur ein Test ist
    try:
        rclpy.spin_once(node, timeout_sec=10)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()