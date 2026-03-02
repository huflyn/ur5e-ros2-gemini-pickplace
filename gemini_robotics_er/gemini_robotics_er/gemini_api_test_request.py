import os
import sys
from google import genai

# Set Google API key from environment variable
GOOGLE_API_KEY = os.environ.get('GEMINI_API_KEY')

# Initialize the GenAI client
client = genai.Client(api_key=GOOGLE_API_KEY)

# Define model
MODEL_ID = "gemini-robotics-er-1.5-preview"

# Define Prompt Logic
# sys.argv[0] is the script name
# sys.argv[1:] are all arguments that come after.
if len(sys.argv) > 1:
    # Joins all words from the terminal back into a sentence (string)
    PROMPT = " ".join(sys.argv[1:])
    print(f"Using prompt: '{PROMPT}'\n---")
else:
    # Fallback / Default, if nothing was entered
    PROMPT = "Are you there?"
    print(f"Using prompt: '{PROMPT}'\nYou can enter a custom prompt as a command line argument between quotes.\n---")

# Generate content / API Call
response = client.models.generate_content(
    model=MODEL_ID,
    contents=PROMPT
)

# Print the response
print(response.text)