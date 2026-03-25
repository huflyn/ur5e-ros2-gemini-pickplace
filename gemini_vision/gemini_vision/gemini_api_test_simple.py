''' A simple script to demonstrate how to use the Gemini API by sending a prompt and printing the response. '''

import os
from google import genai
from google.genai import types

# Set Google API key from environment variable
GOOGLE_API_KEY  = os.environ.get('GEMINI_API_KEY')

# Initialize the GenAI client
client = genai.Client(api_key=GOOGLE_API_KEY)

# Define model and prompt
MODEL_ID = "gemini-2.5-flash" # Example model, adjust based on availability
PROMPT = "Are you there?"

# Generate content / API Call
response = client.models.generate_content(
    model=MODEL_ID, 
    contents=PROMPT
)

# Print the response
print(response.text)