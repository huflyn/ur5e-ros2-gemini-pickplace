# Setup Google Gemini API <!-- omit from toc -->
[![Gemini](https://img.shields.io/badge/-Gemini%20API-black?style=flat-square&logo=googlegemini)](https://ai.google.dev/gemini-api/docs)

- [Step 1: Create Gemini API Key (Docs)](#step-1-create-gemini-api-key-docs)
- [Step 2: Google GenAI SDK Installation (Docs)](#step-2-google-genai-sdk-installation-docs)
- [Step 3: Setting the API key as an environment variable (Docs)](#step-3-setting-the-api-key-as-an-environment-variable-docs)
- [Step 4: Make your first request (Docs)](#step-4-make-your-first-request-docs)


# Step 1: Create Gemini API Key ([Docs](https://ai.google.dev/gemini-api/docs/api-key))

To use the Gemini API, you need an API key. Create and manage your keys in [*Google AI Studio*](https://aistudio.google.com/app/apikey).

# Step 2: Google GenAI SDK Installation ([Docs](https://ai.google.dev/gemini-api/docs/quickstart))

Using Python 3.9+, install the google-genai package using the following pip command:

```bash
pip install -q -U google-genai
```

# Step 3: Setting the API key as an environment variable ([Docs](https://ai.google.dev/gemini-api/docs/api-key#set-api-env-var))

Bash is a common Linux and macOS terminal configuration. You can check if you have a configuration file for it by running the following command:

```bash
~/.bashrc
```

If the response is "No such file or directory", you will need to create this file and open it by running the following commands, or use zsh:

```bash
touch ~/.bashrc
open ~/.bashrc
```

Next, you need to set your API key by adding the following export command at the end of .bashrc:

```bash
export GEMINI_API_KEY=<YOUR_API_KEY_HERE>
```

After saving the file, apply the changes by running:

```bash
source ~/.bashrc
```

Verify the environment variable is set by running:

```bash
echo $GEMINI_API_KEY
```

It should print your API key.


> [!CAUTION] 
> **Keep your API key secure ([Docs](https://ai.google.dev/gemini-api/docs/api-key#security))**
> 
> Your API key is like a password that allows access to your Google Cloud resources. Keep it secure and do not share it publicly or commit it to version control systems. If you believe your API key has been compromised, revoke it immediately in the Google AI Studio and generate a new one.

# Step 4: Make your first request ([Docs](https://ai.google.dev/gemini-api/docs/quickstart#make-first-request))

Here is a simple example of how to use the Google GenAI SDK to make a request to the Gemini API with Gemini Robotics-ER 1.5 model:

```python
import os
from google import genai
from google.genai import types

# Set Google API key from environment variable
GOOGLE_API_KEY  = os.environ.get('GEMINI_API_KEY')

# Initialize the GenAI client
client = genai.Client(api_key=GOOGLE_API_KEY)

# Define model and prompt
MODEL_ID = "gemini-robotics-er-1.5-preview"
PROMPT = "Are you there?"

# Generate content / API Call
response = client.models.generate_content(
    model=MODEL_ID, 
    contents=PROMPT
)

# Print the response
print(response.text)
```

Safe the code in a file named `gemini_api_test.py` and run it using the following command:

```bash
python3 gemini_api_test.py
```

You should see a response from the Gemini Robotics-ER model printed in the terminal, similar to the following:

```text
Yes, I'm here! How can I assist you today?
```

---

Back to [README.md](README.md#setup-google-gemini-api)