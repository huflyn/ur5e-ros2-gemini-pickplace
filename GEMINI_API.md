# Setup Google Gemini API <!-- omit from toc -->
[![gemini][gemini-badge]][gemini-link]

[gemini-badge]: https://img.shields.io/badge/-GEMINI%20API-7C4DFF?style=flat-square&logo=googlegemini&logoColor=white
[gemini-link]: https://ai.google.dev/gemini-api/docs

These instructions will guide you through setting up the Google Gen AI SDK and Gemini API, which we use in our project to enable advanced language understanding and reasoning capabilities for our robotic system.

Official documentattion: [Gemini API](https://ai.google.dev/gemini-api/docs/quickstart)

---

- [I) Google Gen AI SDK](#i-google-gen-ai-sdk)
- [II) Google Gemini API Setup and Usage](#ii-google-gemini-api-setup-and-usage)
  - [Step 1: Create Gemini API Key (Docs)](#step-1-create-gemini-api-key-docs)
  - [Step 2: Google GenAI SDK Installation (Docs)](#step-2-google-genai-sdk-installation-docs)
  - [Step 3: Setting the API key as an environment variable (Docs)](#step-3-setting-the-api-key-as-an-environment-variable-docs)
  - [Step 4: Make your first request (Docs)](#step-4-make-your-first-request-docs)
- [III) Models Overview](#iii-models-overview)

---

# I) Google Gen AI SDK

Google Gen AI Python SDK provides an interface for developers to integrate Google's generative models into their Python applications

Using Python 3.9+, install the google-genai package using the following pip command:

```bash
pip install -q -U google-genai
```

---

# II) Google Gemini API Setup and Usage

To use the Gemini API, you need to create an API key, set it as an environment variable, and then you can make requests to the API using the Google GenAI SDK. Follow the steps below to get started:

## Step 1: Create Gemini API Key ([Docs](https://ai.google.dev/gemini-api/docs/api-key))

To use the Gemini API, you need an API key. Create and manage your keys in [*Google AI Studio*](https://aistudio.google.com/app/apikey).

## Step 2: Google GenAI SDK Installation ([Docs](https://ai.google.dev/gemini-api/docs/quickstart))

Using Python 3.9+, install the google-genai package using the following pip command:

```bash
pip install -q -U google-genai
```

## Step 3: Setting the API key as an environment variable ([Docs](https://ai.google.dev/gemini-api/docs/api-key#set-api-env-var))

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

## Step 4: Make your first request ([Docs](https://ai.google.dev/gemini-api/docs/quickstart#make-first-request))

Here is a simple example of how to use the Google GenAI SDK to make a request to the Gemini API with the Gemini 2.5 Flash model:

```python
import os
from google import genai
from google.genai import types

# Set Google API key from environment variable
GOOGLE_API_KEY  = os.environ.get('GEMINI_API_KEY')

# Initialize the GenAI client
client = genai.Client(api_key=GOOGLE_API_KEY)

# Define model and prompt
MODEL_ID = "gemini-2.5-flash"
PROMPT = "Are you there?"

# Generate content / API Call
response = client.models.generate_content(
    model=MODEL_ID, 
    contents=PROMPT
)

# Print the response
print(response.text)
```

If everything is set up correctly, you should see a response from the Gemini 3 Flash model printed in the terminal, similar to the following:

```text
Yes, I'm here! How can I assist you today?
```

---

# III) Models Overview

The Gemini API provides access to a range of powerful language models, each designed for specific use cases, and with different request limits (you can see your current usage and limits in [Google AI Studio](https://aistudio.google.com/app/rate-limit)).

Following models are recommended for this project (description from official Gemini API [Model Overview](https://ai.google.dev/gemini-api/docs/models)):

| Model | Description | RPD (Free tier) | Latest Update* |
| --- | --- | :---: | :---: |
| [gemini-3-flash-preview](https://ai.google.dev/gemini-api/docs/models/gemini-3-flash-preview) | The best model in the world for multimodal understanding, and our most powerful agentic and vibe-coding model yet, delivering richer visuals and deeper interactivity, all built on a foundation of state-of-the-art reasoning. | 20 | December 2025 |
| [gemini-3.1-flash-lite-preview](https://ai.google.dev/gemini-api/docs/models/gemini-3.1-flash-lite-preview) | Our most cost-efficient multimodal model, offering the fastest performance for high-frequency, lightweight tasks. Gemini 3.1 Flash-Lite is best for high-volume agentic tasks, simple data extraction, and extremely low-latency applications where budget and speed are the primary constraints. | 500 | March 2026 |
| [gemini-robotics-er-1.6-preview](https://ai.google.dev/gemini-api/docs/models/gemini-robotics-er-1.6-preview) | Gemini Robotics-ER 1.6 is a vision-language model (VLM) that brings Gemini's agentic capabilities to robotics. It's designed for advanced reasoning in the physical world, allowing robots to interpret complex visual data, perform spatial reasoning, and plan actions from natural language commands. | 20 | September 2025 |

**Information from March 2026, please refer to the official [Gemini API Models documentation](https://ai.google.dev/gemini-api/docs/models) for the most up-to-date information on available models and their capabilities.*

---

Back to [II) Installation and Setup](/README.md#ii-installation-and-setup)