# voice_server.py

from flask import Flask, request, jsonify
from flask_cors import CORS
from dotenv import load_dotenv
from openai import OpenAI
import os
import json

# Load environment variables from .env
load_dotenv()
api_key = os.getenv("OPENAI_API_KEY")
if not api_key:
    raise RuntimeError(
        "Please set OPENAI_API_KEY in your .env file before running."
    )

# Initialize OpenAI client
client = OpenAI(api_key=api_key)

# Define the function schema—including an 'unrecognized' fallback
functions = [
    {
        "name": "interpret_command",
        "description": (
            "Map a user instruction to exactly one of the robot commands, "
            "or return 'unrecognized' if it does not match any."
        ),
        "parameters": {
            "type": "object",
            "properties": {
                "command": {
                    "type": "string",
                    "enum": [
                        "follow",
                        "return",
                        "stop",
                        "manual",
                        "autonomous",
                        "unrecognized"
                    ]
                }
            },
            "required": ["command"]
        }
    }
]

app = Flask(__name__)
CORS(app)

@app.route("/interpret", methods=["POST"])
def interpret():
    user_input = request.json.get("command", "").strip()
    print(f"Received voice command: {user_input}")
    
    lower = user_input.lower()
    if "control" in lower or "drive" in lower:
        print("Pre-mapped via keyword to: manual")
        return jsonify(result="manual")

    # System prompt now allows 'unrecognized'
    system_msg = (
        "You are a robot assistant. You will receive a single user instruction.  "
        "Your job is to call the function 'interpret_command' with exactly one of these values:\n"
        "- follow\n"
        "- return\n"
        "- stop\n"
        "- manual\n"
        "- autonomous\n"
        "- unrecognized (if the instruction does not match any robot command)\n"
        "Do not output anything else."
    )

    try:
        resp = client.chat.completions.create(
            model="gpt-3.5-turbo",
            messages=[
                {"role": "system", "content": system_msg},
                {"role": "user", "content": user_input}
            ],
            functions=functions,
            function_call={"name": "interpret_command"}
        )
    except Exception as e:
        print(f"Error calling OpenAI API: {e}")
        return jsonify(result="unrecognized")

    msg = resp.choices[0].message

    # Ensure it invoked our function
    if not hasattr(msg, "function_call") or msg.function_call.name != "interpret_command":
        print("No valid function_call returned; defaulting to unrecognized")
        return jsonify(result="unrecognized")

    # Parse the JSON arguments
    try:
        args = json.loads(msg.function_call.arguments)
        cmd = args.get("command")
    except Exception as e:
        print(f"Error parsing function_call.arguments: {e}")
        return jsonify(result="unrecognized")

    # Validate
    if cmd not in ["follow", "return", "stop", "manual", "autonomous", "unrecognized"]:
        print(f"Function returned invalid command: {cmd}")
        cmd = "unrecognized"

    print(f"GPT interpreted as: {cmd}")
    return jsonify(result=cmd)


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5001)
