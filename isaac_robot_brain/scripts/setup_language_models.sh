#!/bin/bash
# setup_language_models.sh
# Script to set up language understanding models for Vision-Language-Action module

set -e  # Exit on any error

echo "Setting up language understanding models..."

# Verify OpenAI API key is set
if [ -z "$OPENAI_API_KEY" ]; then
    echo "Warning: OPENAI_API_KEY environment variable not set. Please set it before using language models."
    echo "You can set it by running: export OPENAI_API_KEY='your-api-key-here'"
fi

# Test OpenAI API connection
echo "Testing OpenAI API connection..."
python3 -c "
import openai
import os
if os.getenv('OPENAI_API_KEY'):
    openai.api_key = os.getenv('OPENAI_API_KEY')
    try:
        response = openai.ChatCompletion.create(
            model='gpt-3.5-turbo',
            messages=[{'role': 'user', 'content': 'test'}],
            max_tokens=5
        )
        print('OpenAI API connection successful!')
    except Exception as e:
        print(f'OpenAI API connection failed: {e}')
else:
    print('OpenAI API key not found. Please set OPENAI_API_KEY environment variable.')
"

# Download required NLP models
echo "Downloading language processing models..."
python3 -c "
import tiktoken
print('Loading tokenizer...')
tokenizer = tiktoken.get_encoding('cl100k_base')
print('Tokenizer loaded successfully.')
"

echo "Language understanding models setup completed!"