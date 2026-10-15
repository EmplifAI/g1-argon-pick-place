"""Small, dependency-free Interactions client. Never expose provider error bodies."""
import json
import re
import urllib.error
import urllib.request

API = 'https://generativelanguage.googleapis.com/v1beta'


def validate_model(model):
    if not isinstance(model, str) or not re.fullmatch(r'[a-zA-Z0-9._-]{1,100}', model):
        raise ValueError('Enter a valid Gemini model ID.')
    return model


def call(key, path, data=None):
    request = urllib.request.Request(API + path,
        data=None if data is None else json.dumps(data).encode(),
        headers={'Content-Type': 'application/json', 'x-goog-api-key': key})
    try:
        with urllib.request.urlopen(request, timeout=120) as response:
            return json.load(response)
    except urllib.error.HTTPError as error:
        messages = {400: 'The model rejected this request. Check the model and input.',
                    401: 'API key rejected.', 403: 'API key or model access denied.',
                    404: 'Model unavailable for this API key.',
                    429: 'Quota exceeded. Check billing and model limits, then retry.'}
        raise ValueError(messages.get(error.code, 'Gemini request failed (HTTP %d).' % error.code)) from None
    except (OSError, ValueError):
        raise ValueError('Could not reach Gemini or read its response. Please retry.') from None


def output_text(result):
    if result.get('status') != 'completed':
        raise ValueError('Model response did not complete. No action was executed.')
    text = ''.join(part.get('text', '') for step in result.get('steps', [])
                   if step.get('type') == 'model_output'
                   for part in step.get('content', []) if part.get('type') == 'text')
    text = text or result.get('output_text', '')
    if not text.strip():
        raise ValueError('The model returned no text. No action was executed.')
    return text
