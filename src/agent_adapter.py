"""Provider boundary. An adapter returns text/data; it never gets robot access."""
from typing import Protocol

from gemini_client import call, output_text


class AgentAdapter(Protocol):
    def respond(self, *, key: str, model: str, thinking: str, inputs: list) -> str:
        """Return the provider's response without executing any proposed action."""
        ...


class GeminiAdapter:
    def respond(self, *, key, model, thinking, inputs):
        return output_text(call(key, '/interactions', {
            'model': model, 'store': False,
            'generation_config': {'thinking_level': thinking, 'max_output_tokens': 2048},
            'input': inputs,
        }))


def validate_practice_action(action):
    """Enforce the arm-only contract independently of model instructions."""
    if not isinstance(action, dict) or action.get('action') not in {
            'enable', 'observe', 'trial_left_arm', 'stop', 'done'}:
        raise ValueError('Action rejected: this session permits bounded left-arm practice only.')
    if action['action'] == 'trial_left_arm':
        joint = action.get('joint_id')
        if type(joint) is not int or joint not in (15, 16, 18):
            raise ValueError('Action rejected: unavailable left-arm trial joint.')
    return action
