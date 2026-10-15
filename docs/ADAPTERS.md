# Agent adapters

`src/agent_adapter.py` defines the provider boundary. The web observation path
uses `Console.adapter`, initially a `GeminiAdapter`. A replacement implements:

```python
class MyAdapter:
    def respond(self, *, key, model, thinking, inputs) -> str:
        # Translate text/image inputs into your provider's request schema.
        # Return text only. Do not execute proposals or pass robot handles.
        ...
```

Inject your implementation by setting `console.adapter = MyAdapter()` in your
launcher. The built-in connection UI validates Gemini keys with Google's model
endpoint; another provider also needs its own credential validation and UI model
choices. This is an extension interface, **not shipped Claude/OpenAI support**.

The hardware practice controller currently uses Gemini directly. Its action
validation is provider-independent (`validate_practice_action`) but swapping its
inference transport still requires integration work. Accepted practice actions
are enable, observe, trial_left_arm, stop and done. A model cannot unlock the
Cartesian or hand tools through its response. The robot-local gateway remains
the authority for generation, boot ID, expiry, joint and velocity checks.

No fabricated benchmark comparisons are included. Contributions implementing
other providers should include mocked transport tests and comparable recorded
scenes before attempting real hardware.
