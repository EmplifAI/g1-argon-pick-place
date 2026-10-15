# Launch copy

Drafts only; these have not been posted.

## X

We open-sourced our Unitree G1 × Gemini Argon experiment.

A bottle carried upright on real hardware. An English, bring-your-own-key console you can try without a robot.

Apache-2.0. Still a lab experiment—help us make it reproducible.

https://github.com/EmplifAI/g1-argon-pick-place

Follow-up: full hardware video https://x.com/OsoneHiroyuki/status/2110614582570164644

## Hacker News

Title: Show HN: A local BYOK console for Unitree G1 and Gemini Argon (Apache-2.0)

I'm Hiroyuki Osone from EmplifAI. We are opening up our experiment connecting a
language model to a suspended Unitree G1's left arm and Inspire hand. The public
video shows our lab work on keeping a bottle upright during transport.

The repo includes an English local web console: run one Python command, enter a
Gemini key, and inspect an image without robot dependencies. Hardware integration
is a separate path, with bounded left-arm practice and robot-local command checks.
The published console does not expose the older full pick-and-place macro.

This is an open experiment, not a general manipulation policy. We have not
published a success-rate benchmark, independent reproduction, or calibrated
RGB-D dataset. We'd especially welcome help with calibration, recorded-scene
evaluation, installation reports and other agent adapters.

Code: https://github.com/EmplifAI/g1-argon-pick-place
Video: https://x.com/OsoneHiroyuki/status/2110614582570164644

## Distribution notes

Share the same evidence and limitations with Unitree's community, robotics
researchers and ROS Discourse where the community's posting rules permit it.
Lead with the actual video, then the one-command observation experience. Do not
claim to be the first, the definitive stack, or to have measured reliability.
Link EmplifAI for deployment support and custom task work; the code stays open
for commercial use. A hosted evaluation service is a future direction, not an
available product. Ask adopters to link reproductions; never invent adoption,
star, citation or benchmark figures.
