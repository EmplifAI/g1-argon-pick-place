# Contributing

Help turn a lab experiment into a reproducible reference implementation.
Useful first contributions: another agent adapter, recorded-scene evaluations,
installation reports, calibration tools, and documented failure cases.

Open an issue describing your hardware and expected behavior. Include the commit,
OS, Python version, dependency versions, and whether the result is simulated,
mocked, or observed on real hardware. Do not publish keys or private sensor data.

## Developer Certificate of Origin

All new contributions must include a `Signed-off-by: Your Name <your@email>`
trailer, certifying the [Developer Certificate of Origin 1.1](https://developercertificate.org/).
Use `git commit -s`. Only sign contributions you have the right to submit,
including any required employer authorization. DCO does not transfer copyright.
Contributions are accepted under Apache-2.0. Preserve upstream notices.

## Checks

```sh
python3 -m unittest discover -s tests -p 'test_web*.py' -v
python3 -m compileall -q src scripts
node --check web/app.js
```

For the full offline robot suite, install the robot dependencies and download
its pinned model as described in [hardware setup](docs/SETUP.md), then run
`python -m unittest discover -s tests -v`.

Model output is untrusted. Adapters must never execute arbitrary code or call
hardware directly. Keep left-only isolation, stop generations, stale-state
rejection, command expiry and bounded trajectories enforced outside the model.
Do not bypass these gates to make a demo succeed. A test pass is not a hardware
safety certification. Changes to physical control require a documented, supervised
hardware review before use.
