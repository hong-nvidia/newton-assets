# Unitree H2 Robot Simulation Assets

## Overview

This package contains a structured USD simulation asset for the
[Unitree H2](https://www.unitree.com/h2) humanoid robot developed by
[Unitree Robotics](https://www.unitree.com/).

The subfolders contain:

- **tools**: Pinned conversion dependencies, a small compatibility patch, and validation checks.
- **usd_structured**: Simulation-ready USD layers preserving the H2 ankle,
  knee, and waist linkage closures.

## Sources

### USD (Structured)

The structured USD model was converted from Unitree's
[`H2_loop.xml`](https://github.com/unitreerobotics/unitree_ros/blob/7d6075f7f58588b189b940130e3edab3c839b2df/robots/h2_description/H2_loop.xml)
MJCF model in the `unitree_ros` repository at revision
`7d6075f7f58588b189b940130e3edab3c839b2df`.

The source was converted with
[`mujoco-usd-converter`](https://github.com/newton-physics/mujoco-usd-converter)
using its standard structured output and three small compatibility adjustments. For conversion,
provenance, and asset-entrypoint details, see the
[`usd_structured` README](usd_structured/README.md).

## Changelog

- 2026-10-06: Switched to converter 0.5.0's standard nested structure and `H2Loop.usda` entrypoint. Retained collision guide geometry and default-import effort limits, removed the source floor, custom restructuring builder, and duplicate source archive, and added source/import/simulation verification.
- 2026-09-29: Added the structured Unitree H2 USD asset with six explicit
  Newton-compatible loop joints.

## License

This model is released under a [BSD-3-Clause License](LICENSE).
