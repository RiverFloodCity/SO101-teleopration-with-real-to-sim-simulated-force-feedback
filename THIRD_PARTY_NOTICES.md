Third-Party Notices
===================

This repository includes or adapts code, data, and assets from third-party
projects. This file documents the known third-party components identified in
the repository at the time of publication.

This notice file is provided for attribution and compliance support only. It is
not legal advice.


1. Hugging Face LeRobot
-----------------------

This repository includes code derived from or adapted from the Hugging Face
LeRobot project.

Upstream project:
- Name: LeRobot
- Copyright: Copyright 2024 The HuggingFace Inc. team
- License: Apache License 2.0

Relevant files in this repository include:
- `hardware/common/motors/motors_bus.py`
- `hardware/common/motors/feetech/feetech.py`
- `hardware/common/motors/feetech/tables.py`

In addition, the SO101 motor/joint range conventions in:
- `mapping/so101_mapping.py`

were copied from or aligned with LeRobot-related conventions, as documented in
the source comments.

The original license headers present in the derived files should be retained.
If this repository is redistributed, the Apache 2.0 license text for the
relevant upstream code should also be preserved in the repository.


2. Feetech / SCServo SDK
------------------------

This repository depends at runtime on the Python package:
- `feetech-servo-sdk`

This dependency provides `scservo_sdk`, which is used by:
- `hardware/common/motors/feetech/feetech.py`

This repository does not vendor the SDK source code directly. Users should
refer to the upstream package and its license terms when installing or
redistributing that dependency.


3. Feetech documentation references
-----------------------------------

This repository includes control-table and protocol-related values based on
Feetech documentation, including references embedded in:
- `hardware/common/motors/feetech/tables.py`

Relevant documentation URLs are cited in source comments.


4. MuJoCo model and mesh assets
-------------------------------

This repository contains MuJoCo XML and mesh assets under `assets/models/`.

Known provenance markers in the repository:
- `assets/models/so101_new_calib.xml` contains the comment
  "Generated using onshape-to-robot"
- the same file references an Onshape document URL
- the same file cites a GitHub URL for gain-calculation context

The repository maintainer should verify that they have the right to publicly
redistribute:
- the generated XML model
- the STL mesh files under `assets/models/assets/`
- any CAD-derived geometry exported from Onshape or other upstream sources

Where asset provenance or licensing is uncertain, do not assume the assets are
covered by the repository's top-level software license.


5. External Python dependencies
-------------------------------

This repository declares external dependencies in `requirements.txt`,
including:
- `mujoco`
- `numpy`
- `pyserial`
- `tqdm`
- `deepdiff`
- `feetech-servo-sdk`

These packages are not bundled into this repository unless separately
committed. Their licenses remain governed by their respective upstream
projects.


Repository maintainer note
--------------------------

If you publish this repository publicly, the safest maintenance practice is:
- keep original upstream copyright/license headers intact
- include a top-level `LICENSE` file for your repository
- keep this `THIRD_PARTY_NOTICES.md` file
- verify redistribution rights for all robot meshes and CAD-derived assets
