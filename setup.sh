#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "$0")"

uv venv --python '3.14+gil' --allow-existing .venv
uv pip install west==1.5.0

mkdir -p .build/manifest
cp west.yml .build/manifest/west.yml
if [[ ! -d .build/.west ]]; then
    uv run west init -l .build/manifest
fi
(cd .build && uv run west update)
uv pip install -r .build/zephyr/scripts/requirements-base.txt
