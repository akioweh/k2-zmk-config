#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "$0")"

uv venv --python '3.14+gil' --allow-existing .venv
uv pip install west==1.5.0
export PATH="$PWD/.venv/bin:$PATH"

mkdir -p .build/manifest
cp west.yml .build/manifest/west.yml
if [[ ! -d .build/.west ]]; then
    west init -l .build/manifest
fi
(cd .build && west update)
uv pip install -r .build/zephyr/scripts/requirements-base.txt
