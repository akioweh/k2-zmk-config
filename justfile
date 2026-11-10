set default-list

export ZEPHYR_TOOLCHAIN_VARIANT := env("ZEPHYR_TOOLCHAIN_VARIANT", "zephyr")

# initialize build directory & update dependencies (run after changing west.yml)
setup:
    #!/usr/bin/env bash
    set -euo pipefail
    uv venv --python '3.14+gil' --allow-existing .venv
    uv pip install west==1.5.0

    mkdir -p .build/manifest
    cp west.yml .build/manifest/west.yml
    if [[ ! -d .build/.west ]]; then
        uv run west init -l .build/manifest
    fi
    (cd .build && uv run west update)
    uv pip install -r .build/zephyr/scripts/requirements-base.txt

# build both halves (-p: pristine build)
build *args: (build-left args) (build-right args)

# build left half (-p: pristine build)
build-left *args: (_build "left" args)

# build right half (-p: pristine build)
build-right *args: (_build "right" args)

_build half *args:
    cd .build && uv run west build zmk/app -d 'build/{{ half }}' -b xiao_ble//zmk {{ args }} -- \
        '-DSHIELD=toucan_{{ half }} rgbled_adapter' \
        '-DZMK_CONFIG={{ justfile_directory() }}/config' \
        '-DZMK_EXTRA_MODULES={{ justfile_directory() }}'

# draw keymap
draw:
    uv run --no-project --python '3.14+gil' --with keymap-drawer==0.23.0 \
        keymap -c keymap_drawer.config.yaml parse -z config/toucan.keymap \
        > keymap-drawer/toucan.yaml
    uv run --no-project --python '3.14+gil' --with keymap-drawer==0.23.0 \
        keymap -c keymap_drawer.config.yaml draw \
        -d hardware/boards/shields/toucan/toucan.dtsi \
        keymap-drawer/toucan.yaml > keymap-drawer/toucan.svg
