#!/bin/bash
set -e

ROOT_DIR="$(cd "$(dirname "$0")" && pwd)"
VENV_DIR="$ROOT_DIR/.venv"

# Create venv if it doesn't exist
if [ ! -d "$VENV_DIR" ]; then
    python3 -m venv "$VENV_DIR"
    echo "Created virtual environment at $VENV_DIR"
fi

"$VENV_DIR/bin/pip" install --upgrade pip --quiet

"$VENV_DIR/bin/pip" install \
    numpy \
    matplotlib \
    numba \
    Pillow \
    jupyter \
    ipykernel \
    --quiet

# Register the venv as a Jupyter kernel named "RAN"
"$VENV_DIR/bin/python" -m ipykernel install --user --name ran --display-name "RAN"

echo ""
echo "Done. Launch with:"
echo "  $VENV_DIR/bin/jupyter notebook"
echo ""
echo "Select kernel 'RAN' inside the notebook."
