#!/usr/bin/env bash
set -euo pipefail

PI_USER="${PI_USER:-aaron}"
PI_HOST="${PI_HOST:-atlas.local}"
PI_WORKSPACE="${PI_WORKSPACE:-/workspace}"

echo "Syncing install/ to ${PI_USER}@${PI_HOST}:${PI_WORKSPACE}/install/"

rsync -avz --delete \
    install/ \
    "${PI_USER}@${PI_HOST}:${PI_WORKSPACE}/install/"

echo "Payload sent to payload :)"
