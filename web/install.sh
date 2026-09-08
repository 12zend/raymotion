#!/usr/bin/env bash
set -euo pipefail

main() {
    if [[ $(uname -s) != Darwin ]]; then
        echo 'raymotion requires macOS.' >&2
        return 1
    fi
    local arch
    case $(uname -m) in
        arm64) arch=arm64 ;;
        x86_64) arch=x64 ;;
        *) echo 'Unsupported CPU architecture.' >&2; return 1 ;;
    esac
    local command
    for command in curl tar shasum python3; do
        command -v "$command" >/dev/null || { echo "Required command: $command" >&2; return 1; }
    done
    python3 -c 'import sys; sys.exit(sys.version_info < (3, 9))' || {
        echo 'Python 3.9 or later is required.' >&2; return 1;
    }

    local prefix="${RAYMOTION_INSTALL_DIR:-$HOME/.local/share/raymotion}"
    local bin_dir="${RAYMOTION_BIN_DIR:-$HOME/.local/bin}"
    if [[ $prefix != /* || $bin_dir != /* ]]; then
        echo 'Install directories must be absolute paths.' >&2
        return 1
    fi
    if [[ -e "$bin_dir/raymotion" && ! -L "$bin_dir/raymotion" ]]; then
        echo "Refusing to replace an existing file: $bin_dir/raymotion" >&2
        return 1
    fi
    local archive="raymotion-macos-$arch.tar.gz"
    local base='https://github.com/12zend/raymotion/releases/latest/download'
    local work
    work=$(mktemp -d)
    trap 'rm -rf "$work"' EXIT
    # Resolve latest once so the archive and checksum come from the same release.
    local checksum_url
    checksum_url=$(curl -fsSL -w '%{url_effective}' -o "$work/release-url" \
        'https://github.com/12zend/raymotion/releases/latest')
    local tag="${checksum_url##*/}"
    [[ $tag == v* ]] || { echo 'Could not resolve the latest release.' >&2; return 1; }
    base="https://github.com/12zend/raymotion/releases/download/$tag"
    curl -fsSL "$base/$archive" -o "$work/$archive"
    curl -fsSL "$base/SHA256SUMS" -o "$work/SHA256SUMS"
    (cd "$work"; awk -v name="$archive" '$2 == name {print}' SHA256SUMS > selected.sha256
        [[ $(wc -l < selected.sha256) -eq 1 ]] || { echo 'Missing or duplicate checksum.' >&2; exit 1; }
        shasum -a 256 -c selected.sha256)
    tar -xzf "$work/$archive" -C "$work"
    [[ -f "$work/raymotion/bin/raymotion" && \
       -f "$work/raymotion/lib/libraymotion_engine.a" && \
       -d "$work/raymotion/include/raymotion" && \
       -f "$work/raymotion/share/raymotion/compiler/raymotion_compiler.py" ]] || {
        echo 'Incomplete release archive.' >&2; return 1;
    }
    # Publish a complete runtime through an atomic symlink replacement.
    python3 - "$work/raymotion" "$prefix" "$bin_dir" <<'PY'
import os
from pathlib import Path
import shutil
import sys
import tempfile

source, prefix, bin_dir = map(Path, sys.argv[1:])
if prefix.exists() and not prefix.is_symlink():
    sys.exit(f'Refusing to replace an existing directory: {prefix}')
prefix.parent.mkdir(parents=True, exist_ok=True)
bin_dir.mkdir(parents=True, exist_ok=True)
runtime = Path(tempfile.mkdtemp(prefix='.raymotion-', dir=prefix.parent))
try:
    shutil.copytree(source, runtime, dirs_exist_ok=True)
    def link(target, destination):
        staging = Path(tempfile.mkdtemp(prefix='.raymotion-link-', dir=destination.parent))
        try:
            (staging / 'link').symlink_to(target)
            os.replace(staging / 'link', destination)
        finally:
            shutil.rmtree(staging)
    link(runtime, prefix)
except BaseException:
    shutil.rmtree(runtime)
    raise
link(prefix / 'bin/raymotion', bin_dir / 'raymotion')
PY
    echo "Installed raymotion $tag in $prefix"
    case ":$PATH:" in
        *":$bin_dir:"*) ;;
        *) printf 'Add this to your shell configuration:\nexport PATH="%s:$PATH"\n' "$bin_dir" ;;
    esac
    rm -rf "$work"
    trap - EXIT
}

main "$@"
