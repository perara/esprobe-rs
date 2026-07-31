#!/usr/bin/env bash
set -euo pipefail

firmware_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
secrets_file="${firmware_dir}/.env.local"

if [[ ! -f "${secrets_file}" ]]; then
    echo "missing ${secrets_file}; copy wifi.env.example and set WIFI_PASSWORD" >&2
    exit 2
fi

set -a
# shellcheck disable=SC1090
source "${secrets_file}"
set +a

: "${WIFI_SSID:?WIFI_SSID is required}"
: "${WIFI_PASSWORD:?WIFI_PASSWORD is required}"
: "${CONTROL_AP_SSID:?CONTROL_AP_SSID is required}"
: "${CONTROL_AP_PASSWORD:?CONTROL_AP_PASSWORD is required}"

if (( ${#CONTROL_AP_PASSWORD} < 8 )); then
    echo "CONTROL_AP_PASSWORD must be at least 8 characters for WPA2" >&2
    exit 2
fi

# bindgen 0.71 (used by the current esp-rs release) misreads forward-declared
# structs with libclang 22. Prefer an installed Android NDK libclang when the
# host distribution has already moved to LLVM 22.
if [[ -z "${LIBCLANG_PATH:-}" ]] && command -v clang >/dev/null; then
    clang_major="$(clang --version | sed -nE 's/.*version ([0-9]+).*/\1/p' | head -n1)"
    if [[ -n "${clang_major}" ]] && (( clang_major >= 22 )); then
        shopt -s nullglob
        libclang_candidates=()
        if [[ -n "${ANDROID_NDK_HOME:-}" ]]; then
            libclang_candidates+=("${ANDROID_NDK_HOME}/toolchains/llvm/prebuilt/linux-x86_64/lib/libclang.so")
        fi
        if [[ -n "${ANDROID_SDK_ROOT:-}" ]]; then
            libclang_candidates+=("${ANDROID_SDK_ROOT}"/ndk/*/toolchains/llvm/prebuilt/linux-x86_64/lib/libclang.so)
        fi
        libclang_candidates+=(/opt/android-sdk/ndk/*/toolchains/llvm/prebuilt/linux-x86_64/lib/libclang.so)
        for candidate in "${libclang_candidates[@]}"; do
            if [[ -f "${candidate}" ]]; then
                export LIBCLANG_PATH
                LIBCLANG_PATH="$(dirname "${candidate}")"
                echo "using LLVM-compatible libclang from ${LIBCLANG_PATH}" >&2
                break
            fi
        done
        if [[ -z "${LIBCLANG_PATH:-}" ]]; then
            echo "libclang 22 is incompatible with esp-rs bindgen 0.71; set LIBCLANG_PATH to libclang 21 or older" >&2
            exit 2
        fi
    fi
fi

cd "${firmware_dir}"
cargo build --release
