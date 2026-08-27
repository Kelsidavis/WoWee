#!/bin/bash
set -euo pipefail

APP_PATH="${1:?usage: verify_signature.sh <app> [identity] [entitlement]}"
IDENTITY="${2:--}"
REQUIRED_ENTITLEMENT="${3:-}"

codesign --verify --deep --strict --verbose=2 "${APP_PATH}"

# A missing entitlement is not a signing error, so nothing above catches one.
# It surfaces at runtime, on the machine of whoever downloaded the release, as
# whatever the denied capability does when refused, and what
# pthread_jit_write_protect_np does without com.apple.security.cs.allow-jit is
# trap the process. Ad-hoc signatures carry no entitlements and want none:
# without the hardened runtime there is nothing to ask for.
if [ -n "${REQUIRED_ENTITLEMENT}" ] && [ "${IDENTITY}" != "-" ]; then
    granted="$(mktemp -t entitlements)"
    trap 'rm -f "${granted}"' EXIT
    # Older codesign writes a binary blob with a header on it and newer ones
    # write XML; plutil normalizes both, and reads the key rather than grepping
    # for it, so an entitlement present and set to false still fails.
    codesign --display --entitlements - --xml "${APP_PATH}" 2>/dev/null \
        | plutil -convert xml1 -o "${granted}" - 2>/dev/null || true
    value="$(/usr/libexec/PlistBuddy -c "Print :${REQUIRED_ENTITLEMENT}" \
        "${granted}" 2>/dev/null || true)"
    if [ "${value}" != "true" ]; then
        echo "ERROR: ${APP_PATH} is missing ${REQUIRED_ENTITLEMENT}" >&2
        exit 1
    fi
fi

checked=0
while IFS= read -r -d '' component; do
    if ! file -b "${component}" | grep -q 'Mach-O'; then
        continue
    fi

    checked=$((checked + 1))
    codesign --verify --strict --verbose=2 "${component}"

    if [ "${IDENTITY}" != "-" ]; then
        details="$(codesign --display --verbose=4 "${component}" 2>&1)"
        if ! grep -Fq "Authority=${IDENTITY}" <<<"${details}"; then
            echo "ERROR: ${component} is not signed by ${IDENTITY}" >&2
            exit 1
        fi
        if ! grep -Eq '^CodeDirectory .* flags=.*\(runtime\)' <<<"${details}"; then
            echo "ERROR: ${component} is missing the hardened runtime flag" >&2
            exit 1
        fi
    fi
done < <(find "${APP_PATH}/Contents" -type f -print0)

if [ "${checked}" -eq 0 ]; then
    echo "ERROR: no signed Mach-O components found in ${APP_PATH}" >&2
    exit 1
fi

echo "Verified ${checked} signed Mach-O component(s) in ${APP_PATH}."
