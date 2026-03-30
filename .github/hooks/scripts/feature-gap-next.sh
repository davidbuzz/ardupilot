#!/usr/bin/env bash
# feature-gap-next.sh — Stop event hook for the Pico2 / RP2350 ArduPilot port.
#
# Purpose: prevent the agent from stopping and asking "what should I do next?"
# Instead, scan FEATURE_GAP.md for the first ❌ (not done / not tested) item
# and inject it as a systemMessage so the agent continues autonomously.
#
# Called by: .github/hooks/feature-gap-continue.json on the Stop event.
# stdin:     JSON event payload from the agent runtime (not used).
# stdout:    JSON {continue: true, systemMessage: "..."} when work remains,
#            or nothing (exit 0) when all items are complete.

set -euo pipefail

FEATURE_GAP="libraries/AP_HAL_ChibiOS/hwdef/Pico2/FEATURE_GAP.md"

# Navigate to the repository root (hook may be invoked from any cwd).
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../../../" && pwd)"
cd "$REPO_ROOT"

# If FEATURE_GAP.md doesn't exist (e.g. different workspace), let agent stop.
if [[ ! -f "$FEATURE_GAP" ]]; then
    exit 0
fi

# Scan the markdown table for the first row whose Status or "Tested in hardware"
# column contains ❌. Rows look like:
#   | Category | Feature | CubeBlack | Pico2 | Status | Tested in hardware | Notes |
next_feature=""
next_notes=""

while IFS= read -r line; do
    # Only consider pipe-delimited table rows containing ❌.
    if [[ "$line" == *"❌"* && "$line" == *"|"* ]]; then
        # Field 3 = Feature, last non-empty field = Notes.
        feature=$(echo "$line" | awk -F'|' '{
            gsub(/^[[:space:]]+|[[:space:]]+$/, "", $3); print $3
        }')
        notes=$(echo "$line" | awk -F'|' '{
            # walk fields right-to-left to find the last non-empty one
            for (i=NF; i>1; i--) {
                gsub(/^[[:space:]]+|[[:space:]]+$/, "", $i)
                if ($i != "") { print $i; break }
            }
        }')

        # Skip the header row.
        if [[ -n "$feature" && "$feature" != "Feature" ]]; then
            next_feature="$feature"
            next_notes="$notes"
            break
        fi
    fi
done < "$FEATURE_GAP"

# If nothing is marked ❌, all done — let the Stop proceed normally.
if [[ -z "$next_feature" ]]; then
    exit 0
fi

# Build the injection message and return it to the agent runtime.
# continue:true prevents the Stop and resumes the agent with the new context.
python3 - "$next_feature" "$next_notes" << 'PYEOF'
import json, sys

feature = sys.argv[1]
notes   = sys.argv[2]

message = (
    "DO NOT STOP — FEATURE_GAP.md still has unactioned items.\n\n"
    f"Next task: **{feature}**\n"
    f"Notes from FEATURE_GAP.md: {notes}\n\n"
    "Action required:\n"
    "1. Read libraries/AP_HAL_ChibiOS/hwdef/Pico2/FEATURE_GAP.md to understand "
    "the full context and current hardware status for this item.\n"
    "2. Plan and implement the work needed to move this item from ❌ to ✅.\n"
    "3. Test on hardware via OpenOCD/GDB as described in CLAUDE.md and AGENTS.md.\n"
    "4. Update FEATURE_GAP.md with the new status and hardware results.\n"
    "5. Git-commit the changes with a descriptive WIP message.\n"
    "6. After completing this item, the Stop hook will fire again and find the "
    "next ❌ item automatically — keep going.\n\n"
    "Do not ask the user what to do next. They built this list for you."
)

print(json.dumps({"continue": True, "systemMessage": message}))
PYEOF
