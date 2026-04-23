#!/bin/bash
# check-no-thermal-port-writes.sh — repo guard against MikroTik SFP port
# write commands targeting thermal cages.
#
# WHY: 2026-04-22/2026-04-23 lost two A6701 units (`:20`, `:4C`). Both
# preceded by sequences that included MikroTik `/interface/ethernet/set
# sfp28-N disabled=...` writes against the thermal cages (sfp28-2 thermal2,
# sfp28-15 thermal1). Memory `feedback_a6701_pleora_flap_state.md` is
# explicit:
#
#   "Don't bounce the MikroTik port to 'help the link come up'.
#    The iPORT's PHY is confused; a switch-side bounce adds one more
#    transition to the exact sequence that put it into the fault state."
#
# This guard fails if any tracked file in the repo contains a string that
# looks like a write to a thermal SFP port. Extension-tools may add
# port-writes against non-thermal cages (lucid, blackfly, ouster, NAS) —
# those are allowed. Only sfp28-2 / sfp28-3 / sfp28-4 / sfp28-15 (the four
# thermal cages per CLAUDE.md §2 topology table) trigger the guard.
#
# Run manually: `bash scripts/check-no-thermal-port-writes.sh`
# Or as pre-commit hook: `ln -s ../../scripts/check-no-thermal-port-writes.sh
#                                .git/hooks/pre-commit`

set -u
cd "$(dirname "$0")/.."

THERMAL_PORTS='sfp28-2|sfp28-3|sfp28-4|sfp28-15'
BAD_PATTERN="(/interface/ethernet/set|interface/ethernet/disable|interface/ethernet/enable|set sfp28-).*($THERMAL_PORTS).*(disabled|enabled|reset)"

# Files allowed to contain references for documentation / refusal-message
# purposes. The guard is for *write commands*, not mentions in comments.
ALLOWLIST_FILES=(
  "scripts/check-no-thermal-port-writes.sh"  # this file
  "scripts/thermal_pleora_recover.sh"        # contains the refusal message
  "docs/CHANGELOG.md"                         # historical record
)

echo "[guard] scanning repo for thermal-cage SFP port-write commands..."
hits=$(git grep -nE "$BAD_PATTERN" 2>/dev/null || true)

# Filter out allowlisted files (refusal messages + historical docs)
filtered=""
while IFS= read -r line; do
  [ -z "$line" ] && continue
  file="${line%%:*}"
  skip=0
  for allowed in "${ALLOWLIST_FILES[@]}"; do
    if [ "$file" = "$allowed" ]; then skip=1; break; fi
  done
  [ "$skip" -eq 0 ] && filtered="${filtered}${line}\n"
done <<< "$hits"

if [ -n "$filtered" ]; then
  echo
  echo "FAIL: thermal-cage SFP port-write commands found:"
  echo
  echo -e "$filtered"
  echo
  echo "Per memory feedback_a6701_pleora_flap_state.md:"
  echo "  'Don't bounce the MikroTik port to help the link come up.'"
  echo
  echo "Two A6701 cameras (\$100k each) bricked following this exact pattern."
  echo "Resolve by removing the offending command or moving the script to"
  echo "the allowlist if it is documented refusal-message text."
  exit 1
fi

echo "[guard] OK — no thermal-cage SFP port-writes found in tree"
exit 0
