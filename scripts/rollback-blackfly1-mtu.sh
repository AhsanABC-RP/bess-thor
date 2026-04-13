#!/usr/bin/env bash
# Rollback script for blackfly1 MTU investigation (2026-04-12)
#
# Reverts:
#   1. MikroTik sfp28-5/9/11/13/15 L2MTU back to original (1584 for -9/11/13/15, 1584 for -5)
#   2. docker-compose.thor.yml blackfly1 back to packet_size=1400, no user_set_load
#   3. config/blackfly_s_thor.yaml back to subset without UserSet params
#
# Usage: sudo bash /home/thor/bess/scripts/rollback-blackfly1-mtu.sh
#        Add --mikrotik-only to skip compose/yaml changes
#        Add --dry-run to print without applying

set -euo pipefail

MIKROTIK_IP="169.254.100.254"
MIKROTIK_PW='8RKUP2PUT9'
BESS_DIR="/home/thor/bess"

DRY_RUN=0
MIKROTIK_ONLY=0
for arg in "$@"; do
  case "$arg" in
    --dry-run) DRY_RUN=1 ;;
    --mikrotik-only) MIKROTIK_ONLY=1 ;;
  esac
done

run() {
  echo "+ $*"
  [ "$DRY_RUN" = 1 ] || "$@"
}

mikrotik() {
  local cmd="$1"
  echo "+ mikrotik: $cmd"
  [ "$DRY_RUN" = 1 ] && return 0
  sshpass -p "$MIKROTIK_PW" ssh -o StrictHostKeyChecking=no "admin@${MIKROTIK_IP}" "$cmd"
}

echo "==> Rolling back MikroTik L2MTU changes"
# sfp28-5 was raised from 1584 to 9216 (Blackfly1 port)
mikrotik '/interface ethernet set sfp28-5 l2mtu=1584'
# sfp28-9/11/13/15 (Thor breakout lanes) were raised from 1584 to 9216
mikrotik '/interface ethernet set sfp28-9,sfp28-11,sfp28-13,sfp28-15 l2mtu=1584'
# sfp28-3,4,7,10,14,16 raised from 1584 to 9216 to lift bridge-wide L2MTU cap
mikrotik '/interface ethernet set sfp28-3,sfp28-4,sfp28-7,sfp28-10,sfp28-14,sfp28-16 l2mtu=1584'

if [ "$MIKROTIK_ONLY" = 1 ]; then
  echo "==> --mikrotik-only: skipping compose/yaml rollback"
  exit 0
fi

echo "==> Rolling back compose blackfly1 to pre-session state"
cd "$BESS_DIR"
# Drop gev_scps_packet_size to 1400 (no change needed if already 1400)
# Remove user_set_selector and user_set_load params
# The only surviving edit from this session is sleep 10→30 which we keep
if grep -q "user_set_selector:=Default" docker-compose.thor.yml; then
  run sed -i '/-p user_set_selector:=Default \\/d' docker-compose.thor.yml
  run sed -i '/-p user_set_load:=1 \\/d' docker-compose.thor.yml
fi
run sed -i 's|-p gev_scps_packet_size:=9000|-p gev_scps_packet_size:=1400|' docker-compose.thor.yml

echo "==> Rolling back blackfly_s_thor.yaml UserSet params"
# Remove the 3 UserSet param entries we added
if grep -q "UserSetControl/UserSetSelector" config/blackfly_s_thor.yaml; then
  run python3 -c "
import re
p = 'config/blackfly_s_thor.yaml'
s = open(p).read()
s = re.sub(r'  - name: user_set_selector\n    type: enum\n    node: UserSetControl/UserSetSelector\n', '', s)
s = re.sub(r'  - name: user_set_load\n    type: command\n    node: UserSetControl/UserSetLoad\n', '', s)
s = re.sub(r'  - name: default_user_set\n    type: enum\n    node: UserSetControl/DefaultUserSet\n', '', s)
open(p,'w').write(s)
"
fi

echo "==> Restarting blackfly1 with rolled-back config"
run docker compose -f "$BESS_DIR/docker-compose.thor.yml" up -d blackfly1

echo "==> Rollback complete. Verify with:"
echo "  docker logs blackfly1 --tail 10 | grep -E 'IN:|ERROR'"
echo "  sshpass -p '$MIKROTIK_PW' ssh admin@$MIKROTIK_IP '/interface ethernet print where name~\"sfp28-\" and l2mtu!=1584'"
