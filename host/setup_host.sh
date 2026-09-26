#!/usr/bin/env bash
# Host-level setup for SocialTech Jetsons (AGX Orin / Orin NX / Orin Nano, JetPack 7.2.1).
# Covers what cannot live in the Docker image: kernel modules, udev, systemd units,
# networking, power, remote access. Idempotent: re-running on a configured host changes nothing.
#
#   sudo ./setup_host.sh [robot.conf]           apply, then verify
#   sudo ./setup_host.sh --check [robot.conf]   verify only; exit 1 if anything is missing
#
# Re-run after an L4T/kernel update: gs_usb is rebuilt for the new kernel.
# Interactive steps (hotspot password, Tailscale login, power-mode reboot) only run with a TTY;
# without one they are skipped with instructions.
set -euo pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
MODE=apply
if [[ ${1:-} == --check ]]; then MODE=check; shift; fi
CONF=${1:-$HERE/robot.conf}

CAN_IFACE=can_tracer   # gs_usb adapter is renamed to this on every board
CAN_BITRATE=500000     # Tracer base
KVER=$(uname -r)
TARGET_USER=${SUDO_USER:-jetson}

log()  { printf '\e[1;34m==>\e[0m %s\n' "$*"; }
warn() { printf '\e[1;33m!!\e[0m  %s\n' "$*" >&2; }
die()  { printf '\e[1;31mxx\e[0m  %s\n' "$*" >&2; exit 1; }

[[ $EUID -eq 0 ]] || die "run as root: sudo $0"
[[ -f $CONF ]] || die "missing $CONF (cp $HERE/robot.conf.example $HERE/robot.conf)"
# shellcheck source=robot.conf.example
source "$CONF"

# Validate config: values end up in hostnames, udev rules and nmcli calls.
[[ $ROBOT_HOSTNAME =~ ^[a-z0-9]([a-z0-9-]{0,61}[a-z0-9])?$ ]] || die "bad ROBOT_HOSTNAME: $ROBOT_HOSTNAME"
[[ -z $LIVOX_IFACE || $LIVOX_HOST_IP =~ ^([0-9]{1,3}\.){3}[0-9]{1,3}$ ]] || die "bad LIVOX_HOST_IP: $LIVOX_HOST_IP"
[[ $JOYSTICK_SERIAL =~ ^[A-Za-z0-9_-]*$ ]] || die "bad JOYSTICK_SERIAL: $JOYSTICK_SERIAL"
[[ ${#HOTSPOT_SSID} -le 32 ]] || die "HOTSPOT_SSID longer than 32 chars"

# ── helpers ────────────────────────────────────────────────────────────────

# put PATH [MODE] < content: install only if content differs. Returns 0 if written, 1 if unchanged.
put() {
  local path=$1 mode=${2:-644} tmp
  tmp=$(mktemp)
  cat >"$tmp"
  if [[ -f $path ]] && cmp -s "$tmp" "$path"; then rm -f "$tmp"; return 1; fi
  install -D -m "$mode" "$tmp" "$path"
  rm -f "$tmp"
  log "wrote $path"
}

pkg_installed() { dpkg-query -W -f='${Status}' "$1" 2>/dev/null | grep -q 'install ok installed'; }
in_group()      { id -nG "$TARGET_USER" | grep -qw "$1"; }
nm_con_exists() { nmcli -t -f NAME connection show | grep -qx "$1"; }
power_mode()    { nvpmodel -q | tail -n1; }

# MAXN id differs per board; prefer MAXN_SUPER (Orin Nano/NX Super) when present.
maxn_id() {
  local id
  id=$(sed -n 's/^< POWER_MODEL ID=\([0-9]*\) NAME=MAXN_SUPER >.*/\1/p' /etc/nvpmodel.conf)
  [[ -n $id ]] || id=$(sed -n 's/^< POWER_MODEL ID=\([0-9]*\) NAME=MAXN >.*/\1/p' /etc/nvpmodel.conf)
  echo "$id"
}

# First onboard NIC of a type, skipping USB gadget, bridges and containers.
first_iface() {
  nmcli -t -f DEVICE,TYPE device | awk -F: -v t="$1" \
    '$2==t && $1 !~ /^(usb|l4tbr|docker|veth|br-|p2p-)/ {print $1; exit}'
}

# ── steps ──────────────────────────────────────────────────────────────────

step_packages() {
  local want=(can-utils systemd-zram-generator) missing=() p
  if [[ $TAILSCALE == 1 ]]; then
    want+=(tailscale)
    if [[ ! -f /etc/apt/sources.list.d/tailscale.list ]]; then
      curl -fsSL https://pkgs.tailscale.com/stable/ubuntu/noble.noarmor.gpg \
        -o /usr/share/keyrings/tailscale-archive-keyring.gpg
      curl -fsSL https://pkgs.tailscale.com/stable/ubuntu/noble.tailscale-keyring.list \
        -o /etc/apt/sources.list.d/tailscale.list
    fi
  fi
  for p in "${want[@]}"; do pkg_installed "$p" || missing+=("$p"); done
  if ((${#missing[@]})); then
    log "installing ${missing[*]}"
    apt-get update -q
    DEBIAN_FRONTEND=noninteractive apt-get install -y -q "${missing[@]}"
  fi
}

step_hostname() {
  if [[ $(hostname) != "$ROBOT_HOSTNAME" ]]; then
    hostnamectl set-hostname "$ROBOT_HOSTNAME"
    log "hostname -> $ROBOT_HOSTNAME"
  fi
  if grep -q '^127\.0\.1\.1' /etc/hosts; then
    sed -i "s/^127\.0\.1\.1.*/127.0.1.1\t$ROBOT_HOSTNAME/" /etc/hosts
  else
    printf '127.0.1.1\t%s\n' "$ROBOT_HOSTNAME" >>/etc/hosts
  fi
}

step_docker() {
  local changed g
  # Merge, don't overwrite: JetPack ships daemon.json with the nvidia runtime entry.
  changed=$(python3 - <<'EOF'
import json, shutil
p = '/etc/docker/daemon.json'
d = json.load(open(p))
if d.get('default-runtime') != 'nvidia':
    shutil.copy(p, p + '.bak')
    d['default-runtime'] = 'nvidia'
    with open(p, 'w') as f:
        json.dump(d, f, indent=4)
    print('yes')
EOF
)
  if [[ $changed == yes ]]; then log "docker default runtime -> nvidia"; systemctl restart docker; fi
  systemctl enable --quiet docker   # containers with restart: unless-stopped come back at boot
  for g in docker dialout; do
    if ! in_group "$g"; then usermod -aG "$g" "$TARGET_USER"; log "$TARGET_USER added to $g (re-login)"; fi
  done
}

step_zram() {
  if put /etc/systemd/zram-generator.conf <<'EOF'
# Compressed swap in RAM (systemd-zram-generator). Disk /swapfile stays as lower-priority fallback.
[zram0]
zram-size = min(ram / 2, 4096)
compression-algorithm = zstd
EOF
  then systemctl daemon-reload; systemctl restart systemd-zram-setup@zram0.service; fi
  if ! swapon --noheadings --show=NAME | grep -q zram0; then systemctl start dev-zram0.swap; fi
}

step_can() {
  local ko=/lib/modules/$KVER/updates/gs_usb.ko d
  # JetPack 7 kernel ships without CONFIG_CAN_GS_USB: build it from the matching stable source.
  if [[ ! -f $ko ]]; then
    [[ -f /lib/modules/$KVER/build/Makefile ]] || die "kernel headers missing for $KVER (nvidia-l4t-kernel-headers)"
    log "building gs_usb for $KVER"
    d=$(mktemp -d)   # left in /tmp, cleared on reboot
    curl -fsSL -o "$d/gs_usb.c" \
      "https://git.kernel.org/pub/scm/linux/kernel/git/stable/linux.git/plain/drivers/net/can/usb/gs_usb.c?h=v${KVER%%-*}"
    echo 'obj-m := gs_usb.o' >"$d/Makefile"
    make -s -C "/lib/modules/$KVER/build" M="$d" modules
    install -D -m 644 "$d/gs_usb.ko" "$ko"
    depmod -a "$KVER"
  fi

  # Stable name: kernel numbering differs per board (AGX has 2 native CAN ifaces, NX/Nano 1).
  if put /etc/systemd/network/10-tracer-can.link <<EOF
# AgileX USB-CAN adapter (gs_usb) -> same interface name on every Jetson.
[Match]
Driver=gs_usb

[Link]
Name=$CAN_IFACE
EOF
  then udevadm control --reload; fi

  # Brought up whenever the interface appears (boot or hot-plug), torn down when it goes.
  if put /etc/systemd/system/can-up@.service <<EOF
[Unit]
Description=Bring up CAN interface %i
BindsTo=sys-subsystem-net-devices-%i.device
After=sys-subsystem-net-devices-%i.device

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/usr/sbin/ip link set %i down
ExecStart=/usr/sbin/ip link set %i up type can bitrate $CAN_BITRATE
ExecStop=-/usr/sbin/ip link set %i down

[Install]
WantedBy=sys-subsystem-net-devices-%i.device
EOF
  then systemctl daemon-reload; fi
  systemctl enable --quiet "can-up@$CAN_IFACE.service"
}

step_udev() {
  local reload=0
  if put /etc/udev/rules.d/99-obsensor-libusb.rules <"$HERE/99-obsensor-libusb.rules"; then reload=1; fi
  if [[ -n $JOYSTICK_SERIAL ]]; then
    if put /etc/udev/rules.d/99-arduino-joystick.rules <<EOF
SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="1002", ATTRS{serial}=="$JOYSTICK_SERIAL", SYMLINK+="arduino_joystick", GROUP="dialout", MODE="0660"
EOF
    then reload=1; fi
  fi
  if ((reload)); then udevadm control --reload-rules; udevadm trigger --subsystem-match=usb --subsystem-match=tty; fi
}

step_livox() {
  local ifc=$LIVOX_IFACE
  if [[ -z $ifc ]]; then log "livox link: skipped"; return 0; fi
  if [[ $ifc == auto ]]; then
    ifc=$(first_iface ethernet)
    [[ -n $ifc ]] || die "no onboard ethernet interface for the Livox"
  fi
  nm_con_exists livox || nmcli connection add type ethernet con-name livox ifname "$ifc" >/dev/null
  # never-default: the lidar subnet must not steal the default route.
  nmcli connection modify livox connection.interface-name "$ifc" \
    connection.autoconnect yes connection.autoconnect-priority 100 \
    ipv4.method manual ipv4.addresses "$LIVOX_HOST_IP/24" ipv4.gateway "" ipv4.never-default yes \
    ipv6.method disabled
}

step_hotspot() {
  local wifi psk=${HOTSPOT_PSK:-}
  if [[ -z $HOTSPOT_SSID ]]; then log "hotspot: skipped"; return 0; fi
  wifi=$(first_iface wifi)
  [[ -n $wifi ]] || die "no Wi-Fi interface for the hotspot"

  if ! nm_con_exists Hotspot; then
    if [[ -z $psk && -t 0 ]]; then read -rsp "Hotspot password for '$HOTSPOT_SSID' (>= 8 chars): " psk; echo; fi
    [[ ${#psk} -ge 8 ]] || die "hotspot needs a password >= 8 chars (run with a TTY or HOTSPOT_PSK=...)"
    nmcli connection add type wifi con-name Hotspot ifname "$wifi" ssid "$HOTSPOT_SSID" >/dev/null
  fi
  local args=(connection.interface-name "$wifi" connection.autoconnect no
    802-11-wireless.ssid "$HOTSPOT_SSID" 802-11-wireless.mode ap 802-11-wireless.band bg
    ipv4.method shared ipv6.method ignore wifi-sec.key-mgmt wpa-psk)
  if [[ -n $psk ]]; then
    [[ ${#psk} -ge 8 ]] || die "hotspot password must be >= 8 chars"
    args+=(wifi-sec.psk "$psk")
  fi
  nmcli connection modify Hotspot "${args[@]}"

  put /usr/local/sbin/hotspot-fallback 755 <<'EOF' || true
#!/bin/sh
# Raise the Hotspot AP when, after boot, no saved client Wi-Fi has connected.
if nmcli -t -f TYPE,NAME connection show --active | grep '^802-11-wireless:' | grep -qv ':Hotspot$'; then
  exit 0
fi
echo "no client Wi-Fi after boot, raising Hotspot" | systemd-cat -t hotspot-fallback
exec nmcli connection up Hotspot
EOF
  put /etc/systemd/system/hotspot-fallback.service <<'EOF' || true
[Unit]
Description=Raise Wi-Fi hotspot if no saved Wi-Fi connected
After=NetworkManager.service

[Service]
Type=oneshot
ExecStart=/usr/local/sbin/hotspot-fallback
EOF
  # Timer, not a sleeping service: a sleep inside a boot service would delay reaching the desktop.
  if put /etc/systemd/system/hotspot-fallback.timer <<'EOF'
[Unit]
Description=Check for client Wi-Fi 45 s after boot

[Timer]
OnBootSec=45

[Install]
WantedBy=timers.target
EOF
  then systemctl daemon-reload; fi
  systemctl enable --quiet hotspot-fallback.timer
}

step_clocks() {
  if [[ $JETSON_CLOCKS != 1 ]]; then
    systemctl disable --now --quiet jetson-clocks.service 2>/dev/null || true
    return 0
  fi
  if put /etc/systemd/system/jetson-clocks.service <<'EOF'
[Unit]
Description=Lock Jetson clocks to max for the current power mode
After=nvpmodel.service

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/usr/bin/jetson_clocks

[Install]
WantedBy=multi-user.target
EOF
  then systemctl daemon-reload; fi
  systemctl enable --now --quiet jetson-clocks.service
}

step_tailscale() {
  if [[ $TAILSCALE != 1 ]]; then return 0; fi
  systemctl enable --now --quiet tailscaled
  if tailscale status >/dev/null 2>&1; then return 0; fi
  if [[ -t 0 ]]; then
    log "Tailscale login: open the URL below"
    tailscale up --hostname="$ROBOT_HOSTNAME"
  else
    warn "Tailscale not logged in: run 'sudo tailscale up --hostname=$ROBOT_HOSTNAME'"
  fi
}

# Last on purpose: switching to MAXN requires a reboot that nvpmodel asks for interactively.
step_power_mode() {
  local want cur
  want=$(maxn_id); cur=$(power_mode)
  [[ -n $want ]] || die "no MAXN mode in /etc/nvpmodel.conf"
  if [[ $cur == "$want" ]]; then return 0; fi
  if [[ -t 0 ]]; then
    warn "power mode $cur -> MAXN ($want) needs a reboot; answer YES to reboot now"
    nvpmodel -m "$want" || warn "power mode not changed"
  else
    warn "power mode $cur -> MAXN ($want) needs a reboot: run 'sudo nvpmodel -m $want'"
  fi
}

# ── verification ───────────────────────────────────────────────────────────

FAILS=0
check() {
  local name=$1; shift
  if "$@" >/dev/null 2>&1; then printf '  PASS  %s\n' "$name"
  else printf '  FAIL  %s\n' "$name"; FAILS=$((FAILS + 1)); fi
}

run_checks() {
  log "verifying"
  check "hostname = $ROBOT_HOSTNAME" test "$(hostname)" = "$ROBOT_HOSTNAME"
  check "can-utils installed" pkg_installed can-utils
  check "zram generator installed" pkg_installed systemd-zram-generator
  check "docker default runtime = nvidia" test "$(docker info --format '{{.DefaultRuntime}}')" = nvidia
  check "$TARGET_USER in docker" in_group docker
  check "$TARGET_USER in dialout" in_group dialout
  check "zram swap active" sh -c 'swapon --noheadings --show=NAME | grep -q zram0'
  check "gs_usb built for $KVER" modinfo -k "$KVER" gs_usb
  check "gs_usb -> $CAN_IFACE rename rule" test -f /etc/systemd/network/10-tracer-can.link
  check "can-up@$CAN_IFACE enabled" systemctl is-enabled "can-up@$CAN_IFACE.service"
  check "orbbec udev rules" cmp -s "$HERE/99-obsensor-libusb.rules" /etc/udev/rules.d/99-obsensor-libusb.rules
  if [[ -n $JOYSTICK_SERIAL ]]; then
    check "joystick udev rule" grep -q "$JOYSTICK_SERIAL" /etc/udev/rules.d/99-arduino-joystick.rules
  fi
  if [[ -n $LIVOX_IFACE ]]; then
    check "livox link $LIVOX_HOST_IP/24" test "$(nmcli -g ipv4.addresses connection show livox)" = "$LIVOX_HOST_IP/24"
  fi
  if [[ -n $HOTSPOT_SSID ]]; then
    check "hotspot connection" nm_con_exists Hotspot
    check "hotspot fallback timer enabled" systemctl is-enabled hotspot-fallback.timer
  fi
  if [[ $JETSON_CLOCKS == 1 ]]; then check "jetson-clocks enabled" systemctl is-enabled jetson-clocks.service; fi
  if [[ $TAILSCALE == 1 ]]; then check "tailscale logged in" tailscale status; fi
  check "power mode = MAXN" test "$(power_mode)" = "$(maxn_id)"
  if ((FAILS)); then warn "$FAILS check(s) failed"; else log "all checks passed"; fi
}

# ── main ───────────────────────────────────────────────────────────────────

if [[ $MODE == apply ]]; then
  step_packages
  step_hostname
  step_docker
  step_zram
  step_can
  step_udev
  step_livox
  step_hotspot
  step_clocks
  step_tailscale
fi
run_checks
if [[ $MODE == apply ]]; then step_power_mode; fi
exit $((FAILS > 0))
