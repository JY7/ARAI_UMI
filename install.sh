#!/bin/bash
# UMI collection PC setup (Ubuntu 20.04-24.04): collection, alignment, GR00T dataset conversion (not GR00T training)
#   - .venv with uv-managed Python 3.12 (independent of system python)
#   - Steam + SteamVR (trackers only, no HMD)
# Run: bash install.sh (from anywhere). Safe to re-run: finished steps are skipped
# After the first run: log in to Steam -> install SteamVR -> log out/in (dialout, input groups) -> re-run (no-HMD config)

UMI_DIR="$(cd "$(dirname "$0")" && pwd)"
echo "umi_collect: $UMI_DIR"

## packages: curl (uv download), build-essential (evdev build), espeak (voice prompts), bluez (BT buttons), ffmpeg (video encoding)
sudo apt update
sudo apt -y install curl build-essential espeak bluez ffmpeg

## device access: U2D2 (/dev/ttyUSB*) = dialout, BT buttons (/dev/input/*) = input; applies after log out/in
if id -nG "$USER" | grep -qw dialout && id -nG "$USER" | grep -qw input; then
  echo "dialout/input groups OK, skipping"
else
  echo "Adding $USER to dialout, input (log out/in after this script)"
  sudo usermod -aG dialout,input "$USER"
fi

## U2D2 (FTDI) USB latency 16 ms -> 1 ms (Dynamixel loop 50-60 Hz -> ~100 Hz)
RULE=/etc/udev/rules.d/99-dynamixel-latency.rules
if test -f "$RULE"; then
  echo "Dynamixel latency rule exists, skipping"
else
  echo 'ACTION=="add", SUBSYSTEM=="usb-serial", DRIVER=="ftdi_sio", ATTR{latency_timer}="1"' | sudo tee "$RULE"
  sudo udevadm control --reload-rules
  sudo udevadm trigger --action=add --subsystem-match=usb-serial --subsystem-match=tty   # rule matches "add" only
fi

## uv + Python 3.12 venv
export PATH="$HOME/.local/bin:$PATH"
command -v uv >/dev/null || curl -LsSf https://astral.sh/uv/install.sh | sh
if "$UMI_DIR/.venv/bin/python" -c 'import sys; sys.exit(sys.version_info[:2] != (3, 12))' 2>/dev/null; then
  echo "venv (python 3.12) exists, updating packages"
else
  echo "Creating venv (python 3.12)..."
  rm -rf "${UMI_DIR:?}/.venv"
  uv venv --python 3.12 --seed "$UMI_DIR/.venv"   # --seed: pip inside the venv
fi
"$UMI_DIR/.venv/bin/pip" install -r "$UMI_DIR/requirements.txt"
A="alias umi='cd $UMI_DIR && source .venv/bin/activate'"
grep -qxF "$A" ~/.bashrc || echo "$A" >> ~/.bashrc

## Rerun viewer mp4 playback needs FFmpeg >= 5.1 (20.04/22.04 apt ship 4.x) -> static ffmpeg next to the viewer binary
## System ffmpeg is untouched; reinstalling rerun-sdk removes it, so check every run
RERUN_DIR="$("$UMI_DIR/.venv/bin/python" -c 'import rerun_cli, os; print(os.path.dirname(rerun_cli.__file__))')"
if ffmpeg -version | head -1 | grep -qE "version (5\.[1-9]|[6-9]\.|[1-9][0-9])" || test -x "$RERUN_DIR/ffmpeg"; then
  echo "ffmpeg for the Rerun viewer OK, skipping"
else
  echo "Installing static ffmpeg for the Rerun viewer..."
  T="$(mktemp -d)"
  curl -fsSL https://johnvansickle.com/ffmpeg/releases/ffmpeg-release-amd64-static.tar.xz | tar -xJ -C "$T" --wildcards '*/ffmpeg' \
    && cp "$T"/ffmpeg-*-amd64-static/ffmpeg "$RERUN_DIR/ffmpeg" || echo "FAILED: static ffmpeg (replay of mp4 episodes in Rerun)"
  rm -rf "${T:?}"
fi

## Steam (+ steam-devices: Vive dongle udev rules); SteamVR needs a Steam login, so it is installed from the Steam app
if command -v steam >/dev/null; then
  echo "Steam installed, skipping"
else
  echo "Installing Steam..."
  sudo add-apt-repository -y multiverse
  sudo dpkg --add-architecture i386
  sudo apt update
  sudo apt -y install steam-installer steam-devices || sudo apt -y install steam steam-devices   # 24.04: steam-installer, 20.04: steam
fi
STEAM="$HOME/.steam/root"
STEAMVR="$STEAM/steamapps/common/SteamVR"
if test -d "$STEAMVR"; then   # trackers only, no HMD
  mkdir -p "$STEAM/config"
  cp "$UMI_DIR/steamvr/steamvr.vrsettings.nohmd" "$STEAM/config/steamvr.vrsettings"
  cp "$UMI_DIR/steamvr/default.vrsettings.nohmd" "$STEAMVR/drivers/null/resources/settings/default.vrsettings"
  STEAMVR_MSG="SteamVR: OK (no-HMD config applied)"
else
  STEAMVR_MSG="SteamVR: NOT INSTALLED -> log in to Steam, click Install in the SteamVR dialog, then re-run this script"
  nohup steam steam://install/250820 >/dev/null 2>&1 &
fi

echo
echo "Done (umi_collect)."
echo " $STEAMVR_MSG"
echo " 1) Log out/in (dialout, input groups)"
echo " 2) Trackers: umi && python3 collect.py --list-vive"
