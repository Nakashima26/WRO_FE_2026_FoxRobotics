#!/usr/bin/env bash
# Instala el dashboard en la Pi y deja lista la red sin Internet.
#   bash scripts/instalar_dashboard_pi.sh [clave_del_hotspot]
# No activa el hotspot: solo crea los perfiles. Así no se cae el SSH por Wi-Fi.
set -euo pipefail
REPO="$(cd "$(dirname "$0")/.." && pwd)"
PASS="${1:-foxrobotics}"

sudo install -m 644 "$REPO/deploy/systemd/wro-dashboard.service" /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now wro-dashboard

# El dashboard (usuario user) arranca/para el runtime y apaga radios sin contraseña.
if ! sudo -n -l -U user 2>/dev/null | grep -q "NOPASSWD: ALL"; then
  echo "user ALL=(root) NOPASSWD: /usr/bin/systemctl start wro-runtime, /usr/bin/systemctl stop wro-runtime, /usr/bin/tee /sys/class/rfkill/rfkill*/soft" \
    | sudo tee /etc/sudoers.d/fox-dashboard >/dev/null
  sudo chmod 440 /etc/sudoers.d/fox-dashboard
fi

# Hotspot propio: prioridad baja, así en casa gana el Wi-Fi conocido y en la sede (sin Wi-Fi conocido) se levanta solo.
if ! nmcli -t -f NAME con show | grep -qx fox-ap; then
  sudo nmcli con add type wifi ifname wlan0 con-name fox-ap ssid FoxCar autoconnect yes \
    connection.autoconnect-priority -10 802-11-wireless.mode ap 802-11-wireless.band bg \
    ipv4.method shared ipv4.addresses 10.42.0.1/24 ipv6.method disabled \
    wifi-sec.key-mgmt wpa-psk wifi-sec.psk "$PASS"
fi
# Cable directo laptop↔Pi: la Pi da IP por DHCP (sirve con los radios apagados, regla 11.10).
if ! nmcli -t -f NAME con show | grep -qx fox-eth; then
  sudo nmcli con add type ethernet ifname eth0 con-name fox-eth autoconnect yes \
    ipv4.method shared ipv4.addresses 10.43.0.1/24 ipv6.method disabled
fi
echo "Listo. Dashboard: http://<ip>:8080  ·  hotspot FoxCar → http://10.42.0.1:8080  ·  cable → http://10.43.0.1:8080"
