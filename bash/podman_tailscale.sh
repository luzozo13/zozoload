#!/bin/bash

# 🚀 Tailscale installation script for Podman container
# 📦 Purpose: Secure VPN access to Home Assistant without exposing ports
# 🧠 Author: Copilot for Ludovic

# 📁 Configuration file path
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_FILE="$SCRIPT_DIR/tailscale_config.env"🚀 Tailscale installation script for Podman container
# 📦 Purpose: Secure VPN access to Home Assistant without exposing ports
# 🧠 Author: Copilot for Ludovic

# 📁 Configuration file path 📁 Configuration file pathbash

# 🚀 Tailscale installation script for Podman container
# 📦 Purpose: Secure VPN access to Home Assistant without exposing ports
# 🧠 Author: Copilot for Ludovic

# � Chemin du fichier de configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_FILE="$SCRIPT_DIR/tailscale_config.env"

# 🔍 Check if configuration file exists
if [[ ! -f "$CONFIG_FILE" ]]; then
    echo "❌ Error: Configuration file not found: $CONFIG_FILE"
    echo "📝 Please create the tailscale_config.env file with the necessary variables"
    exit 1
fi

# 📋 Load variables from configuration file
echo "📋 Loading configuration from: $CONFIG_FILE"
source "$CONFIG_FILE"

# ✅ Check that mandatory variables are defined
if [[ -z "$CONTAINER_NAME" || -z "$TAILSCALE_IMAGE" || -z "$SUBNET" || -z "$AUTHKEY" || -z "$CONFIG_DIR" ]]; then
    echo "❌ Error: Missing mandatory variables in $CONFIG_FILE"
    echo "📝 Required variables: CONTAINER_NAME, TAILSCALE_IMAGE, SUBNET, AUTHKEY, CONFIG_DIR"
    exit 1
fi

# 🔑 Check that the authentication key has been modified
if [[ "$AUTHKEY" == "tskey-xxxxxxxxxxxxxxxx" ]] || [[ "$AUTHKEY" == "YOUR_TAILSCALE_AUTH_KEY" ]]; then
    echo "❌ Error: Please set a real Tailscale authentication key in $CONFIG_FILE"
    echo "🔗 Generate a key at: https://login.tailscale.com/admin/settings/authkeys"
    exit 1
fi

# 🔐 Check if running as root or with sudo access
if [[ $EUID -eq 0 ]]; then
    echo "⚠️  Running as root - this is required for Tailscale network operations"
elif ! sudo -n true 2>/dev/null; then
    echo "❌ Error: This script requires sudo access for network operations"
    echo "💡 Alternatives:"
    echo "   1. Run with: sudo $0"
    echo "   2. Configure rootless Podman with user namespaces"
    echo "   3. Add user to podman group (if available)"
    exit 1
fi

# 📁 Create persistent config directory
sudo mkdir -p "$CONFIG_DIR"
sudo chown $(whoami):$(whoami) "$CONFIG_DIR"

# 🛠️ Create Tailscale container (requires sudo for TUN/TAP and network capabilities)
echo "🔐 Running with sudo for required network privileges..."
sudo podman run -d \
  --name "$CONTAINER_NAME" \
  --cap-add NET_ADMIN \
  --cap-add SYS_MODULE \
  --network host \
  --privileged \
  -v /dev/net/tun:/dev/net/tun \
  -v "$CONFIG_DIR":"$CONFIG_DIR" \
  "$TAILSCALE_IMAGE" \
  tailscaled

# ⏳ Wait a few seconds for tailscaled to start
sleep 5

# 🔗 Connect container to Tailscale network with subnet routing
sudo podman exec -it "$CONTAINER_NAME" tailscale up \
  --authkey "$AUTHKEY" \
  --advertise-routes="$SUBNET" \
  --accept-routes

# 🧬 Generate systemd service for automatic startup
sudo podman generate systemd --name "$CONTAINER_NAME" --files --restart-policy always

# 📦 Move service file to the correct directory
sudo mv container-"$CONTAINER_NAME".service /etc/systemd/system/

# 🔄 Enable and start the service
sudo systemctl enable container-"$CONTAINER_NAME"
sudo systemctl start container-"$CONTAINER_NAME"

# ✅ Final summary
echo "✅ Tailscale container launched and connected to private network."
echo "🌐 Subnet route announced: $SUBNET"
echo "🔁 Automatic startup enabled via systemd."
echo "🧭 Remember to activate the route in the Tailscale interface: https://login.tailscale.com/admin/machines"
