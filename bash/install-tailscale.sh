#!/bin/bash

# 🚀 Global Tailscale setup script
# 📦 Downloads and runs the latest version from GitHub

REPO_URL="https://raw.githubusercontent.com/luzozo13/zozoload/main/bash"
TEMP_DIR="/tmp/tailscale-setup"

# Create temp directory
mkdir -p "$TEMP_DIR"
cd "$TEMP_DIR"

# Download files
echo "📥 Downloading Tailscale setup files..."
wget -q "$REPO_URL/podman_tailscale.sh" -O podman_tailscale.sh
wget -q "$REPO_URL/tailscale_config.env.template" -O tailscale_config.env.template

if [[ ! -f "podman_tailscale.sh" ]] || [[ ! -f "tailscale_config.env.template" ]]; then
    echo "❌ Error: Failed to download files from GitHub"
    exit 1
fi

# Check if config exists
if [[ ! -f "tailscale_config.env" ]]; then
    echo "📋 Creating configuration file..."
    cp tailscale_config.env.template tailscale_config.env
    echo "⚠️  Please edit tailscale_config.env with your settings:"
    echo "   nano $TEMP_DIR/tailscale_config.env"
    echo ""
    echo "Then run this command again to install."
    exit 0
fi

# Make executable and run
chmod +x podman_tailscale.sh
echo "🚀 Running Tailscale setup..."
sudo ./podman_tailscale.sh

# Cleanup
cd /
rm -rf "$TEMP_DIR"
