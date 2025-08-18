# Tailscale Podman Setup

## 📖 Description

Tailscale installation script for Podman container to provide secure VPN access to Home Assistant without exposing ports.

## 🚀 Installation

1. **Copy the configuration template:**
   ```bash
   cp tailscale_config.env.template tailscale_config.env
   ```

2. **Edit the configuration:**
   ```bash
   nano tailscale_config.env
   ```

3. **Customize the variables:**
   - `CONTAINER_NAME`: Container name (default: `tailscale`)
   - `TAILSCALE_IMAGE`: Docker image to use (default: `tailscale/tailscale`)
   - `SUBNET`: Home Assistant local subnet (e.g.: `192.168.1.0/24`)
   - `AUTHKEY`: Tailscale authentication key (generate at https://login.tailscale.com/admin/settings/authkeys)
   - `CONFIG_DIR`: Persistent configuration directory (default: `/etc/tailscale`)

4. **Execute the script with sudo (required for network operations):**
   ```bash
   chmod +x podman_tailscale.sh
   sudo ./podman_tailscale.sh
   ```

## ⚙️ Required Configuration

### Mandatory variables in `tailscale_config.env`:

- **CONTAINER_NAME**: Unique container name
- **TAILSCALE_IMAGE**: Tailscale Docker/Podman image
- **SUBNET**: Subnet to route via Tailscale
- **AUTHKEY**: Valid Tailscale authentication key
- **CONFIG_DIR**: Configuration storage directory

### Authentication key generation:

1. Go to https://login.tailscale.com/admin/settings/authkeys
2. Click on "Generate auth key"
3. Configure according to your needs (duration, reusable, etc.)
4. Copy the generated key into `tailscale_config.env`

## 🔧 Usage

The script:
1. Checks for the presence of the configuration file
2. Validates that all variables are defined
3. Verifies sudo access (required for network operations)
4. Creates the Tailscale container with subnet routing
5. Configures systemd service for automatic startup
6. Enables and starts the service

### ⚠️ Sudo Requirement

Tailscale requires privileged access for:
- TUN/TAP device access (`/dev/net/tun`)
- Network namespace operations
- Subnet routing capabilities

**Alternatives to sudo:**
- **Rootless Podman**: Configure user namespaces (complex setup)
- **User groups**: Add user to `podman` group (if configured)
- **Docker**: Consider using Docker instead of Podman

## 🛡️ Security

- The `tailscale_config.env` file contains sensitive information
- Do not commit this file to a public repository
- Add `tailscale_config.env` to `.gitignore`

## 📝 Logs

Check container logs:
```bash
sudo podman logs tailscale
```

Check service status:
```bash
systemctl status container-tailscale
```

## 🔧 Troubleshooting

### Permission Issues
```bash
# Check if container is running
sudo podman ps

# Restart container if needed
sudo podman restart tailscale

# Check Tailscale status inside container
sudo podman exec tailscale tailscale status
```

### Rootless Podman Setup (Alternative)
If you want to avoid sudo, configure rootless Podman:
```bash
# Enable user namespaces
echo 'user.max_user_namespaces=28633' | sudo tee -a /etc/sysctl.conf
sudo sysctl -p

# Configure subuid/subgid
sudo usermod --add-subuids 100000-165535 $(whoami)
sudo usermod --add-subgids 100000-165535 $(whoami)

# Reboot required for changes to take effect
```
