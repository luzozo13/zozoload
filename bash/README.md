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

4. **Execute the script:**
   ```bash
   chmod +x podman_tailscale.sh
   ./podman_tailscale.sh
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
3. Creates the Tailscale container with subnet routing
4. Configures systemd service for automatic startup
5. Enables and starts the service

## 🛡️ Security

- The `tailscale_config.env` file contains sensitive information
- Do not commit this file to a public repository
- Add `tailscale_config.env` to `.gitignore`

## 📝 Logs

Check container logs:
```bash
podman logs tailscale
```

Check service status:
```bash
systemctl status container-tailscale
```
