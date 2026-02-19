# Local Server Setup Guide

**Date**: 2026-01-27
**Purpose**: Testing wowee with a local WoW 3.3.5a server
**Status**: Ready for testing

---

## Overview

The wowee client is pre-configured to connect to a local WoW 3.3.5a private server. This guide explains how to set up and test with popular server emulators like TrinityCore or AzerothCore.

## Default Configuration

The authentication screen comes with local server defaults:

| Setting | Default Value | Description |
|---------|---------------|-------------|
| **Hostname** | localhost | Localhost (your machine) |
| **Port** | 3724 | Standard auth server port |
| **Username** | (empty) | Your account username |
| **Password** | (empty) | Your account password |

These values can be changed in the UI before connecting.

## Server Requirements

You need a WoW 3.3.5a (Wrath of the Lich King) server emulator running on your local machine or network.

### Supported Server Emulators

**Recommended:**
- **TrinityCore 3.3.5a** - Most stable and feature-complete
  - GitHub: https://github.com/TrinityCore/TrinityCore (3.3.5 branch)
  - Documentation: https://trinitycore.info/

- **AzerothCore** - Active community, good documentation
  - GitHub: https://github.com/azerothcore/azerothcore-wotlk
  - Documentation: https://www.azerothcore.org/wiki/

- **MaNGOS WotLK** - Classic emulator, stable
  - GitHub: https://github.com/cmangos/mangos-wotlk
  - Documentation: https://github.com/cmangos/mangos-wotlk/wiki

## Quick Setup (TrinityCore)

### 1. Install Prerequisites

**Ubuntu/Debian:**
```bash
sudo apt-get update
sudo apt-get install git cmake make gcc g++ libssl-dev \
    libmysqlclient-dev libreadline-dev zlib1g-dev libbz2-dev \
    libboost-all-dev mysql-server
```

**macOS:**
```bash
brew install cmake boost openssl readline mysql
```

### 2. Download TrinityCore

```bash
cd ~/
git clone -b 3.3.5 https://github.com/TrinityCore/TrinityCore.git
cd TrinityCore
```

### 3. Compile Server

```bash
mkdir build && cd build
cmake ../ -DCMAKE_INSTALL_PREFIX=$HOME/trinitycore-server
make -j$(nproc)
make install
```

**Compilation time:** ~30-60 minutes depending on your system.

### 4. Setup Database

```bash
# Create MySQL databases
mysql -u root -p

CREATE DATABASE world;
CREATE DATABASE characters;
CREATE DATABASE auth;
CREATE USER 'trinity'@'localhost' IDENTIFIED BY 'trinity';
GRANT ALL PRIVILEGES ON world.* TO 'trinity'@'localhost';
GRANT ALL PRIVILEGES ON characters.* TO 'trinity'@'localhost';
GRANT ALL PRIVILEGES ON auth.* TO 'trinity'@'localhost';
FLUSH PRIVILEGES;
EXIT;

# Import base database
cd ~/TrinityCore
mysql -u trinity -ptrinity auth < sql/base/auth_database.sql
mysql -u trinity -ptrinity characters < sql/base/characters_database.sql
mysql -u trinity -ptrinity world < sql/base/world_database.sql

# Download world database (TDB)
wget https://github.com/TrinityCore/TrinityCore/releases/download/TDB335.23041/TDB_full_world_335.23041_2023_04_11.sql
mysql -u trinity -ptrinity world < TDB_full_world_335.23041_2023_04_11.sql
```

### 5. Configure Server

```bash
cd ~/trinitycore-server/etc/

# Copy configuration templates
cp authserver.conf.dist authserver.conf
cp worldserver.conf.dist worldserver.conf

# Edit authserver.conf
nano authserver.conf
```

**Key settings in `authserver.conf`:**
```ini
LoginDatabaseInfo = "localhost;3306;trinity;trinity;auth"
RealmServerPort = 3724
BindIP = "localhost"
```

**Key settings in `worldserver.conf`:**
```ini
LoginDatabaseInfo = "localhost;3306;trinity;trinity;auth"
WorldDatabaseInfo = "localhost;3306;trinity;trinity;world"
CharacterDatabaseInfo = "localhost;3306;trinity;trinity;characters"

DataDir = "/path/to/your/WoW-3.3.5a/Data"  # Your WoW client data directory
```

### 6. Create Account

```bash
cd ~/trinitycore-server/bin/

# Start authserver first
./authserver

# In another terminal, start worldserver
./worldserver

# Wait for worldserver to fully load, then in worldserver console:
account create testuser testpass
account set gmlevel testuser 3 -1
```

### 7. Setup Realm

In the worldserver console:
```
realm add "Local Test Realm" localhost:8085 0 1
```

Or directly in database:
```sql
mysql -u trinity -ptrinity auth

INSERT INTO realmlist (name, address, port, icon, realmflags, timezone, allowedSecurityLevel)
VALUES ('Local Test Realm', 'localhost', 8085, 1, 0, 1, 0);
```

## Running the Server

### Start Services

**Terminal 1 - Auth Server:**
```bash
cd ~/trinitycore-server/bin/
./authserver
```

**Terminal 2 - World Server:**
```bash
cd ~/trinitycore-server/bin/
./worldserver
```

### Server Console Commands

**Useful commands in worldserver console:**

```bash
# Create account
account create username password

# Set GM level (0=player, 1=moderator, 2=GM, 3=admin)
account set gmlevel username 3 -1

# Teleport character
.tele ironforge
.tele stormwind

# Get server info
server info
server set motd Welcome to Test Server!

# List online players
account onlinelist

# Shutdown server
server shutdown 10  # Shutdown in 10 seconds
```

## Connecting with WoWee

### 1. Start the Client

```bash
cd /path/to/wowee
./build/bin/wowee
```

### 2. Login Screen

You'll see the authentication screen with default values:
- **Hostname:** localhost (already set)
- **Port:** 3724 (already set)
- **Username:** (enter your account username)
- **Password:** (enter your account password)

### 3. Connect

1. Enter your credentials (e.g., `testuser` / `testpass`)
2. Click **Connect**
3. You should see "Authentication successful!"
4. Select your realm from the realm list
5. Create or select a character
6. Enter the world!

## Troubleshooting

### Connection Refused

**Problem:** Cannot connect to auth server

**Solutions:**
```bash
# Check if authserver is running
ps aux | grep authserver

# Check if port is listening
netstat -an | grep 3724
sudo lsof -i :3724

# Check firewall
sudo ufw allow 3724
sudo ufw status

# Verify MySQL is running
sudo systemctl status mysql
```

### Authentication Failed

**Problem:** "Authentication failed" error

**Solutions:**
```bash
# Verify account exists
mysql -u trinity -ptrinity auth
SELECT * FROM account WHERE username='testuser';

# Reset password
# In worldserver console:
account set password testuser newpass newpass

# Check auth server logs
tail -f ~/trinitycore-server/logs/Auth.log
```

### Realm List Empty

**Problem:** No realms showing after login

**Solutions:**
```bash
# Check realm configuration in database
mysql -u trinity -ptrinity auth
SELECT * FROM realmlist;

# Verify world server is running
ps aux | grep worldserver

# Check world server port
netstat -an | grep 8085

# Update realmlist address
UPDATE realmlist SET address='localhost' WHERE id=1;
```

### Cannot Enter World

**Problem:** Stuck at character selection or disconnect when entering world

**Solutions:**
```bash
# Check worldserver logs
tail -f ~/trinitycore-server/logs/Server.log

# Verify Data directory in worldserver.conf
DataDir = "/path/to/WoW-3.3.5a/Data"

# Ensure maps are extracted
cd ~/WoW-3.3.5a/
ls -la maps/  # Should have .map files

# Extract maps if needed (from TrinityCore tools)
cd ~/trinitycore-server/bin/
./mapextractor
./vmap4extractor
./vmap4assembler
./mmaps_generator
```

## Network Configuration

### Local Network Testing

To test from another machine on your network:

**1. Find your local IP:**
```bash
ip addr show | grep inet
# Or
ifconfig | grep inet
```

**2. Update server configuration:**

Edit `authserver.conf`:
```ini
# Local-only testing on the same machine:
BindIP = "127.0.0.1"

# LAN/remote testing (listen on all interfaces):
# BindIP = "0.0.0.0"
```

Edit database:
```sql
mysql -u trinity -ptrinity auth
UPDATE realmlist SET address='<your-lan-ip>' WHERE id=1;  # Your local IP
```

**3. Configure firewall:**
```bash
sudo ufw allow 3724  # Auth server
sudo ufw allow 8085  # World server
```

**4. In wowee:**
- Change hostname to your server's local IP (e.g., <your-lan-ip>)
- Keep port as 3724
- Connect

### Remote Server Testing

For testing with a remote server (VPS, dedicated server):

**Client configuration:**
- **Hostname:** server.example.com or remote IP
- **Port:** 3724 (or custom port)

**Server configuration:**
```ini
# authserver.conf
# Public/remote access:
BindIP = "0.0.0.0"

# Database
UPDATE realmlist SET address='your.server.ip' WHERE id=1;
```

## WoW Data Files

The client needs access to extracted WoW data (terrain, models, textures) indexed by `manifest.json`.

If you have a fresh WoW install (MPQs only), extract once with:

```bash
./extract_assets.sh /path/to/WoW-3.3.5a/Data wotlk
```

### Setting WOW_DATA_PATH

```bash
# Linux/Mac
export WOW_DATA_PATH="/path/to/extracted/Data"

# Or add to ~/.bashrc
echo 'export WOW_DATA_PATH="/path/to/extracted/Data"' >> ~/.bashrc
source ~/.bashrc

# Run client
cd /path/to/wowee
./build/bin/wowee
```

### Data Directory Structure

Your extracted data directory should contain (example):
```
Data/
├── manifest.json
├── interface/
├── sound/
├── world/
└── expansions/
```

## Testing Features

### In-Game Testing

Once connected and in-world, test client features:

**Camera Controls:**
- **WASD** - Move camera
- **Mouse** - Look around
- **Shift** - Move faster

**UI/Gameplay Windows:**
- **B** - Toggle bags
- **C** - Toggle character
- **P** - Toggle spellbook
- **N** - Toggle talents
- **L** - Toggle quest log
- **M** - Toggle world map
- **O** - Toggle guild roster

**Debug Features:**
- **F1** - Toggle performance HUD
- **F4** - Toggle shadows

### Performance Monitoring

Press **F1** to show/hide the performance HUD which displays:
- **FPS** - Frames per second (color-coded: green=60+, yellow=30-60, red=<30)
- **Frame time** - Milliseconds per frame
- **Renderer stats** - Draw calls, triangles
- **WMO stats** - Building models and instances
- **Camera position** - X, Y, Z coordinates

## Server Administration

### GM Commands (in worldserver console or in-game)

**Character Management:**
```
.character level 80              # Set level to 80
.character rename                # Flag character for rename
.character customize             # Flag for appearance change
.levelup 80                      # Increase level by 80
```

**Item/Gold:**
```
.additem 25 10                   # Add 10 of item ID 25
.modify money 1000000            # Add 10 gold (in copper)
.lookup item sword               # Find item IDs
```

**Teleportation:**
```
.tele stormwind                  # Teleport to Stormwind
.tele ironforge                  # Teleport to Ironforge
.gps                             # Show current position
```

**World Management:**
```
.server set motd Welcome!        # Set message of the day
.announce Message                # Server-wide announcement
.server shutdown 60              # Shutdown in 60 seconds
```

## Performance Tips

### Server Optimization

**worldserver.conf settings for testing:**
```ini
# Faster respawn times for testing
Corpse.Decay.NORMAL = 30
Corpse.Decay.RARE = 60
Corpse.Decay.ELITE = 60

# Faster leveling for testing
Rate.XP.Kill = 2
Rate.XP.Quest = 2

# More gold for testing
Rate.Drop.Money = 2

# Instant flight paths (testing)
Rate.Creature.Normal.Damage = 1
Rate.Player.Haste = 1
```

### Client Performance

- Keep performance HUD (F1) enabled to monitor FPS
- Reduce quality/effects from Settings if FPS drops

## Security Notes

⚠️ **For Local Testing Only**

This setup is for **local development and testing** purposes:
- Default passwords are insecure
- No SSL/TLS encryption
- MySQL permissions are permissive
- Ports are open without authentication

**Do not expose these settings to the internet without proper security configuration.**

## Additional Resources

### Server Emulators
- **TrinityCore**: https://trinitycore.info/
- **AzerothCore**: https://www.azerothcore.org/
- **MaNGOS**: https://getmangos.eu/

### Database Tools
- **Keira3** - Visual database editor: https://github.com/azerothcore/Keira3
- **HeidiSQL** - MySQL client: https://www.heidisql.com/

### WoW Development
- **WoWDev Wiki**: https://wowdev.wiki/
- **TrinityCore Forum**: https://community.trinitycore.org/
- **AzerothCore Discord**: https://discord.gg/azerothcore

### Map/DBC Tools
- **WoW Blender Studio**: https://github.com/Marlamin/WoW-Blender-Studio
- **WDBXEditor**: https://github.com/WowDevTools/WDBXEditor

## Example Testing Session

### Complete Workflow

1. **Start Server:**
```bash
# Terminal 1
cd ~/trinitycore-server/bin && ./authserver

# Terminal 2
cd ~/trinitycore-server/bin && ./worldserver
```

2. **Create Test Account (in worldserver console):**
```
account create demo demopass
account set gmlevel demo 3 -1
```

3. **Start Client:**
```bash
cd /path/to/wowee
export WOW_DATA_PATH="/path/to/extracted/Data"
./build/bin/wowee
```

4. **Connect:**
- Username: `demo`
- Password: `demopass`
- Click Connect

5. **Test Features:**
- Create a character
- Enter world
- Test windows (`B`, `C`, `P`, `N`, `L`, `M`, `O`)
- Test vendor flow (buy, sell, buyback)
- Test quest flow (accept, progress tracking, turn-in markers)
- Test movement (WASD, mouse)

6. **Stop Server (worldserver console):**
```
server shutdown 10
```

## Troubleshooting Checklist

- [ ] MySQL server running
- [ ] Databases created and populated
- [ ] authserver running and listening on port 3724
- [ ] worldserver running and listening on port 8085
- [ ] Realmlist configured with correct address
- [ ] Account created with proper credentials
- [ ] Firewall allows ports 3724 and 8085
- [ ] WOW_DATA_PATH set correctly (if using MPQ assets)
- [ ] Client can resolve hostname (localhost for localhost)

## Next Steps

Once you have a working local server connection:
1. Test network protocol implementation
2. Validate packet handling
3. Test character creation and login
4. Verify world entry and movement
5. Test rendering with real terrain data (requires WOW_DATA_PATH)
6. Profile performance with actual game data

---

**Status**: Ready for local server testing
**Last Updated**: 2026-01-27
**Client Version**: 1.0.3
**Server Compatibility**: WoW 3.3.5a (12340)
