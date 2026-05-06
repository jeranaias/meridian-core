# Deployment Guide

## GitHub Pages (recommended — free HTTPS, PWA install works)

1. Push this repo to GitHub
2. Go to **Settings → Pages**
3. Source: **Deploy from branch → main → / (root)**
4. Save — live at `https://[your-org].github.io/[repo-name]` in ~60 seconds

On tablet, open the URL → browser shows **"Add to Home Screen"** prompt automatically.

---

## Local network (ship / no internet)

```bash
# On the laptop/companion computer
python3 -m http.server 8080 --bind 0.0.0.0

# On tablet (same WiFi)
http://[laptop-ip]:8080
```

> Note: PWA install prompt requires HTTPS. Local HTTP works as a web app but won't show the install banner. Use GitHub Pages or the option below for full PWA install.

---

## HTTPS on local network (full PWA install without internet)

```bash
# Install mkcert (one-time)
brew install mkcert        # macOS
# or: https://github.com/FiloSottile/mkcert

mkcert -install
mkcert localhost [your-laptop-ip]

# Serve with HTTPS
npx serve . --ssl-cert localhost+1.pem --ssl-key localhost+1-key.pem -p 8443

# On tablet
https://[laptop-ip]:8443
# Accept the certificate → Add to Home Screen
```

---

## Netlify (one-drag deploy, free HTTPS)

1. Go to [netlify.com/drop](https://app.netlify.com/drop)
2. Drag the `vanguard_pwa/` folder onto the page
3. Done — live HTTPS URL in 30 seconds

---

## Connecting to Meridian bridge

Once deployed, update the WebSocket URL in `index.html`:

```javascript
// Find this line and update with your bridge IP
const BRIDGE_URL = 'ws://192.168.1.100:5760/telemetry';
```

The companion computer running `meridian-orca-bridge` must be on the same network as the tablet.

```bash
# Verify bridge is running
systemctl --user status meridian-orca-bridge

# Check logs
journalctl -u meridian-orca-bridge -n 200
```
