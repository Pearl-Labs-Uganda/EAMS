# RC Car — USB Ethernet Gadget Link

**Date:** 16 July 2026
**Addendum to:** RC Car Project — Progress Report & Replication Guide

---

## 1. Where we were

The Pi was reachable only over WiFi at `172.20.10.2`, on a phone hotspot shared with the dev laptop (`172.20.10.3`). This is the link used for `rsync` deploys, SSH, and the browser app on port 8080.

Problem: the Zero 2 W's WiFi is known to be flaky under sustained load (see requirements §9). Every deploy, every telemetry test, and every debugging session rides on it. A dropout mid-`rsync` or mid-telemetry-test is indistinguishable from a code bug.

## 2. Where we got to

A **USB Ethernet gadget link**: the Pi presents itself as a network adapter over the USB data cable. Wired, point-to-point, no hotspot in the loop.

| | |
|---|---|
| Pi | `10.55.0.1/24` on `usb0` |
| Laptop | `10.55.0.2/24` on `enp0s20f0u1` |
| Persistence | Both sides survive reboot/unplug (NetworkManager profiles) |
| WiFi | Still works, unchanged, in parallel |
| Auth | SSH key installed on the Pi |

Verified: post-reboot `uptime` 2 min, `usb0` self-assigned `10.55.0.1`, `nmcli` shows it connected via the `usb0` profile (not the competing netplan one), ping 0% loss at ~0.3 ms.

---

## 3. How to replicate

### 3.1 Enable the dwc2 overlay (Pi)

```bash
sudo sed -i '$ a dtoverlay=dwc2' /boot/firmware/config.txt
```
*Objective:* switch the Pi's USB port from host-only to dual-role, so it can act as a **device** (gadget) rather than only accepting peripherals.

### 3.2 Load the gadget modules at boot (Pi)

```bash
sudo sed -i 's/rootwait/rootwait modules-load=dwc2,g_ether/' /boot/firmware/cmdline.txt
```
*Objective:* load `dwc2` (the dual-role USB controller driver) and `g_ether` (the Ethernet-gadget driver, which creates `usb0`) early in boot.

**Verify before rebooting** — a malformed `cmdline.txt` will not boot:
```bash
tail -3 /boot/firmware/config.txt
cat /boot/firmware/cmdline.txt
```
`cmdline.txt` must remain a **single line**. `sed` appends in place here, so this is safe, but check anyway.

### 3.3 Give usb0 a static address (Pi)

```bash
sudo nmcli con add type ethernet ifname usb0 con-name usb0 \
  ipv4.method manual ipv4.addresses 10.55.0.1/24 autoconnect yes
```
*Objective:* a fixed address on the gadget interface. `10.55.0.0/24` is arbitrary — just needs to not collide with the hotspot's `172.20.10.0/24`.

### 3.4 Reboot

```bash
sudo reboot
```
Note: plain `reboot` fails with `Interactive authentication required` — it needs `sudo`.

Cable: laptop → the Pi's **USB** port (inner), **not PWR**. Must be a data cable, not charge-only. Power stays on PWR as normal.

### 3.5 Fix the profile race (Pi)

After reboot, `ip -4 addr show usb0` showed **no address**. Cause: a netplan-generated profile (`netplan-eth0`) also claims `usb0` and was sitting in `connecting (getting IP configuration)` — waiting forever on a DHCP server that doesn't exist.

```bash
nmcli -f NAME,DEVICE,STATE con show      # diagnosis: netplan-eth0 holding usb0
sudo nmcli device set usb0 managed yes
sudo nmcli con up usb0
sudo nmcli con mod usb0 connection.autoconnect-priority 100
sudo nmcli con mod usb0 connection.interface-name usb0
```
*Objective:* activate our profile and give it a higher autoconnect priority so it wins the race against `netplan-eth0` on every subsequent boot. This is the step that makes the link persistent rather than a one-off.

### 3.6 Laptop side

Find the interface (`ip link` → e.g. `enp0s20f0u1`), then create a persistent profile:

```bash
sudo nmcli con add type ethernet ifname enp0s20f0u1 con-name pi-usb \
  ipv4.method manual ipv4.addresses 10.55.0.2/24 ipv4.never-default yes autoconnect yes
```
*Objective:* fixed address that survives unplug. `ipv4.never-default yes` is important — without it the gadget link can steal the laptop's default route and break normal internet.

Temporary equivalent (dies on reboot, fine for testing):
```bash
sudo ip addr add 10.55.0.2/24 dev enp0s20f0u1
sudo ip link set enp0s20f0u1 up
```

### 3.7 SSH key

```bash
ssh-copy-id eams-pi@10.55.0.1
```

---

## 4. Verification

```bash
ping -c2 10.55.0.1
ssh eams-pi@10.55.0.1 'uptime -p; ip -4 addr show usb0; nmcli device status'
```

Expect: uptime consistent with the reboot you just did (proves it's not a stale pre-shutdown response), `inet 10.55.0.1/24` on `usb0`, and `usb0 ethernet connected usb0` — **connection column must read `usb0`, not `netplan-eth0`**.

---

## 5. Using it

Deploy over the wire instead of WiFi:
```bash
rsync -avz --delete rccar/ eams-pi@10.55.0.1:/home/eams-pi/rccar/
```

The web app is also reachable at `http://10.55.0.1:8080` from the laptop — useful for testing telemetry rate and WebSocket behaviour without WiFi flakiness as a confounding variable. (Phone control still goes over WiFi; the gadget link is laptop-only by nature.)

Optional `~/.ssh/config` on the laptop:
```
Host pi
  HostName 10.55.0.1
  User eams-pi
  IdentityFile ~/.ssh/id_ed25519
```
Then `ssh pi` and `rsync -avz --delete rccar/ pi:/home/eams-pi/rccar/`.

---

## 6. Open items

1. **Duplicate `pi-usb` profile on the laptop.** `nmcli con add` warned that a connection with that name already existed and added a second one anyway. Check and delete the stale one, or NM may activate the wrong profile:
   ```bash
   nmcli -f NAME,UUID,DEVICE,STATE con show | grep pi-usb
   sudo nmcli con delete <old-uuid>
   ```
2. **`netplan-eth0` still exists** and still claims `usb0`. The priority bump beats it, but it's fragile — a netplan regeneration could reset things. Cleaner fix if it ever misbehaves: remove the netplan yaml that generates it, or `sudo nmcli con delete netplan-eth0`.
3. **SSH agent passphrase prompt.** `sign_and_send_pubkey: agent refused operation` on first key use — the agent couldn't unlock `id_ed25519`. Resolved (passphrase recovered). If it recurs: `ssh-add -D && ssh-add ~/.ssh/id_ed25519`.
4. **The Pi's USB port is now gadget-only.** No USB host peripherals on that port. PWR port is unaffected.

---

## 7. Glossary

- **dwc2** — The Pi's USB controller driver in dual-role mode; lets the port act as a device rather than only a host.
- **g_ether** — Kernel gadget driver that makes the Pi appear to the host as a USB Ethernet adapter, creating the `usb0` interface.
- **USB gadget** — A device that presents itself as a peripheral over USB (keyboard, drive, network adapter) instead of hosting peripherals.
- **cmdline.txt** — Kernel command line, read at boot. Must be exactly one line.
- **NetworkManager / nmcli** — The service that configures network interfaces on Trixie, and its CLI.
- **autoconnect-priority** — Which NM profile wins when two profiles claim the same interface. Higher number wins.
- **never-default** — Tells NM not to use this connection as the default route for general internet traffic.
- **LOWER_UP** — Flag in `ip link` meaning the physical link is live (cable plugged in and the other end responding).
