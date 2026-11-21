# Phone remote

Built into `cubi_fw.ino`. Cubi serves a web app over Wi-Fi; each button sends one serial command letter.

- **Default**: she runs her own network "Cubi". Join it and open http://192.168.4.1. Set your own password in `cubi_secrets.h` (see `cubi_secrets.example.h`).
- **Home Wi-Fi**: define `CUBI_HOME_SSID` / `CUBI_HOME_PASS` in `cubi_secrets.h`; she's at http://cubi.local and falls back to her own network after 8 s.
- The web server runs in its own task on core 1 and queues commands (32 chars). `/status` reports battery voltage, polled every 4 s.
- Buttons: modes M/P/F; walk w/b; U/L/c/Z; tricks 1-9; G; I; steer ( ); tuning (+ - [ ] k K y Y n g T); sensors (D R e d v t m h f o W); raw text field; pinned STOP (x).

## Auto mind mode
With no USB host at power-on she skips the clap and stress tests and enters mind mode. With the Serial Monitor open she waits 5 s; any key keeps command mode.

## Not done
No serial output in the app. No turn command for a joystick. Walks and tricks don't check for stop while running.
