# Project launcher convention

- The user's single launcher is the root `Start.bat`. Recommend it in responses and documentation.
- Double-clicking it opens the latest integrated forklift project in Unreal Editor AND the local live dashboard; the user then presses Play.
- Keep future features in the default `integrated` preset in `Scripts/Start.ps1`. Do not add new `Start-*.bat` files for features or versions.
- Optional older fixtures use `Start.bat -Mode ...`; `-Game` selects standalone mode. `Scripts/Run.ps1` remains the low-level launcher and the automated test scripts remain separate.
- `Start.bat -Website` opens only the live dashboard. The user explicitly requested the root `Website.bat` as a website-only shortcut; it delegates to `Start.bat -Website` and is the sole exception to the single-launcher convention. The Node loopback bridge receives actual Unreal telemetry; never replace live values with a browser physics calculator or demo defaults. `-NoWebsite` disables the dashboard; `-WebPort` selects the local port. No other start batch files.
