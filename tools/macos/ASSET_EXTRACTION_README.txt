Wowee requires assets extracted from a compatible World of Warcraft client.

1. Drag both Wowee.app and "Wowee Asset Extractor.app" to the Applications
   shortcut. Running them from the disk image works, but a copy on the disk
   cannot be given permissions that stick.
2. Open Wowee.app. With nothing extracted yet, it opens the asset builder in
   place of the login screen. "Wowee Asset Extractor.app" opens the same
   builder as a window of its own, from a Terminal window that stays open
   while it runs.
3. Choose your World of Warcraft installation - the game folder or the Data
   folder inside it - then which game your server runs and whether you want
   the updated assets, and press "Build my assets".
4. When the build finishes, reopen Wowee.app.

To add a second game or rebuild the one you have, use "more options" on the
login screen.

Extracted files are stored in:
~/Library/Application Support/Wowee/Data

They remain available when Wowee.app is upgraded and do not modify the signed
application bundle.
