# Maintainer: <your name> <your@email>
# Contributor: <your name> <your@email>

pkgname=wowee-git
pkgver=r.1
pkgrel=1
pkgdesc="Open-source World of Warcraft client with Vulkan renderer (WotLK 3.3.5a / TBC / Classic)"
arch=('x86_64')
url="https://github.com/Kelsidavis/WoWee"
license=('MIT')
depends=(
  'sdl3'              # Windowing and event loop
  'vulkan-icd-loader' # Vulkan runtime (GPU driver communication)
  'openssl'           # SRP6a auth protocol (key exchange + RC4 encryption)
  'zlib'              # Network packet decompression and Warden module inflate
  'ffmpeg'            # Video playback (login cinematics)
  'unicorn'           # Warden anti-cheat x86 emulation (cross-platform, no Wine)
  'libx11'            # X11 windowing support
  'stormlib'          # AUR - MPQ extraction (wowee-extract-assets uses libstorm.so)
)
makedepends=(
  'git'               # Clone submodules (imgui, vk-bootstrap)
  'cmake'             # Build system
  'pkgconf'           # Dependency detection
  'glm'               # Header-only math library (vectors, matrices, quaternions)
  'vulkan-headers'    # Vulkan API definitions (build-time only)
  'shaderc'           # GLSL → SPIR-V shader compilation
  'python'            # Opcode registry generation and DBC validation scripts
)
provides=('wowee')
conflicts=('wowee')
source=("${pkgname}::git+https://github.com/Kelsidavis/WoWee.git#branch=master"
        "git+https://github.com/ocornut/imgui.git"
        "git+https://github.com/charles-lunarg/vk-bootstrap.git")
sha256sums=('SKIP' 'SKIP' 'SKIP')

pkgver() {
  cd "${pkgname}"
  printf "r%s.%s" "$(git rev-list --count HEAD)" "$(git rev-parse --short HEAD)"
}

prepare() {
  cd "${pkgname}"
  git submodule init
  git config submodule.extern/imgui.url "${srcdir}/imgui"
  git config submodule.extern/vk-bootstrap.url "${srcdir}/vk-bootstrap"
  git -c protocol.file.allow=always submodule update
}

build() {
  cmake -S "${pkgname}" -B build \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/usr \
    -Wno-dev
  cmake --build build --parallel "$(nproc)"
}

package() {
  DESTDIR="${pkgdir}" cmake --install build

  # Relocate real binaries from /usr/bin → /usr/lib/wowee/
  # so wrapper scripts can live at /usr/bin instead.
  install -dm755 "${pkgdir}/usr/lib/wowee"
  for bin in wowee asset_extract dbc_to_csv auth_probe auth_login_probe blp_convert; do
    if [[ -f "${pkgdir}/usr/bin/${bin}" ]]; then
      mv "${pkgdir}/usr/bin/${bin}" "${pkgdir}/usr/lib/wowee/${bin}"
    fi
  done

  # Main launcher: sets WOW_DATA_PATH to the user's XDG data dir.
  # The app uses WOW_DATA_PATH to locate Data/manifest.json at runtime.
  install -Dm755 /dev/stdin "${pkgdir}/usr/bin/wowee" <<'EOF'
#!/bin/sh
export WOW_DATA_PATH="${XDG_DATA_HOME:-$HOME/.local/share}/wowee/Data"
exec /usr/lib/wowee/wowee "$@"
EOF

  # Asset extraction helper: runs asset_extract and outputs to the XDG data dir.
  # Usage: wowee-extract-assets /path/to/WoW/Data [wotlk|tbc|classic]
  install -Dm755 /dev/stdin "${pkgdir}/usr/bin/wowee-extract-assets" <<'EOF'
#!/bin/sh
if [ -z "$1" ]; then
  echo "Usage: wowee-extract-assets /path/to/WoW/Data [wotlk|tbc|classic]"
  exit 1
fi
OUTPUT="${XDG_DATA_HOME:-$HOME/.local/share}/wowee/Data"
mkdir -p "${OUTPUT}"
exec /usr/lib/wowee/asset_extract --mpq-dir "$1" --output "${OUTPUT}" ${2:+--expansion "$2"}
EOF

  # License
  install -Dm644 "${pkgname}/LICENSE" \
    "${pkgdir}/usr/share/licenses/${pkgname}/LICENSE"

  # Post-install instructions (shown by pacman helpers that support it)
  install -Dm644 /dev/stdin \
    "${pkgdir}/usr/share/doc/${pkgname}/POST_INSTALL" <<'EOF'
==> WoWee requires game assets extracted from your own WoW client.
==> Run the following once, pointing at your WoW Data/ directory:
==>
==>   wowee-extract-assets /path/to/WoW-3.3.5a/Data wotlk
==>
==> Assets are written to ~/.local/share/wowee/Data/ (or $XDG_DATA_HOME/wowee/Data/).
==> Then launch the client with:  wowee
EOF
}
