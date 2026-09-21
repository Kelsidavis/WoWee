#pragma once

/// Realm-supplied patch archives, and the rules that keep one realm's
/// downloads out of every other realm's.
///
/// A patch MPQ is not only art. FrameXML loads Lua out of the archives, so a
/// file a realm hands over is closer to a plugin than to a texture pack, and
/// the realm that supplies it is one whose address somebody typed into a box.
/// Two things follow, and both live here rather than at the call sites:
///
///   - Every realm gets a directory of its own, named from its address by a
///     rule that cannot produce a path. Nothing a manifest says can widen
///     that, because the directory is derived from the address the player
///     connected to and never from the manifest.
///   - Every name inside a manifest is checked before it is joined to
///     anything. A name is a bare file name or it is refused; there is no
///     sanitising pass that tries to repair one.
///
/// The parsing and the naming are pure functions on purpose: this is the part
/// that has to be right, and it is the part that can be tested without a
/// network or a server.

#include <cstdint>
#include <string>
#include <vector>

namespace wowee::core {

/// One archive a realm offers.
struct PatchEntry {
    /// A bare file name, already checked by isSafePatchFileName.
    std::string file;
    /// Bytes, as the manifest declares them. A download that does not match
    /// is refused rather than truncated.
    std::uint64_t size = 0;
    /// Lower-case hex, 64 characters. Checked after download, before the file
    /// is moved anywhere the client will read it.
    std::string sha256;
};

struct PatchManifest {
    bool ok = false;
    /// Why it was refused. Empty when ok.
    std::string error;
    std::vector<PatchEntry> entries;
};

/// The most a single archive may declare, and the most one realm may.
///
/// A cap rather than trust: the size decides how much is written to the
/// player's disk, and it arrives from the same place as everything else here.
inline constexpr std::uint64_t kMaxPatchBytes = 4ull * 1024 * 1024 * 1024;
inline constexpr std::uint64_t kMaxTotalPatchBytes = 8ull * 1024 * 1024 * 1024;
inline constexpr std::size_t kMaxPatchEntries = 64;

/// Whether `name` may be joined to a realm's patch directory.
///
/// A bare file name ending in .mpq, of the characters an MPQ is ever actually
/// called: letters, digits, dot, dash, underscore. Everything else is refused,
/// including every spelling of a path - separators of both kinds, "..", a
/// leading dot, a drive letter, a UNC prefix - and the device names Windows
/// resolves before it looks at the filesystem.
///
/// Refusing rather than sanitising is the point. A repaired name is a name
/// somebody chose and the client then changed, and the interesting cases are
/// the ones where the repair produces something that still resolves.
[[nodiscard]] bool isSafePatchFileName(const std::string& name);

/// The directory this realm's archives live in, relative to the patch root.
///
/// Derived from the address connected to, never from anything a server sends.
/// The readable part is for a person looking in the folder; the hash is what
/// makes it unambiguous, since sanitising two different hostnames can arrive
/// at the same letters and they must not arrive at the same directory.
[[nodiscard]] std::string realmPatchDirName(const std::string& host, int port);

/// Parse a manifest document.
///
/// Refuses the whole thing rather than skipping a bad entry: a manifest with
/// one unusable name in it is one this client does not understand, and
/// installing the rest would be deciding on the player's behalf which parts of
/// a realm's patch set they can do without.
[[nodiscard]] PatchManifest parsePatchManifest(const std::string& json);

}  // namespace wowee::core
