import Foundation

/// Everything the app needs to know about the world before it can run.
public enum ExtractorLocation {

    /// The name of the extraction binary inside Wowee.app.
    public static let executableName = "asset_extract"

    /// Find `asset_extract`, or say why it could not be found.
    ///
    /// The search order is the one the shipped shell launcher uses - beside
    /// this app first, then the two places a Mac app gets installed - with one
    /// difference that matters. The shell version accepts any directory named
    /// `Wowee.app`; this one only accepts a bundle that actually contains an
    /// executable extractor. An old or half-copied Wowee.app in /Applications
    /// otherwise shadows a good one in ~/Applications, and the user is told to
    /// look in the very folders they already installed into.
    /// The two places a Mac app is installed, in the order they are tried.
    public static func standardInstallLocations(
        fileManager: FileManager = .default
    ) -> [URL] {
        [
            URL(fileURLWithPath: "/Applications/Wowee.app"),
            fileManager.homeDirectoryForCurrentUser
                .appendingPathComponent("Applications/Wowee.app"),
        ]
    }

    public static func findExtractor(
        nextTo appURL: URL? = Bundle.main.bundleURL,
        installLocations: [URL]? = nil,
        fileManager: FileManager = .default
    ) -> URL? {
        var candidates: [URL] = []

        // Beside this app: the DMG layout, and what the README asks for.
        if let siblingRoot = appURL?.deletingLastPathComponent() {
            candidates.append(siblingRoot.appendingPathComponent("Wowee.app"))
        }
        candidates.append(
            contentsOf: installLocations ?? standardInstallLocations(fileManager: fileManager)
        )
        // A developer build, run straight out of the tree.
        if let appURL {
            candidates.append(appURL.deletingLastPathComponent())
        }

        for candidate in candidates {
            let executable = candidate
                .appendingPathComponent("Contents/MacOS")
                .appendingPathComponent(executableName)
            if fileManager.isExecutableFile(atPath: executable.path) {
                return executable
            }
            // A loose binary rather than a bundle, which is how it looks in a
            // build directory.
            let loose = candidate.appendingPathComponent(executableName)
            if fileManager.isExecutableFile(atPath: loose.path) {
                return loose
            }
        }
        return nil
    }

    /// The expansion profiles and opcode tables shipped inside Wowee.app.
    ///
    /// These are seeded into the output tree so a fresh extraction has the
    /// configuration the client needs. Returns nil when the app is not found
    /// or carries no bundled Data, which is not fatal - the extractor writes
    /// its own manifest either way.
    public static func bundledData(
        besideExtractor extractor: URL,
        fileManager: FileManager = .default
    ) -> URL? {
        // .../Wowee.app/Contents/MacOS/asset_extract -> .../Contents/Resources/Data
        let resources = extractor
            .deletingLastPathComponent()      // MacOS
            .deletingLastPathComponent()      // Contents
            .appendingPathComponent("Resources/Data")
        var isDirectory: ObjCBool = false
        guard fileManager.fileExists(atPath: resources.path, isDirectory: &isDirectory),
              isDirectory.boolValue else { return nil }
        return resources
    }

    /// Where extracted data goes, and where the client looks for it.
    ///
    /// Must stay in step with `selectMacUserDataPath()` in src/main.cpp, which
    /// reads exactly this path when WOW_DATA_PATH is unset.
    public static func defaultOutputDirectory(
        fileManager: FileManager = .default
    ) -> URL {
        fileManager.homeDirectoryForCurrentUser
            .appendingPathComponent("Library/Application Support/Wowee/Data")
    }
}

/// What a folder the user dropped turns out to be.
public struct DataFolderInspection: Equatable, Sendable {
    public let archiveCount: Int
    public let localeSubdirectories: [String]

    public var looksLikeWoWData: Bool { archiveCount > 0 }

    public init(archiveCount: Int, localeSubdirectories: [String]) {
        self.archiveCount = archiveCount
        self.localeSubdirectories = localeSubdirectories
    }

    /// Look for MPQ archives in the folder and one level down.
    ///
    /// One level matters: a WoW Data directory keeps its locale archives in
    /// `enUS/`, `frFR/` and so on, and some installs put nothing but those
    /// subfolders at the top. Refusing such a folder would reject a perfectly
    /// good Data directory - the extractor itself walks them
    /// (`extractor.cpp:598` is the error the user would otherwise hit only
    /// after committing to a run).
    public static func inspect(
        _ folder: URL,
        fileManager: FileManager = .default
    ) -> DataFolderInspection {
        func archives(in directory: URL) -> Int {
            let contents = (try? fileManager.contentsOfDirectory(
                at: directory,
                includingPropertiesForKeys: nil,
                options: [.skipsHiddenFiles]
            )) ?? []
            return contents.filter { $0.pathExtension.lowercased() == "mpq" }.count
        }

        var total = archives(in: folder)
        var locales: [String] = []

        let subdirectories = (try? fileManager.contentsOfDirectory(
            at: folder,
            includingPropertiesForKeys: [.isDirectoryKey],
            options: [.skipsHiddenFiles]
        )) ?? []

        for entry in subdirectories {
            let isDirectory = (try? entry.resourceValues(forKeys: [.isDirectoryKey]))?
                .isDirectory ?? false
            guard isDirectory else { continue }
            let count = archives(in: entry)
            if count > 0 {
                total += count
                locales.append(entry.lastPathComponent)
            }
        }

        return DataFolderInspection(
            archiveCount: total,
            localeSubdirectories: locales.sorted()
        )
    }
}

/// Free space, and whether it is plausibly enough.
public enum DiskSpace {

    /// A full extraction is around 18 GB (README.md's own figure). Warn below
    /// that rather than filling the disk and failing somewhere in the middle,
    /// which is how the shell launcher behaves today: it checks nothing.
    public static let recommendedFreeBytes: Int64 = 20 * 1_000_000_000

    public static func availableBytes(
        at url: URL,
        fileManager: FileManager = .default
    ) -> Int64? {
        // Walk up to the nearest existing ancestor: the output directory is
        // usually about to be created and cannot be queried yet.
        var probe = url
        while !fileManager.fileExists(atPath: probe.path) {
            let parent = probe.deletingLastPathComponent()
            if parent.path == probe.path { return nil }
            probe = parent
        }
        let values = try? probe.resourceValues(
            forKeys: [.volumeAvailableCapacityForImportantUsageKey]
        )
        return values?.volumeAvailableCapacityForImportantUsage
    }

    public static func isTight(_ available: Int64?) -> Bool {
        guard let available else { return false }
        return available < recommendedFreeBytes
    }
}
