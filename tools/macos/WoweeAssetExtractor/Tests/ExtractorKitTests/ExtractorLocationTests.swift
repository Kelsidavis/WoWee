import XCTest
@testable import ExtractorKit

final class ExtractorLocationTests: XCTestCase {

    private var sandbox: URL!

    override func setUpWithError() throws {
        sandbox = URL(fileURLWithPath: NSTemporaryDirectory())
            .appendingPathComponent("wowee-extractor-tests-\(UUID().uuidString)")
        try FileManager.default.createDirectory(
            at: sandbox, withIntermediateDirectories: true
        )
    }

    override func tearDownWithError() throws {
        try? FileManager.default.removeItem(at: sandbox)
    }

    private func makeAppBundle(
        at url: URL,
        withExecutable: Bool
    ) throws {
        let macOS = url.appendingPathComponent("Contents/MacOS")
        try FileManager.default.createDirectory(at: macOS, withIntermediateDirectories: true)
        guard withExecutable else { return }
        let binary = macOS.appendingPathComponent(ExtractorLocation.executableName)
        try "#!/bin/sh\nexit 0\n".write(to: binary, atomically: true, encoding: .utf8)
        try FileManager.default.setAttributes(
            [.posixPermissions: 0o755], ofItemAtPath: binary.path
        )
    }

    // MARK: - Locating the extractor

    func testFindsExtractorBesideTheApp() throws {
        let wowee = sandbox.appendingPathComponent("Wowee.app")
        try makeAppBundle(at: wowee, withExecutable: true)
        let me = sandbox.appendingPathComponent("Wowee Asset Extractor.app")

        // installLocations is emptied on purpose: this machine may well have a
        // real Wowee.app in /Applications, and a test that silently finds it
        // is testing the tester's Mac rather than the search order.
        let found = ExtractorLocation.findExtractor(nextTo: me, installLocations: [])

        XCTAssertEqual(
            found?.standardizedFileURL.path,
            wowee.appendingPathComponent("Contents/MacOS/asset_extract")
                .standardizedFileURL.path
        )
    }

    func testAnEmptyWoweeAppBesideUsIsNotAccepted() throws {
        // The bug this guards against: the shipped shell launcher tests
        // `[ -d Wowee.app ]`, so an incomplete bundle satisfies it and the
        // search stops there. Here a bundle with no executable must be
        // skipped so a good install elsewhere still wins.
        let hollow = sandbox.appendingPathComponent("Wowee.app")
        try makeAppBundle(at: hollow, withExecutable: false)
        let me = sandbox.appendingPathComponent("Wowee Asset Extractor.app")

        let found = ExtractorLocation.findExtractor(nextTo: me, installLocations: [])

        XCTAssertNil(
            found,
            "a Wowee.app carrying no extractor must not count as a hit"
        )
    }

    func testFindsALooseBinaryInABuildDirectory() throws {
        let binary = sandbox.appendingPathComponent(ExtractorLocation.executableName)
        try "#!/bin/sh\nexit 0\n".write(to: binary, atomically: true, encoding: .utf8)
        try FileManager.default.setAttributes(
            [.posixPermissions: 0o755], ofItemAtPath: binary.path
        )
        let me = sandbox.appendingPathComponent("WoweeAssetExtractor")

        XCTAssertEqual(
            ExtractorLocation.findExtractor(
                nextTo: me, installLocations: []
            )?.standardizedFileURL.path,
            binary.standardizedFileURL.path
        )
    }

    // MARK: - Inspecting a dropped folder

    func testFolderWithArchivesIsRecognised() throws {
        for name in ["common.MPQ", "expansion.MPQ", "patch.mpq"] {
            try "x".write(
                to: sandbox.appendingPathComponent(name),
                atomically: true, encoding: .utf8
            )
        }

        let inspection = DataFolderInspection.inspect(sandbox)

        XCTAssertTrue(inspection.looksLikeWoWData)
        XCTAssertEqual(inspection.archiveCount, 3, "the .mpq extension is case-insensitive")
    }

    func testLocaleSubdirectoriesCount() throws {
        // A Data folder whose archives all live in enUS/ is still a Data
        // folder. Rejecting it would turn a valid install into "ce dossier ne
        // contient aucune archive MPQ".
        let locale = sandbox.appendingPathComponent("frFR")
        try FileManager.default.createDirectory(at: locale, withIntermediateDirectories: true)
        try "x".write(
            to: locale.appendingPathComponent("locale-frFR.MPQ"),
            atomically: true, encoding: .utf8
        )

        let inspection = DataFolderInspection.inspect(sandbox)

        XCTAssertTrue(inspection.looksLikeWoWData)
        XCTAssertEqual(inspection.localeSubdirectories, ["frFR"])
    }

    func testAnUnrelatedFolderIsRejected() throws {
        try "x".write(
            to: sandbox.appendingPathComponent("notes.txt"),
            atomically: true, encoding: .utf8
        )

        XCTAssertFalse(DataFolderInspection.inspect(sandbox).looksLikeWoWData)
    }

    // MARK: - Seeding profiles

    func testSeedingDoesNotOverwriteAnEditedProfile() throws {
        // The whole point of not using ditto: a user who put their server's
        // wardenRsaModulus in expansion.json must still have it after the
        // next extraction.
        let source = sandbox.appendingPathComponent("bundled")
        let destination = sandbox.appendingPathComponent("out")
        let relative = "expansions/wotlk/expansion.json"

        try FileManager.default.createDirectory(
            at: source.appendingPathComponent("expansions/wotlk"),
            withIntermediateDirectories: true
        )
        try #"{"shipped": true}"#.write(
            to: source.appendingPathComponent(relative),
            atomically: true, encoding: .utf8
        )

        try FileManager.default.createDirectory(
            at: destination.appendingPathComponent("expansions/wotlk"),
            withIntermediateDirectories: true
        )
        try #"{"wardenRsaModulus": "mine"}"#.write(
            to: destination.appendingPathComponent(relative),
            atomically: true, encoding: .utf8
        )

        try ExtractionRunner.seedProfiles(from: source, into: destination)

        let kept = try String(
            contentsOf: destination.appendingPathComponent(relative), encoding: .utf8
        )
        XCTAssertEqual(kept, #"{"wardenRsaModulus": "mine"}"#)
    }

    func testSeedingCopiesWhatIsMissing() throws {
        let source = sandbox.appendingPathComponent("bundled")
        let destination = sandbox.appendingPathComponent("out")
        try FileManager.default.createDirectory(
            at: source.appendingPathComponent("expansions/tbc"),
            withIntermediateDirectories: true
        )
        try "{}".write(
            to: source.appendingPathComponent("expansions/tbc/opcodes.json"),
            atomically: true, encoding: .utf8
        )

        try ExtractionRunner.seedProfiles(from: source, into: destination)

        XCTAssertTrue(FileManager.default.fileExists(
            atPath: destination.appendingPathComponent("expansions/tbc/opcodes.json").path
        ))
    }

    // MARK: - Arguments

    func testAutoExpansionOmitsTheFlag() {
        let request = ExtractionRequest(
            dataFolder: URL(fileURLWithPath: "/Data"),
            outputFolder: URL(fileURLWithPath: "/out"),
            expansion: .auto
        )
        XCTAssertFalse(request.arguments().contains("--expansion"))
        XCTAssertTrue(request.arguments().contains("--expansion-subdir"))
    }

    func testExplicitExpansionIsPassedThrough() {
        let request = ExtractionRequest(
            dataFolder: URL(fileURLWithPath: "/Data"),
            outputFolder: URL(fileURLWithPath: "/out"),
            expansion: .wotlk,
            verify: true
        )
        let arguments = request.arguments()
        XCTAssertTrue(arguments.contains("--expansion"))
        XCTAssertTrue(arguments.contains("wotlk"))
        XCTAssertTrue(arguments.contains("--verify"))
    }

    // MARK: - Disk space

    func testTightSpaceIsFlagged() {
        XCTAssertTrue(DiskSpace.isTight(1_000_000_000))
        XCTAssertFalse(DiskSpace.isTight(500_000_000_000))
        XCTAssertFalse(DiskSpace.isTight(nil), "an unknown volume is not a warning")
    }
}
