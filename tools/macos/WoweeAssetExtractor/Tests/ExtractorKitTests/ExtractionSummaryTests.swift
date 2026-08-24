import XCTest
@testable import ExtractorKit

final class ExtractionSummaryTests: XCTestCase {

    /// The line as extractor.cpp:874 actually prints it, leading spaces and all.
    func testClosingTallyIsRead() {
        let summary = ExtractionSummary.parse(
            "  Extracted 431221 files (17845 MB), 12 skipped, 3 failed"
        )
        XCTAssertEqual(summary?.filesExtracted, 431221)
        XCTAssertEqual(summary?.megabytesWritten, 17845)
        XCTAssertEqual(summary?.filesSkipped, 12)
        XCTAssertEqual(summary?.filesFailed, 3)
        XCTAssertEqual(summary?.hasFailures, true)
    }

    /// THE failure this parser exists to avoid.
    ///
    /// The progress line opens with the same two words. Reading one as the
    /// other puts a half-finished count on the success screen - and, worse,
    /// reads "431221" as a byte total.
    func testProgressLineIsNotATally() {
        XCTAssertNil(ExtractionSummary.parse("  Extracted 12000 / 431221 files..."))
        XCTAssertNil(ExtractionSummary.parse("Extracted 1 / 2 files..."))
    }

    func testUnrelatedLinesAreNotTallies() {
        XCTAssertNil(ExtractionSummary.parse("Found 14 MPQ archives"))
        XCTAssertNil(ExtractionSummary.parse("Enumerated 431221 unique files"))
        XCTAssertNil(ExtractionSummary.parse("Wrote manifest: /tmp/manifest.json (431221 entries)"))
        XCTAssertNil(ExtractionSummary.parse(""))
    }

    /// An older extractor prints the count and the size but not the two
    /// counters. That line still has to give a usable summary rather than none.
    func testTallyWithoutSkippedOrFailed() {
        let summary = ExtractionSummary.parse("  Extracted 100 files (5 MB)")
        XCTAssertEqual(summary?.filesExtracted, 100)
        XCTAssertEqual(summary?.megabytesWritten, 5)
        XCTAssertEqual(summary?.filesSkipped, 0)
        XCTAssertEqual(summary?.filesFailed, 0)
        XCTAssertEqual(summary?.hasFailures, false)
    }

    func testMegabytesConvertToBytes() {
        let summary = ExtractionSummary(
            filesExtracted: 1, megabytesWritten: 2, filesSkipped: 0, filesFailed: 0
        )
        XCTAssertEqual(summary.bytesWritten, 2 * 1024 * 1024)
    }

    /// Fed as a whole run, the parser keeps the tally and does not let the
    /// closing line disturb the phase it already settled on.
    func testParserKeepsTheTallyFromAFullRun() {
        let parser = ExtractionOutputParser()
        parser.feed("Found 2 MPQ archives\n")
        parser.feed("  Scanning: common.MPQ\n  Scanning: patch.MPQ\n")
        parser.feed("Enumerated 500 unique files\n")
        parser.feed("\r  Extracted 250 / 500 files...")
        XCTAssertNil(parser.summary, "no tally has been printed yet")

        parser.feed("\r  Extracted 500 files (42 MB), 0 skipped, 0 failed\n")
        XCTAssertEqual(parser.summary?.filesExtracted, 500)
        XCTAssertEqual(parser.summary?.megabytesWritten, 42)

        parser.feed("Wrote manifest: /tmp/manifest.json (500 entries)\n")
        XCTAssertEqual(parser.lastKnownPhase, .finishing)
        XCTAssertEqual(parser.summary?.filesExtracted, 500, "the manifest line must not clear it")
    }

    /// A run that never reaches its closing line reports nothing rather than
    /// a made-up figure.
    func testInterruptedRunHasNoTally() {
        let parser = ExtractionOutputParser()
        parser.feed("Found 2 MPQ archives\n")
        parser.feed("\r  Extracted 250 / 500 files...")
        XCTAssertNil(parser.summary)
    }
}

final class ExtractionFailureTests: XCTestCase {

    /// The regression this type exists to prevent: the advice was computed on
    /// every failure and dropped before it reached the screen.
    func testFailureCarriesTheRecoverySuggestion() {
        let failure = ExtractionFailure(.extractorNotFound)
        XCTAssertEqual(failure.message, "Wowee.app est introuvable.")
        XCTAssertEqual(
            failure.suggestion,
            "Placez cette application à côté de Wowee.app, ou installez Wowee.app dans Applications."
        )
    }

    func testNotAWoWDataFolderAlsoCarriesItsAdvice() {
        let failure = ExtractionFailure(.notAWoWDataFolder)
        XCTAssertEqual(failure.message, "Ce dossier ne contient aucune archive MPQ.")
        XCTAssertNotNil(failure.suggestion)
        XCTAssertTrue(failure.suggestion?.contains(".MPQ") == true)
    }

    /// A non-empty tail is the message; the exit code alone is the fallback.
    func testProcessFailureUsesTheTailWhenThereIsOne() {
        XCTAssertEqual(
            ExtractionFailure(.failed(exitCode: 1, tail: "No space left on device")).message,
            "No space left on device"
        )
        XCTAssertEqual(
            ExtractionFailure(.failed(exitCode: 9, tail: "")).message,
            "L'extraction s'est arrêtée (code 9)."
        )
    }
}
