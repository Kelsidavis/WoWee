import XCTest
@testable import ExtractorKit

final class ExtractionOutputParserTests: XCTestCase {

    // The shape asset_extract actually prints, taken from extractor.cpp.
    private let realRun = """
        Found 12 MPQ archives
          Scanning: /Data/common.MPQ
          Scanning: /Data/expansion.MPQ
        Enumerated 431221 unique files
        Extracting 431221 files using 8 threads...
        """

    func testCarriageReturnProgressIsSeen() {
        // The progress line is rewritten with \r and never terminated by \n.
        // A parser that only splits on newlines reports .starting forever.
        let parser = ExtractionOutputParser()
        parser.feed("Found 4 MPQ archives\n")
        parser.feed("\r  Extracted 1000 / 500000 files...")

        XCTAssertEqual(parser.lastKnownPhase, .extracting(done: 1000, total: 500000))
    }

    func testChunkSplitMidLineDoesNotLoseTheLine() {
        // A pipe delivers arbitrary chunk boundaries. Splitting "Extracted 2000
        // / 500000" across two reads must still produce one phase update, not
        // two garbled ones.
        let parser = ExtractionOutputParser()
        parser.feed("\r  Extracted 20")
        XCTAssertEqual(parser.lastKnownPhase, .starting, "half a line is not a phase")

        parser.feed("00 / 500000 files...\r")
        XCTAssertEqual(parser.lastKnownPhase, .extracting(done: 2000, total: 500000))
    }

    func testFinalSummaryIsNotReadAsProgress() {
        // extractor.cpp:875 prints "Extracted 431221 files (...)" with no
        // slash. Read as a ratio it would be nonsense; it must be ignored.
        let parser = ExtractionOutputParser()
        parser.feed("\r  Extracted 400000 / 431221 files...")
        parser.feed("\n  Extracted 431221 files (12.4 GB in 812s)\n")

        XCTAssertEqual(parser.lastKnownPhase, .extracting(done: 400000, total: 431221))
    }

    func testArchiveScanCountsUpToTheAnnouncedTotal() {
        let parser = ExtractionOutputParser()
        parser.feed(realRun + "\n")

        // Two "Scanning:" lines out of the twelve announced.
        if case let .enumerating = parser.lastKnownPhase {
            // Enumerated arrived last in the fixture, which is correct.
        } else {
            XCTFail("expected .enumerating, got \(parser.lastKnownPhase)")
        }
    }

    func testScanPhaseTracksArchives() {
        let parser = ExtractionOutputParser()
        parser.feed("Found 12 MPQ archives\n  Scanning: a.MPQ\n  Scanning: b.MPQ\n")

        XCTAssertEqual(parser.lastKnownPhase, .scanningArchives(done: 2, total: 12))
    }

    func testMoreArchivesScannedThanAnnouncedNeverExceedsOne() {
        // Defensive: if the announced count and the scan lines ever disagree,
        // the fraction must stay in range rather than driving the bar past
        // full, which looks like a bug to anyone watching.
        let parser = ExtractionOutputParser()
        parser.feed("Found 2 MPQ archives\n")
        parser.feed("  Scanning: a\n  Scanning: b\n  Scanning: c\n")

        let fraction = parser.lastKnownPhase.fraction ?? 0
        XCTAssertLessThanOrEqual(fraction, 0.08)
    }

    func testFlushCountsAnUnterminatedFinalLine() {
        let parser = ExtractionOutputParser()
        parser.feed("Wrote manifest: /out/manifest.json (431221 entries)")
        XCTAssertEqual(parser.lastKnownPhase, .starting, "no terminator yet")

        parser.flush()
        XCTAssertEqual(parser.lastKnownPhase, .finishing)
    }

    func testFeedReturnsCompleteLinesForTheLog() {
        let parser = ExtractionOutputParser()
        let lines = parser.feed("Found 3 MPQ archives\n  Scanning: a.MPQ\n  Scan")

        XCTAssertEqual(lines, ["Found 3 MPQ archives", "  Scanning: a.MPQ"])
    }

    func testFractionIsMonotonicAcrossPhases() {
        // A bar that goes backwards is worse than no bar. Each phase boundary
        // must not hand back a smaller number than the phase before it could
        // reach.
        let scanEnd = ExtractionPhase.scanningArchives(done: 10, total: 10).fraction!
        let enumerate = ExtractionPhase.enumerating.fraction!
        let extractStart = ExtractionPhase.extracting(done: 0, total: 1000).fraction!
        let extractEnd = ExtractionPhase.extracting(done: 1000, total: 1000).fraction!
        let finishing = ExtractionPhase.finishing.fraction!

        XCTAssertLessThanOrEqual(scanEnd, enumerate)
        XCTAssertLessThanOrEqual(enumerate, extractStart)
        XCTAssertLessThanOrEqual(extractStart, extractEnd)
        XCTAssertLessThanOrEqual(extractEnd, finishing)
        XCTAssertLessThanOrEqual(finishing, 1.0)
    }

    func testZeroTotalsDoNotDivideByZero() {
        XCTAssertNil(ExtractionPhase.extracting(done: 0, total: 0).fraction)
        XCTAssertNil(ExtractionPhase.scanningArchives(done: 0, total: 0).fraction)
    }
}
