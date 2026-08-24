import Foundation

/// What an extraction actually did, read off the extractor's own closing line.
///
/// `extractor.cpp:874` prints the whole tally in one go:
///
///     Extracted 431221 files (17845 MB), 12 skipped, 3 failed
///
/// So none of this is measured a second time on the Swift side. Walking the
/// output tree to weigh 400 000 files, or parsing a 20 MB manifest to add up
/// its `"s"` fields, would both take seconds to arrive at a number the
/// extractor already printed.
///
/// `skipped` and `failed` are the reason this is worth showing at all: a run
/// that ends green while quietly failing three thousand files is the failure
/// mode a success screen is supposed to catch.
public struct ExtractionSummary: Equatable, Sendable {
    public let filesExtracted: Int
    public let megabytesWritten: Int
    public let filesSkipped: Int
    public let filesFailed: Int

    public init(filesExtracted: Int, megabytesWritten: Int, filesSkipped: Int, filesFailed: Int) {
        self.filesExtracted = filesExtracted
        self.megabytesWritten = megabytesWritten
        self.filesSkipped = filesSkipped
        self.filesFailed = filesFailed
    }

    /// True when the run finished but did not come out clean.
    ///
    /// Skipped files are normal - the extractor skips what is already there.
    /// Failed ones never are.
    public var hasFailures: Bool { filesFailed > 0 }

    public var bytesWritten: Int64 { Int64(megabytesWritten) * 1024 * 1024 }

    /// Parse the closing line, or return nil for any other line.
    ///
    /// The discriminator against the *progress* line - which also opens with
    /// "Extracted " - is the slash. `Extracted 12000 / 431221 files...` has
    /// one; this line has none and carries a parenthesised size instead.
    /// Reading one as the other is what would put a half-finished count on the
    /// success screen.
    public static func parse(_ rawLine: String) -> ExtractionSummary? {
        let line = rawLine.trimmingCharacters(in: .whitespaces)
        guard line.hasPrefix("Extracted "), !line.contains("/") else { return nil }

        guard let extracted = ExtractionOutputParser.integer(
            in: line, after: "Extracted ", before: " files"
        ) else { return nil }

        // The size is the only number inside parentheses.
        guard let megabytes = ExtractionOutputParser.integer(
            in: line, after: "(", before: " MB)"
        ) else { return nil }

        // These two are absent from older builds of the extractor, so a line
        // without them still parses - it just reports zero rather than nothing.
        let skipped = trailingCount(in: line, before: " skipped") ?? 0
        let failed = trailingCount(in: line, before: " failed") ?? 0

        return ExtractionSummary(
            filesExtracted: extracted,
            megabytesWritten: megabytes,
            filesSkipped: skipped,
            filesFailed: failed
        )
    }

    /// The digits immediately to the left of `suffix`.
    private static func trailingCount(in line: String, before suffix: String) -> Int? {
        guard let end = line.range(of: suffix) else { return nil }
        let digits = line[line.startIndex..<end.lowerBound]
            .reversed().prefix { $0.isNumber }.reversed()
        return Int(String(digits))
    }
}
