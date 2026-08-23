import Foundation

/// Where the extractor currently is, and how far into it.
///
/// `asset_extract` announces each phase on stdout before it starts, and only
/// the extraction phase reports a running count. Everything else here is
/// derived from those announcements rather than guessed at, so a bar that
/// says 40% is answering for 40% of a real total.
public enum ExtractionPhase: Equatable, Sendable {
    /// Launched, nothing said yet. stdout is block-buffered through a pipe, so
    /// the opening lines can sit in the buffer until the first flush.
    case starting
    /// Reading the MPQ table of contents, one archive at a time.
    case scanningArchives(done: Int, total: Int)
    /// Archives read, building the unique file list.
    case enumerating
    /// The long one. Counted in files.
    case extracting(done: Int, total: Int)
    /// Files are on disk; writing manifest.json, and optionally verifying.
    case finishing
    case finished
    case failed(String)
    case cancelled

    /// A single 0...1 number for the whole run, or nil when the phase carries
    /// no honest fraction.
    ///
    /// The phases do not take equal time, so they do not get equal shares.
    /// Scanning is a handful of archives and enumeration is one pass over a
    /// list; extraction is minutes to hours. The weights below say so: the bar
    /// reaches 8% quickly, then spends the rest of its life in extraction,
    /// which is what the user actually watches.
    public var fraction: Double? {
        switch self {
        case .starting:
            return nil
        case let .scanningArchives(done, total):
            guard total > 0 else { return nil }
            return 0.08 * (Double(done) / Double(total))
        case .enumerating:
            return 0.08
        case let .extracting(done, total):
            guard total > 0 else { return nil }
            return 0.10 + 0.88 * (Double(done) / Double(total))
        case .finishing:
            return 0.98
        case .finished:
            return 1.0
        case .failed, .cancelled:
            return nil
        }
    }

    public var isTerminal: Bool {
        switch self {
        case .finished, .failed, .cancelled: return true
        default: return false
        }
    }
}

/// Turns `asset_extract`'s stdout into phases.
///
/// TWO THINGS MAKE THIS LESS TRIVIAL THAN IT LOOKS
///
/// The progress line is rewritten in place with a carriage return and no
/// newline (`extractor.cpp:851`), so splitting the stream on "\n" alone never
/// yields it until the run ends. Both terminators are separators here.
///
/// And a pipe is not a terminal: reads arrive in arbitrary chunks, so a line
/// can be delivered in halves. The parser keeps a tail between feeds rather
/// than assuming each chunk ends on a boundary - the failure that produces is
/// a bar that freezes at a round number and then jumps, which reads exactly
/// like a hung extraction.
public final class ExtractionOutputParser {
    private var tail = ""
    private var archivesFound = 0
    private var archivesScanned = 0

    public private(set) var lastKnownPhase: ExtractionPhase = .starting

    public init() {}

    /// Feed a chunk of stdout. Returns every complete line it contained, in
    /// order, so the caller can also show them as a log.
    @discardableResult
    public func feed(_ chunk: String) -> [String] {
        tail += chunk
        var lines: [String] = []

        // Split on either terminator, keeping whatever follows the last one as
        // the tail. "a\rb\n" yields ["a", "b"] and an empty tail; "a\rb"
        // yields ["a"] and holds "b" back until it is complete.
        while let index = tail.firstIndex(where: { $0 == "\n" || $0 == "\r" }) {
            let line = String(tail[tail.startIndex..<index])
            tail = String(tail[tail.index(after: index)...])
            if !line.isEmpty {
                lines.append(line)
                apply(line)
            }
        }

        // The progress line is only ever terminated by the *next* one, because
        // each tick opens with the carriage return that closes its
        // predecessor. Waiting for a terminator therefore shows a bar one tick
        // behind, and drops the final tick entirely. So read the unterminated
        // tail too - without consuming it, and only for the one line shape
        // that is safe to apply twice.
        applyIdempotently(tail)

        return lines
    }

    /// The extractor can exit with its last progress line still unterminated.
    /// Flushing makes that line count instead of being dropped.
    @discardableResult
    public func flush() -> [String] {
        let remainder = tail.trimmingCharacters(in: .whitespaces)
        tail = ""
        guard !remainder.isEmpty else { return [] }
        apply(remainder)
        return [remainder]
    }

    /// Apply only what a half-read line can safely say.
    ///
    /// `Scanning:` increments a counter, so reading it from an unconsumed tail
    /// would count the same archive on every subsequent feed. The extraction
    /// ratio carries its own absolute numbers, so applying it repeatedly lands
    /// on the same phase - that one is safe.
    private func applyIdempotently(_ rawTail: String) {
        let line = rawTail.trimmingCharacters(in: .whitespaces)
        guard line.hasPrefix("Extracted "), let (done, total) = Self.ratio(in: line) else { return }
        lastKnownPhase = .extracting(done: done, total: total)
    }

    private func apply(_ rawLine: String) {
        let line = rawLine.trimmingCharacters(in: .whitespaces)

        // "Found 12 MPQ archives" - extractor.cpp:602
        if let count = Self.integer(in: line, after: "Found ", before: " MPQ archives") {
            archivesFound = count
            archivesScanned = 0
            lastKnownPhase = .scanningArchives(done: 0, total: count)
            return
        }

        // "Scanning: <path>" - extractor.cpp:628, one per archive
        if line.hasPrefix("Scanning:") {
            archivesScanned += 1
            lastKnownPhase = .scanningArchives(
                done: min(archivesScanned, max(archivesFound, archivesScanned)),
                total: max(archivesFound, archivesScanned)
            )
            return
        }

        // "Enumerated 431221 unique files" - extractor.cpp:659
        if line.hasPrefix("Enumerated ") {
            lastKnownPhase = .enumerating
            return
        }

        // "Extracted 12000 / 431221 files..." - extractor.cpp:851, and the
        // final "Extracted 431221 files (1.2 GB in 240s)" at :875, which has
        // no slash and must not be read as a count of one.
        if line.hasPrefix("Extracted "), let (done, total) = Self.ratio(in: line) {
            lastKnownPhase = .extracting(done: done, total: total)
            return
        }

        // "Wrote manifest: ... (431221 entries)" - extractor.cpp:923
        if line.hasPrefix("Wrote manifest:") || line.hasPrefix("Verifying extracted files") {
            lastKnownPhase = .finishing
            return
        }
    }

    /// `12000 / 431221` anywhere in the line.
    ///
    /// The extractor puts spaces around the slash, so each half is trimmed
    /// before the digits are read off its inner edge. Without the trim the
    /// trailing-digit scan hits a space first, finds nothing, and the whole
    /// line is silently dropped - a bar that never moves.
    static func ratio(in line: String) -> (Int, Int)? {
        let parts = line.split(separator: "/")
        guard parts.count == 2 else { return nil }
        let left = parts[0].trimmingCharacters(in: .whitespaces)
        let right = parts[1].trimmingCharacters(in: .whitespaces)
        guard let done = trailingInteger(in: left),
              let total = leadingInteger(in: right),
              total > 0 else { return nil }
        return (done, total)
    }

    static func integer(in line: String, after prefix: String, before suffix: String) -> Int? {
        guard let start = line.range(of: prefix),
              let end = line.range(of: suffix, range: start.upperBound..<line.endIndex)
        else { return nil }
        return Int(line[start.upperBound..<end.lowerBound].trimmingCharacters(in: .whitespaces))
    }

    private static func trailingInteger(in text: String) -> Int? {
        let digits = text.reversed().prefix { $0.isNumber }.reversed()
        return Int(String(digits))
    }

    private static func leadingInteger(in text: String) -> Int? {
        let trimmed = text.drop { $0 == " " }
        return Int(String(trimmed.prefix { $0.isNumber }))
    }
}
