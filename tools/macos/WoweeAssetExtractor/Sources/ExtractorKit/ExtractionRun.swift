import Foundation

public enum Expansion: String, CaseIterable, Identifiable, Sendable {
    case auto
    case classic
    case turtle
    case tbc
    case wotlk

    public var id: String { rawValue }

    public var label: String {
        switch self {
        case .auto: return "Détection automatique"
        case .classic: return "Classic (1.12)"
        case .turtle: return "Turtle WoW"
        case .tbc: return "The Burning Crusade (2.4.3)"
        case .wotlk: return "Wrath of the Lich King (3.3.5a)"
        }
    }

    /// `--expansion` is omitted for auto, which is how the extractor asks to
    /// detect it itself.
    var commandLineValue: String? { self == .auto ? nil : rawValue }
}

public struct ExtractionRequest: Sendable {
    public var dataFolder: URL
    public var outputFolder: URL
    public var expansion: Expansion
    public var verify: Bool

    public init(
        dataFolder: URL,
        outputFolder: URL = ExtractorLocation.defaultOutputDirectory(),
        expansion: Expansion = .auto,
        verify: Bool = false
    ) {
        self.dataFolder = dataFolder
        self.outputFolder = outputFolder
        self.expansion = expansion
        self.verify = verify
    }

    public func arguments() -> [String] {
        var arguments = [
            "--mpq-dir", dataFolder.path,
            "--output", outputFolder.path,
            // Keep each client in its own subtree so extracting a second
            // expansion cannot overwrite the first.
            "--expansion-subdir",
        ]
        if let expansion = expansion.commandLineValue {
            arguments.append(contentsOf: ["--expansion", expansion])
        }
        if verify {
            arguments.append("--verify")
        }
        return arguments
    }
}

public enum ExtractionError: LocalizedError {
    case extractorNotFound
    case notAWoWDataFolder
    case failed(exitCode: Int32, tail: String)

    public var errorDescription: String? {
        switch self {
        case .extractorNotFound:
            return "Wowee.app est introuvable."
        case .notAWoWDataFolder:
            return "Ce dossier ne contient aucune archive MPQ."
        case let .failed(exitCode, tail):
            return tail.isEmpty
                ? "L'extraction s'est arrêtée (code \(exitCode))."
                : tail
        }
    }

    public var recoverySuggestion: String? {
        switch self {
        case .extractorNotFound:
            return """
                Placez cette application à côté de Wowee.app, ou installez \
                Wowee.app dans Applications.
                """
        case .notAWoWDataFolder:
            return """
                Choisissez le dossier Data de votre client World of Warcraft, \
                celui qui contient les fichiers .MPQ.
                """
        case .failed:
            return "Le journal ci-dessous contient le détail."
        }
    }
}

/// Runs `asset_extract` and reports where it is.
///
/// Deliberately not an ObservableObject: the UI owns the observable state and
/// this owns the process. Keeping them apart is what lets the parser and the
/// launch rules be tested without a window.
public final class ExtractionRunner: @unchecked Sendable {
    private var process: Process?
    private let queue = DispatchQueue(label: "com.wowee.asset-extractor.run")

    public init() {}

    public var isRunning: Bool {
        queue.sync { process?.isRunning ?? false }
    }

    /// Copy the bundled expansion profiles into the output tree.
    ///
    /// Existing files are left alone. The shipped shell launcher uses a plain
    /// `ditto`, which overwrites - and `expansion.json` is a file users are
    /// documented to edit by hand (CONTRIBUTING.md describes putting a
    /// server's own `wardenRsaModulus` there). Overwriting it on every run
    /// throws that away silently, so this seeds only what is missing.
    public static func seedProfiles(
        from source: URL,
        into destination: URL,
        fileManager: FileManager = .default
    ) throws {
        // Compare resolved paths on both sides. /var is a symlink to
        // /private/var on macOS, and an enumerator can hand back either form -
        // a plain string subtraction then leaves the whole absolute path in
        // place and every file is copied to the wrong depth, or to nowhere.
        let base = source.resolvingSymlinksInPath().standardizedFileURL
        let baseComponents = base.pathComponents

        guard let walker = fileManager.enumerator(
            at: base,
            includingPropertiesForKeys: [.isDirectoryKey]
        ) else { return }

        for case let item as URL in walker {
            let isDirectory = (try? item.resourceValues(forKeys: [.isDirectoryKey]))?
                .isDirectory ?? false
            let itemComponents = item.resolvingSymlinksInPath()
                .standardizedFileURL.pathComponents
            guard itemComponents.count > baseComponents.count,
                  Array(itemComponents.prefix(baseComponents.count)) == baseComponents
            else { continue }
            let relative = itemComponents.dropFirst(baseComponents.count).joined(separator: "/")
            guard !relative.isEmpty else { continue }
            let target = destination.appendingPathComponent(relative)

            if isDirectory {
                try fileManager.createDirectory(
                    at: target,
                    withIntermediateDirectories: true
                )
            } else if !fileManager.fileExists(atPath: target.path) {
                try fileManager.createDirectory(
                    at: target.deletingLastPathComponent(),
                    withIntermediateDirectories: true
                )
                try fileManager.copyItem(at: item, to: target)
            }
        }
    }

    /// Start the extraction.
    ///
    /// `onPhase` and `onLine` are called on the main queue so the UI can bind
    /// to them directly. `onFinish` is called exactly once.
    public func start(
        _ request: ExtractionRequest,
        extractor: URL,
        onPhase: @escaping (ExtractionPhase) -> Void,
        onLine: @escaping (String) -> Void,
        onFinish: @escaping (Result<Void, ExtractionError>) -> Void
    ) {
        let parser = ExtractionOutputParser()
        let process = Process()
        process.executableURL = extractor
        process.arguments = request.arguments()

        let pipe = Pipe()
        process.standardOutput = pipe
        process.standardError = pipe

        // Keep the last few lines so a failure can say something better than
        // its exit code.
        let recentLines = RecentLines(limit: 12)

        pipe.fileHandleForReading.readabilityHandler = { handle in
            let data = handle.availableData
            guard !data.isEmpty else { return }
            guard let text = String(data: data, encoding: .utf8) else { return }

            let lines = parser.feed(text)
            let phase = parser.lastKnownPhase
            for line in lines { recentLines.append(line) }

            DispatchQueue.main.async {
                for line in lines { onLine(line) }
                onPhase(phase)
            }
        }

        process.terminationHandler = { finished in
            pipe.fileHandleForReading.readabilityHandler = nil
            // Anything still sitting in the pipe after the process exits.
            let rest = pipe.fileHandleForReading.readDataToEndOfFile()
            if let text = String(data: rest, encoding: .utf8), !text.isEmpty {
                for line in parser.feed(text) { recentLines.append(line) }
            }
            for line in parser.flush() { recentLines.append(line) }

            self.queue.sync { self.process = nil }

            DispatchQueue.main.async {
                if finished.terminationReason == .uncaughtSignal {
                    onPhase(.cancelled)
                    onFinish(.failure(.failed(exitCode: -1, tail: "Extraction annulée.")))
                } else if finished.terminationStatus == 0 {
                    onPhase(.finished)
                    onFinish(.success(()))
                } else {
                    let tail = recentLines.joined()
                    onPhase(.failed(tail))
                    onFinish(.failure(.failed(
                        exitCode: finished.terminationStatus,
                        tail: tail
                    )))
                }
            }
        }

        queue.sync { self.process = process }

        do {
            try process.run()
            DispatchQueue.main.async { onPhase(.starting) }
        } catch {
            queue.sync { self.process = nil }
            DispatchQueue.main.async {
                onPhase(.failed(error.localizedDescription))
                onFinish(.failure(.failed(exitCode: -1, tail: error.localizedDescription)))
            }
        }
    }

    public func cancel() {
        queue.sync { process?.terminate() }
    }
}

/// A tiny ring of the most recent output lines, for error messages.
final class RecentLines: @unchecked Sendable {
    private var lines: [String] = []
    private let limit: Int
    private let lock = NSLock()

    init(limit: Int) { self.limit = limit }

    func append(_ line: String) {
        lock.lock()
        defer { lock.unlock() }
        lines.append(line)
        if lines.count > limit { lines.removeFirst(lines.count - limit) }
    }

    func joined() -> String {
        lock.lock()
        defer { lock.unlock() }
        return lines.joined(separator: "\n")
    }
}
