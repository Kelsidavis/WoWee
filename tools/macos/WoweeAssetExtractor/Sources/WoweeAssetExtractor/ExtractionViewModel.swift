import Foundation
import SwiftUI
import ExtractorKit

@MainActor
final class ExtractionViewModel: ObservableObject {

    enum Stage: Equatable {
        case idle
        case running
        /// Carries the extractor's own tally, or nil when it exited clean
        /// without printing one.
        case succeeded(ExtractionSummary?)
        /// Carries the advice as well as the message. See ExtractionFailure.
        case failed(ExtractionFailure)

        var isRunning: Bool { self == .running }
    }

    @Published var stage: Stage = .idle
    @Published var phase: ExtractionPhase = .starting
    @Published var dataFolder: URL?
    @Published var inspection: DataFolderInspection?
    @Published var outputFolder: URL = ExtractorLocation.defaultOutputDirectory()
    @Published var expansion: Expansion = .auto
    @Published var verify = false
    @Published var log: [String] = []
    @Published var isTargeted = false
    @Published var startedAt: Date?
    /// How long the last run took, wall clock, kept when it ends.
    ///
    /// Measured here rather than read from the extractor's own timing: what
    /// the user waited includes the archive scan and the manifest write, not
    /// just the extraction the C++ times.
    @Published var elapsed: Duration?

    private let runner = ExtractionRunner()

    // MARK: - Derived state

    var extractorURL: URL? { ExtractorLocation.findExtractor() }

    var canStart: Bool {
        !stage.isRunning && dataFolder != nil
            && (inspection?.looksLikeWoWData ?? false) && extractorURL != nil
    }

    var availableBytes: Int64? { DiskSpace.availableBytes(at: outputFolder) }
    var isDiskTight: Bool { DiskSpace.isTight(availableBytes) }

    /// The message under the drop zone. Kept as one computed property so the
    /// view never has to work out which of several half-states applies.
    var subtitle: String {
        if extractorURL == nil {
            return "Wowee.app est introuvable — placez cette app à côté."
        }
        guard let inspection else {
            return "Glissez le dossier Data de World of Warcraft"
        }
        if !inspection.looksLikeWoWData {
            return "Aucune archive MPQ dans ce dossier"
        }
        var text = "\(inspection.archiveCount) archives MPQ trouvées"
        if !inspection.localeSubdirectories.isEmpty {
            text += " · \(inspection.localeSubdirectories.joined(separator: ", "))"
        }
        return text
    }

    var phaseTitle: String {
        switch phase {
        case .starting: return "Préparation…"
        case let .scanningArchives(done, total): return "Lecture des archives \(done)/\(total)"
        case .enumerating: return "Inventaire des fichiers…"
        case let .extracting(done, total):
            return "Extraction \(done.formatted()) / \(total.formatted()) fichiers"
        case .finishing: return "Écriture du manifeste…"
        case .finished: return "Terminé"
        case .cancelled: return "Annulé"
        case .failed: return "Échec"
        }
    }

    /// Rough remaining time, from the rate achieved so far.
    ///
    /// Only offered during extraction: the other phases are too short, and too
    /// unlike each other, for an extrapolation to mean anything.
    var estimatedRemaining: Duration? {
        guard case let .extracting(done, total) = phase,
              let startedAt, done > 0, total > done else { return nil }
        let elapsed = Date().timeIntervalSince(startedAt)
        guard elapsed > 5 else { return nil }
        let perFile = elapsed / Double(done)
        return .seconds(perFile * Double(total - done))
    }

    // MARK: - Actions

    /// What VoiceOver reads for the progress bar.
    ///
    /// The visible label is built for a glance ("Extraction 12 000 / 431 221
    /// fichiers"); read aloud it needs the percentage too, since a listener
    /// cannot see where the bar sits.
    var accessibleProgressDescription: String {
        guard let fraction = phase.fraction else { return phaseTitle }
        let percent = fraction.formatted(.percent.precision(.fractionLength(0)))
        return "\(phaseTitle), \(percent)"
    }

    func accept(folder: URL) {
        dataFolder = folder
        inspection = DataFolderInspection.inspect(folder)
        if !stage.isRunning { stage = .idle }
    }

    func clearDataFolder() {
        dataFolder = nil
        inspection = nil
    }

    func chooseDataFolder() {
        let panel = NSOpenPanel()
        panel.canChooseDirectories = true
        panel.canChooseFiles = false
        panel.allowsMultipleSelection = false
        panel.prompt = "Choisir"
        panel.message = "Sélectionnez le dossier Data de World of Warcraft"
        if panel.runModal() == .OK, let url = panel.url {
            accept(folder: url)
        }
    }

    func chooseOutputFolder() {
        let panel = NSOpenPanel()
        panel.canChooseDirectories = true
        panel.canChooseFiles = false
        panel.canCreateDirectories = true
        panel.prompt = "Choisir"
        panel.message = "Où écrire les fichiers extraits"
        if panel.runModal() == .OK, let url = panel.url {
            outputFolder = url
        }
    }

    func start() {
        guard let dataFolder, let extractor = extractorURL else { return }

        log.removeAll()
        phase = .starting
        stage = .running
        startedAt = Date()

        // Seed the shipped expansion profiles before extracting, skipping any
        // the user already has. See ExtractionRunner.seedProfiles.
        if let bundled = ExtractorLocation.bundledData(besideExtractor: extractor) {
            do {
                try ExtractionRunner.seedProfiles(from: bundled, into: outputFolder)
            } catch {
                log.append("Profils non copiés : \(error.localizedDescription)")
            }
        }

        let request = ExtractionRequest(
            dataFolder: dataFolder,
            outputFolder: outputFolder,
            expansion: expansion,
            verify: verify
        )

        runner.start(
            request,
            extractor: extractor,
            onPhase: { [weak self] phase in
                self?.phase = phase
            },
            onLine: { [weak self] line in
                guard let self else { return }
                self.log.append(line)
                // A run prints hundreds of thousands of progress ticks. Keeping
                // them all is a memory leak with a scrollbar.
                if self.log.count > 400 { self.log.removeFirst(self.log.count - 400) }
            },
            onFinish: { [weak self] result in
                guard let self else { return }
                if let startedAt = self.startedAt {
                    self.elapsed = .seconds(Date().timeIntervalSince(startedAt))
                }
                switch result {
                case let .success(summary):
                    self.stage = .succeeded(summary)
                case let .failure(error):
                    self.stage = .failed(ExtractionFailure(error))
                }
            }
        )
    }

    func cancel() {
        runner.cancel()
    }

    func reset() {
        stage = .idle
        phase = .starting
        log.removeAll()
        startedAt = nil
        elapsed = nil
    }

    func revealOutput() {
        NSWorkspace.shared.selectFile(nil, inFileViewerRootedAtPath: outputFolder.path)
    }
}
