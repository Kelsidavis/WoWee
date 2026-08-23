import SwiftUI
import UniformTypeIdentifiers
import ExtractorKit

struct ContentView: View {
    @StateObject private var model = ExtractionViewModel()
    @State private var showLog = false

    var body: some View {
        VStack(spacing: 0) {
            header

            Divider()

            Group {
                switch model.stage {
                case .idle:
                    setupView
                case .running:
                    progressView
                case .succeeded:
                    successView
                case let .failed(message):
                    failureView(message)
                }
            }
            .frame(maxWidth: .infinity, maxHeight: .infinity)
            .padding(24)

            if showLog {
                Divider()
                logView
            }
        }
        .frame(minWidth: 520, minHeight: 460)
        .background(.background)
    }

    // MARK: - Header

    private var header: some View {
        HStack(spacing: 12) {
            Image(systemName: "cube.transparent")
                .font(.system(size: 22, weight: .light))
                .foregroundStyle(.tint)

            VStack(alignment: .leading, spacing: 1) {
                Text("Extracteur d'assets WoWee")
                    .font(.headline)
                Text("Prépare les données du client pour WoWee")
                    .font(.caption)
                    .foregroundStyle(.secondary)
            }

            Spacer()

            Button {
                withAnimation(.easeInOut(duration: 0.18)) { showLog.toggle() }
            } label: {
                Label("Journal", systemImage: showLog ? "chevron.down" : "chevron.right")
                    .labelStyle(.titleAndIcon)
                    .font(.caption)
            }
            .buttonStyle(.borderless)
            .foregroundStyle(.secondary)
        }
        .padding(.horizontal, 20)
        .padding(.vertical, 14)
    }

    // MARK: - Idle

    private var setupView: some View {
        VStack(spacing: 20) {
            dropZone

            if model.dataFolder != nil, model.inspection?.looksLikeWoWData == true {
                settings
                    .transition(.opacity.combined(with: .move(edge: .top)))
            }

            Spacer(minLength: 0)

            if model.isDiskTight, let available = model.availableBytes {
                Label(
                    "Espace libre : \(format(bytes: available)). Une extraction complète "
                    + "occupe environ 18 Go.",
                    systemImage: "exclamationmark.triangle"
                )
                .font(.caption)
                .foregroundStyle(.orange)
                .fixedSize(horizontal: false, vertical: true)
            }

            Button(action: model.start) {
                Text("Extraire")
                    .frame(maxWidth: .infinity)
            }
            .keyboardShortcut(.defaultAction)
            .controlSize(.large)
            .buttonStyle(.borderedProminent)
            .disabled(!model.canStart)
        }
        .animation(.easeInOut(duration: 0.2), value: model.dataFolder)
    }

    private var dropZone: some View {
        VStack(spacing: 10) {
            Image(systemName: dropIcon)
                .font(.system(size: 34, weight: .thin))
                .foregroundStyle(model.isTargeted ? AnyShapeStyle(.tint) : AnyShapeStyle(.secondary))

            Text(model.dataFolder?.lastPathComponent ?? "Dossier Data")
                .font(.system(.body, design: .rounded).weight(.medium))
                .lineLimit(1)
                .truncationMode(.middle)

            Text(model.subtitle)
                .font(.caption)
                .foregroundStyle(subtitleColor)
                .multilineTextAlignment(.center)

            Button("Parcourir…", action: model.chooseDataFolder)
                .buttonStyle(.link)
                .font(.caption)
        }
        .frame(maxWidth: .infinity)
        .padding(.vertical, 30)
        .background(
            RoundedRectangle(cornerRadius: 12)
                .fill(model.isTargeted ? AnyShapeStyle(.tint.opacity(0.08))
                                       : AnyShapeStyle(Color.secondary.opacity(0.05)))
        )
        .overlay(
            RoundedRectangle(cornerRadius: 12)
                .strokeBorder(
                    model.isTargeted ? AnyShapeStyle(.tint) : AnyShapeStyle(.separator),
                    style: StrokeStyle(lineWidth: model.isTargeted ? 2 : 1, dash: [7, 5])
                )
        )
        // A folder arrives as a file URL; .dropDestination gives us the URL
        // directly rather than an NSItemProvider to unwrap by hand.
        .dropDestination(for: URL.self) { urls, _ in
            guard let url = urls.first, url.hasDirectoryPath else { return false }
            model.accept(folder: url)
            return true
        } isTargeted: { targeted in
            withAnimation(.easeOut(duration: 0.12)) { model.isTargeted = targeted }
        }
    }

    private var dropIcon: String {
        guard let inspection = model.inspection else {
            return model.isTargeted ? "folder.fill" : "folder"
        }
        return inspection.looksLikeWoWData ? "checkmark.circle.fill" : "questionmark.folder"
    }

    private var subtitleColor: Color {
        guard let inspection = model.inspection else { return .secondary }
        return inspection.looksLikeWoWData ? .green : .orange
    }

    private var settings: some View {
        VStack(spacing: 12) {
            HStack {
                Text("Extension")
                    .foregroundStyle(.secondary)
                Spacer()
                Picker("", selection: $model.expansion) {
                    ForEach(Expansion.allCases) { expansion in
                        Text(expansion.label).tag(expansion)
                    }
                }
                .labelsHidden()
                .frame(maxWidth: 260)
            }

            HStack {
                Text("Destination")
                    .foregroundStyle(.secondary)
                Spacer()
                Button {
                    model.chooseOutputFolder()
                } label: {
                    Text(model.outputFolder.path.replacingOccurrences(
                        of: FileManager.default.homeDirectoryForCurrentUser.path,
                        with: "~"
                    ))
                    .lineLimit(1)
                    .truncationMode(.head)
                }
                .buttonStyle(.link)
                .help(model.outputFolder.path)
            }

            Toggle("Vérifier les fichiers extraits (CRC32, plus lent)", isOn: $model.verify)
                .toggleStyle(.checkbox)
                .frame(maxWidth: .infinity, alignment: .leading)
                .font(.callout)
        }
        .font(.callout)
        .padding(14)
        .background(RoundedRectangle(cornerRadius: 10).fill(Color.secondary.opacity(0.05)))
    }

    // MARK: - Running

    private var progressView: some View {
        VStack(spacing: 18) {
            Spacer(minLength: 0)

            VStack(spacing: 10) {
                Text(model.phaseTitle)
                    .font(.system(.title3, design: .rounded).weight(.medium))
                    .monospacedDigit()
                    .contentTransition(.numericText())

                // A determinate bar whenever the extractor gives real numbers,
                // and an indeterminate one only while it genuinely has nothing
                // to report - rather than a fake bar that invents a position.
                if let fraction = model.phase.fraction {
                    ProgressView(value: fraction)
                        .progressViewStyle(.linear)
                        .animation(.easeOut(duration: 0.3), value: fraction)
                    Text(fraction.formatted(.percent.precision(.fractionLength(0))))
                        .font(.caption)
                        .monospacedDigit()
                        .foregroundStyle(.secondary)
                } else {
                    ProgressView()
                        .progressViewStyle(.linear)
                }

                if let remaining = model.estimatedRemaining {
                    Text("Environ \(remaining.formatted(.units(allowed: [.hours, .minutes], width: .wide))) restantes")
                        .font(.caption)
                        .foregroundStyle(.secondary)
                        .transition(.opacity)
                }
            }
            .frame(maxWidth: 380)

            Spacer(minLength: 0)

            Button("Annuler", role: .cancel, action: model.cancel)
                .controlSize(.large)
        }
    }

    // MARK: - Terminal states

    private var successView: some View {
        VStack(spacing: 14) {
            Spacer(minLength: 0)
            Image(systemName: "checkmark.circle.fill")
                .font(.system(size: 46))
                .foregroundStyle(.green)
            Text("Extraction terminée")
                .font(.title3.weight(.medium))
            Text("Vous pouvez lancer Wowee.app.")
                .font(.callout)
                .foregroundStyle(.secondary)
            Spacer(minLength: 0)
            HStack(spacing: 10) {
                Button("Afficher dans le Finder", action: model.revealOutput)
                Button("Nouvelle extraction", action: model.reset)
                    .buttonStyle(.borderedProminent)
            }
            .controlSize(.large)
        }
    }

    private func failureView(_ message: String) -> some View {
        VStack(spacing: 14) {
            Spacer(minLength: 0)
            Image(systemName: "exclamationmark.triangle.fill")
                .font(.system(size: 40))
                .foregroundStyle(.orange)
            Text("L'extraction n'a pas abouti")
                .font(.title3.weight(.medium))
            ScrollView {
                Text(message)
                    .font(.system(.caption, design: .monospaced))
                    .foregroundStyle(.secondary)
                    .textSelection(.enabled)
                    .frame(maxWidth: .infinity, alignment: .leading)
            }
            .frame(maxHeight: 110)
            Spacer(minLength: 0)
            HStack(spacing: 10) {
                Button("Voir le journal") {
                    withAnimation { showLog = true }
                }
                Button("Recommencer", action: model.reset)
                    .buttonStyle(.borderedProminent)
            }
            .controlSize(.large)
        }
    }

    // MARK: - Log

    private var logView: some View {
        ScrollViewReader { proxy in
            ScrollView {
                LazyVStack(alignment: .leading, spacing: 1) {
                    ForEach(Array(model.log.enumerated()), id: \.offset) { index, line in
                        Text(line)
                            .font(.system(size: 10.5, design: .monospaced))
                            .foregroundStyle(.secondary)
                            .frame(maxWidth: .infinity, alignment: .leading)
                            .id(index)
                    }
                }
                .padding(.horizontal, 14)
                .padding(.vertical, 8)
            }
            .frame(height: 130)
            .background(Color.secondary.opacity(0.04))
            .onChange(of: model.log.count) { _ in
                proxy.scrollTo(model.log.count - 1, anchor: .bottom)
            }
        }
    }

    private func format(bytes: Int64) -> String {
        ByteCountFormatter.string(fromByteCount: bytes, countStyle: .file)
    }
}
