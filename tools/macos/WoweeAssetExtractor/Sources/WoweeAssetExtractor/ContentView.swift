import SwiftUI
import UniformTypeIdentifiers
import ExtractorKit

struct ContentView: View {
    @EnvironmentObject private var model: ExtractionViewModel
    @Binding var showLog: Bool

    // A Mac app answers to these three the way the system asks it to, not the
    // way the design would prefer.
    @Environment(\.accessibilityReduceMotion) private var reduceMotion
    @Environment(\.colorSchemeContrast) private var contrast
    @Environment(\.legibilityWeight) private var legibilityWeight

    @State private var isHoveringDropZone = false

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
            .padding(20)

            if showLog {
                Divider()
                logView
            }
        }
        // Freely resizable with a floor that keeps the layout usable. A fixed
        // window is one of the things that most reliably marks an app as not
        // written for the Mac.
        .frame(minWidth: 460, minHeight: 420)
        .background(.background)
    }

    /// One place to decide whether an animation happens at all.
    private func animated<V: Equatable>(_ animation: Animation, value: V) -> Animation? {
        reduceMotion ? nil : animation
    }

    // MARK: - Header

    private var header: some View {
        HStack(spacing: 12) {
            Image(systemName: "cube.transparent")
                .font(.system(size: 22, weight: .light))
                .foregroundStyle(.tint)
                .accessibilityHidden(true)

            VStack(alignment: .leading, spacing: 1) {
                Text("Extracteur d'assets WoWee")
                    .font(.headline)
                Text("Prépare les données du client pour WoWee")
                    .font(.caption)
                    .foregroundStyle(.secondary)
            }
            .accessibilityElement(children: .combine)

            Spacer()

            Button {
                withAnimation(animated(.easeInOut(duration: 0.18), value: showLog)) {
                    showLog.toggle()
                }
            } label: {
                Label("Journal", systemImage: showLog ? "chevron.down" : "chevron.right")
                    .font(.caption)
            }
            .buttonStyle(.borderless)
            .foregroundStyle(.secondary)
            .accessibilityLabel(showLog ? "Masquer le journal" : "Afficher le journal")
            .help("Afficher ou masquer la sortie détaillée de l'extracteur (⌘L)")
        }
        .padding(.horizontal, 20)
        .padding(.vertical, 12)
    }

    // MARK: - Idle

    private var setupView: some View {
        VStack(spacing: 16) {
            dropZone

            if model.dataFolder != nil, model.inspection?.looksLikeWoWData == true {
                settings
                    .transition(reduceMotion ? .identity : .opacity)
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
                .accessibilityLabel(
                    "Attention, espace disque faible : \(format(bytes: available)) disponibles."
                )
            }

            Button(action: model.start) {
                Text("Extraire")
                    .frame(maxWidth: .infinity)
            }
            .keyboardShortcut(.defaultAction)
            .controlSize(.large)
            .buttonStyle(.borderedProminent)
            .disabled(!model.canStart)
            .help(model.canStart ? "Lancer l'extraction (⌘R)" : model.subtitle)
        }
        .animation(animated(.easeInOut(duration: 0.2), value: model.dataFolder),
                   value: model.dataFolder)
    }

    private var dropZone: some View {
        VStack(spacing: 8) {
            Image(systemName: dropIcon)
                .font(.system(size: 34, weight: .thin))
                .foregroundStyle(dropIconStyle)
                .accessibilityHidden(true)

            Text(model.dataFolder?.lastPathComponent ?? "Dossier Data")
                .font(.system(.body, design: .rounded))
                .fontWeight(legibilityWeight == .bold ? .semibold : .medium)
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
        .padding(.vertical, 28)
        .background(
            RoundedRectangle(cornerRadius: 12)
                .fill(dropZoneFill)
        )
        .overlay(
            RoundedRectangle(cornerRadius: 12)
                .strokeBorder(
                    dropZoneBorder,
                    style: StrokeStyle(lineWidth: model.isTargeted ? 2 : 1, dash: [7, 5])
                )
        )
        .contentShape(Rectangle())
        .onHover { hovering in
            withAnimation(animated(.easeOut(duration: 0.12), value: hovering)) {
                isHoveringDropZone = hovering
            }
        }
        // A folder arrives as a file URL; .dropDestination hands it over
        // directly rather than an NSItemProvider to unwrap by hand.
        .dropDestination(for: URL.self) { urls, _ in
            guard let url = urls.first, url.hasDirectoryPath else { return false }
            model.accept(folder: url)
            return true
        } isTargeted: { targeted in
            withAnimation(animated(.easeOut(duration: 0.12), value: targeted)) {
                model.isTargeted = targeted
            }
        }
        .contextMenu {
            Button("Choisir le dossier Data…", action: model.chooseDataFolder)
            if model.dataFolder != nil {
                Button("Afficher dans le Finder") {
                    if let folder = model.dataFolder {
                        NSWorkspace.shared.activateFileViewerSelecting([folder])
                    }
                }
                Divider()
                Button("Retirer", role: .destructive) { model.clearDataFolder() }
            }
        }
        // Announced as one control, because a drop target split into four
        // unlabelled fragments tells VoiceOver nothing about what it is for.
        .accessibilityElement(children: .ignore)
        .accessibilityLabel("Dossier Data de World of Warcraft")
        .accessibilityValue(
            model.dataFolder.map { "\($0.lastPathComponent). \(model.subtitle)" }
                ?? "Aucun dossier choisi. \(model.subtitle)"
        )
        .accessibilityHint("Déposez un dossier ici, ou activez pour le choisir.")
        .accessibilityAddTraits(.isButton)
        .accessibilityAction { model.chooseDataFolder() }
    }

    private var dropIcon: String {
        guard let inspection = model.inspection else {
            return model.isTargeted ? "folder.fill" : "folder"
        }
        return inspection.looksLikeWoWData ? "checkmark.circle.fill" : "questionmark.folder"
    }

    private var dropIconStyle: AnyShapeStyle {
        model.isTargeted ? AnyShapeStyle(.tint) : AnyShapeStyle(.secondary)
    }

    private var dropZoneFill: AnyShapeStyle {
        if model.isTargeted { return AnyShapeStyle(.tint.opacity(0.08)) }
        if isHoveringDropZone { return AnyShapeStyle(Color.secondary.opacity(0.09)) }
        return AnyShapeStyle(Color.secondary.opacity(0.05))
    }

    /// With Increase Contrast on, a hairline separator is not a visible edge -
    /// and this border is the only thing saying where the drop target is.
    private var dropZoneBorder: AnyShapeStyle {
        if model.isTargeted { return AnyShapeStyle(.tint) }
        return contrast == .increased
            ? AnyShapeStyle(Color.primary)
            : AnyShapeStyle(Color.secondary.opacity(0.5))
    }

    private var subtitleColor: Color {
        guard let inspection = model.inspection else { return .secondary }
        return inspection.looksLikeWoWData ? .green : .orange
    }

    private var settings: some View {
        VStack(spacing: 10) {
            HStack {
                Text("Extension")
                    .foregroundStyle(.secondary)
                Spacer()
                Picker("Extension", selection: $model.expansion) {
                    ForEach(Expansion.allCases) { expansion in
                        Text(expansion.label).tag(expansion)
                    }
                }
                .labelsHidden()
                .frame(maxWidth: 260)
                .accessibilityLabel("Extension à extraire")
            }

            HStack {
                Text("Destination")
                    .foregroundStyle(.secondary)
                Spacer()
                Button(action: model.chooseOutputFolder) {
                    Text(abbreviated(model.outputFolder))
                        .lineLimit(1)
                        .truncationMode(.head)
                }
                .buttonStyle(.link)
                .help(model.outputFolder.path)
                .accessibilityLabel("Destination : \(abbreviated(model.outputFolder))")
                .accessibilityHint("Activez pour choisir un autre dossier.")
            }

            Toggle("Vérifier les fichiers extraits (CRC32, plus lent)", isOn: $model.verify)
                .toggleStyle(.checkbox)
                .frame(maxWidth: .infinity, alignment: .leading)
        }
        .font(.callout)
        .padding(12)
        .background(RoundedRectangle(cornerRadius: 10).fill(Color.secondary.opacity(0.05)))
    }

    // MARK: - Running

    private var progressView: some View {
        VStack(spacing: 16) {
            Spacer(minLength: 0)

            VStack(spacing: 8) {
                Text(model.phaseTitle)
                    .font(.system(.title3, design: .rounded))
                    .fontWeight(legibilityWeight == .bold ? .semibold : .medium)
                    .monospacedDigit()
                    .contentTransition(.numericText())

                // Determinate whenever the extractor reports real numbers, and
                // indeterminate only while it genuinely has nothing to say -
                // rather than a bar that invents a position.
                if let fraction = model.phase.fraction {
                    ProgressView(value: fraction)
                        .progressViewStyle(.linear)
                        .animation(animated(.easeOut(duration: 0.3), value: fraction),
                                   value: fraction)
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
                }
            }
            .frame(maxWidth: 380)
            .accessibilityElement(children: .combine)
            .accessibilityLabel("Progression de l'extraction")
            .accessibilityValue(model.accessibleProgressDescription)

            Spacer(minLength: 0)

            Button("Annuler", role: .cancel, action: model.cancel)
                .keyboardShortcut(.cancelAction)
                .controlSize(.large)
                .help("Interrompre l'extraction (⌘.)")
        }
    }

    // MARK: - Terminal states

    private var successView: some View {
        VStack(spacing: 12) {
            Spacer(minLength: 0)
            Image(systemName: "checkmark.circle.fill")
                .font(.system(size: 46))
                .foregroundStyle(.green)
                .accessibilityHidden(true)
            Text("Extraction terminée")
                .font(.title3)
                .fontWeight(legibilityWeight == .bold ? .semibold : .medium)
            Text("Vous pouvez lancer Wowee.app.")
                .font(.callout)
                .foregroundStyle(.secondary)
            Spacer(minLength: 0)
            HStack(spacing: 10) {
                Button("Afficher dans le Finder", action: model.revealOutput)
                    .keyboardShortcut("r", modifiers: [.command, .shift])
                Button("Nouvelle extraction", action: model.reset)
                    .buttonStyle(.borderedProminent)
                    .keyboardShortcut(.defaultAction)
            }
            .controlSize(.large)
        }
    }

    private func failureView(_ message: String) -> some View {
        VStack(spacing: 12) {
            Spacer(minLength: 0)
            Image(systemName: "exclamationmark.triangle.fill")
                .font(.system(size: 40))
                .foregroundStyle(.orange)
                .accessibilityHidden(true)
            Text("L'extraction n'a pas abouti")
                .font(.title3)
                .fontWeight(legibilityWeight == .bold ? .semibold : .medium)
            ScrollView {
                Text(message)
                    .font(.system(.caption, design: .monospaced))
                    .foregroundStyle(.secondary)
                    .textSelection(.enabled)
                    .frame(maxWidth: .infinity, alignment: .leading)
            }
            .frame(maxHeight: 110)
            .accessibilityLabel("Détail de l'erreur : \(message)")
            Spacer(minLength: 0)
            HStack(spacing: 10) {
                Button("Voir le journal") {
                    withAnimation(animated(.easeInOut(duration: 0.18), value: showLog)) {
                        showLog = true
                    }
                }
                .keyboardShortcut("l", modifiers: .command)
                Button("Recommencer", action: model.reset)
                    .buttonStyle(.borderedProminent)
                    .keyboardShortcut(.defaultAction)
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
                            .font(.system(.caption2, design: .monospaced))
                            .foregroundStyle(.secondary)
                            .textSelection(.enabled)
                            .frame(maxWidth: .infinity, alignment: .leading)
                            .id(index)
                    }
                }
                .padding(.horizontal, 14)
                .padding(.vertical, 8)
            }
            .frame(height: 130)
            .background(Color.secondary.opacity(0.04))
            .contextMenu {
                Button("Copier le journal") {
                    NSPasteboard.general.clearContents()
                    NSPasteboard.general.setString(
                        model.log.joined(separator: "\n"), forType: .string
                    )
                }
                .disabled(model.log.isEmpty)
            }
            .accessibilityLabel("Journal de l'extracteur")
            .onChange(of: model.log.count) { _ in
                guard !reduceMotion else {
                    proxy.scrollTo(model.log.count - 1, anchor: .bottom)
                    return
                }
                withAnimation(.easeOut(duration: 0.15)) {
                    proxy.scrollTo(model.log.count - 1, anchor: .bottom)
                }
            }
        }
    }

    private func abbreviated(_ url: URL) -> String {
        url.path.replacingOccurrences(
            of: FileManager.default.homeDirectoryForCurrentUser.path,
            with: "~"
        )
    }

    private func format(bytes: Int64) -> String {
        ByteCountFormatter.string(fromByteCount: bytes, countStyle: .file)
    }
}
