import SwiftUI
import UniformTypeIdentifiers
import ExtractorKit

struct ContentView: View {
    @EnvironmentObject private var model: ExtractionViewModel
    @Binding var showLog: Bool

    // A Mac app answers to these three the way the system asks it to, not the
    // way the design would prefer. The Atelier palette fixes its own colours,
    // but not these.
    @Environment(\.accessibilityReduceMotion) private var reduceMotion
    @Environment(\.colorSchemeContrast) private var contrast
    @Environment(\.legibilityWeight) private var legibilityWeight

    @State private var isHoveringDropZone = false

    private var increasedContrast: Bool { contrast == .increased }
    private var titleWeight: Font.Weight { legibilityWeight == .bold ? .bold : .semibold }

    var body: some View {
        VStack(spacing: 0) {
            Group {
                switch model.stage {
                case .idle:
                    setupView
                case .running:
                    progressView
                case let .succeeded(summary):
                    successView(summary)
                case let .failed(failure):
                    failureView(failure)
                }
            }
            .frame(maxWidth: .infinity, maxHeight: .infinity)
            .padding(20)

            if showLog {
                Rectangle()
                    .fill(Atelier.line(Atelier.hairline, increasedContrast: increasedContrast))
                    .frame(height: 1)
                logView
            }
        }
        // Freely resizable with a floor that keeps the layout usable. A fixed
        // window is one of the things that most reliably marks an app as not
        // written for the Mac.
        .frame(minWidth: 460, minHeight: 420)
        .background(Atelier.ground)
        // The toolbar is where a Mac user looks for a view control, and it
        // buys back the vertical space the old bespoke header spent on a logo
        // and a subtitle the title bar was already carrying.
        .toolbar {
            ToolbarItem(placement: .primaryAction) {
                Button {
                    withAnimation(animated(.easeInOut(duration: 0.18), value: showLog)) {
                        showLog.toggle()
                    }
                } label: {
                    Label("Journal", systemImage: "text.alignleft")
                }
                .help("Afficher ou masquer la sortie détaillée de l'extracteur (⌘L)")
                .accessibilityLabel(showLog ? "Masquer le journal" : "Afficher le journal")
            }
        }
    }

    /// One place to decide whether an animation happens at all.
    private func animated<V: Equatable>(_ animation: Animation, value: V) -> Animation? {
        reduceMotion ? nil : animation
    }

    // MARK: - Idle

    private var setupView: some View {
        VStack(spacing: 16) {
            mark

            dropZone

            if model.dataFolder != nil, model.inspection?.looksLikeWoWData == true {
                settingsSummary
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
                .foregroundStyle(Atelier.ember)
                .fixedSize(horizontal: false, vertical: true)
                .accessibilityLabel(
                    "Attention, espace disque faible : \(format(bytes: available)) disponibles."
                )
            }

            Button(action: model.start) {
                Text("Extraire")
                    .frame(maxWidth: .infinity)
            }
            .buttonStyle(.borderedProminent)
            .tint(Atelier.brass)
            .controlSize(.large)
            .keyboardShortcut(.defaultAction)
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
                .foregroundStyle(dropIconColor)
                .accessibilityHidden(true)

            Text(model.dataFolder?.lastPathComponent ?? "Dossier Data")
                .font(.system(.body, design: .rounded))
                .fontWeight(legibilityWeight == .bold ? .semibold : .medium)
                .foregroundStyle(Atelier.ink)
                .lineLimit(1)
                .truncationMode(.middle)

            Text(model.subtitle)
                .font(.caption)
                .foregroundStyle(subtitleColor)
                .multilineTextAlignment(.center)

            Button("Parcourir…", action: model.chooseDataFolder)
                .buttonStyle(.link)
                .tint(Atelier.brass)
                .font(.caption)
        }
        .frame(maxWidth: .infinity)
        .padding(.vertical, 28)
        .background(
            RoundedRectangle(cornerRadius: Atelier.dropRadius, style: .continuous)
                .fill(dropZoneFill)
        )
        .overlay(
            RoundedRectangle(cornerRadius: Atelier.dropRadius, style: .continuous)
                .strokeBorder(
                    dropZoneBorder,
                    style: StrokeStyle(lineWidth: model.isTargeted ? 2 : 1.5, dash: [7, 5])
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

    private var dropIconColor: Color {
        if model.isTargeted { return Atelier.brass }
        guard let inspection = model.inspection else { return Atelier.inkTertiary }
        return inspection.looksLikeWoWData ? Atelier.verdigris : Atelier.ember
    }

    private var dropZoneFill: Color {
        if model.isTargeted { return Atelier.brass.opacity(0.10) }
        if isHoveringDropZone { return Atelier.brass.opacity(0.05) }
        return Atelier.raised.opacity(0.6)
    }

    /// With Increase Contrast on, a hairline is not a visible edge - and this
    /// border is the only thing saying where the drop target is.
    private var dropZoneBorder: Color {
        if model.isTargeted { return Atelier.brass }
        return increasedContrast ? Atelier.ink : Atelier.brass.opacity(0.45)
    }

    private var subtitleColor: Color {
        guard let inspection = model.inspection else { return Atelier.inkTertiary }
        return inspection.looksLikeWoWData ? Atelier.verdigris : Atelier.ember
    }

    /// The mark, given the room the header never had.
    private var mark: some View {
        VStack(spacing: 9) {
            Image("AppMark", bundle: .assets)
                .resizable()
                .interpolation(.high)
                .frame(width: 60, height: 60)
                .clipShape(RoundedRectangle(cornerRadius: 13, style: .continuous))
                .accessibilityHidden(true)

            Text("Prépare les données du client pour WoWee")
                .font(.caption)
                .foregroundStyle(Atelier.inkTertiary)
        }
    }

    /// What ⌘, is set to, in one line.
    ///
    /// The settings themselves moved to the Settings window; leaving nothing
    /// behind would mean a user could start an 18 GB run without ever seeing
    /// which expansion it picked or where it lands.
    private var settingsSummary: some View {
        HStack(spacing: 8) {
            Text(model.expansion.label)
                .foregroundStyle(Atelier.inkSecondary)
            Text("·").foregroundStyle(Atelier.inkQuaternary)
            Text(abbreviated(model.outputFolder))
                .foregroundStyle(Atelier.inkTertiary)
                .lineLimit(1)
                .truncationMode(.head)
                .help(model.outputFolder.path)
            if model.verify {
                Text("·").foregroundStyle(Atelier.inkQuaternary)
                Label("CRC32", systemImage: "checkmark.shield")
                    .foregroundStyle(Atelier.inkTertiary)
            }
            Spacer(minLength: 8)
            Button("Réglages…") { openSettings() }
                .buttonStyle(.link)
                .tint(Atelier.brass)
        }
        .font(.caption)
        .padding(.horizontal, 13)
        .padding(.vertical, 9)
        .background(card)
        .accessibilityElement(children: .combine)
        .accessibilityLabel(
            "Réglages : \(model.expansion.label), destination \(abbreviated(model.outputFolder))"
            + (model.verify ? ", vérification CRC32 activée" : "")
        )
    }

    // MARK: - Running

    private var progressView: some View {
        VStack(spacing: 18) {
            Spacer(minLength: 0)

            VStack(spacing: 10) {
                Text(model.phaseTitle)
                    .font(Atelier.title(.title3, weight: titleWeight))
                    .foregroundStyle(Atelier.ink)
                    .monospacedDigit()
                    .contentTransition(.numericText())

                // Determinate whenever the extractor reports real numbers, and
                // indeterminate only while it genuinely has nothing to say -
                // rather than a bar that invents a position.
                if let fraction = model.phase.fraction {
                    ProgressView(value: fraction)
                        .progressViewStyle(.linear)
                        .tint(Atelier.brass)
                        .animation(animated(.easeOut(duration: 0.3), value: fraction),
                                   value: fraction)
                    HStack {
                        Text(fraction.formatted(.percent.precision(.fractionLength(0))))
                        Spacer()
                        if let remaining = model.estimatedRemaining {
                            Text("Environ \(remaining.formatted(.units(allowed: [.hours, .minutes], width: .wide))) restantes")
                        }
                    }
                    .font(.caption)
                    .monospacedDigit()
                    .foregroundStyle(Atelier.inkTertiary)
                } else {
                    ProgressView()
                        .progressViewStyle(.linear)
                }
            }
            .frame(maxWidth: 420)
            .accessibilityElement(children: .combine)
            .accessibilityLabel("Progression de l'extraction")
            .accessibilityValue(model.accessibleProgressDescription)

            phaseTrack
                .frame(maxWidth: 460)

            Spacer(minLength: 0)

            Button("Annuler", action: model.cancel)
                .buttonStyle(.bordered)
                .controlSize(.large)
                .keyboardShortcut(.cancelAction)
                .help("Interrompre l'extraction (⌘.)")
        }
    }

    /// The four phases, so a long extraction says where it is in the whole run
    /// and not only how far along its own step it happens to be.
    private var phaseTrack: some View {
        HStack(spacing: 8) {
            phaseChip("Archives", state: phaseState(0))
            phaseChip("Inventaire", state: phaseState(1))
            phaseChip("Extraction", state: phaseState(2))
            phaseChip("Manifeste", state: phaseState(3))
        }
        .accessibilityHidden(true)   // the progress bar above already says this
    }

    private enum PhaseState { case done, current, pending }

    private func phaseState(_ index: Int) -> PhaseState {
        let current: Int
        switch model.phase {
        case .starting: current = 0
        case .scanningArchives: current = 0
        case .enumerating: current = 1
        case .extracting: current = 2
        case .finishing: current = 3
        case .finished: current = 4
        case .failed, .cancelled: current = -1
        }
        if index < current { return .done }
        if index == current { return .current }
        return .pending
    }

    private func phaseChip(_ label: String, state: PhaseState) -> some View {
        Text(label)
            .font(.system(size: 10, weight: .medium))
            .textCase(.uppercase)
            .kerning(0.6)
            .foregroundStyle(
                state == .current ? Atelier.brass
                    : state == .done ? Atelier.inkTertiary : Atelier.inkQuaternary
            )
            .frame(maxWidth: .infinity)
            .padding(.vertical, 8)
            .background(
                RoundedRectangle(cornerRadius: Atelier.controlRadius, style: .continuous)
                    .fill(state == .current ? Atelier.control : Atelier.raised)
            )
            .overlay(
                RoundedRectangle(cornerRadius: Atelier.controlRadius, style: .continuous)
                    .strokeBorder(
                        state == .current
                            ? Atelier.brass
                            : Atelier.line(Atelier.hairline, increasedContrast: increasedContrast)
                    )
            )
    }

    // MARK: - Terminal states

    private func successView(_ summary: ExtractionSummary?) -> some View {
        VStack(spacing: 16) {
            Spacer(minLength: 0)

            Image(systemName: "checkmark.circle")
                .font(.system(size: 50, weight: .light))
                .foregroundStyle(Atelier.verdigris)
                .accessibilityHidden(true)

            VStack(spacing: 5) {
                Text("Extraction terminée")
                    .font(Atelier.title(.title2, weight: titleWeight))
                    .foregroundStyle(Atelier.ink)
                Text("Vous pouvez lancer Wowee.app.")
                    .font(.callout)
                    .foregroundStyle(Atelier.inkTertiary)
            }

            // Only when the extractor actually printed its tally. No figures
            // beats invented ones.
            if let summary {
                summaryTiles(summary)
                    .frame(maxWidth: 440)

                if summary.hasFailures {
                    Label(
                        "\(summary.filesFailed.formatted()) fichiers n'ont pas pu être extraits. "
                        + "Le journal en donne le détail.",
                        systemImage: "exclamationmark.triangle"
                    )
                    .font(.caption)
                    .foregroundStyle(Atelier.ember)
                    .fixedSize(horizontal: false, vertical: true)
                }
            }

            Label(abbreviated(model.outputFolder), systemImage: "folder")
                .font(.caption)
                .foregroundStyle(Atelier.inkQuaternary)
                .lineLimit(1)
                .truncationMode(.head)

            Spacer(minLength: 0)

            HStack(spacing: 10) {
                Button("Afficher dans le Finder", action: model.revealOutput)
                    .buttonStyle(.bordered)
                    .keyboardShortcut("r", modifiers: [.command, .shift])
                Button("Nouvelle extraction", action: model.reset)
                    .buttonStyle(.borderedProminent)
                    .tint(Atelier.brass)
                    .keyboardShortcut(.defaultAction)
            }
        }
    }

    private func summaryTiles(_ summary: ExtractionSummary) -> some View {
        HStack(spacing: 1) {
            summaryTile(summary.filesExtracted.formatted(), "fichiers")
            summaryTile(format(bytes: summary.bytesWritten), "écrits")
            if let elapsed = model.elapsed {
                summaryTile(
                    elapsed.formatted(.units(allowed: [.hours, .minutes], width: .narrow)),
                    "durée"
                )
            }
        }
        .background(Atelier.hairline)
        .clipShape(RoundedRectangle(cornerRadius: Atelier.cardRadius, style: .continuous))
        .overlay(
            RoundedRectangle(cornerRadius: Atelier.cardRadius, style: .continuous)
                .strokeBorder(Atelier.line(Atelier.hairline, increasedContrast: increasedContrast))
        )
    }

    private func summaryTile(_ value: String, _ label: String) -> some View {
        VStack(spacing: 4) {
            Text(value)
                .font(Atelier.title(.body, weight: titleWeight))
                .foregroundStyle(Atelier.ink)
                .monospacedDigit()
            Text(label)
                .font(.system(size: 10, weight: .medium))
                .textCase(.uppercase)
                .kerning(0.5)
                .foregroundStyle(Atelier.inkQuaternary)
        }
        .frame(maxWidth: .infinity)
        .padding(.vertical, 13)
        .background(Atelier.raised)
        .accessibilityElement(children: .combine)
        .accessibilityLabel("\(value) \(label)")
    }

    /// The screen this app exists for.
    ///
    /// The shipped AppleScript applet is an agent (LSUIElement), so a failure
    /// before the Terminal opens produces nothing on screen at all. So the
    /// order here is deliberately the reverse of the old view's: what to DO
    /// first, the technical trace second and in a lower register.
    private func failureView(_ failure: ExtractionFailure) -> some View {
        VStack(alignment: .leading, spacing: 14) {
            HStack(alignment: .top, spacing: 13) {
                Image(systemName: "exclamationmark.triangle")
                    .font(.system(size: 32, weight: .light))
                    .foregroundStyle(Atelier.ember)
                    .accessibilityHidden(true)

                VStack(alignment: .leading, spacing: 3) {
                    Text("L'extraction n'a pas abouti")
                        .font(Atelier.title(.title3, weight: titleWeight))
                        .foregroundStyle(Atelier.ink)
                    Text("Rien n'a été écrit dans la destination. Vos fichiers d'origine sont intacts.")
                        .font(.caption)
                        .foregroundStyle(Atelier.inkTertiary)
                        .fixedSize(horizontal: false, vertical: true)
                }
            }

            // The advice ExtractionError has always computed and nothing ever
            // showed. See ExtractionFailure.
            if let suggestion = failure.suggestion {
                HStack(alignment: .top, spacing: 10) {
                    Image(systemName: "info.circle")
                        .foregroundStyle(Atelier.brass)
                        .accessibilityHidden(true)
                    VStack(alignment: .leading, spacing: 4) {
                        Text("Que faire")
                            .font(.callout.weight(.semibold))
                            .foregroundStyle(Atelier.ink)
                        Text(suggestion)
                            .font(.callout)
                            .foregroundStyle(Atelier.inkSecondary)
                            .fixedSize(horizontal: false, vertical: true)
                    }
                }
                .padding(13)
                .frame(maxWidth: .infinity, alignment: .leading)
                .background(
                    RoundedRectangle(cornerRadius: Atelier.cardRadius, style: .continuous)
                        .fill(Atelier.brass.opacity(0.07))
                )
                .overlay(alignment: .leading) {
                    Rectangle()
                        .fill(Atelier.brass)
                        .frame(width: 3)
                }
                .clipShape(RoundedRectangle(cornerRadius: Atelier.cardRadius, style: .continuous))
                .accessibilityElement(children: .combine)
            }

            VStack(alignment: .leading, spacing: 6) {
                Text("Détail de l'erreur")
                    .font(.system(size: 10, weight: .medium))
                    .textCase(.uppercase)
                    .kerning(0.6)
                    .foregroundStyle(Atelier.inkQuaternary)

                ScrollView {
                    Text(failure.message)
                        .font(.system(.caption, design: .monospaced))
                        .foregroundStyle(Atelier.inkTertiary)
                        .textSelection(.enabled)
                        .frame(maxWidth: .infinity, alignment: .leading)
                        .padding(12)
                }
                .frame(maxHeight: 120)
                .background(
                    RoundedRectangle(cornerRadius: 8, style: .continuous)
                        .fill(Atelier.recessed)
                )
                .overlay(
                    RoundedRectangle(cornerRadius: 8, style: .continuous)
                        .strokeBorder(
                            Atelier.line(Atelier.hairline, increasedContrast: increasedContrast)
                        )
                )
                .accessibilityLabel("Détail de l'erreur : \(failure.message)")
            }

            Spacer(minLength: 0)

            HStack(spacing: 10) {
                Spacer()
                Button("Voir le journal") {
                    withAnimation(animated(.easeInOut(duration: 0.18), value: showLog)) {
                        showLog = true
                    }
                }
                .buttonStyle(.bordered)
                .keyboardShortcut("l", modifiers: .command)

                Button("Recommencer", action: model.reset)
                    .buttonStyle(.borderedProminent)
                    .tint(Atelier.brass)
                    .keyboardShortcut(.defaultAction)
            }
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
                            .foregroundStyle(Atelier.inkQuaternary)
                            .textSelection(.enabled)
                            .frame(maxWidth: .infinity, alignment: .leading)
                            .id(index)
                    }
                }
                .padding(.horizontal, 16)
                .padding(.vertical, 10)
            }
            .frame(height: 130)
            .background(Atelier.recessed)
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

    // MARK: - Pieces

    private var card: some View {
        RoundedRectangle(cornerRadius: Atelier.cardRadius, style: .continuous)
            .fill(Atelier.raised)
            .overlay(
                RoundedRectangle(cornerRadius: Atelier.cardRadius, style: .continuous)
                    .strokeBorder(
                        Atelier.line(Atelier.hairline, increasedContrast: increasedContrast)
                    )
            )
    }

    /// Open the Settings scene.
    ///
    /// `SettingsLink` would be one line, and needs macOS 14; this app's floor
    /// is 13, matching the release workflow's. So the action goes through the
    /// responder chain - where Apple RENAMED the selector in 14, from
    /// `showPreferencesWindow:` to `showSettingsWindow:`. Sending only one of
    /// them gives a button that silently does nothing on half the supported
    /// systems, which is why both are tried.
    private func openSettings() {
        let selectors = ["showSettingsWindow:", "showPreferencesWindow:"]
        for name in selectors {
            let selector = Selector((name))
            if NSApp.sendAction(selector, to: nil, from: nil) { return }
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
