import SwiftUI
import ExtractorKit

/// Extraction settings, in the place ⌘, opens.
///
/// These three lived in the main window, which put a form in front of a user
/// who, nine times out of ten, wants to drop a folder and press one button.
/// They are settings in the Mac sense - chosen rarely, remembered, and the
/// same for every run - so this is where they belong. The window keeps a
/// one-line summary and a way back here.
struct SettingsView: View {
    @EnvironmentObject private var model: ExtractionViewModel

    var body: some View {
        Form {
            Picker("Extension :", selection: $model.expansion) {
                ForEach(Expansion.allCases) { expansion in
                    Text(expansion.label).tag(expansion)
                }
            }
            .accessibilityLabel("Extension à extraire")

            LabeledContent("Destination :") {
                HStack(spacing: 8) {
                    Text(abbreviated(model.outputFolder))
                        .lineLimit(1)
                        .truncationMode(.head)
                        .help(model.outputFolder.path)
                    Button("Modifier…", action: model.chooseOutputFolder)
                }
            }
            .accessibilityLabel("Destination : \(abbreviated(model.outputFolder))")

            Toggle("Vérifier les fichiers extraits", isOn: $model.verify)

            Text("La vérification relit chaque fichier écrit et compare son CRC32 "
                 + "à celui du manifeste. Elle double environ la durée de l'extraction.")
                .font(.caption)
                .foregroundStyle(.secondary)
                .fixedSize(horizontal: false, vertical: true)
        }
        .formStyle(.grouped)
        // Settings windows are not resizable by convention, and this one has a
        // fixed amount to say.
        .frame(width: 460)
        .disabled(model.stage.isRunning)
        .overlay(alignment: .bottom) {
            if model.stage.isRunning {
                Text("Une extraction est en cours. Ces réglages s'appliqueront à la suivante.")
                    .font(.caption)
                    .foregroundStyle(Atelier.ember)
                    .padding(.bottom, 8)
            }
        }
    }

    private func abbreviated(_ url: URL) -> String {
        url.path.replacingOccurrences(
            of: FileManager.default.homeDirectoryForCurrentUser.path,
            with: "~"
        )
    }
}
