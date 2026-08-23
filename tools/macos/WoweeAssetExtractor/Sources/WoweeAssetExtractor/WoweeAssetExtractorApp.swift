import SwiftUI

@main
struct WoweeAssetExtractorApp: App {

    // Owned here rather than in ContentView so the menu bar can drive the same
    // state the window shows. A Mac app whose actions exist only as buttons is
    // missing the surface users look at first.
    @StateObject private var model = ExtractionViewModel()
    @AppStorage("showLog") private var showLog = false

    var body: some Scene {
        Window("Extracteur d'assets WoWee", id: "main") {
            ContentView(showLog: $showLog)
                .environmentObject(model)
        }
        .defaultSize(width: 560, height: 500)
        .commands {
            // This app opens no documents, so New has nothing to make.
            CommandGroup(replacing: .newItem) {
                Button("Choisir le dossier Data…") {
                    model.chooseDataFolder()
                }
                .keyboardShortcut("o", modifiers: .command)
            }

            CommandGroup(after: .saveItem) {
                Button("Afficher les fichiers extraits dans le Finder") {
                    model.revealOutput()
                }
                .keyboardShortcut("r", modifiers: [.command, .shift])
            }

            CommandMenu("Extraction") {
                Button("Extraire") { model.start() }
                    .keyboardShortcut("r", modifiers: .command)
                    .disabled(!model.canStart)

                Button("Annuler") { model.cancel() }
                    .keyboardShortcut(".", modifiers: .command)
                    .disabled(model.stage != .running)

                Divider()

                Button("Choisir la destination…") { model.chooseOutputFolder() }
                    .disabled(model.stage == .running)

                Toggle("Vérifier les fichiers extraits", isOn: $model.verify)
                    .disabled(model.stage == .running)

                Divider()

                Button("Recommencer") { model.reset() }
                    .disabled(model.stage == .running || model.stage == .idle)
            }

            CommandGroup(after: .toolbar) {
                // The title changes with the state, which is how a Mac menu
                // item says what it will do rather than what it is about.
                Button(showLog ? "Masquer le journal" : "Afficher le journal") {
                    showLog.toggle()
                }
                .keyboardShortcut("l", modifiers: .command)
            }
        }
    }
}
