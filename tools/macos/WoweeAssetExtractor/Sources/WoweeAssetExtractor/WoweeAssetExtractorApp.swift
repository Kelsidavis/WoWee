import SwiftUI

@main
struct WoweeAssetExtractorApp: App {
    var body: some Scene {
        Window("Extracteur d'assets WoWee", id: "main") {
            ContentView()
        }
        // The window is the app. Resizing it wider buys nothing, and this is
        // a utility that should open the same size every time.
        .windowResizability(.contentSize)
        .commands {
            CommandGroup(replacing: .newItem) {}
        }
    }
}
