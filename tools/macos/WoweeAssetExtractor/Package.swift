// swift-tools-version: 5.9
import PackageDescription

// The app is split in two on purpose. Everything that can be tested without a
// window - locating the extractor, parsing its output, deciding whether a
// folder is a WoW Data directory - lives in ExtractorKit, which the test
// target imports. The executable is the SwiftUI shell around it and holds no
// logic worth testing.
let package = Package(
    name: "WoweeAssetExtractor",
    platforms: [.macOS(.v13)],
    targets: [
        .target(name: "ExtractorKit"),
        .executableTarget(
            name: "WoweeAssetExtractor",
            dependencies: ["ExtractorKit"]
        ),
        .testTarget(
            name: "ExtractorKitTests",
            dependencies: ["ExtractorKit"]
        ),
    ]
)
