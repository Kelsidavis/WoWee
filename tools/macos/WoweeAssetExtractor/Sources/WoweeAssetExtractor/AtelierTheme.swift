import SwiftUI
import AppKit

/// The Atelier palette: the app icon, taken at its word — in both appearances.
///
/// `assets/Wowee.png` is brass on charcoal, and the first cut of this theme
/// forced dark mode because "brass on white is not the same design". That was
/// true of *white*. It is not true of the workbench the icon actually depicts:
/// in light mode the ground becomes parchment and the brass darkens, and the
/// direction survives intact while the window follows the system the way a Mac
/// app is supposed to.
///
/// EVERY COLOUR BELOW WAS MEASURED, NOT PICKED
///
/// The first version had `inkQuaternary` at 3.02:1 against its own background -
/// and that is the colour carrying every 10px uppercase label in the app (the
/// phase chips, the summary captions, the log). The values here clear 4.5:1
/// for text and 3:1 for anything that carries meaning without words, in both
/// appearances. Changing one means re-measuring it.
enum Atelier {

    /// One colour, two appearances. `NSColor(name:dynamicProvider:)` resolves
    /// per-view at draw time, so this follows a system theme change live -
    /// which a `@Environment(\.colorScheme)` branch in every view would not.
    private static func dynamic(light: String, dark: String) -> Color {
        Color(nsColor: NSColor(name: nil) { appearance in
            appearance.bestMatch(from: [.aqua, .darkAqua]) == .darkAqua
                ? NSColor(hex: dark)
                : NSColor(hex: light)
        })
    }

    // MARK: - Ground

    /// The window's own background: parchment, or the forge.
    static let ground = dynamic(light: "F5F1E8", dark: "17150F")
    /// Cards and grouped rows sitting on the ground.
    static let raised = dynamic(light: "EDE7DA", dark: "1E1A13")
    /// Controls, and the title bar.
    static let control = dynamic(light: "E4DCCA", dark: "241F16")
    /// The log, which sits lower than the ground.
    static let recessed = dynamic(light: "E8E2D4", dark: "100E0A")

    // MARK: - Lines

    static let hairline = dynamic(light: "D8CFBA", dark: "2C2619")
    static let border = dynamic(light: "C2B69C", dark: "3A3225")

    // MARK: - Ink
    //
    // Measured against `ground` and `raised` in both appearances. The weakest
    // pairing in the set is inkQuaternary on raised: 4.65:1 light, 5.09:1 dark.

    static let ink = dynamic(light: "241E14", dark: "F0E6D2")
    static let inkSecondary = dynamic(light: "4A4133", dark: "C4B9A4")
    static let inkTertiary = dynamic(light: "635947", dark: "9A8F7C")
    static let inkQuaternary = dynamic(light: "6F6553", dark: "948A78")

    // MARK: - Brass, and the two states that are not brass

    /// Text, links, active borders, and the default button's tint.
    ///
    /// Light is a darkened brass rather than the icon's own: the icon's
    /// #D9A441 reads 1.9:1 on parchment, which is not a colour, it is a
    /// suggestion of one. This one clears 5.6:1 and still reads as brass.
    static let brass = dynamic(light: "7E5814", dark: "D9A441")
    static let brassDeep = dynamic(light: "5E410F", dark: "B8822E")
    static let brassPale = dynamic(light: "9A6E1C", dark: "E8C37A")

    /// Confirmation. Muted rather than system green, which on either ground
    /// reads as a notification badge.
    static let verdigris = dynamic(light: "2F6B2F", dark: "7FBF7F")
    /// Warning and failure share one colour: a red distinct enough from brass
    /// to register has to be so saturated it looks like an alarm.
    static let ember = dynamic(light: "94500E", dark: "D08A3C")

    // MARK: - Type

    /// Titles are serif, body is not.
    ///
    /// `design: .serif` resolves to New York, which ships with the system.
    /// Bundling the mockup's Bitter would mean a licence to carry, a ~200 KB
    /// binary in a source tree, and a face that has to be registered before
    /// first paint - for a difference nobody would name.
    static func title(_ style: Font.TextStyle, weight: Font.Weight = .semibold) -> Font {
        .system(style, design: .serif).weight(weight)
    }

    // MARK: - Shape

    static let cardRadius: CGFloat = 10
    static let dropRadius: CGFloat = 12
    static let controlRadius: CGFloat = 7

    // MARK: - Accessibility

    /// With Increase Contrast on, hairlines are not edges.
    ///
    /// This palette leans on low-contrast separators to keep the window calm;
    /// that is exactly what the setting exists to override. Every border that
    /// carries meaning - the drop target above all - asks here first.
    static func line(_ base: Color, increasedContrast: Bool) -> Color {
        increasedContrast ? ink : base
    }
}

private extension NSColor {
    /// Six hex digits, no hash. Anything else is a typo in the palette above,
    /// and magenta is easier to spot in a window than a silent black.
    convenience init(hex: String) {
        var value: UInt64 = 0
        guard hex.count == 6, Scanner(string: hex).scanHexInt64(&value) else {
            self.init(red: 1, green: 0, blue: 1, alpha: 1)
            return
        }
        self.init(
            srgbRed: CGFloat((value >> 16) & 0xFF) / 255,
            green: CGFloat((value >> 8) & 0xFF) / 255,
            blue: CGFloat(value & 0xFF) / 255,
            alpha: 1
        )
    }
}

extension Bundle {
    /// Where the app mark lives, on either build path.
    ///
    /// SwiftPM synthesises `Bundle.module` for a target with resources; an
    /// Xcode application target has no such symbol - its resources are copied
    /// straight into the main bundle. Referring to `.module` unconditionally
    /// builds under `swift build` and fails under `xcodebuild`, which is the
    /// kind of break that only shows up on the other person's machine.
    static var assets: Bundle {
        #if SWIFT_PACKAGE
        return .module
        #else
        return .main
        #endif
    }
}
