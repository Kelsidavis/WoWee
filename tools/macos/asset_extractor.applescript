on run
    set appPath to POSIX path of (path to me)
    set runnerPath to appPath & "Contents/Resources/extract-assets-terminal.sh"

    -- Opened through LaunchServices rather than by telling Terminal to do it.
    --
    -- "tell application \"Terminal\" to do script" is an Apple event, and
    -- since macOS 10.14 sending one is gated behind the Automation privacy
    -- permission. An app whose Info.plist carries no
    -- NSAppleEventsUsageDescription is refused outright and never prompts, so
    -- the extractor failed with "Not authorized to send Apple events to
    -- Terminal" and no way for anyone to say yes. Reported in #117, where the
    -- reporter got past it by running the inner applet binary by hand.
    --
    -- "open -a" asks LaunchServices to open the file with Terminal, which
    -- needs no permission at all and still gets the window the long
    -- extraction wants.
    do shell script "/usr/bin/open -a Terminal " & quoted form of runnerPath
end run
