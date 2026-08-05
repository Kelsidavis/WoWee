# CPack, the install rules that follow it, and the bundled-addon copy.
# Extracted from the top-level CMakeLists.txt, which had grown past two
# thousand lines and was asked about in #107. Moved verbatim: this is the same
# text in a file of its own, included from the same scope, so every variable it
# reads and every one it sets behaves exactly as before.

# ---- CPack packaging ----
set(CPACK_PACKAGE_NAME "wowee")
set(CPACK_PACKAGE_VERSION "${PROJECT_VERSION}")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "World of Warcraft client emulator")
set(CPACK_PACKAGE_VENDOR "Wowee")
set(CPACK_PACKAGE_INSTALL_DIRECTORY "Wowee")
set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")

if(WIN32)
    set(CPACK_GENERATOR "NSIS")
    set(CPACK_NSIS_DISPLAY_NAME "Wowee")
    set(CPACK_NSIS_PACKAGE_NAME "Wowee")
    set(CPACK_NSIS_INSTALL_ROOT "$PROGRAMFILES64")
    set(CPACK_NSIS_ENABLE_UNINSTALL_BEFORE_INSTALL ON)
    # Run wowee from bin/ so that ./assets/ resolves correctly.
    # SetOutPath sets the shortcut's working directory in NSIS.
    set(CPACK_NSIS_CREATE_ICONS_EXTRA
        "SetOutPath '$INSTDIR\\\\bin'\nCreateShortCut '$SMPROGRAMS\\\\$STARTMENU_FOLDER\\\\Wowee.lnk' '$INSTDIR\\\\bin\\\\wowee.exe'")
    set(CPACK_NSIS_DELETE_ICONS_EXTRA
        "Delete '$SMPROGRAMS\\\\$STARTMENU_FOLDER\\\\Wowee.lnk'")
elseif(APPLE)
    set(CPACK_GENERATOR "DragNDrop")
else()
    # Linux — generate postinst/prerm wrapper scripts
    file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/packaging)
    # postinst: write a wrapper script at /usr/local/bin/wowee that cd's to
    # the install dir so ./assets/ resolves correctly.
    file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/packaging/postinst
[[#!/bin/sh
cat > /usr/local/bin/wowee << 'WOWEE_WRAPPER'
#!/bin/sh
cd /opt/wowee/bin
exec ./wowee "$@"
WOWEE_WRAPPER
chmod +x /usr/local/bin/wowee
]])
    file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/packaging/prerm
"#!/bin/sh\nrm -f /usr/local/bin/wowee\n")
    file(CHMOD
        ${CMAKE_CURRENT_BINARY_DIR}/packaging/postinst
        ${CMAKE_CURRENT_BINARY_DIR}/packaging/prerm
        PERMISSIONS
            OWNER_EXECUTE OWNER_WRITE OWNER_READ
            GROUP_EXECUTE GROUP_READ
            WORLD_EXECUTE WORLD_READ)

    set(CPACK_GENERATOR "DEB")
    set(CPACK_PACKAGING_INSTALL_PREFIX "/opt/wowee")
    set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Wowee")
    set(CPACK_DEBIAN_PACKAGE_SECTION "games")
    set(CPACK_DEBIAN_PACKAGE_DEPENDS
        "libsdl2-2.0-0, libvulkan1, libssl3, zlib1g")
    set(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA
        "${CMAKE_CURRENT_BINARY_DIR}/packaging/postinst;${CMAKE_CURRENT_BINARY_DIR}/packaging/prerm")
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64")
        set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "amd64")
    elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|ARM64")
        set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "arm64")
    endif()
endif()

include(CPack)

# Bundled addons ship next to the executable, so the client finds them without
# anyone copying files into an extracted game install. AddonManager searches
# here as well as the game data's own Interface/AddOns.
add_custom_target(copy_bundled_addons ALL
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            ${CMAKE_SOURCE_DIR}/addons $<TARGET_FILE_DIR:wowee>/addons
    COMMENT "Copying bundled addons")
add_dependencies(copy_bundled_addons wowee)
