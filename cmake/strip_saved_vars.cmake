# Saved variables belong to whoever ran the client, not to the package.
#
# The bundled addons are staged with copy_directory, which takes the directory
# as it finds it - and the client writes an addon's SavedVariables next to the
# addon they belong to, including for the ones in addons/. So a developer who
# has run the client once has a .lua.saved in the source tree, and every build
# after that staged it: the package would ship with their window positions and
# their settings as its defaults.
#
# Invoked with -DDIR=<staged addons dir>.
file(GLOB_RECURSE staged_saved_vars "${DIR}/*.lua.saved")
if(staged_saved_vars)
    file(REMOVE ${staged_saved_vars})
endif()
