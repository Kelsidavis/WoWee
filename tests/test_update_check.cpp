/// Version ordering for the update notice.
///
/// The comparison is the whole of the decision: wrong one way it nags
/// somebody who is already current, wrong the other it stays quiet for
/// somebody who is not. No network here - only the ordering.

#include <catch_amalgamated.hpp>

#include "core/update_check.hpp"

using wowee::core::isNewerVersion;

TEST_CASE("a later release is newer", "[update]") {
    CHECK(isNewerVersion("v3.1.31", "v3.1.30"));
    CHECK(isNewerVersion("v3.2.0", "v3.1.30"));
    CHECK(isNewerVersion("v4.0.0", "v3.99.99"));
    // Ten is not one, which is what a string comparison would make of it.
    CHECK(isNewerVersion("v3.1.30", "v3.1.9"));
    CHECK(isNewerVersion("v3.10.0", "v3.9.0"));
}

TEST_CASE("the same release is not newer", "[update]") {
    CHECK_FALSE(isNewerVersion("v3.1.30", "v3.1.30"));
}

TEST_CASE("an older release is not newer", "[update]") {
    CHECK_FALSE(isNewerVersion("v3.1.29", "v3.1.30"));
    CHECK_FALSE(isNewerVersion("v2.9.9", "v3.0.0"));
    CHECK_FALSE(isNewerVersion("v3.1.9", "v3.1.30"));
}

TEST_CASE("a build past the tag is not sent backwards", "[update]") {
    // git describe on a build with commits on top of v3.1.30. It is ahead of
    // that release, so that release is not something to go and download.
    CHECK_FALSE(isNewerVersion("v3.1.30", "v3.1.30-7-gabc1234"));
    // A genuinely later release still shows, though.
    CHECK(isNewerVersion("v3.1.31", "v3.1.30-7-gabc1234"));
}

TEST_CASE("nothing to compare means no notice", "[update]") {
    CHECK_FALSE(isNewerVersion("", "v3.1.30"));
    CHECK_FALSE(isNewerVersion("v3.1.31", ""));
    CHECK_FALSE(isNewerVersion("", ""));
    // A tag that is not a version at all reads as 0.0.0 and must not be
    // announced as an upgrade over a real one.
    CHECK_FALSE(isNewerVersion("nightly", "v3.1.30"));
}

TEST_CASE("the v is optional on either side", "[update]") {
    CHECK(isNewerVersion("3.1.31", "v3.1.30"));
    CHECK(isNewerVersion("v3.1.31", "3.1.30"));
    CHECK_FALSE(isNewerVersion("3.1.30", "3.1.30"));
}
