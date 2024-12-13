"""
"""

load("@bzlmodrio//private/non_bzlmod:download_dependencies.bzl", "download_dependencies")
load("@rules_jvm_external//:repositories.bzl", "rules_jvm_external_deps")
load("//build_scripts/bazel/deps:versions.bzl", "NAVX_FRC_VERSION", "PATHPLANNERLIB_VERSION", "PHOENIX6_VERSION", "PHOENIX_VERSION", "PHOTONLIB_JSON_1_0_VERSION", "REVLIB_VERSION", "WPILIB_VERSION", "CHOREOLIB_VERSION")

SNOBOT_SIM_VERSION = "2022.2.2.0"

def setup_dependencies():
    """
    Second step of dependency initialization.
    """
    rules_jvm_external_deps()

    download_dependencies(
        allwpilib_version = WPILIB_VERSION,
        apriltaglib_version = None,
        imgui_version = None,
        libssh_version = None,
        navx_version = NAVX_FRC_VERSION,
        phoenix_version = PHOENIX_VERSION,
        phoenix6_version = PHOENIX6_VERSION,
        revlib_version = REVLIB_VERSION,
        rules_pmd_version = "6.43.0",
        rules_wpi_styleguide_version = "1.0.0",
        photonlib_version = PHOTONLIB_JSON_1_0_VERSION,
        pathplannerlib_version = PATHPLANNERLIB_VERSION,
        choreolib_version = CHOREOLIB_VERSION,

        # Always the default version of these libraries
        # rules_spotless_version = None,
        # rules_wpi_styleguide_version = None,
        # ni_version = None,
        # opencv_version = None,
        # rules_bazelrio_version = None,
        # rules_toolchains_version = None,
        # rules_checkstyle_version = None,
    )
