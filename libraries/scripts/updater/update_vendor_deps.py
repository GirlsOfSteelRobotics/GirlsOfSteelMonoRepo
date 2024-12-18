"""
Downloads the latest version of the vendor dependences, and replaces all copies of them in the repository
"""
import os
import shutil
import tempfile
from libraries.scripts.updater.utils import (
    walk_with_blacklist,
    auto_retry_download,
    PINNED_VSCODE_WPILIB_COMMITISH,
)
from libraries.scripts.git.git_python_wrappers import commit_all_changes

CACHE_DIRECTORY = os.path.join(tempfile.gettempdir(), "gos_vendor_cache")


def download_latest_vendordeps(ignore_cache):
    year = "2025"
    # fmt: off
    vendor_dep_urls = {}
    vendor_dep_urls["navx_frc.json"] = f"https://dev.studica.com/releases/{year}/NavX-2025.1.1-beta-1.json"
    vendor_dep_urls["Phoenix.json"] = f"https://maven.ctr-electronics.com/release/com/ctre/phoenix/Phoenix5-frc{year}-beta-latest.json"
    vendor_dep_urls["Phoenix6.json"] = f"https://maven.ctr-electronics.com/release/com/ctre/phoenix6/latest/Phoenix6-frc{year}-beta-latest.json"
    vendor_dep_urls["REVLib.json"] = f"https://software-metadata.revrobotics.com/REVLib-{year}.json"
    vendor_dep_urls["SnobotSim.json"] = "http://raw.githubusercontent.com/snobotsim/maven_repo/master/development/SnobotSim.json"
    vendor_dep_urls["PhotonLib-json-1.0.json"] = "https://maven.photonvision.org/repository/internal/org/photonvision/photonlib-json/1.0/photonlib-json-1.0.json"
    vendor_dep_urls["PathplannerLib.json"] = "https://3015rangerrobotics.github.io/pathplannerlib/PathplannerLib-beta.json"
    vendor_dep_urls["ChoreoLib.json"] = f"https://sleipnirgroup.github.io/ChoreoLib/dep/ChoreoLib{year}Beta.json"
    vendor_dep_urls["WPILibNewCommands.json"] = f"https://raw.githubusercontent.com/wpilibsuite/allwpilib/{PINNED_VSCODE_WPILIB_COMMITISH}/wpilibNewCommands/WPILibNewCommands.json"
    # fmt: on

    vendor_files = []

    if not os.path.exists(CACHE_DIRECTORY):
        os.mkdir(CACHE_DIRECTORY)

    for name, url in vendor_dep_urls.items():
        cached_file = os.path.join(CACHE_DIRECTORY, name)
        vendor_files.append(cached_file)
        if os.path.exists(cached_file) and not ignore_cache:
            continue

        print(f"Downloading {cached_file} - {url}")

        with open(cached_file, "wb") as f:
            f.write(auto_retry_download(url))

    return vendor_files


def update_vendor_deps(ignore_cache=True, auto_commit=True):
    print(f"Writing vendordep cache to {CACHE_DIRECTORY}")

    # Used when a vendor changes its filename name
    vendor_replacements = {}

    download_latest_vendordeps(ignore_cache)

    for root, dirs, files in walk_with_blacklist("."):
        if root.endswith("vendordeps"):
            for vendor_file in files:
                if vendor_file in vendor_replacements:
                    new_file = os.path.join(CACHE_DIRECTORY, vendor_replacements[vendor_file])
                    file_to_write = os.path.join(root, vendor_replacements[vendor_file])
                    os.remove(os.path.join(root, vendor_file))
                else:
                    new_file = os.path.join(CACHE_DIRECTORY, vendor_file)
                    file_to_write = os.path.join(root, vendor_file)

                shutil.copy(new_file, file_to_write)

    if auto_commit:
        commit_all_changes("Auto-Update: Updating vendor deps")


def main():
    if "BUILD_WORKSPACE_DIRECTORY" in os.environ:
        os.chdir(os.environ["BUILD_WORKSPACE_DIRECTORY"])

    update_vendor_deps(ignore_cache=True, auto_commit=False)


if __name__ == "__main__":
    # py -m libraries.scripts.updater.update_vendor_deps
    main()
