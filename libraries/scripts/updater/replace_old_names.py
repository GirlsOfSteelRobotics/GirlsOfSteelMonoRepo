"""
Runs a regex replace on any of the class names that got changed in the past year updates of wpilib / vendor deps
"""
import json
from libraries.scripts.updater.utils import (
    PINNED_VSCODE_WPILIB_COMMITISH,
    walk_for_extension,
    regex_replace_file,
    auto_retry_download,
)
from libraries.scripts.git.git_python_wrappers import commit_all_changes


def __run_replacement(replacements, root=".", dir_blacklist=None):
    java_files = walk_for_extension(root, "java", dir_blacklist=dir_blacklist)

    for java_file in java_files:
        regex_replace_file(java_file, replacements)


def run_standard_replacement(auto_commit):
    # Last sync Dec 19, 2021
    wpilib_replacements_url = f"https://raw.githubusercontent.com/wpilibsuite/vscode-wpilib/{PINNED_VSCODE_WPILIB_COMMITISH}/vscode-wpilib/resources/java_replacements.json"

    raw_json_data = auto_retry_download(wpilib_replacements_url).decode("utf-8")
    json_data = json.loads(raw_json_data)

    replacements = []
    for replacement_json in json_data[0]["replacements"]:
        replacement_to = replacement_json["to"]
        # Python-ize the replacement substitution
        replacement_to = replacement_to.replace("$1", r"\1").replace("$2", r"\2")
        if "$" in replacement_to:
            raise Exception(f"Make this smarter. To = '{replacement_to}")
        replacements.append((replacement_json["from"], replacement_to))

    # Undo some of the auto-changes
    replacements.append(("GosCommand", "GosCommandBase"))
    replacements.append(("import edu.wpi.first.units.measure.measure", "import edu.wpi.first.units.measure"))

    # Run these on all the files
    __run_replacement(replacements)

    if auto_commit:
        commit_all_changes("Auto-Update: Ran standard vscode replacements")


def run_our_additional_replacements(auto_commit):
    replacements = []

    # Put our smarter-than-wpilib replacements here
    # fmt: off
    # fmt: on

    # Run these on all the files
    __run_replacement(replacements)

    if auto_commit:
        commit_all_changes("Auto-Update: Ran our additional replacements")


def run_all_replacements(auto_commit=True):
    run_standard_replacement(auto_commit=auto_commit)
    run_our_additional_replacements(auto_commit=auto_commit)


if __name__ == "__main__":
    #  py -m libraries.scripts.updater.replace_old_names
    run_all_replacements(auto_commit=False)
