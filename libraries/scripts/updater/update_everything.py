import os
import sys
from libraries.scripts.updater.replace_gradlerio_files import (
    replace_gradlerio_files,
)
from libraries.scripts.updater.update_vendor_deps import update_vendor_deps
from libraries.scripts.git.git_python_wrappers import commit_all_changes
from libraries.scripts.updater.replace_old_names import (
    run_all_replacements,
    run_standard_replacement,
    run_our_additional_replacements,
)
from libraries.scripts.updater.update_bazelrio import update_bazelrio
from libraries.scripts.gradle.run_spotless import run_smart_spotless


def update_everything():
    if "BUILD_WORKSPACE_DIRECTORY" in os.environ:
        os.chdir(os.environ["BUILD_WORKSPACE_DIRECTORY"])

    ignore_download_cache = True
    run_replacements_in_batch = True

    update_vendor_deps(ignore_cache=ignore_download_cache)
    replace_gradlerio_files(run_custom_updates=True)
    if run_replacements_in_batch:
        # run_standard_replacement(auto_commit=False)
        run_our_additional_replacements(auto_commit=False)
        run_smart_spotless(commands=["spotlessGroovyGradleApply"])
    else:
        run_all_replacements()
    update_bazelrio(ignore_cache=ignore_download_cache)
    run_smart_spotless(
        commands=["spotlessGroovyGradleApply", "spotlessMiscApply", "spotlessXmlApply"]
    )


if __name__ == "__main__":
    # py -m libraries.scripts.updater.update_everything
    update_everything()
