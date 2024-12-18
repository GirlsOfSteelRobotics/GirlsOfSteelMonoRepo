import os
import re
from urllib.request import urlopen, Request
import http
import time
import urllib

DEFAULT_DIR_BLACKLIST = [
    ".git",
    "bazel-bin",
    "bazel-out",
    "bazel-testlogs",
    "bazel-gos_monorepo",
    "bazel-girlsofsteelfrc",
    ".gradle",
    "gradle",
    ".idea",
    ".ijwb",
    ".github",
    "build",
    "venv",
]

PINNED_VSCODE_WPILIB_COMMITISH = "main"


def walk_with_blacklist(search_root, dir_blacklist=None):
    if dir_blacklist is None:
        dir_blacklist = DEFAULT_DIR_BLACKLIST

    for root, dirs, files in os.walk(search_root):
        dirs[:] = [d for d in dirs if d not in dir_blacklist]

        yield root, dirs, files


def walk_for_extension(search_root, extension, dir_blacklist=None):
    out_files = []
    for root, dirs, files in walk_with_blacklist(search_root, dir_blacklist=dir_blacklist):
        out_files.extend([os.path.join(root, f) for f in files if f.endswith(extension)])

    return out_files


def regex_replace_file(file, replacements):
    with open(file, "rb") as f:
        contents = f.read().decode("utf-8")
    for repl_from, repl_to in replacements:
        contents = re.sub(repl_from, repl_to, contents)

    with open(file, "wb") as f:
        f.write(contents.encode("utf-8"))


def auto_retry_download(url):
    print(f"Downloading from {url}")

    req = Request(url)
    req.add_header(
        "User-Agent",
        "Mozilla/5.0 (Windows; U; Windows NT 5.1; en-US; rv:1.9.0.7) Gecko/2009021910 Firefox/3.0.7",
    )

    retries = 0

    while retries < 100:
        retries += 1

        try:
            with urlopen(req) as x:
                return x.read()
        except (http.client.RemoteDisconnected, urllib.error.URLError) as e:
            print(f"  Trying again... - {e}")
            time.sleep(0.1)
        except Exception as e:
            print("Got a different failure...")
            raise e

    raise Exception(f"Could not {url}")
