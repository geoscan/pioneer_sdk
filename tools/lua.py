"""
API to Lua-related functionality
"""

import os, pathlib


def compile(lua_source) -> str:
    """ Compiles a binary from Lua source file """

    out = "hw.luac"

    if os.name == "nt":  # "Windows"
        tool = pathlib.Path("luac.exe")
    elif os.name == "posix":  # "Linux"
        tool = pathlib.Path("luac")

    os.system(f"{tool} -s -o {out} {lua_source}")

    return pathlib.Path(out)
