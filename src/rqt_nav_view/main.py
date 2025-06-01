#!/usr/bin/env python3

import sys

from rqt_gui.main import Main
from rqt_nav_view.nav_view_plugin import NavViewPlugin


def main():
    plugin = "rqt_nav_view.nav_view_plugin.NavViewPlugin"
    main = Main(filename=plugin)
    sys.exit(
        main.main(
            standalone=plugin,
            plugin_argument_provider=NavViewPlugin.add_arguments,
        )
    )


if __name__ == "__main__":
    main()
