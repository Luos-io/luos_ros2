#!/usr/bin/env python3

from luos_interface.containers import _make as containers
from luos_interface.containers import *

for name, Class in containers.items():
    # Call the publisher's constructor with node = None to print the doc

    print("# Container", name)
    print("## ROS topics")
    print("| **Topic name** | **Message type** |")
    print("|:----|:---:|")

    c = Class(None, None, None)
    print(c.get_markdown_doc())
    print("\n\n")