#!/usr/bin/env python

import os
import subprocess

for part in ["nose_cone", "body", "fin", "plume"]:
    out_filename = "./stl/{}.stl".format(part)
    openscad_cmd = "openscad -o {} -Dpart=\"{}\" rocket.scad ".format(out_filename, part)
    print(openscad_cmd)
    with open(out_filename, "w+") as f:
        subprocess.call(openscad_cmd.split())

subprocess.call("blender --python subdivide.py".split())
