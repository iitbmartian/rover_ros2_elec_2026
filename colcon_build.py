#!/usr/bin/env python3

import os
import subprocess

def main():
    base = os.getcwd()

    for name in os.listdir(base):
        full = os.path.join(base, name)

        # check only directories ending with "controls"
        if os.path.isdir(full) and name.endswith("controls"):
            print(f"Building in: {full}")

            # run colcon build inside that folder
            try:
                subprocess.run(
                    ["colcon", "build"],
                    cwd=full,
                    check=True
                )
            except subprocess.CalledProcessError as e:
                print(f"Build failed in {full}: {e}")

if __name__ == "__main__":
    main()

