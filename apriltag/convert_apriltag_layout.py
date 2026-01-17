#!/usr/bin/env python3

"""
Convert AprilTag field layout CSV files to WPILib JSON format.

ASSUMPTIONS:
- CSV files are in the SAME directory as this Python file
- Output JSON files will be written to the SAME directory
"""

import csv
import json
import os
import math
import argparse

from wpimath import geometry, units
import numpy as np


def convert_apriltag_layouts(fmap=False):
    fieldLengthMeters = 17.548
    fieldWidthMeters = 8.052

    # --- Absolute directory of THIS SCRIPT ---
    script_dir = os.path.dirname(os.path.abspath(__file__))

    print("Working directory:", script_dir)

    # Find CSV files in the same directory
    filenames = [
        os.path.join(script_dir, f)
        for f in os.listdir(script_dir)
        if f.lower().endswith(".csv")
    ]

    if not filenames:
        print("ERROR: No CSV files found in script directory.")
        return

    for filename in filenames:
        json_data = {
            "tags": [],
            "field": {
                "length": fieldLengthMeters,
                "width": fieldWidthMeters
            }
        }

        fmap_data = {
            "type": "frc",
            "fiducials": []
        }

        tags = []
        xMin, xMax = 0.0, fieldLengthMeters
        yMin, yMax = 0.0, fieldWidthMeters

        # --- Read CSV ---
        with open(filename, newline="") as csvfile:
            reader = csv.reader(csvfile)
            next(reader)  # skip header

            for row in reader:
                id = int(row[0])
                x = float(row[1])
                y = float(row[2])
                z = float(row[3])
                zRot = float(row[4])
                yRot = float(row[5])

                xM = units.inchesToMeters(x)
                yM = units.inchesToMeters(y)
                zM = units.inchesToMeters(z)

                xMin, xMax = min(xMin, xM), max(xMax, xM)
                yMin, yMax = min(yMin, yM), max(yMax, yM)

                tags.append((id, xM, yM, zM, zRot, yRot))

        for id, xM, yM, zM, zRot, yRot in tags:
            q = geometry.Rotation3d(
                0.0,
                units.degreesToRadians(yRot),
                units.degreesToRadians(zRot),
            ).getQuaternion()

            json_data["tags"].append({
                "ID": id,
                "pose": {
                    "translation": {"x": xM, "y": yM, "z": zM},
                    "rotation": {
                        "quaternion": {
                            "W": q.W(),
                            "X": q.X(),
                            "Y": q.Y(),
                            "Z": q.Z(),
                        }
                    },
                },
            })

            zRad = units.degreesToRadians(zRot)
            yRad = units.degreesToRadians(yRot)

            zCos, zSin = math.cos(zRad), math.sin(zRad)
            yCos, ySin = math.cos(yRad), math.sin(yRad)

            xC = xM - ((xMax - xMin) / 2) - xMin
            yC = yM - ((yMax - yMin) / 2) - yMin

            zMatrix = np.array([
                [zCos, -zSin, 0, xC],
                [zSin,  zCos, 0, yC],
                [0,        0, 1, zM],
                [0,        0, 0, 1],
            ])

            yMatrix = np.array([
                [ yCos, 0, ySin, 0],
                [    0, 1,    0, 0],
                [-ySin, 0, yCos, 0],
                [    0, 0,    0, 1],
            ])

            transform = zMatrix @ yMatrix

            fmap_data["fiducials"].append({
                "family": "apriltag3_36h11_classic",
                "id": id,
                "size": 165.1,
                "transform": transform.ravel().tolist(),
                "unique": 1,
            })

        # --- Write output to SAME directory ---
        base = os.path.splitext(os.path.basename(filename))[0]
        json_path = os.path.join(script_dir, base + ".json")

        with open(json_path, "w") as f:
            json.dump(json_data, f, indent=2)
            f.write("\n")

        print("Wrote JSON:", json_path)

        if fmap:
            fmap_path = os.path.join(script_dir, base + ".fmap")
            with open(fmap_path, "w") as f:
                json.dump(fmap_data, f, indent=2)
                f.write("\n")
            print("Wrote FMAP:", fmap_path)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-f", "--fmap", action="store_true")
    args = parser.parse_args()

    convert_apriltag_layouts(args.fmap)
