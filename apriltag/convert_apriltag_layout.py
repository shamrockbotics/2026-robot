#!/usr/bin/env python3

"""
This script converts all AprilTag field layout CSV files in
src/main/native/resources/edu/wpi/first/apriltag to the JSON format
AprilTagFields expects.

The input CSV has the following format:

* Columns: ID, X, Y, Z, Z Rotation, Y Rotation
* ID is a positive integer
* X, Y, and Z are decimal inches
* Z Rotation is yaw in degrees
* Y Rotation is pitch in degrees

The values come from a table in the layout marking diagram (e.g.,
https://firstfrc.blob.core.windows.net/frc2024/FieldAssets/2024LayoutMarkingDiagram.pdf).
"""

import csv
import json
import os
import math
import argparse

from wpimath import geometry, units
from json import encoder

import numpy as np

def convert_apriltag_layouts(fmap = False):
    fieldLengthMeters = 17.548
    fieldWidthMeters = 8.052

    # Find AprilTag field layout CSVs
    filenames = [
        os.path.join(dp, f)
        for dp, dn, fn in os.walk(".")
        for f in fn
        if f.endswith(".csv")
    ]

    for filename in filenames:
        json_data = {"tags": [], "field": {"length": fieldLengthMeters, "width": fieldWidthMeters}}
        fmap_data = {"type": "frc", "fiducials": []}

        tags = []
        xMin = 0.0
        xMax = fieldLengthMeters
        yMin = 0.0
        yMax = fieldWidthMeters

        # Read CSV and fill in JSON data
        with open(filename, newline="") as csvfile:
            reader = csv.reader(csvfile, delimiter=",")

            # Skip header
            next(reader)

            for row in reader:
                # Unpack row elements
                id = int(row[0])
                x = float(row[1])
                y = float(row[2])
                z = float(row[3])
                zRotation = float(row[4])
                yRotation = float(row[5])

                xMeters = units.inchesToMeters(x)
                yMeters = units.inchesToMeters(y)
                zMeters = units.inchesToMeters(z)

                if(xMeters < xMin):
                    xMin = xMeters
                if(xMeters > xMax):
                    xMax = xMeters
                if(yMeters < yMin):
                    yMin = yMeters
                if(yMeters > yMax):
                    yMax = yMeters

                tags.append(
                    {
                        "id": id,
                        "xMeters": xMeters,
                        "yMeters": yMeters,
                        "zMeters": zMeters,
                        "zRotation": zRotation,
                        "yRotation": yRotation
                    }
                )

        for tag in tags:
            id = tag["id"]
            xMeters = tag["xMeters"]
            yMeters = tag["yMeters"]
            zMeters = tag["zMeters"]
            zRotation = tag["zRotation"]
            yRotation = tag["yRotation"]

            # Turn yaw/pitch into quaternion
            q = geometry.Rotation3d(
                units.radians(0),
                units.degreesToRadians(yRotation),
                units.degreesToRadians(zRotation),
            ).getQuaternion()

            json_data["tags"].append(
                {
                    "ID": id,
                    "pose": {
                        "translation": {
                            "x": xMeters,
                            "y": yMeters,
                            "z": zMeters,
                        },
                        "rotation": {
                            "quaternion": {
                                "W": q.W(),
                                "X": q.X(),
                                "Y": q.Y(),
                                "Z": q.Z(),
                            }
                        },
                    },
                }
            )

            zRotationCos = math.cos(units.degreesToRadians(zRotation))
            zRotationSin = math.sin(units.degreesToRadians(zRotation))
            yRotationCos = math.cos(units.degreesToRadians(yRotation))
            yRotationSin = math.sin(units.degreesToRadians(yRotation))

            xMetersFromCenter = xMeters - ((xMax - xMin) / 2) - xMin
            yMetersFromCenter = yMeters - ((yMax - yMin) / 2) - yMin
            zMetersFromCenter = zMeters

            zMatrix = ([zRotationCos, -zRotationSin, 0, xMetersFromCenter],
                       [zRotationSin,  zRotationCos, 0, yMetersFromCenter],
                       [           0,             0, 1, zMetersFromCenter],
                       [           0,             0, 0,                 1])

            yMatrix = ([ yRotationCos, 0, yRotationSin, 0],
                       [            0, 1,            0, 0],
                       [-yRotationSin, 0, yRotationCos, 0],
                       [            0, 0,            0, 1])

            transform = np.array(zMatrix).dot(np.array(yMatrix))

            fmap_data["fiducials"].append(
                {
                    "family": "apriltag3_36h11_classic",
                    "id": id,
                    "size": 165.1,
                    "transform": transform.ravel().tolist(),
                    "unique": 1
                }
            )

        # Write JSON
        with open("../src/main/deploy/"+filename.replace(".csv", ".json"), "w") as f:
            json.dump(json_data, f, indent=2)
            f.write("\n")
        # Write FMAP
        if (fmap):
            with open("./"+filename.replace(".csv", ".fmap"), "w") as f:
                json.dump(fmap_data, f, indent=2)
                f.write("\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-f', '--fmap', action='store_true', default=False)
    args = parser.parse_args()
    convert_apriltag_layouts(args.fmap)