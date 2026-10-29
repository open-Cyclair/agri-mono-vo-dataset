#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Download a dataset (ROS 2 bag) from a MARV Robotics server.

The dataset is identified by the hash at the end of its MARV URL. For example,
for https://marv.mesocentre.uca.fr/?filter=#/detail/<hash>, pass <hash>.

Credentials are never stored in this script. They are read from the
MARV_USER / MARV_PASSWORD environment variables, or from --user with an
interactive password prompt.
"""

import argparse
import getpass
import json
import os
import pathlib
import shutil
import sys

import requests

DEFAULT_MARV_API = "https://marv.mesocentre.uca.fr/marv/api/"
TIMEOUT_S = 60

parser = argparse.ArgumentParser(
    prog=os.path.basename(sys.argv[0]),
    description="""\
        Download a dataset from marv using the hash at the end of the dataset URL. For example, for the dataset at https://marv.mesocentre.uca.fr/?filter=#/detail/<hash>, the hash is <hash>.
      """,
)
parser.add_argument("dataset", type=str, help="dataset hash on marv")
parser.add_argument(
    "--output-dir",
    default=".",
    help="Directory where the bag folder is written (default: current directory).",
)
parser.add_argument(
    "--api",
    default=os.environ.get("MARV_API", DEFAULT_MARV_API),
    help="MARV API base URL (default: $MARV_API or %(default)s).",
)
parser.add_argument(
    "--user",
    default=os.environ.get("MARV_USER"),
    help="MARV user name (default: $MARV_USER).",
)
parser.add_argument(
    "--verbose",
    action="store_true",
    help="Print the dataset query and file list returned by the server.",
)
args = parser.parse_args()

MARV_API = args.api if args.api.endswith("/") else args.api + "/"

MARV_USER = args.user or input("MARV user: ")
MARV_PASS = os.environ.get("MARV_PASSWORD") or getpass.getpass("MARV password: ")

json_data = {"username": MARV_USER, "password": MARV_PASS}

response = requests.post(MARV_API + "auth", json=json_data, timeout=TIMEOUT_S)
response.raise_for_status()
print("Authenticated on " + MARV_API)

token = response.json()["access_token"]

headers = {"Authorization": "Bearer " + token, "Content-Type": "application/json"}

json_req = {
    "rpcs": [
        {
            "query": {
                "model": "dataset",
                "filters": [
                    {
                        "op": "eq",
                        "name": "setid",
                        "value": args.dataset,
                    },
                ],
            },
        },
    ]
}

r_query = requests.post(
    MARV_API + "v1/rpcs", headers=headers, data=json.dumps(json_req), timeout=TIMEOUT_S
)
r_query.raise_for_status()
r_query_json = r_query.json()
if args.verbose:
    print(json.dumps(r_query_json, indent=2))

datasets = r_query_json["data"]["dataset"]
if not datasets:
    sys.exit("No dataset found on MARV for hash: " + args.dataset)

r_file_list = requests.post(
    MARV_API + "file-list",
    headers=headers,
    data="[" + str(datasets[0]["id"]) + "]",
    timeout=TIMEOUT_S,
)
r_file_list.raise_for_status()
file_list = r_file_list.json()
if args.verbose:
    print(json.dumps(file_list, indent=2))

output_root = pathlib.Path(args.output_dir)
for id, path in enumerate(file_list["paths"]):
    with requests.get(
        MARV_API + file_list["urls"][id], headers=headers, stream=True, timeout=TIMEOUT_S
    ) as r_file:
        r_file.raise_for_status()
        # Keep the original layout: <bag_folder>/<file> (metadata.yaml, *.mcap).
        file_name = path.split("/")[-2] + "/" + path.split("/")[-1]
        print("Downloading " + str(file_name) + " from " + str(file_list["urls"][id]))
        output_path = output_root / file_name
        output_path.parent.mkdir(exist_ok=True, parents=True)
        with open(output_path, "wb") as f:
            shutil.copyfileobj(r_file.raw, f)
