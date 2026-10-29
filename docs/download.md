# Downloading the ROS 2 Bags

The sequences are distributed as ROS 2 bags (MCAP storage) on a MARV Robotics
server in Clermont-Ferrand. Each sequence is a MARV *dataset*, identified by
the hash at the end of its URL:

```text
https://marv.mesocentre.uca.fr/?filter=#/detail/<dataset_hash>
                                                 ^^^^^^^^^^^^^^
```

> **TODO:** list the dataset hashes and links for each sequence (see the
> sequence table in the main [README](../README.md#download-dataset)).

## Option 1: Web interface

Open the sequence link in a browser and download the files from the MARV
detail page.

## Option 2: `scripts/download_marv.py`

```bash
pip install -r requirements.txt

export MARV_USER=<user>
export MARV_PASSWORD=<password>     # or omit it to be prompted
python3 scripts/download_marv.py <dataset_hash> --output-dir data/bags
```

The script:

1. authenticates on the MARV API (`https://marv.mesocentre.uca.fr/marv/api/` by
   default; override with `--api` or `$MARV_API`),
2. looks up the dataset by its hash,
3. downloads every file of the dataset, keeping the bag folder layout:

```text
data/bags/
└── <bag_name>/
    ├── metadata.yaml
    └── <bag_name>_0.mcap
```

The `<bag_name>/` folder is a regular ROS 2 bag. Pass it straight to the
converter (`--bag data/bags/<bag_name>`), or inspect it with:

```bash
ros2 bag info data/bags/<bag_name>
```

Options:

| Option          | Description                                               |
|-----------------|-----------------------------------------------------------|
| `--output-dir`  | Target directory (default: current directory)             |
| `--api`         | MARV API base URL (default: `$MARV_API` or the public server) |
| `--user`        | MARV user name (default: `$MARV_USER`, else prompted)     |
| `--verbose`     | Print the dataset query and file list returned by MARV    |

Credentials are never stored in the script. Never commit them to the
repository.

> **TODO:** document the access model for the public release (public account,
> per-user registration, or anonymous access).
