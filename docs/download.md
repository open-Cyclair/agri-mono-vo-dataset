# Downloading the ROS 2 Bags

The sequences are distributed as ROS 2 bags (MCAP storage) on a MARV Robotics
server in Clermont-Ferrand. Each sequence is a MARV *dataset*, identified by
the hash at the end of its URL:

```text
https://marv.mesocentre.uca.fr/?filter=#/detail/<dataset_hash>
                                                 ^^^^^^^^^^^^^^
```

| Seq. | Bag name | Dataset hash | Chunks |
|------|----------|--------------|--------|
| 01 | `vo_dataset_2026-06-08_pressac_sugarbeet_bbch19+_fast_run001` | [`ivbpj7lvzzf4oszbl43fhao47a`](https://marv.mesocentre.uca.fr/?filter=#/detail/ivbpj7lvzzf4oszbl43fhao47a) | 37 |
| 02 | `vo_dataset_2026-06-08_pressac_maize_bbch14_nominal_run001` | [`is2pnph7qz525macitzcykw6je`](https://marv.mesocentre.uca.fr/?filter=#/detail/is2pnph7qz525macitzcykw6je) | 70 |
| 03 | `vo_dataset_2026-06-08_pressac_maize_bbch14_fast_run002` | [`iw2tkttilegoghmywjenwwvsmu`](https://marv.mesocentre.uca.fr/?filter=#/detail/iw2tkttilegoghmywjenwwvsmu) | 52 |
| 04 | `vo_dataset_2026-06-08_pressac_maize_bbch14-15_fast_run003` | [`vzqpk3ndgbgdx5sszpl4ewhgsy`](https://marv.mesocentre.uca.fr/?filter=#/detail/vzqpk3ndgbgdx5sszpl4ewhgsy) | 63 |
| 05 | `vo_dataset_2026-06-16_pressac_maize_bbch12_fast_run001` | [`si5ebrozsfxoz5xhnintmquaby`](https://marv.mesocentre.uca.fr/?filter=#/detail/si5ebrozsfxoz5xhnintmquaby) | 56 |
| 06 | `vo_dataset_2026-06-16_pressac_maize_bbch12_nominal_run001` | [`izyy3b2oecnfe4oorb5af6elya`](https://marv.mesocentre.uca.fr/?filter=#/detail/izyy3b2oecnfe4oorb5af6elya) | 56 |
| 07 | `vo_dataset_2026-06-16_pressac_maize_bbch18_fast_run001` | [`4nkxqx43a7typmhijr4zr6lk2a`](https://marv.mesocentre.uca.fr/?filter=#/detail/4nkxqx43a7typmhijr4zr6lk2a) | 31 |
| 08 | `vo_dataset_2026-06-16_pressac_sunflower_bbch10_fast_run001` | [`67l3q2z4w6cyklkmsmtencszd4`](https://marv.mesocentre.uca.fr/?filter=#/detail/67l3q2z4w6cyklkmsmtencszd4) | 48 |
| 09 | `vo_dataset_2026-07-08_pressac_maize_bbch19+_fast_run001` | [`7erefiw7itxiwscdwzol2xuwbi`](https://marv.mesocentre.uca.fr/?filter=#/detail/7erefiw7itxiwscdwzol2xuwbi) | 1 |
| 10 | `vo_dataset_2026-07-08_pressac_maize_bbch19+_fast_run002` | [`il5puddscg5ygv3n5anrfm6df4`](https://marv.mesocentre.uca.fr/?filter=#/detail/il5puddscg5ygv3n5anrfm6df4) | 45 |
| 11 | `vo_dataset_2026-07-08_pressac_sunflower_bbch19+_fast_run001` | [`2hic5cfyqc3jvdunnke6ohpqpa`](https://marv.mesocentre.uca.fr/?filter=#/detail/2hic5cfyqc3jvdunnke6ohpqpa) | 47 |

Each `.mcap` chunk holds about 1 GB. Sequence 09 is stored as a single 42 GB
file, so it can only be downloaded as a whole.

## Option 1: Web interface

Open the sequence link in a browser and download the files from the MARV
detail page. The same page shows the images, motion plots and trajectory of the
sequence, so you can inspect it before downloading.

## Option 2: `scripts/download_marv.py`

With the Docker image of this repository (see
[conversion.md](conversion.md#running-in-docker) to build it):

```bash
mkdir -p <data_dir>
export MARV_USER=<user>
export MARV_PASSWORD=<password>
docker run --rm --user "$(id -u):$(id -g)" \
  -e MARV_USER -e MARV_PASSWORD \
  -v <data_dir>:/data \
  gs-agrifield-tools download <dataset_hash> --output-dir /data
```

Omit `MARV_PASSWORD` and add `-it` to `docker run` to be prompted for it.

Without Docker, the script only needs Python 3 and `requests`, on Linux,
macOS or Windows:

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
<data_dir>/
└── <bag_name>/
    ├── metadata.yaml
    ├── <bag_name>_0.mcap
    ├── <bag_name>_1.mcap
    └── ...
```

The `<bag_name>/` folder is a regular ROS 2 bag that can be passed straight to
the converter.

Options:

| Option          | Description                                               |
|-----------------|-----------------------------------------------------------|
| `--output-dir`  | Target directory (default: current directory)             |
| `--api`         | MARV API base URL (default: `$MARV_API` or the public server) |
| `--user`        | MARV user name (default: `$MARV_USER`, else prompted)     |
| `--chunks`      | Only download these chunks, by index or range (e.g. `0-3 30`); `metadata.yaml` is always added |
| `--files`       | Only download the files matching these name patterns      |
| `--verbose`     | Print the dataset query and file list returned by MARV    |

## Downloading part of a sequence

A sequence can be downloaded whole, or only some of its chunks. For example,
the first four chunks and their metadata:

```bash
docker run --rm --user "$(id -u):$(id -g)" -e MARV_USER -e MARV_PASSWORD \
  -v <data_dir>:/data gs-agrifield-tools download <dataset_hash> \
  --output-dir /data --chunks 0-3
```

`--files` does the same with file name patterns, e.g.
`--files '*_45.mcap' metadata.yaml`.

The converter reads a partial bag folder as it is: it converts the chunks that
are present, in order, and lists the missing ones in `info.txt`. Keep chunk 0
if you need the camera extrinsics: `/tf_static` is recorded at the start of the
bag, so `nav_camera_to_imu_link.txt` is only found when chunk 0 is converted.
Chunks you download later can be added to the same folder.

Credentials are never stored in the script. Never commit them to the
repository.

## Access

Access is free. Fill in the [request form](https://marv-gate.cyclair.fr/) to
receive the MARV access link and credentials automatically by email, without
manual approval.
