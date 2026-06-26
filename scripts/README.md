# Scripts

This directory contains utility scripts released together with Elevator-LIO.

## bag_runner_ui.py

`bag_runner_ui.py` is a rosbag manager released together with Elevator-LIO. It is designed to help organize rosbag sequences, run batch tests, manage event flags, and review generated results during algorithm development and evaluation.

Usage video:

[Bilibili: Elevator-LIO rosbag manager usage](https://www.bilibili.com/video/BV1n3jt64Eoi/?share_source=copy_web&vd_source=392db04838f1edf7d12e58a3d68775d8)

Run:

```bash
python3 scripts/bag_runner_ui.py
```

## download_dataset.sh

`download_dataset.sh` downloads the Elevator-LIO dataset from Hugging Face. By default, it downloads the main dataset to a sibling directory:

```text
../Elevator-LIO-Dataset
```

Basic usage:

```bash
./scripts/download_dataset.sh
```

Specify an output directory:

```bash
./scripts/download_dataset.sh --output-dir /data/Elevator-LIO-Dataset
```

Also download community-contributed bags:

```bash
./scripts/download_dataset.sh --include-community
```

Show the SJTU cloud mirror link:

```bash
./scripts/download_dataset.sh --sjtu
```

The script uses `huggingface_hub` and resumes interrupted downloads when rerun with the same options.
