# Factory data server

**Stackable filtering:** see [FILTERS.md](FILTERS.md) for HTTP and ROS examples,
filter rules, response fields and upgrade instructions.

Python 3.10+; no pip dependencies. Start from the extracted directory:

```bash
cd factory_data_server
python3 server.py
```

Put CSVs in `data/` or any subfolder. Every `GET /data` recursively scans filenames and reads matching CSVs,
returns one JSON document and atomically replaces `output/factory_data.json`.
There is no cached response. `GET /health` checks server availability without
reading CSVs. No ROS installation is needed on this machine.

```bash
curl http://127.0.0.1:8000/data
```

## Filename rules

`YYYY-MM-DD_Station_<station_id>_W<worker_id>_<shift>.csv`

Example: `2026-09-30_Station_03_W0012_M.csv`

- Dates must be real calendar dates, with two-digit months and days.
- IDs must contain one or more ASCII letters, digits or hyphens. They remain strings.
- Shift is exactly `A` (afternoon), `M` (morning) or `N` (night).
- `Station`, `W` and the lowercase `.csv` extension are case-sensitive.
- `.CSV` files are found but reported as incorrectly named and skipped.
- Duplicate basenames in different folders are retained with distinct `relative_path` values.
- Non-CSV files are ignored. Symlink directories are not traversed; symlink CSVs are skipped.

Invalid names, dates, encodings, duplicate/empty column names, or malformed rows
are logged as ERROR and the entire faulty file is skipped. Other files continue.
Directory scan failures fail the request with HTTP 500 rather than silently omitting
a directory. An empty folder or a folder with only invalid files returns HTTP 200
with an empty `files` array and the appropriate error counts.

## CSV interpretation

The first row is a header. Cell values are preserved as strings, including
leading zeros, decimal commas and empty fields. Each row becomes a JSON object.
No numerical conversion or factory-specific interpretation is assumed.
Completely blank lines are ignored. Header-only files are valid with zero rows.
Default encoding is UTF-8, with or without a BOM.

Delimiter detection tries comma, semicolon, tab and pipe independently per file.
Detection is heuristic; use an explicit delimiter if detection is ambiguous:

```bash
python3 server.py --delimiter semicolon
python3 server.py --data-dir /path/to/data --output /path/to/factory_data.json --encoding cp1252
python3 server.py --host 0.0.0.0 --port 8000
```

Data and output default paths are relative to `server.py`, not the shell directory.
Command-line relative paths are relative to the current shell directory.
The output filename must end in `.json`.

## JSON schema

```json
{
  "schema_version": 1,
  "generated_at": "2026-09-30T06:40:00+00:00",
  "file_count": 1,
  "row_count": 1,
  "skipped_count": 0,
  "files": [{
    "filename": "2026-09-30_Station_03_W0012_M.csv",
    "relative_path": "line1/2026-09-30_Station_03_W0012_M.csv",
    "date": "2026-09-30",
    "station_id": "03",
    "worker_id": "0012",
    "shift": "M",
    "shift_name": "morning",
    "columns": ["part", "quantity"],
    "row_count": 1,
    "rows": [{"part": "001", "quantity": "12"}]
  }],
  "errors": []
}
```

Each error has `relative_path` and `message`. `generated_at` is UTC; the filename
date is preserved without inferring shift times or a time zone.

## Operation

The server listens on all interfaces by default so a remote ROS machine can
connect. This simple HTTP server has no authentication or TLS and is intended for
a trusted internal network. Use `--host 127.0.0.1` for same-machine-only access.

Requests are processed serially. The complete aggregate is held in RAM; choose a
larger ROS `timeout_sec` and `max_response_bytes` if needed. These packages target
modest datasets rather than streaming very large factory archives.

Write incoming CSVs to temporary non-CSV filenames and rename them into place
when complete. A scan is not a transaction across the entire directory; avoid
editing the dataset during collection when you need a consistent snapshot.
A detected change during an individual file read causes that file to be skipped.

## Tests

```bash
python3 -m unittest discover -s tests -v
```
