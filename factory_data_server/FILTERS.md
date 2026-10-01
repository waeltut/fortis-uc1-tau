# Factory data filters — API v0.2.0

Every supplied filter is combined with **AND**. Omit filters to fetch everything.
Filtering uses metadata in filenames, not values or timestamps inside CSV rows.
One value per filter is supported; repeated keys, unknown filters, empty HTTP
values and invalid values return HTTP 400. No matches returns HTTP 200 with an
empty `files` array. The ROS service reports invalid filters with `success=false`.

| Filter | HTTP value | ROS field / unset value | Meaning |
| --- | --- | --- | --- |
| `worker` | `12` | string / `''` | Exact worker ID, without `W` |
| `station` | `5` | string / `''` | Exact station ID, without `Station_` |
| `day` | `1`–`31` | uint32 / `0` | Day of month |
| `month` | `1`–`12` | uint32 / `0` | Month of year; October = 10 |
| `year` | `1`–`9999` | uint32 / `0` | Calendar year |
| `shift` | `A`, `M`, `N` | string / `''` | Afternoon, morning, night |
| `date` | `2026-10-01` | string / `''` | Exact date in YYYY-MM-DD format |

IDs match exactly and are case-sensitive: station `5` does **not** match `05`.
Use `station=05` if that is the ID used in your filenames. Leading zeros are
preserved in IDs. For day/month/year, both `10` and `010` are interpreted numerically.
`day=1` matches the first of every month; `month=10` matches October of every year.
An exact `date` must agree with any supplied day/month/year. Impossible calendar
combinations are rejected; February 29 without a year can match leap years.
HTTP `month=0` is invalid; ROS `month: 0` means omit that filter.

## HTTP API

Start the server with `python3 server.py`. The endpoint is `GET /data`.
The host address below assumes the server is on the same machine.

October at station 5 (any year):

```bash
curl 'http://127.0.0.1:8000/data?month=10&station=5'
```

October 2026 at station 5:

```bash
curl 'http://127.0.0.1:8000/data?year=2026&month=10&station=5'
```

Worker 12 on 1 October 2026, morning shift:

```bash
curl 'http://127.0.0.1:8000/data?worker=12&date=2026-10-01&shift=M'
```

All six requested filters together:

```bash
curl 'http://127.0.0.1:8000/data?worker=12&day=1&month=10&year=2026&station=5&shift=M'
```

Python, using only the standard library:

```python
import json
from urllib.parse import urlencode
from urllib.request import urlopen

filters = {'month': 10, 'year': 2026, 'station': '5'}
url = 'http://127.0.0.1:8000/data?' + urlencode(filters)
with urlopen(url, timeout=60) as response:
    data = json.load(response)
print(data['file_count'], data['files'])
```

To add a filter, add its `key=value` to the URL separated by `&`, or add it to the
Python dictionary. Quote shell URLs containing `&`. `/health` accepts no filters.

## ROS 2 API

Keep `server_url` free of query parameters. Supply filters per service request:

```bash
ros2 launch factory_data_bridge factory_data.launch.py server_url:=http://127.0.0.1:8000/data
```

October at station 5:

```bash
ros2 service call /get_factory_data factory_data_interfaces/srv/GetFactoryData "{month: 10, station: '5'}"
```

October 2026 at station 5:

```bash
ros2 service call /get_factory_data factory_data_interfaces/srv/GetFactoryData "{year: 2026, month: 10, station: '5'}"
```

All six filters:

```bash
ros2 service call /get_factory_data factory_data_interfaces/srv/GetFactoryData "{worker: '12', day: 1, month: 10, year: 2026, station: '5', shift: 'M'}"
```

Exact date and worker:

```bash
ros2 service call /get_factory_data factory_data_interfaces/srv/GetFactoryData "{date: '2026-10-01', worker: '12'}"
```

No filters:

```bash
ros2 service call /get_factory_data factory_data_interfaces/srv/GetFactoryData '{}'
```

In a Python ROS client, populate the request before `call_async`:

```python
request = GetFactoryData.Request()
request.month = 10
request.station = '5'
future = client.call_async(request)
# Spin/wait for future completion, check response.success, then json.loads(response.json_data).
```

The included example client accepts the same filters:

```bash
python3 examples/request_data.py --month 10 --year 2026 --station 5
```

## Response and saved JSON

The existing fields and response interface remain. Three added JSON fields describe
filtering, for example:

```json
{
  "filters": {"month": 10, "station": "5"},
  "filtered_out_count": 7,
  "skipped_count": 1
}
```

- `filters`: validated filters applied to this response; `{}` if none.
- `filtered_out_count`: correctly named files excluded by the filters; not errors.
- `skipped_count` / `errors`: bad filenames anywhere in the tree, plus CSV/read
  errors in matching files. All filenames are checked, but excluded CSV contents
  are not opened or validated.
- `file_count` / `row_count`: only successfully read matching files and rows.

Every successful request replaces `output/factory_data.json` with **that request's
filtered result**. It is the last successful snapshot, not always the full dataset.
Invalid requests leave the saved snapshot unchanged. GET `/data` without filters
rebuilds the full snapshot. No caching or pagination is used.

## Updating an existing installation

1. Stop the old server and ROS service node.
2. Replace `server.py`; keep your existing `data/` directory and CSVs. Restart the server.
3. Replace both ROS package directories with the updated versions. Do not keep a
   second copy of packages with the same names elsewhere under `src`.
4. In a fresh terminal, source Humble, rebuild both packages and source the workspace:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to factory_data_bridge
source install/setup.bash
ros2 interface show factory_data_interfaces/srv/GetFactoryData
```

The service request definition changed. Rebuild/restart all ROS clients that use
this interface, including C++ clients; old generated interfaces are not compatible.
The request now contains worker, day, month, year, station, shift and date. The
response remains success, message and json_data. Empty `{}` requests still fetch all.
