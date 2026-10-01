#!/usr/bin/env python3
"""On-demand, recursive CSV aggregation. Python 3.10+, standard library only."""
import argparse
import csv
from datetime import date, datetime, timezone
from functools import partial
from http.server import BaseHTTPRequestHandler, HTTPServer
import io
import json
import logging
import os
from pathlib import Path
import re
import tempfile
from urllib.parse import parse_qsl, urlsplit

LOG = logging.getLogger('factory_data')
NAME = re.compile(r'(?P<date>[0-9]{4}-[0-9]{2}-[0-9]{2})_Station_(?P<station_id>[A-Za-z0-9-]+)_W(?P<worker_id>[A-Za-z0-9-]+)_(?P<shift>[AMN])\.csv')
SHIFTS = {'A': 'afternoon', 'M': 'morning', 'N': 'night'}


def metadata(filename):
    match = NAME.fullmatch(filename)
    if not match:
        raise ValueError('expected YYYY-MM-DD_Station_<id>_W<id>_<A|M|N>.csv; IDs: letters, digits or hyphens')
    result = match.groupdict()
    date.fromisoformat(result['date'])  # Reject impossible dates, e.g. 2026-02-30.
    result['shift_name'] = SHIFTS[result['shift']]
    return result


FILTER_KEYS = ('worker', 'day', 'month', 'year', 'station', 'shift', 'date')


def parse_filters(query):
    """Parse and validate public query parameters. Every supplied filter uses AND."""

    if not query:
        return {}

    pairs = parse_qsl(
        query,
        keep_blank_values=True,
        strict_parsing=True,
        max_num_fields=20,
    )
    
    filters = {}
    for key, value in pairs:
        if key not in FILTER_KEYS:
            raise ValueError(f'unknown filter {key!r}; allowed: {", ".join(FILTER_KEYS)}')
        if key in filters:
            raise ValueError(f'duplicate filter {key!r}; use one value per filter')
        if not value:
            raise ValueError(f'filter {key!r} must not be empty; omit unused filters')
        if key in ('day', 'month', 'year'):
            maximum = {'day': 31, 'month': 12, 'year': 9999}[key]
            if not re.fullmatch(r'[0-9]{1,4}', value) or not 1 <= int(value) <= maximum:
                raise ValueError(f'{key} must be an integer from 1 to {maximum}')
            value = int(value)
        elif key in ('worker', 'station'):
            if not re.fullmatch(r'[A-Za-z0-9-]+', value):
                raise ValueError(f'{key} must contain only letters, digits or hyphens')
        elif key == 'shift':
            if value not in SHIFTS:
                raise ValueError('shift must be A, M or N')
        elif key == 'date':
            if not re.fullmatch(r'[0-9]{4}-[0-9]{2}-[0-9]{2}', value):
                raise ValueError('date must be YYYY-MM-DD')
            date.fromisoformat(value)
        filters[key] = value
    if 'date' in filters:
        exact = date.fromisoformat(filters['date'])
        for key in ('year', 'month', 'day'):
            if key in filters and filters[key] != getattr(exact, key):
                raise ValueError(f'{key} conflicts with date')
    if 'month' in filters and 'day' in filters:
        # Use a leap year if year is unspecified: February 29 can match leap years.
        date(filters.get('year', 2000), filters['month'], filters['day'])
    return filters


def matches_filters(info, filters):
    file_date = date.fromisoformat(info['date'])
    for key, value in filters.items():
        if key in ('year', 'month', 'day'):
            actual = getattr(file_date, key)
        else:
            actual = info[{'worker': 'worker_id', 'station': 'station_id'}.get(key, key)]
        if actual != value:
            return False
    return True


def read_csv(path, delimiter='auto', encoding='utf-8-sig'):
    before = path.stat()
    text = path.read_text(encoding=encoding)
    after = path.stat()
    if (before.st_size, before.st_mtime_ns) != (after.st_size, after.st_mtime_ns):
        raise ValueError('file changed while reading; retry after writing finishes')
    if '\x00' in text:
        raise ValueError('CSV contains NUL bytes; check encoding')
    if not text.strip():
        raise ValueError('empty CSV: header is required')
    if delimiter == 'auto':
        try:
            delimiter = csv.Sniffer().sniff(text[:65536], delimiters=',;\t|').delimiter
        except csv.Error:
            # An unambiguous single-column CSV is valid. Otherwise fail explicitly.
            if any(c in text for c in ',;\t|'):
                raise ValueError('cannot detect delimiter; supply --delimiter comma/semicolon/tab/pipe')
            delimiter = ','
    reader = csv.reader(io.StringIO(text, newline=''), delimiter=delimiter, strict=True)
    columns = next(reader)
    if not columns or any(not name.strip() for name in columns):
        raise ValueError('all header names must be non-empty')
    if len(set(columns)) != len(columns):
        raise ValueError('duplicate header names')
    rows = []
    for row in reader:
        if not row:  # Ignore completely blank lines, preserve rows of empty fields.
            continue
        if len(row) != len(columns):
            raise ValueError(f'CSV line {reader.line_num}: expected {len(columns)} fields, got {len(row)}')
        rows.append(dict(zip(columns, row)))
    return columns, rows


def aggregate(data_dir, delimiter='auto', encoding='utf-8-sig', filters=None):
    filters = {} if filters is None else filters
    filtered_out_count = 0
    root = Path(data_dir).resolve()
    if not root.is_dir():
        raise OSError(f'data directory does not exist: {root}')
    files, errors = [], []
    candidates = []
    def walk_error(error):
        raise error  # Do not silently serve incomplete data after a directory scan failure.
    for folder, dirs, names in os.walk(root, followlinks=False, onerror=walk_error):
        dirs[:] = sorted(d for d in dirs if not (Path(folder) / d).is_symlink())
        for name in sorted(names):
            if name.lower().endswith('.csv'):
                candidates.append(Path(folder) / name)
    for path in sorted(candidates):
        relative = path.relative_to(root).as_posix()
        try:
            if path.is_symlink():
                raise ValueError('symbolic-link CSV files are not supported')
            info = metadata(path.name)
            if not matches_filters(info, filters):
                filtered_out_count += 1
                continue
            columns, rows = read_csv(path, delimiter, encoding)
            files.append({'filename': path.name, 'relative_path': relative, **info,
                          'columns': columns, 'row_count': len(rows), 'rows': rows})
        except (ValueError, OSError, UnicodeError, csv.Error) as error:
            message = str(error)
            LOG.error('Skipping %s: %s', relative, message)
            errors.append({'relative_path': relative, 'message': message})
    result = {'schema_version': 1, 'generated_at': datetime.now(timezone.utc).isoformat(),
              'file_count': len(files), 'row_count': sum(f['row_count'] for f in files),
              'skipped_count': len(errors), 'filtered_out_count': filtered_out_count,
              'filters': dict(filters), 'files': files, 'errors': errors}
    LOG.info('Loaded %d CSVs, %d rows; skipped %d files', result['file_count'], result['row_count'], len(errors))
    return result


def save_json(path, payload):
    """Replace the last snapshot atomically; failed requests leave it unchanged."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(dir=path.parent, delete=False) as stream:
            temporary = Path(stream.name)
            stream.write(payload)
        os.replace(temporary, path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


class Handler(BaseHTTPRequestHandler):
    def __init__(self, *args, data_dir, output, delimiter, encoding, **kwargs):
        self.data_dir, self.output = data_dir, output
        self.delimiter, self.encoding = delimiter, encoding
        super().__init__(*args, **kwargs)

    def do_GET(self):
        route = urlsplit(self.path)
        if route.path not in ('/health', '/data'):
            self.reply(404, {'error': 'Use GET /data or GET /health'})
            return
        if route.path == '/health':
            if route.query:
                self.reply(400, {'error': '/health does not accept query parameters'})
                return
            self.reply(200, {'status': 'ok'})
            return
        try:
            filters = parse_filters(route.query)
        except ValueError as error:
            self.reply(400, {'error': str(error)})
            return
        try:
            document = aggregate(self.data_dir, self.delimiter, self.encoding, filters)
            payload = json.dumps(document, ensure_ascii=False, indent=2).encode('utf-8')
            save_json(self.output, payload)
        except Exception:
            LOG.exception('Could not generate factory data')
            self.reply(500, {'error': 'Could not generate factory data; see server console'})
            return
        self.reply(200, payload)

    def reply(self, status, data):
        payload = data if isinstance(data, bytes) else json.dumps(data).encode('utf-8')
        self.send_response(status)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(payload)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        try:
            self.wfile.write(payload)
        except (BrokenPipeError, ConnectionResetError):
            LOG.warning('Client disconnected before response finished')

    def log_message(self, fmt, *args):
        LOG.info('%s - %s', self.client_address[0], fmt % args)


def main():
    here = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=8000)
    parser.add_argument('--data-dir', type=Path, default=here / 'data')
    parser.add_argument('--output', type=Path, default=here / 'output' / 'factory_data.json')
    parser.add_argument('--delimiter', choices=['auto', 'comma', 'semicolon', 'tab', 'pipe'], default='auto')
    parser.add_argument('--encoding', default='utf-8-sig')
    args = parser.parse_args()
    if not args.data_dir.is_dir():
        parser.error(f'data directory not found: {args.data_dir}')
    if args.output.suffix.lower() != '.json':
        parser.error('--output must end in .json')
    delimiter = {'auto': 'auto', 'comma': ',', 'semicolon': ';', 'tab': '\t', 'pipe': '|'}[args.delimiter]
    logging.basicConfig(level=logging.INFO, format='%(asctime)s %(levelname)s %(message)s')
    handler = partial(Handler, data_dir=args.data_dir, output=args.output, delimiter=delimiter, encoding=args.encoding)
    # Serialize snapshots so concurrent requests cannot race on the saved JSON.
    with HTTPServer((args.host, args.port), handler) as server:
        LOG.info('Listening on %s:%d; data=%s; output=%s', args.host, args.port,
                 args.data_dir.resolve(), args.output.resolve())
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            LOG.info('Server stopped')


if __name__ == '__main__':
    main()
