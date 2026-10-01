from functools import partial
from http.server import HTTPServer
import json
from pathlib import Path
import sys
from threading import Thread
from urllib.error import HTTPError
from urllib.request import urlopen
import tempfile
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from server import aggregate, Handler, parse_filters


class FilterTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        for folder, filename in [
            ('', '2026-10-01_Station_5_W12_M.csv'),
            ('nested/deeper', '2025-10-02_Station_5_W13_N.csv'),
            ('', '2026-09-01_Station_5_W12_A.csv'),
            ('', '2026-10-01_Station_6_W12_M.csv'),
            ('', '2026-10-01_Station_05_W0012_M.csv')]:
            self.write(str(Path(folder) / filename), 'part,quantity\n001,12\n')

    def tearDown(self):
        self.temp.cleanup()

    def write(self, name, content):
        path = self.root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content, encoding='utf-8')
        return path

    def test_individual_and_stacked_filters(self):
        for query, count in [('worker=12', 3), ('day=1', 4), ('month=10', 4),
                             ('year=2026', 4), ('station=5', 3), ('shift=M', 3),
                             ('month=10&station=5', 2),
                             ('month=10&station=5&year=2026', 1),
                             ('worker=12&day=1&month=10&year=2026&station=5&shift=M', 1),
                             ('date=2026-10-01', 3), ('station=05&worker=0012', 1),
                             ('month=12', 0), ('month=010', 4), ('', 5)]:
            with self.subTest(query=query):
                result = aggregate(self.root, filters=parse_filters(query))
                self.assertEqual(result['file_count'], count)
                self.assertEqual(result['filtered_out_count'], 5 - count)
                self.assertEqual(result['skipped_count'], 0)

    def test_invalid_filters(self):
        for query in ['month=0', 'month=13', 'day=32', 'year=10000', 'year=-1',
                      'month=ten', 'worker=', 'station=', 'shift=morning',
                      'month=10&month=11', 'unknown=x', 'date=2026-02-29',
                      'date=2026-10-01&month=11', 'month=2&day=30',
                      'month=2&day=29&year=2025', 'month', 'date=2026-1-01']:
            with self.subTest(query=query), self.assertRaises(ValueError):
                parse_filters(query)
        self.assertEqual(parse_filters('month=2&day=29'), {'month': 2, 'day': 29})

    def test_matching_read_errors_and_bad_names(self):
        self.write('bad.csv', 'x\n1\n')
        self.write('2026-10-01_Station_7_W12_M.csv', 'x,x\n1,2\n')
        self.write('2026-09-01_Station_7_W12_M.csv', 'x,x\n1,2\n')
        with self.assertLogs('factory_data', level='ERROR'):
            result = aggregate(self.root, filters=parse_filters('month=10'))
        self.assertEqual(result['skipped_count'], 2)
        self.assertEqual(result['filtered_out_count'], 2)

    def test_http_and_bridge(self):
        sys.path.insert(0, str(Path(__file__).resolve().parents[2] /
                              'factory_data_ros2/factory_data_bridge'))
        from factory_data_bridge.http_client import fetch_data
        snapshot = self.root / 'snapshot.json'
        handler = partial(Handler, data_dir=self.root, output=snapshot,
                          delimiter='auto', encoding='utf-8-sig')
        server = HTTPServer(('127.0.0.1', 0), handler)
        worker = Thread(target=server.serve_forever, daemon=True)
        worker.start()
        url = f'http://127.0.0.1:{server.server_port}/data'
        try:
            text, result = fetch_data(url, filters={'month': 10, 'station': '5', 'year': 0, 'worker': ''})
            self.assertEqual(result['file_count'], 2)
            self.assertEqual(result['filters'], {'month': 10, 'station': '5'})
            self.assertEqual(snapshot.read_text(), text)
            with self.assertRaisesRegex(ValueError, 'HTTP 400.*month'):
                fetch_data(url, filters={'month': 13})
            self.assertEqual(snapshot.read_text(), text)
            with self.assertRaises(HTTPError) as caught:
                urlopen(url + '?unknown=1')
            self.assertEqual(caught.exception.code, 400)
            caught.exception.close()
            self.assertEqual(fetch_data(url)[1]['file_count'], 5)
            self.assertEqual(fetch_data(url, filters={'month': 12})[1]['file_count'], 0)
        finally:
            server.shutdown()
            server.server_close()
            worker.join()
