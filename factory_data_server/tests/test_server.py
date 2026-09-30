import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from server import aggregate, metadata, read_csv, save_json


class ServerTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)

    def tearDown(self):
        self.temp.cleanup()

    def write(self, name, content):
        path = self.root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content, encoding='utf-8')
        return path

    def test_recursive_and_invalid(self):
        self.write('2026-09-30_Station_03_W0012_M.csv', 'part,quantity\n001,12\n')
        self.write('a/b/2026-09-30_Station_03_W0012_M.csv', 'part;value\n002;"1,5"\n')
        self.write('bad.csv', 'x\n1\n')
        self.write('2026-02-30_Station_1_W2_N.csv', 'x\n1\n')
        self.write('2026-09-30_Station_1_W2_A.CSV', 'x\n1\n')
        self.write('note.txt', 'ignored')
        with self.assertLogs('factory_data', level='ERROR') as logs:
            result = aggregate(self.root)
        self.assertEqual((result['file_count'], result['row_count'], result['skipped_count']), (2, 2, 3))
        self.assertEqual(len(logs.output), 3)
        self.assertEqual(result['files'][0]['worker_id'], '0012')
        self.assertEqual(result['files'][0]['rows'][0]['part'], '001')
        self.assertEqual(result['files'][1]['rows'][0]['value'], '1,5')

    def test_bad_csv_skips_entire_file(self):
        self.write('2026-09-30_Station_1_W2_N.csv', 'x,x\n1,2\n')
        self.write('2026-09-30_Station_2_W2_N.csv', 'x,y\n1,2\n3\n')
        with self.assertLogs('factory_data', level='ERROR'):
            result = aggregate(self.root, delimiter=',')
        self.assertEqual(result['file_count'], 0)
        self.assertEqual(result['skipped_count'], 2)

    def test_bom_quotes_multiline_and_blank(self):
        path = self.write('x.csv', '\ufeffpart,description\n001,"first\nsecond"\n\n002,"a,b"\n003,\n')
        columns, rows = read_csv(path, ',')
        self.assertEqual(columns, ['part', 'description'])
        self.assertEqual(rows[0]['description'], 'first\nsecond')
        self.assertEqual(rows[1]['description'], 'a,b')
        self.assertEqual(rows[2]['description'], '')

    def test_empty_header_only_single_column(self):
        self.assertEqual(aggregate(self.root)['files'], [])
        path = self.write('x.csv', 'quantity\n12\n')
        self.assertEqual(read_csv(path)[1], [{'quantity': '12'}])
        path.write_text('quantity\n')
        self.assertEqual(read_csv(path)[1], [])
        path.write_text('')
        with self.assertRaises(ValueError):
            read_csv(path)

    def test_calendar_and_shift(self):
        self.assertEqual(metadata('2024-02-29_Station_A-1_W0001_N.csv')['shift_name'], 'night')
        for filename in ['2025-02-29_Station_1_W2_M.csv', '2026-9-30_Station_1_W2_M.csv',
                         '2026-09-30_Station_1_W2_X.csv', '2026-09-30_Station__W2_M.csv']:
            with self.assertRaises(ValueError):
                metadata(filename)

    def test_snapshot_replace(self):
        output = self.root / 'output' / 'factory_data.json'
        save_json(output, b'{"a":1}')
        save_json(output, b'{"a":2}')
        self.assertEqual(json.loads(output.read_bytes()), {'a': 2})
        self.assertEqual(list(output.parent.iterdir()), [output])


if __name__ == '__main__':
    unittest.main()
