"""HTTP transport independent of ROS, for easy testing."""
import json
from urllib.request import Request, urlopen
from urllib.parse import urlencode, urlsplit, urlunsplit
from urllib.error import HTTPError


def fetch_data(url, timeout_sec=60.0, max_response_bytes=52428800, filters=None):
    parsed = urlsplit(url)
    if parsed.scheme not in ('http', 'https') or not parsed.netloc:
        raise ValueError('server_url must be an absolute HTTP(S) URL')
    if timeout_sec <= 0 or max_response_bytes <= 0:
        raise ValueError('timeout_sec and max_response_bytes must be positive')
    if parsed.query or parsed.fragment:
        raise ValueError('server_url must have no query or fragment; use request filters')
    query = urlencode({key: value for key, value in (filters or {}).items()
                       if value != '' and value != 0})
    url = urlunsplit((parsed.scheme, parsed.netloc, parsed.path, query, ''))
    request = Request(url, headers={'Accept': 'application/json'})
    try:
        response_stream = urlopen(request, timeout=timeout_sec)
    except HTTPError as error:
        try:
            body = json.loads(error.read(8192).decode('utf-8'))
            detail = body.get('error', error.reason) if isinstance(body, dict) else error.reason
        except (ValueError, UnicodeError):
            detail = error.reason
        finally:
            error.close()
        raise ValueError(f'HTTP {error.code}: {detail}') from error
    with response_stream as response:
        if response.status != 200:
            raise ValueError(f'HTTP status {response.status}')
        if response.headers.get_content_type() != 'application/json':
            raise ValueError('server did not return application/json')
        payload = response.read(max_response_bytes + 1)
    if len(payload) > max_response_bytes:
        raise ValueError(f'response exceeds max_response_bytes ({max_response_bytes})')
    text = payload.decode('utf-8')
    document = json.loads(text)
    if not isinstance(document, dict) or document.get('schema_version') != 1:
        raise ValueError('unsupported factory-data JSON schema')
    if not isinstance(document.get('files'), list) or not isinstance(document.get('errors'), list):
        raise ValueError('factory-data JSON must contain files and errors arrays')
    for key in ('file_count', 'row_count', 'skipped_count'):
        if type(document.get(key)) is not int or document[key] < 0:
            raise ValueError(f'invalid {key}')
    if document['file_count'] != len(document['files']) or document['skipped_count'] != len(document['errors']):
        raise ValueError('inconsistent file counts')
    return text, document
