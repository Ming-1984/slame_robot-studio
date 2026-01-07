#!/usr/bin/env python3

import argparse
import functools
import http.server
import os


class NoCacheRequestHandler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, directory=None, **kwargs):
        super().__init__(*args, directory=directory, **kwargs)

    def end_headers(self):
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
        self.send_header("Pragma", "no-cache")
        self.send_header("Expires", "0")
        super().end_headers()


def main():
    parser = argparse.ArgumentParser(description="Robot Studio Web Interface Server (no-cache)")
    parser.add_argument("--bind", default="0.0.0.0", help="Bind address (default: 0.0.0.0)")
    parser.add_argument("--port", type=int, default=8080, help="Listen port (default: 8080)")
    parser.add_argument("--directory", default="web_interface", help="Serve directory (default: web_interface)")
    args = parser.parse_args()

    directory = os.path.abspath(args.directory)
    handler = functools.partial(NoCacheRequestHandler, directory=directory)
    server = http.server.ThreadingHTTPServer((args.bind, args.port), handler)
    server.serve_forever()


if __name__ == "__main__":
    main()

