"""Inject simulation results into the dashboard template.

Produces
  demo/twin_dashboard.html  page body (what is published as the web artifact)
  demo/index.html           standalone page - open it in any browser, works offline
"""
import pathlib

ROOT = pathlib.Path(__file__).resolve().parents[1]
DEMO = ROOT / "demo"
DATA = ROOT / "results" / "twin" / "dashboard_data.json"


def main():
    body = (DEMO / "dashboard_template.html").read_text()
    body = body.replace("/*__DATA__*/null", DATA.read_text().strip())
    (DEMO / "twin_dashboard.html").write_text(body)
    page = ("<!doctype html>\n<html lang=\"en\">\n<head>\n<meta charset=\"utf-8\">\n"
            "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1, viewport-fit=cover\">\n"
            "<style>body{margin:0}</style>\n</head>\n<body>\n" + body + "\n</body>\n</html>\n")
    (DEMO / "index.html").write_text(page)
    print(f"Wrote {DEMO / 'index.html'} ({len(page) / 1e3:.0f} kB)")


if __name__ == "__main__":
    main()
