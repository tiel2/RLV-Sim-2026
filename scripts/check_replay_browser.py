"""Check real WebGL geometry against replay records (optional Playwright dependency).

Run after generating the ascent, single-engine landing and fuel-depletion replays.
Use --browser /path/to/chrome for a system browser, or install Playwright Chromium.
"""

import argparse
import functools
import http.server
import json
import threading
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--browser")
    parser.add_argument("--output", type=Path, default=ROOT / "results/visual_checks/ascent")
    parser.add_argument("--ascent", default="results/ascent_physics_replay/replay.json")
    parser.add_argument("--landing", default="results/replay_landing_regression/replay.json")
    parser.add_argument("--depletion", default="results/replay_fuel_depletion/replay.json")
    args = parser.parse_args()
    from playwright.sync_api import sync_playwright

    args.output.mkdir(parents=True, exist_ok=True)

    class QuietHandler(http.server.SimpleHTTPRequestHandler):
        def log_message(self, *_):
            pass

    server = http.server.ThreadingHTTPServer(
        ("127.0.0.1", 0), functools.partial(QuietHandler, directory=str(ROOT))
    )
    threading.Thread(target=server.serve_forever, daemon=True).start()
    reports = []
    try:
        with sync_playwright() as pw:
            browser = pw.chromium.launch(
                executable_path=args.browser,
                headless=True,
                args=["--no-sandbox", "--use-angle=swiftshader", "--enable-unsafe-swiftshader"],
            )
            for name, filename in (
                ("ascent", args.ascent),
                ("landing", args.landing),
                ("depletion", args.depletion),
            ):
                path = (ROOT / filename).resolve()
                replay = json.loads(path.read_text())
                page = browser.new_page(
                    viewport={"width": 1440, "height": 1000}, ignore_https_errors=True
                )
                errors = []
                page.on("pageerror", lambda error, errors=errors: errors.append(str(error)))
                url = f"http://127.0.0.1:{server.server_port}/viz/index.html"
                page.goto(f"{url}?inspect&replay=/{path.relative_to(ROOT).as_posix()}")
                page.wait_for_function("window.replayInspection !== undefined", timeout=30000)
                if name == "landing":
                    page.wait_for_function(
                        "window.replayInspection.rocket.hasGlbLegs()", timeout=30000
                    )
                # Includes backwards scrubbing across cutoff and gimbal commands.
                epochs = [0, 11, 147, 11] if name == "ascent" else [replay["frames"][-1]["t"]]
                for epoch in epochs:
                    frame = min(replay["frames"], key=lambda f: abs(f["t"] - epoch))
                    check = page.evaluate(
                        """async ({frame, vehicle, site}) => {
                        const T = await import('three');
                        const d = window.replayInspection;
                        d.seek(frame.t); d.scene.updateMatrixWorld(true);
                        const r = d.rocket, issues = [];
                        const same = (a, b) => a.distanceTo(b) < 1e-7;
                        const cg = r.inner.localToWorld(new T.Vector3(frame.xcg, 0, 0));
                        if (!same(cg, new T.Vector3(...frame.p))) issues.push('CG world transform');
                        if (r.engines.length !== vehicle.n_engines) issues.push('engine count');
                        r.engines.forEach((e, i) => {
                            if (!same(e.gim.position, new T.Vector3(...vehicle.engine_pos[i])))
                                issues.push('engine position ' + i);
                            const force = new T.Vector3(...frame.engine_force_body[i]);
                            if (force.length() > 1e-9) {
                                const direction = new T.Vector3(1, 0, 0)
                                    .applyQuaternion(e.gim.quaternion);
                                if (!same(direction, force.normalize()))
                                    issues.push('thrust axis ' + i);
                            } else if (e.plume.visible) issues.push('zero-thrust plume ' + i);
                        });
                        if (Boolean(r.inner.getObjectByName('attached-upper-stack')) !==
                            Boolean(vehicle.attached_component)) issues.push('attached stack');
                        r.finFolds.forEach(f => {
                            const expected = frame.fin_deployment ? 0 : Math.PI / 2;
                            if (Math.abs(f.rotation.z - expected) > 1e-9) issues.push('fin state');
                        });
                        if (site.kind === 'land' && !d.scene.getObjectByName('launch-pad'))
                            issues.push('launch site');
                        if (site.kind === 'land' && frame.t === 0) {
                            const engine = r.engines[0].gim.getWorldPosition(new T.Vector3());
                            if (Math.abs(engine.y - site.ground_y) > 1e-7)
                                issues.push('pad height');
                        }
                        return {issues, engines: r.engines.length, time_s: frame.t,
                                plumes: r.engines.filter(e => e.plume.visible).length};
                    }""",
                        {
                            "frame": frame,
                            "vehicle": replay["meta"]["vehicle"],
                            "site": replay["meta"]["scene"],
                        },
                    )
                    reports.append({"case": name, **check})
                    page.screenshot(path=str(args.output / f"{name}_{epoch:g}s.png"))
                    assert not check["issues"], check
                assert not errors, errors
                page.close()
            browser.close()
    finally:
        server.shutdown()
        server.server_close()
    (args.output / "browser_report.json").write_text(json.dumps(reports, indent=2) + "\n")
    print(f"Passed {len(reports)} WebGL state/geometry checks; screenshots: {args.output}")


if __name__ == "__main__":
    main()
