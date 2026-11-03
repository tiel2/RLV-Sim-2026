"""Compare consecutive WebGL background pixels around T+05:36.

Unlike geometry assertions, this detects visible horizon flicker. --legacy-control
also measures the former two-triangle terrain in the same browser and camera path.
Uses Playwright/Chrome, with the same optional dependencies as check_replay_browser.py.
"""

import argparse
import functools
import http.server
import json
import threading
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def sample(page, mode, legacy):
    return page.evaluate(
        """async ({mode, legacy}) => {
        const T = await import('three'), d = window.replayInspection;
        const terrain = d.scene.getObjectByName('land-surface');
        if (!window.recordedTerrain) window.recordedTerrain = terrain.geometry;
        terrain.geometry = legacy
            ? new T.PlaneGeometry(20000000, 20000000) : window.recordedTerrain;
        d.seek(335); d.setCamera(mode);
        const canvas = document.querySelector('canvas');
        const gl = canvas.getContext('webgl2') || canvas.getContext('webgl');
        // Right-hand background, outside the booster, engine HUD and transport band.
        const x = Math.round(canvas.width * 0.78), y = Math.round(canvas.height * 0.27);
        const width = Math.round(canvas.width * 0.18);
        const height = Math.round(canvas.height * 0.45), pixels = width * height;
        let previous = null, worstFraction = 0, worstTime = 335, maxMeanDelta = 0;
        let minMeanIntensity = 255;
        const frames = [];
        for (let k = 0; k <= 180; k++) {
            const t = 335 + k / 60;
            d.advance(t);
            const current = new Uint8Array(pixels * 4);
            // Read immediately after rendering, before the browser clears its backbuffer.
            gl.readPixels(x, y, width, height, gl.RGBA, gl.UNSIGNED_BYTE, current);
            if (gl.getError() !== gl.NO_ERROR) throw Error('WebGL readback failed');
            let changed = 0, sum = 0, intensity = 0;
            for (let i = 0; i < current.length; i += 4) {
                intensity += current[i] + current[i+1] + current[i+2];
                if (previous) {
                    const r = Math.abs(current[i] - previous[i]);
                    const g = Math.abs(current[i+1] - previous[i+1]);
                    const b = Math.abs(current[i+2] - previous[i+2]);
                    if (Math.max(r,g,b) > 25) changed++;
                    sum += r+g+b;
                }
            }
            minMeanIntensity = Math.min(minMeanIntensity, intensity / (3 * pixels));
            if (previous) {
                const fraction = changed / pixels, meanDelta = sum / (3 * pixels);
                frames.push({t, changed_fraction:fraction, mean_rgb_delta:meanDelta});
                if (fraction > worstFraction) { worstFraction = fraction; worstTime = t; }
                maxMeanDelta = Math.max(maxMeanDelta, meanDelta);
            }
            previous = current;
        }
        if (legacy) terrain.geometry.dispose();
        terrain.geometry = window.recordedTerrain;
        d.advance(336.73333333333335);
        return {mode, legacy, samples:181, start_s:335, end_s:338, sample_hz:60,
            worst_changed_fraction:worstFraction, worst_time_s:worstTime,
            maximum_mean_rgb_delta:maxMeanDelta, minimum_mean_intensity:minMeanIntensity,
            region:{x,y,width,height}, frames};
        }""",
        {"mode": mode, "legacy": legacy},
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--browser", default="/opt/google/chrome/chrome")
    parser.add_argument("--replay", default="results/ascent_boostback/replay.json")
    parser.add_argument("--legacy-control", action="store_true")
    parser.add_argument(
        "--output", type=Path, default=ROOT / "results/visual_checks/background_temporal"
    )
    args = parser.parse_args()
    from playwright.sync_api import sync_playwright

    class QuietHandler(http.server.SimpleHTTPRequestHandler):
        def log_message(self, *_):
            pass

    args.output.mkdir(parents=True, exist_ok=True)
    server = http.server.ThreadingHTTPServer(
        ("127.0.0.1", 0), functools.partial(QuietHandler, directory=str(ROOT))
    )
    threading.Thread(target=server.serve_forever, daemon=True).start()
    reports, errors = [], []
    try:
        with sync_playwright() as pw:
            browser = pw.chromium.launch(
                executable_path=args.browser,
                headless=True,
                args=["--no-sandbox", "--use-angle=swiftshader", "--enable-unsafe-swiftshader"],
            )
            page = browser.new_page(
                viewport={"width": 1200, "height": 850}, ignore_https_errors=True
            )
            page.on("pageerror", lambda error: errors.append(str(error)))
            url = f"http://127.0.0.1:{server.server_port}/viz/index.html"
            page.goto(f"{url}?inspect&replay=/{args.replay}")
            page.wait_for_function("window.replayInspection !== undefined", timeout=30000)
            for legacy in [True, False] if args.legacy_control else [False]:
                for mode in ("orbit", "chase"):
                    result = sample(page, mode, legacy)
                    result["limit_changed_fraction"] = 0.01
                    result["passed"] = (
                        result["worst_changed_fraction"] <= 0.01
                        and result["minimum_mean_intensity"] > 1
                    )
                    reports.append(result)
                    print(
                        json.dumps({k: v for k, v in result.items() if k != "frames"}),
                        flush=True,
                    )
                    if not legacy:
                        page.screenshot(path=str(args.output / f"{mode}_336.73s.png"))
            browser.close()
    finally:
        server.shutdown()
        server.server_close()
    report = {
        "status": "passed" if all(r["passed"] for r in reports if not r["legacy"]) else "failed",
        "scope": "181 consecutive rendered frames per camera, 335–338 s; RGB delta >25/255",
        "legacy_control": "Previous two-triangle terrain; diagnostic, expected to flicker",
        "results": reports,
        "page_errors": errors,
    }
    (args.output / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    assert not errors, errors
    assert report["status"] == "passed", "visible background discontinuity; inspect report.json"


if __name__ == "__main__":
    main()
