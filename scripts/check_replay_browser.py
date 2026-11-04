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
    parser.add_argument("--separation", help="optional ascent_separation replay")
    parser.add_argument("--boostback", help="optional ascent_boostback replay")
    parser.add_argument(
        "--project2-root", type=Path, help="root containing the five Project 2 run folders"
    )
    parser.add_argument(
        "--project2-only", action="store_true", help="inspect only the five phase runs"
    )
    args = parser.parse_args()
    if args.project2_only and args.project2_root is None:
        parser.error("--project2-only requires --project2-root")
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
            cases = [
                ("ascent", args.ascent),
                ("landing", args.landing),
                ("depletion", args.depletion),
            ]
            if args.project2_only:
                cases = []
            if args.project2_root:
                cases.extend(
                    ("project2_" + name, str(args.project2_root / name / "replay.json"))
                    for name in (
                        "ascent",
                        "entry_burn",
                        "aero_descent",
                        "powered_descent",
                        "landing",
                    )
                )
            if args.separation:
                cases.append(("separation", args.separation))
            if args.boostback:
                cases.append(("boostback", args.boostback))
            for name, filename in cases:
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
                if name.startswith("project2_"):
                    final = replay["frames"][-1]["t"]
                    epochs = [0, final / 2, final, final / 2, 0]
                if name == "separation":
                    epochs = [149.8, 150, 160, 149.8, 150]
                if name == "boostback":
                    transitions = {
                        e["phase_after"]: e["t"]
                        for e in replay["meta"]["events"]
                        if e["kind"] == "mission_transition"
                    }
                    epochs = [
                        150,
                        transitions[11] + 30,
                        transitions[13] + 1,
                        transitions[15],
                        transitions[13] + 1,
                        transitions[11] + 30,
                    ]
                for epoch in epochs:
                    frame = min(replay["frames"], key=lambda f: abs(f["t"] - epoch))
                    # At a duplicate separation epoch, the active state is the right limit.
                    frame = next(f for f in reversed(replay["frames"]) if f["t"] == frame["t"])
                    vehicle = replay["meta"].get("vehicles", [replay["meta"]["vehicle"]])[
                        frame.get("configuration", 0)
                    ]
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
                        const expectedPlumes = frame.engine_force_body.filter(f =>
                            new T.Vector3(...f).length() > 0.02 * vehicle.thrust_vac[0]).length;
                        if (r.engines.filter(e => e.plume.visible).length !== expectedPlumes)
                            issues.push('active plume count');
                        const attitude = new T.Quaternion(...frame.q).normalize();
                        if (r.group.quaternion.angleTo(attitude) > 1e-6)
                            issues.push('body attitude');
                        return {issues, engines: r.engines.length, time_s: frame.t,
                                plumes: r.engines.filter(e => e.plume.visible).length};
                    }""",
                        {
                            "frame": frame,
                            "vehicle": vehicle,
                            "site": replay["meta"]["scene"],
                        },
                    )
                    reports.append({"case": name, **check})
                    page.screenshot(path=str(args.output / f"{name}_{epoch:g}s.png"))
                    assert not check["issues"], check
                if name == "separation":
                    event = next(
                        e for e in replay["meta"]["events"] if e["kind"] == "stage_separation"
                    )
                    continuity = page.evaluate(
                        """async t => {
                        const T = await import('three'), d = window.replayInspection;
                        d.seek(t - 1e-8); d.scene.updateMatrixWorld(true);
                        const before = d.rocket.engines[0].gim.getWorldPosition(new T.Vector3());
                        const attached = !!d.rocket.inner.getObjectByName('attached-upper-stack');
                        d.seek(t); d.scene.updateMatrixWorld(true);
                        const after = d.rocket.engines[0].gim.getWorldPosition(new T.Vector3());
                        return {distance_m: before.distanceTo(after), attached,
                            separated: !d.rocket.inner.getObjectByName('attached-upper-stack')};
                    }""",
                        event["t"],
                    )
                    assert continuity["distance_m"] < 1e-3, continuity
                    assert continuity["attached"] and continuity["separated"], continuity
                    reports.append({"case": "separation_continuity", **continuity})
                if name == "boostback":
                    assert replay["meta"].get("rcs_activity"), "refresh replay RCS channels first"
                    # Check exact on/off boundaries and backward seeks, independently of
                    # pose sampling. A failed or opposite-sense nozzle must stay dark.
                    burst = next(
                        row
                        for row in replay["meta"]["rcs_activity"]
                        if row["t"] > 160 and max(row["duty"]) >= 0.3
                    )
                    for epoch in [burst["t"] - 1e-5, burst["t"], 145, burst["t"], 371.38]:
                        expected = next(
                            row["duty"]
                            for row in reversed(replay["meta"]["rcs_activity"])
                            if row["t"] <= epoch
                        )
                        check = page.evaluate(
                            """async ({t, duties}) => {
                            const T = await import('three'), d = window.replayInspection;
                            d.seek(t); d.scene.updateMatrixWorld(true);
                            const r = d.rocket, issues = [];
                            r.rcs.forEach(({mount, plume}, i) => {
                                if (plume.visible !== (duties[i] > 0))
                                    issues.push('RCS on/off ' + i);
                                if (Math.abs(plume.userData.duty - duties[i]) > 1e-10)
                                    issues.push('RCS duty ' + i);
                                const exhaust = new T.Vector3(1,0,0)
                                    .applyQuaternion(mount.quaternion);
                                const force = new T.Vector3(...r.group.userData.rcs.directions[i]);
                                if (exhaust.dot(force) > -0.999999) issues.push('RCS exhaust ' + i);
                                if (mount.position.distanceTo(new T.Vector3(
                                    ...r.group.userData.rcs.positions[i])) > 1e-8)
                                    issues.push('RCS mount ' + i);
                            });
                            if (r.rcs.length !== duties.length) issues.push('RCS nozzle count');
                            const count = duties.filter(x => x > 0).length;
                            const status = document.getElementById('rcsState').textContent;
                            if (status !== (count ? 'ON' : 'OFF')) issues.push('RCS HUD state');
                            const cells = [...document.getElementById('rcsNozzles').children];
                            if (cells.length !== duties.length) issues.push('RCS HUD nozzle count');
                            cells.forEach((cell,i) => {
                                if (cell.classList.contains('active') !== (duties[i] > 0) ||
                                    Math.abs(Number(cell.dataset.duty) - duties[i]) > 1e-10)
                                    issues.push('RCS HUD nozzle ' + i);
                            });
                            const expectedCount = `${count} / ${duties.length} NOZZLES`;
                            if (document.getElementById('rcsCount').textContent !== expectedCount)
                                issues.push('RCS HUD active count');
                            const maximum = Math.round(Math.max(...duties) * 100);
                            if (document.getElementById('rcsDuty').textContent !==
                                `MAX DUTY ${maximum}%`) issues.push('RCS HUD duty');
                            return {time_s:t, issues,
                                active: r.rcs.filter(r => r.plume.visible).length};
                            }""",
                            {"t": epoch, "duties": expected},
                        )
                        assert not check["issues"], check
                        reports.append({"case": "rcs_display", **check})
                    page.evaluate("window.replayInspection.seek(170)")
                    page.evaluate("window.replayInspection.setCamera('chase')")
                    page.screenshot(path=str(args.output / "rcs_chase.png"))
                    low_duty = next(
                        row
                        for row in replay["meta"]["rcs_activity"]
                        if row["t"] > 160 and abs(max(row["duty"]) - 0.05) < 1e-10
                    )
                    for label, row in (("low", low_duty), ("high", burst)):
                        # Compare rendered pixels with/without the gas sprites at
                        # the same recorded state; nominal low duty must be visible.
                        check = page.evaluate(
                            """t => {
                            const d = window.replayInspection;
                            d.seek(t); d.setCamera('chase'); d.advance(t);
                            const canvas = document.querySelector('canvas');
                            const gl = canvas.getContext('webgl2') || canvas.getContext('webgl');
                            const pixels = canvas.width * canvas.height;
                            const withGas = new Uint8Array(pixels * 4);
                            const withoutGas = new Uint8Array(pixels * 4);
                            gl.readPixels(0,0,canvas.width,canvas.height,
                                gl.RGBA,gl.UNSIGNED_BYTE,withGas);
                            const puffs = d.rocket.rcs.flatMap(r => r.plume.children);
                            puffs.forEach(p => { p.visible = false; });
                            d.advance(t);
                            gl.readPixels(0,0,canvas.width,canvas.height,
                                gl.RGBA,gl.UNSIGNED_BYTE,withoutGas);
                            puffs.forEach(p => { p.visible = true; });
                            d.advance(t);
                            let visiblePixels = 0;
                            for (let i=0; i<withGas.length; i+=4) {
                                if (Math.max(...[0,1,2].map(j =>
                                    Math.abs(withGas[i+j]-withoutGas[i+j]))) > 25)
                                    visiblePixels++;
                            }
                            return {time_s:t, visible_pixels:visiblePixels,
                                webgl_error:gl.getError()};
                            }""",
                            row["t"],
                        )
                        assert check["webgl_error"] == 0, check
                        assert check["visible_pixels"] >= 500, check
                        reports.append({"case": "rcs_visibility", "duty": label, **check})
                        page.screenshot(path=str(args.output / f"rcs_{label}_duty.png"))
                    page.evaluate("window.replayInspection.seek(145)")
                    page.screenshot(path=str(args.output / "rcs_off.png"))
                    # Real playback updates (no camera reframe between frames) at 1x,
                    # 8x and 32x, including release, large seeks and backwards seeking.
                    for speed in (1, 8, 32):
                        check = page.evaluate(
                            """async speed => {
                            const T = await import('three'), d = window.replayInspection;
                            d.seek(148); d.setCamera('chase');
                            let maxTargetError = 0, maxScreen = 0;
                            const issues = [];
                            const epochs = Array.from({length:40}, (_,i) => 148+i*speed/60)
                                .concat([150,160,250,335.18,346.6,371.38,180,149.8]);
                            for (const t of epochs) {
                                d.advance(t); d.scene.updateMatrixWorld(true);
                                maxTargetError = Math.max(maxTargetError,
                                    d.controls.target.distanceTo(d.rocket.group.position));
                                for (const x of [0, -d.rocket.group.userData.length]) {
                                    const point = d.rocket.inner.localToWorld(new T.Vector3(x,0,0));
                                    point.project(d.camera);
                                    maxScreen = Math.max(maxScreen,
                                        Math.abs(point.x), Math.abs(point.y));
                                    if (point.z < -1 || point.z > 1) issues.push('body clipped');
                                }
                                const sky = d.scene.getObjectByName('sky-background');
                                if (sky.position.distanceTo(d.camera.position) > 1e-7 ||
                                    sky.material.depthWrite || sky.material.depthTest)
                                    issues.push('sky/terrain depth overlap');
                                const ground = d.scene.getObjectByName('land-surface');
                                if (ground.geometry.parameters.width/2 <= d.camera.far ||
                                    d.scene.fog.far >= d.camera.far)
                                    issues.push('finite terrain edge in view');
                            }
                            if (maxTargetError > 1e-6) issues.push('camera lag');
                            if (maxScreen > 0.9) issues.push('body outside frame');
                            return {speed, maxTargetError, maxScreen, issues};
                            }""",
                            speed,
                        )
                        assert not check["issues"], check
                        reports.append({"case": "chase_and_background", **check})
                    page.evaluate("window.replayInspection.seek(346.6)")
                    page.screenshot(path=str(args.output / "high_altitude_chase.png"))
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
