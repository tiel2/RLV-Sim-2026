"""Export the selected flight reconstruction versus observations, with Korean figures.

Reads verified final-run artifacts; does not simulate or fit parameters. Historical
campaign artifacts stay unchanged. Metrics use every eligible observed interval
supported by this run, separately for altitude and speed.
"""

import argparse
import hashlib
import json
import shutil
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / "results/verification/public_flight_comparison"
NAMES = dict(
    ascent="상승",
    entry_burn="재진입 연소",
    aero_descent="공력 하강",
    powered_descent="최종 동력 하강",
    landing="착륙",
)


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write(path, value):
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False) + "\n")


def plot(folder, name, p, a, report):
    fig, axes = plt.subplots(2, 3, figsize=(15, 8))
    times = a["reference_time_s"]
    epoch = report["window_s"][0]
    for col, (key, pk, label, unit) in enumerate(
        [("altitude_m", "altitude_m", "고도", "m"), ("speed_m_s", "speed_mps", "속력", "m/s")]
    ):
        used = a[f"{key}_quality_eligible"] & ~a["initialization_used"]
        axes[0, col].plot(
            times[~used],
            a[f"reference_{key}"][~used],
            "x",
            ms=3,
            color=".75",
            label="평가에서 제외한 관측",
        )
        axes[0, col].plot(
            times[used],
            a[f"reference_{key}"][used],
            ".",
            ms=2,
            color="black",
            label="공개 비행 관측",
        )
        axes[0, col].plot(p["t"] + epoch, p[pk], color="#197a97", label="시뮬레이션")
        axes[0, col].set(title=f"{label} 비교", ylabel=f"{label} [{unit}]")
        axes[1, col].plot(times, a[f"error_{key}"], ".", ms=2, color="#197a97")
        axes[1, col].axhline(0, color=".4", lw=0.7)
        axes[1, col].set(title=f"{label} 차이", ylabel=f"시뮬레이션 - 관측 [{unit}]")
        for row in range(2):
            axes[row, col].set(xlim=report["window_s"], xlabel="발사 후 시간 [초]")
            axes[row, col].grid(alpha=0.25)
        axes[0, col].legend(fontsize=8)
    valid = (
        a["altitude_m_quality_eligible"]
        & a["speed_m_s_quality_eligible"]
        & ~a["initialization_used"]
    )
    axes[0, 2].plot(
        a["reference_altitude_m"][valid],
        a["reference_speed_m_s"][valid],
        ".",
        ms=2,
        color="black",
        label="공개 비행 관측",
    )
    axes[0, 2].plot(p["altitude_m"], p["speed_mps"], color="#197a97", label="시뮬레이션")
    axes[0, 2].set(title="고도에 따른 속력", xlabel="고도 [m]", ylabel="속력 [m/s]")
    axes[0, 2].legend(fontsize=8)
    axes[0, 2].grid(alpha=0.25)
    lines = ["최종 적용안과 공개 비행 데이터의 차이", ""]
    for key, label, unit in (("altitude_m", "고도", "m"), ("speed_m_s", "속력", "m/s")):
        m = report["metrics"][key]
        lines += [
            f"{label} RMSE: {m['rmse']:.3f} {unit}",
            f"비교한 시간: {m['covered_duration_s']:.3f}초",
            f"평가 가능한 관측: {m['available_observation_duration_s']:.3f}초",
            "",
        ]
    if name == "landing":
        lines += [
            "모의 접지 뒤의 속력 관측은 비교하지 못함.",
            "접지 뒤를 속력 0으로 채우지 않음.",
            "명령을 즉시 따르는 이상적인 구동기 사용.",
        ]
    axes[1, 2].axis("off")
    axes[1, 2].text(0, 1, "\n".join(lines), va="top", fontsize=10)
    fig.suptitle(f"CRS-11 {NAMES[name]}: 공개 비행 관측과 시뮬레이션", fontsize=15)
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    for extension in ("png", "pdf"):
        fig.savefig(folder / f"comparison.{extension}", dpi=160)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=BASE / "integrated_20261107")
    parser.add_argument("--output", type=Path, default=BASE / "final_report_20261107")
    args = parser.parse_args()
    source, out = args.source.resolve(), args.output.resolve()
    manifest = json.loads((source / "artifact_manifest.json").read_text())
    required = ["summary.json"] + [
        f"{name}/after/{filename}"
        for name in NAMES
        for filename in (
            "resolved_config.yaml",
            "physics.npz",
            "actuators.npz",
            "flight_comparison.npz",
            "flight_comparison.json",
            "kpi.json",
        )
    ]
    for file in required:
        if sha(source / file) != manifest[file]:
            raise ValueError(f"archived evidence changed: {file}")
    s = json.loads((source / "summary.json").read_text())
    out.mkdir(parents=True, exist_ok=False)
    plt.rcParams.update({"font.family": "NanumGothic", "axes.unicode_minus": False})
    reports = {}
    for name in NAMES:
        folder = out / name
        folder.mkdir()
        r = s["phases"][name]
        report = dict(
            phase=name,
            window_s=r["window_s"],
            settings=r["versions"]["after"],
            metrics=r["metrics"]["full_overlap"]["after"],
            numerical_verification=r["numerical_verification"],
            sensitivity_verdict=r["p4_verdict"],
            touchdown_truth=r["touchdown_truth"],
        )
        if name == "landing":
            report["landing_independent_plant_replay"] = r["landing_independent_plant_replay"]
        for file in required:
            if file.startswith(f"{name}/after/"):
                shutil.copy2(source / file, folder / Path(file).name)
        p = dict(np.load(folder / "physics.npz"))
        a = dict(np.load(folder / "flight_comparison.npz"))
        plot(folder, name, p, a, report)
        write(folder / "summary.json", report)
        reports[name] = report
    write(
        out / "summary.json",
        dict(
            phases=reports,
            scope="selected simulation versus observations",
            limitations=s["limitations"],
        ),
    )
    write(
        out / "input_manifest.json",
        dict(
            source=str(source),
            sha256={file: manifest[file] for file in required},
            script_sha256=sha(Path(__file__)),
        ),
    )
    write(
        out / "artifact_manifest.json",
        {str(p.relative_to(out)): sha(p) for p in out.rglob("*") if p.is_file()},
    )
    print(f"Exported five final-run figures and results: {out}")


if __name__ == "__main__":
    main()
