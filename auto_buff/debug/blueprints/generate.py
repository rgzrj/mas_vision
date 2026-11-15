"""Generate the embedded layouts; only needed when changing layouts, not at runtime."""

import argparse
from pathlib import Path

import rerun as rr
import rerun.blueprint as rrb


def make_blueprint(mask: int) -> rrb.Blueprint:
    columns = []
    for bit, name in enumerate(("detect", "track", "shoot")):
        if not mask & (1 << bit):
            continue
        # Same pixel origin and one camera image, with only this module's overlays.
        image = rrb.Spatial2DView(
            name=name,
            origin="/",
            contents=["+ /camera/image", f"+ /{name}/image/**"],
        )
        if name == "detect":
            plots = [
                rrb.TimeSeriesView(name="Confidence", origin="/detect/confidence"),
                rrb.TimeSeriesView(name="Counts", origin="/detect/count"),
                rrb.TimeSeriesView(name="Classes", origin="/detect/classes"),
            ]
        elif name == "track":
            plots = [
                rrb.TimeSeriesView(name="Angles (rad)", origin="/track", contents=[
                    "+ /track/angle_rad", "+ /track/ekf/theta_rad", "+ /track/ekf/face_yaw_rad",
                ]),
                rrb.TimeSeriesView(name="Angular speed (rad/s)", origin="/track", contents=[
                    "+ /track/speed_rad_s", "+ /track/ekf/omega_rad_s",
                ]),
                rrb.TimeSeriesView(name="Position (m)", origin="/track/ekf", contents=[
                    "+ /track/ekf/x_m", "+ /track/ekf/y_m", "+ /track/ekf/z_m",
                ]),
                rrb.TimeSeriesView(name="Position variance (m2)", origin="/track/ekf/P_diag", contents=[
                    "+ /track/ekf/P_diag/x_m", "+ /track/ekf/P_diag/y_m", "+ /track/ekf/P_diag/z_m",
                ]),
                rrb.TimeSeriesView(name="Angle variance (rad2)", origin="/track/ekf/P_diag", contents=[
                    "+ /track/ekf/P_diag/theta_rad", "+ /track/ekf/P_diag/face_yaw_rad",
                ]),
                rrb.TimeSeriesView(name="Speed variance (rad2/s2)", origin="/track/ekf/P_diag", contents=[
                    "+ /track/ekf/P_diag/omega_rad_s",
                ]),
                rrb.TimeSeriesView(name="Fit error", origin="/track", contents=["+ /track/fit_error"]),
                rrb.TimeSeriesView(name="Track state", origin="/track", contents=[
                    "+ /track/initialized", "+ /track/observed", "+ /track/ready",
                    "+ /track/matched", "+ /track/sine_valid",
                ]),
                rrb.Spatial3DView(name="World", origin="/world"),
            ]
        else:
            plots = [
                rrb.TimeSeriesView(name="Angles (rad)", origin="/shoot", contents=[
                    "+ /shoot/yaw_rad", "+ /shoot/pitch_rad",
                    "+ /shoot/gimbal_yaw_rad", "+ /shoot/gimbal_pitch_rad",
                ]),
                rrb.TimeSeriesView(name="Timing (s)", origin="/shoot", contents=[
                    "+ /shoot/flight_s", "+ /shoot/prediction_s",
                ]),
                rrb.TimeSeriesView(name="Fire state", origin="/shoot", contents=[
                    "+ /shoot/valid", "+ /shoot/should_shoot",
                ]),
            ]
            if not mask & 2:
                plots.append(rrb.Spatial3DView(name="World", origin="/world"))
        columns.append(rrb.Vertical(image, rrb.Tabs(*plots), row_shares=[3, 1]))
    return rrb.Blueprint(
        rrb.Horizontal(*columns),
        rrb.TimePanel(timeline="exposure", play_state="Following", state="collapsed"),
        rrb.BlueprintPanel(state="collapsed"),
        rrb.SelectionPanel(state="collapsed"),
        auto_layout=False,
        auto_views=False,
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=Path(__file__).parent)
    args = parser.parse_args()
    if rr.__version__ != "0.41.0":
        raise SystemExit("Use rerun-sdk==0.41.0, matching the bundled C++ SDK and Viewer")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    for mask in range(1, 8):
        make_blueprint(mask).save("mas_vision", args.output_dir / f"{mask}.rbl")
    print(f"Generated 7 layouts in {args.output_dir}")
