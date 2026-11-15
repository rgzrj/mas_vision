"""生成装甲板调试布局；仅更新布局时运行，要求 rerun-sdk==0.41.0。"""

from pathlib import Path

import rerun as rr
import rerun.blueprint as rrb


# 位顺序与 ArmorDebug::enabled_views 一致，不能按窗口排列顺序调整。
VIEWS = (
    ("HikCamera", "detect"),
    ("HikCamera_track", "track"),
    ("HikCamera_shoot", "shoot"),
    ("HikCamera_track_pose", "pose"),
)

CURVE_GROUPS = {
    "detect": ("count",),
    "track": ("position", "velocity", "rotation", "geometry", "covariance"),
    "shoot": ("angles", "state"),
    "pose": (),
}


if __name__ == "__main__":
    if rr.__version__ != "0.41.0":
        raise SystemExit("Use rerun-sdk==0.41.0")

    # 四个开关共有 15 种非空组合，每种布局保存为对应位掩码的 RBL 文件。
    for mask in range(1, 16):
        columns = []
        for bit, (name, stage) in enumerate(VIEWS):
            if not mask & (1 << bit):
                continue

            image = rrb.Spatial2DView(
                name=name,
                origin="/",
                contents=["+ /camera/image", f"+ /{name}/image/**"],
            )
            plots = [
                rrb.TimeSeriesView(name=group, origin=f"/armor/{stage}/{group}")
                for group in CURVE_GROUPS[stage]
            ]

            # 有曲线的阶段上方放图像，下方按物理量分组；pose 只显示图像。
            columns.append(
                rrb.Vertical(image, rrb.Tabs(*plots), row_shares=[3, 1])
                if plots else image
            )

        rrb.Blueprint(
            rrb.Horizontal(*columns),
            rrb.TimePanel(timeline="exposure", play_state="Following", state="collapsed"),
            rrb.BlueprintPanel(state="collapsed"),
            rrb.SelectionPanel(state="collapsed"),
            auto_layout=False,
            auto_views=False,
        ).save("mas_vision", Path(__file__).parent / f"{mask}.rbl")

    print("Generated 15 armor layouts with original MAS window names")
