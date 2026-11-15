# YOLO检测入口

深大与自训练模型共用一套接口：输入FP32 `[1,3,480,640]`，输出FP32 `[1,18,6300]`；三类为未激活/小符已激活/大符已激活，五点为top/left/R/right/bottom。旧三类/五类Pose权重不再接入。

- yolo.hpp/cpp：模块内读YAML，OpenVINO CPU单流推理，RGB/NCHW/255及居中letterbox。
- pose_output.hpp/cpp：18通道解码、点有效性、同组中心距离去重，拆分R和符板公共观测。
- pixel_geometry.hpp/cpp：原图ROI判色和可选像素精修，精修失败保留有效原始点。
- 后续pose沿用RMCS的R＋四端点PnP，track和shoot不区分权重来源。

颜色配置仅支持 `red` / `blue`，缺省为 `red`；电控切色仍使用这两个值，不再支持 `auto`。配置示例见[tests/auto_buff/yolo_pose.yaml](tests/auto_buff/yolo_pose.yaml)。模型换成自训练交付文件时只修改model，不能改变类别或点序；nms_distance的单位为原图像素，不再使用iou参数。detect.debug控制检测模块自己的显示。

基准模型保存在auto_buff/buff_detect/yolo/model/rune_szu.onnx，上游来源/哈希见同目录SZU_MODEL.json，许可见SZU_LICENSE.txt。当前基准不是本项目量化的INT8；INT8交付要求、数据迁移和训练验收统一见[TRAINING_YOLO_POSE.md](TRAINING_YOLO_POSE.md)。

类别0完整四点可初始化；类别1提供已击打板中心；类别2保留独立灯臂观测，由 `buff_track/pose/big_activated.cpp` 转换为已激活板心，再调用原有 RMCS correct。该转换没有额外开关，不能将灯臂四点直接用作靶板 PnP 端点。

唯一精修开关仍为 `detect.feature_points: none/rmcs_pixels`：关闭时各类保持原始点；开启时待击板/小符使用现有实现，大符灯臂走预留接口，当前返回原始点。精修开关仅由检测器读取 YAML 并统一传入三类处理流程，不在 Board/LightArm 中存储副本；精修函数返回值表达本次是否成功，`source` 保留物理观测类型。YOLO 原始三类五点始终保存在 `raw_poses`，不受 debug 开关影响。

大符几何输入与可选角点精修分开：类别2按有效网络点确定局部ROI，参考深大做颜色差分、5×5平滑、外轮廓提取及关键点支持检查，保存 contour/contour_status，不覆盖网络点。灯臂四点在杆上，不能套用“网络四点均值是靶板中心”的轮廓分类条件。

同帧必须有 R 和 1～2 块完整待击板通过已有 RMCS PnP，提供符中心及竖直符面。固定这份几何，使用深大56点激活灯臂物理模型逐根匹配实测轮廓，通过有界一维转角搜索及距离变换求相位，再将0.7 m半径处的靶心投影回原图。投影包含原相机畸变；SZU模型轴转换为RMCS轴，灯臂-11.1 mm偏移只计算一次。模型来源与数值在 big_activated_SOURCE.json / szu_light_arm_model.hpp。这是深大方法的适配实现，不是完整Projector或Ceres移植，不提前分配72°槽位，不补出未观测杆的中心。

缺R、无完整待击板、截断或无有效轮廓、匹配支持不足/歧义时不输出大符板心；已有完整轮廓可以容忍一个无效网络端点。只看到灯臂时不沿用历史符面；不读取EKF预测。直接中心优先并去重。推导中心保留big_activated类别，映射RMCS时为active=true且不携带初始化端点，不刷新待击有效期。各推导中心与待击板共享符面，误差相关，当前RMCS噪声保持原样。

实机/配对回放的模式在输入适配层筛选：小符保留0/1，大符保留0/2；原始raw_poses全保留，普通无模式视频保留三类。模式不用于强制选择RMCS线性/正弦模型，不新增配置开关。深大模型尺寸及色差阈值（红50/蓝62）尚待本机实拍验证；更换权重不能替代相机和机械尺寸核对。

日志新增 `raw_poses`、`light_arms` 及 `track.geometry_boards`；后者区分反解板心与直接检测，并记录 match_error_px/match_support；light_arms记录实测轮廓及失败状态，rune_mode记录输入模式。此修改位于主工程，`versions/` 旧试验副本不是运行依赖。来源记录见 `auto_buff/buff_track/pose/big_activated_SOURCE.json`。合成几何检查与已有视频回归通过不代表真实大符精度已验收。
