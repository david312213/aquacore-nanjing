# 下视模型 20261016

当前比赛场地优先版本，替换原 1013 下视模型。六类顺序：帐篷、碉堡、桥梁、装甲车、红十字、H。

静态 FP32 ONNX；输入 1×3×640×640，输出 1×10×8400；letterbox、RGB/255、外部 NMS 0.70，置信度保留 0.45。开环、闭环和 H 下视共用本权重。

`config/mission.yaml` 与调试配置的 `runtime.down_model`、`down_model_selection.copy/hybrid` 已设为此版本。原 `yolo11n_20261013` 和 `yolo11n`（1014）模型仍保留。具体安装/回退路径见 `data/deploy_backups/down_model_20261016_*` 中的 `installation.json` 和 `rollback.sh`；回退只恢复模型选择，不回退现场调参。

已知表现：新场地有提升；蓝圈碎片、黑画面仍可能误检，旧场景 H 召回下降。当前全录像回放部分抽帧参加过训练，不属于独立新航次验收。
