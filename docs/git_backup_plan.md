# Git 备份与上传建议

## 应优先备份（源码与配置）
- px4_offboard/offboard_square.py
- px4_offboard/subagent_px4_offboard.md
- openvins/plan.md
- openvins/want_px4.md
- openvins/subagent_openvins.md
- bridges/imu_bridge/**
- bridges/vio_to_px4/**
- bridges/open_vins_patches/**
- scripts/install_deps_jazzy.sh
- docs/AGENT_workspace.md

## 你当前工作区中已检测到的关键“未提交改动”
- ros2_ws/src/open_vins:
  - ov_core/src/test_tracking.cpp
  - ov_msckf/src/ros/ROS1Visualizer.h
  - ov_msckf/src/ros/ROS2Visualizer.h
  - ov_msckf/src/ros/ROSVisualizerHelper.h
  - config/px4_mono/*

## 不建议纳入 git 备份
- build/
- install/
- log/
- __pycache__/
- *.pyc
- *:Zone.Identifier

## 上传策略
1. 本仓库用于“工程集成快照”与可复现构建说明。
2. 上游第三方仓库（如 open_vins、px4_msgs）建议单独 fork/子模块管理，不建议整仓复制。
3. 如需保留上游差异，推荐同时输出 patch 文件或 PR 链接。
