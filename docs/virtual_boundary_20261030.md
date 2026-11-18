# 场地四角虚拟围墙交付（2026-10-30）

部署位置：drone24@192.168.90.236:/home/drone24/aquacore-nanjing。

## 使用

保持原有传感器、定位和控制器运行；录四角、录航线和执行使用同一次定位。未解锁时拿着飞机沿场地周边顺时针或逆时针走到四个角，运行：

```sh
cd /home/drone24/aquacore-nanjing
sh flight/record_boundary.sh
```

每角空格记录，x 撤销最后一角，q 保存四角。独立写入 data/routes/field_boundary.yaml，不覆盖 field_loop.yaml 或 waypoints.yaml。只记 XY；角点沿边排列，不能交叉、重合或共线。实测四角按原始位置构成凸四边形，不强制拟合正交矩形。

flight/flight.yaml 新增 common.planning.virtual_boundary：

```yaml
enabled: false
file: data/routes/field_boundary.yaml
bottom_m: null
top_m: null
```

现场录好后把 enabled 改为 true。null 使墙覆盖完整地图高度；填写数值时按本次起飞点的 Z 基准计算墙底/墙顶。功能只影响下次任务，运行中不重读文件。启用时把四角与来源 SHA256 冻结进任务配置。

```sh
sh flight/fly_points.sh --route data/routes/field_loop.yaml --dry-run
sh flight/fly_points.sh --route data/routes/field_loop.yaml
```

预览显示围墙开关、文件、角点和高度范围。关闭用 enabled=false。

## 规划行为

墙外和内侧膨胀带均作为障碍；采用当前巡航/过门膨胀并按地图分辨率向上取整。当前现场巡航膨胀 0.45m，因此录制的飞行航点要留在墙内可通行区域。墙不会随点云地图重置/刷新消失。

虚拟墙走现有 EGO 碰撞查询接口，不修改目标点、不新增等待/停稳/超时/重试；不控制手动接管，也不约束绕过 EGO 的原生起降和移动指令。它是规划地图约束，不是 PX4 级电子围栏或实飞绝不越界的保证。

RViz Marker：/drone_0_ego_planner_node/grid_map/virtual_boundary。橙色框线表示录制的几何边界，膨胀区域位于其内侧。关闭后发布删除标记，清掉旧显示。

## 验证

- Python 四角录制、顺逆时针、非法形状、独立文件格式、保存失败保留旧文件、冻结快照及真实录点离线入口 4 项通过。
- C++ 几何测试覆盖内外、边上、膨胀变化、顺逆序、旋转场地、墙体高度范围和错误点序。
- 本地与机载原有航点测试通过（64 次用例执行，包含原测试类导入/继承的重复回归项）。项目结构检查及现有 13 个 dry-run 通过。
- 机载全量 catkin_make -j2 构建成功。
- 独立 ROS master 下启动实际 ego_worker.launch，验证四边外侧、角部和内侧占用；0.10/0.35/0.10m 切换随之改变靠墙占用。
- 暂停新点云时围墙仍有效，恢复点云刷新后也保留。旧目标重试不覆盖当前膨胀。
- 启用时发布 12 条三维框线；关闭回归中确认删除旧 Marker，原障碍膨胀行为保持。
- 实际机载 dry-run 读取边界快照通过，所有模拟文件保存在 data/fixes/20261030-virtual-boundary，未作为现场边界启用。
- 未连接飞控进行测试，未解锁、起飞、投放或实飞验证。没有改动现场已有航点、巡航/过门参数、定位和投放逻辑。

证据、部署清单及修改前源码备份：data/fixes/20261030-virtual-boundary/。现场虚拟墙默认关闭，等待用户录制真实四角并启用。
