# 无人机智能化应用系统

基于Streamlit开发的无人机智能化应用系统，包含航线规划、飞行监控、心跳检测和坐标转换功能。

## 功能特性

### 🗺️ 航线规划
- **卫星地图展示** - 使用ESRI卫星地图，美观清晰
- **航点添加** - 支持地图点击添加和手动输入坐标
- **障碍物多边形圈选** - 支持多边形圈选障碍物，含高度设置
- **安全半径设置** - 可配置安全半径，实时检测航线冲突
- **绕飞路径规划** - 支持向左绕飞、向右绕飞、最优路径（弧线）
- **障碍物高度设置** - 根据飞行高度自动判断是否需要绕飞
- **JSON文件保存** - 航线和障碍物数据自动保存到JSON文件
- **坐标转换工具** - WGS-84/GCJ-02/BD-09坐标转换

### 🚁 飞行监控
- 按航点飞行模拟
- 实时状态监测（速度、高度、航向）
- 电量模拟与预警
- 飞行数据可视化
- **通信链路状态** - GCS-OBC-FCU链路监控
- **系统拓扑图** - GCS-OBC-FCU可视化拓扑
- **MAVLink数据流** - 实时报文显示与统计

## 技术栈

- **前端框架**: Streamlit
- **数据可视化**: Plotly
- **地图服务**: Folium (ESRI卫星地图)
- **编程语言**: Python 3.8+

## 本地运行

### 1. 克隆仓库

```bash
git clone https://github.com/your-username/uav-monitor.git
cd uav-monitor
```

### 2. 安装依赖

```bash
pip install -r requirements.txt
```

### 3. 运行应用

```bash
streamlit run app.py
```

## 部署到Streamlit Cloud

### 1. 推送代码到GitHub

```bash
git add .
git commit -m "Initial commit"
git push origin main
```

### 2. 在Streamlit Cloud部署

1. 访问 [share.streamlit.io](https://share.streamlit.io)
2. 使用GitHub账号登录
3. 点击 "New app"
4. 选择仓库和分支
5. 设置主文件路径为 `app.py`
6. 点击 "Deploy"

## 项目结构

```
uav_monitor/
├── app.py                    # 主应用入口
├── route_planner.py          # 航线规划核心模块
├── heartbeat.py              # 心跳包模拟模块
├── coordinate_converter.py   # 坐标转换模块
├── flight_monitor.py         # 飞行监控页面
├── route_planning.py         # 航线规划页面（备用）
├── requirements.txt          # 依赖列表
├── .github/workflows/        # GitHub Actions配置
│   └── deploy.yml
├── .streamlit/
│   └── config.toml           # Streamlit配置
└── route.json                # 航线数据存储
```

## 版本更新

### v2.0.0 (2026-06-23)
- ✅ 优化地图显示，使用ESRI卫星地图
- ✅ 增强多边形圈选障碍物功能
- ✅ 添加障碍物高度设置与JSON文件保存
- ✅ 添加飞行高度/安全半径设置
- ✅ 优化飞越/绕飞路径规划算法（新增弧形绕飞）
- ✅ 修复安全半径显示问题（使用真实米数）
- ✅ 同步RoutePlanner与session_state数据
- ✅ 添加坐标转换工具（WGS-84/GCJ-02/BD-09）
- ✅ 添加通信链路状态监控
- ✅ 添加GCS-OBC-FCU系统拓扑图
- ✅ 添加MAVLink数据流与报文显示

## 许可证

MIT License

---

© 2024-2026 南京科技职业学院 | 无人机智能化应用项目
