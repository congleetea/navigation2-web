# Nav2 Lab

Navigation2 算法的中文交互教学实验室。将不同主题放在同一个仓库中，分别维护、运行和扩展。

仓库：<https://github.com/congleetea/navigation2-web>

## 目录结构

```text
nav2-lab/                       # 本地仓库根目录
├── README.md
├── package.json                # 统一测试与启动入口，无 npm 依赖
└── nav2-planner-lab/            # 全局规划教学实验室
    ├── dist/                   # 手写 HTML / CSS / JS，可直接打开，不是可删除的构建缓存
    ├── tests/                  # Node 内置测试
    ├── .openai/hosting.json    # 当前 planner lab 的 Sites 部署配置，无凭据
    └── README.md
```

后续控制器实验室可以添加为同级的 `nav2-controller-lab/`，无需创建独立仓库。当前尚未实现控制器实验室。

## 快速开始

```sh
git clone git@github.com:congleetea/navigation2-web.git nav2-lab
cd nav2-lab
npm start
```

打开 <http://localhost:8765>。启动使用 Python 3，不需要运行 `npm install`。也可以直接打开 `nav2-planner-lab/dist/index.html`。

不使用 npm 时：

```sh
python3 -m http.server 8765 --directory nav2-planner-lab/dist
```

## 测试

需要 Node.js 18 或以上版本，建议 Node.js 22。

```sh
npm test
# 或
node --test nav2-planner-lab/tests/*.test.cjs
```

## Planner Lab

包含 NavFn Dijkstra / A*、Theta*、Smac 2D、Hybrid-A*、State Lattice 六种教学模式：

- 设置起终点、位姿朝向，编辑障碍，切换地图代价权重。
- 在地图正下方播放、暂停、前后单步，或拖动时间线。
- 格内显示 f、g/P、h 或地图代价 C，明确标注颜色范围。
- 黄色圈标出当前节点，绿色圈预告下一处理节点。
- Theta* 用虚线显示原父节点连接，LOS 接受后用实线显示新连接。

详细说明与实现限制见 [Planner Lab README](nav2-planner-lab/README.md)。浏览器实现用于理解算法，不连接 ROS，也不能代替真实 Nav2 的性能与可行性验证。

## 扩展约定

每个实验室放在 `nav2-<主题>-lab/` 中，拥有独立页面、测试和说明文档。仓库根目录保留统一入口与测试命令。增加 controller lab 后，再把它的测试加入根目录的测试命令。

当前 planner lab 的私有 Sites 部署与 GitHub 仓库分别管理，GitHub 是整个项目的协作仓库。发布凭据不写入源码或 Git。
