# 第三方软件声明

Elevator-LIO 整体以 `GPL-2.0-or-later` 许可证发布。独立以 MIT License 提供的文件保留其文件级 SPDX 标识。

## ikd-Tree

- 上游项目：[hku-mars/ikd-Tree](https://github.com/hku-mars/ikd-Tree)
- 原作者：Yixi Cai、Wei Xu、Fu Zhang 及贡献者
- 上游许可证：GNU General Public License, version 2
- 本仓库中的文件：`include/ikd_tree/ikd_Tree.h` 和 `include/ikd_tree/ikd_Tree.cpp`
- 本地修改：集成到 Elevator-LIO，适配路径和类型，并补充注释

上游项目声明其源代码以 GPLv2 发布。Elevator-LIO 在此记录上游声明，并在源码和包元数据中将仓库许可证标识为 `GPL-2.0-or-later`。公开发布前，维护者应保留上游声明，并确认上游授权是 GPL-2.0-only，还是包含 later-version 选项。

在学术工作中使用 ikd-Tree 时，应引用以下论文：

```bibtex
@article{cai2021ikd,
  title   = {ikd-Tree: An Incremental KD Tree for Robotic Applications},
  author  = {Cai, Yixi and Xu, Wei and Zhang, Fu},
  journal = {arXiv preprint arXiv:2102.10808},
  year    = {2021}
}
```

## ikd-Tree 中文注释

- 来源项目：[KennyWGH/ikd-Tree-detailed](https://github.com/KennyWGH/ikd-Tree-detailed)
- 关系：`hku-mars/ikd-Tree` 的 fork
- 许可证：GNU General Public License, version 2
- 使用内容：`include/ikd_tree/ikd_Tree.h` 和 `include/ikd_tree/ikd_Tree.cpp` 中的部分中文解释性注释

## 外部依赖

ROS、PCL、Eigen、OpenCV、yaml-cpp、TBB、OpenMP 及其传递依赖未内置到本仓库中。它们仍受各自许可证约束。
