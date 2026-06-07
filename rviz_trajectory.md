## Intro

记录解决trajectory无法在rivz中显示的问题的过程

## 问题1

首先要明确rviz显示trajectory是不是通过ros2 topic完成的

是

## 问题2

是通过topic, 那么排查方式呢?

- 发布的type是什么

关于ros2 topic的一个基础问题：durability（持久性），更具体地说，是不是支持 late-joining subscriber（后加入的订阅者）

使用`ros2 topic list`, `ros2 topic info -v`, “我已经证明了这个 topic 和一个 publisher 在 ROS graph 中存在，但我还没有证明这个 publisher 当前正在出数据，也没有证明我的订阅者和它的 QoS 一定兼容。”

改了一下，行了又没有完全行，好像是轨迹播放的太快了(改了播放速度后能正常播放了)，或许是发布频率的问题

来讨论下conflict only的数据表示，为当前的senario_search添加相应的功能，导出一份额外的json。注意，这份json也要包含整个实验的数据