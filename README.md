## Introduction

Implement Stop-N-Go(https://ieeexplore.ieee.org/document/11127576) from scratch

## Extra

Stop-N-Go is also a music(https://www.youtube.com/watch?v=3G1iSMh6OX4), enjoy it

## .gitignore中忽略的文件无法被opencode读取

https://opencode.ai/docs/zh-cn/tools/

```
在内部，grep、glob 和 list 等工具底层使用 ripgrep。默认情况下，ripgrep 遵循 .gitignore 中的模式，这意味着 .gitignore 中列出的文件和目录将被排除在搜索和列表结果之外。

要包含通常会被忽略的文件，请在项目根目录下创建一个 .ignore 文件。该文件可以显式允许某些路径。
```