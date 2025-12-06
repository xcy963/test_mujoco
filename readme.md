# 搬家用的

## 常用的
```bash
--cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=YES
--cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

## bug说明
1. opencv窗口不更新:需要在一个经常刷新的函数里面写上
`cv::waitKey(1);`

2. 关于matplotlibcpp 
- 使用`PyErr_Print();`可以在调用python函数失败时打印错误信息，帮助调试。
- 使用`const std::map<std::string, std::string>& keywords` 作为参数,像python那样子传递参数
        

## 关于加力矩,
### 1 按住ctrl 鼠标左键是旋转力矩
### 2 按住ctrl 鼠标右键是带方向的力

## 常用的快捷键
1. ctrl + A 是视角快速回到初始位置
2. `是显示AABB(最小外接矩形)
3. 调试坐标系可以在,这个label立选body![alt text](image.png)
4. 