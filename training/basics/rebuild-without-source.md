# 演示：编译后不 source，输出还是旧的

先记结论。同一个工作空间、同一个终端里，`colcon build` 之后不重新 `source install/setup.bash`，一般不会出现旧输出，因为 `install/` 的路径没变，可执行文件就在原地被覆盖。真正会出现旧输出的，是下层已经有同名包的时候。

## 复现用的最小工程

两个工作空间，包名相同，代码不同。下层打印 `v1`，上层打印 `v2`。

```text
underlay/src/hello_demo/hello_demo/hello.py    打印 v1
overlay/src/hello_demo/hello_demo/hello.py     打印 v2
```

两个 `hello.py` 的内容：

```python
def main():
    print("v1")
```

```python
def main():
    print("v2")
```

`setup.py` 里只加一处入口，两个包都一样：

```python
    entry_points={
        "console_scripts": [
            "hello = hello_demo.hello:main",
        ],
    },
```

## 操作步骤

```bash
# 1 建下层，编译，把它 source 进当前终端
cd underlay/src
ros2 pkg create --build-type ament_python hello_demo --dependencies rclpy
# 按上面的内容写 hello_demo/hello_demo/hello.py 和 setup.py
cd .. && colcon build --symlink-install
source install/setup.bash
ros2 run hello_demo hello        # 输出 v1

# 2 建上层，包名完全相同，编译，但故意不 source
cd ../overlay/src
ros2 pkg create --build-type ament_python hello_demo --dependencies rclpy
# 同样写文件，这次打印 v2
cd .. && colcon build --symlink-install
ros2 run hello_demo hello        # 仍然输出 v1
ros2 pkg prefix hello_demo       # 指向 underlay/install，说明用的是下层那份

# 3 source 上层，再看一次
source install/setup.bash
ros2 run hello_demo hello        # 这次输出 v2
ros2 pkg prefix hello_demo       # 指向 overlay/install
```

第 2 步的 `ros2 run` 输出 `v1`，就是"编译过了但没 source，输出和之前一样"。原因是 `ros2` 按环境变量 `AMENT_PREFIX_PATH` 的顺序找包，先命中的是下层。

## 另一个常见版本：新终端没 source

```bash
colcon build --symlink-install     # 编译完了
ros2 run hello_demo hello          # Package 'hello_demo' not found
source install/setup.bash
ros2 run hello_demo hello          # 正常输出
```

报错是找不到包，不是输出旧结果。两个现象要分开讲，学生才不会记错因果。

## 收尾

演示完删掉两个工作空间即可，它们和车上的 `dev_ws` 没有关系。查询当前用的是哪一份包，记住这一条命令：

```bash
ros2 pkg prefix <包名>
```
