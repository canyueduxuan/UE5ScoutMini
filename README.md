# Scout Mini Simulation for UE 5.0

面向 AgileX Scout Mini 的专用四轮滑移转向仿真插件。插件包含 Scout Mini Pawn、运动组件和可直接放入关卡的蓝图资产。

## 内容

- `AScoutMiniPawn`：底盘、四个轮轴 Pivot、可替换静态网格、第三人称相机和键盘输入。
- `UScoutMiniMovementComponent`：四点射线悬架、坡面切向驱动、差速偏航、轮胎摩擦、滚阻和空气阻力。
- `/ScoutMiniSimulation/ScoutMini/Scoutmini`：项目现有 Scout Mini 蓝图迁移后的插件资产。

## URDF 基准参数

| 参数 | 数值 |
|---|---:|
| 车体尺寸 | 0.620 x 0.585 x 0.235 m |
| 轴距 | 0.463951 m |
| 轮距 | 0.416503 m |
| 轮半径 | 0.160 m |
| 车轮垂直偏移 | -0.100998 m |
| 总仿真质量 | 72 kg |
| 转动惯量 | 2.288641, 5.103976, 3.431465 kg m^2 |
| 轮胎摩擦系数 | 1.0 |

总质量由 URDF 的 60 kg 底盘和四个 3 kg 车轮合并得到。根碰撞盒使用惯量缩放近似 URDF 惯量张量。

## 使用

1. 启用 **Scout Mini Simulation** 插件并重启编辑器。
2. 在内容浏览器中启用“显示插件内容”。
3. 将 `ScoutMiniSimulation/ScoutMini/Scoutmini` 拖入关卡。
4. 手动模式使用 W/S 前后移动、A/D 差速转向。
5. 程序控制时调用：

```cpp
VehicleMovement->SetControlMode(EScoutMiniControlMode::Programmatic);
VehicleMovement->SetVelocityCommand(2.0f, 0.5f);
```

公开接口使用 SI 单位：线速度为 m/s，角速度为 rad/s，长度为 m，质量为 kg。

## 网格与坐标

底盘和四个车轮网格均可在蓝图中替换。当前导入网格共享 `base_link` 原点，因此四个轮子放在各自轮轴 Pivot 下，并使用反向位置偏移；运行时旋转 Pivot，使网格绕真实轮心旋转。

坐标约定为 UE 的 X 前、Y 右、Z 上。左轮位于负 Y，右轮位于正 Y。

## 动力学保真度

四个轮点独立计算弹簧、阻尼和地面法线。驱动与横向力沿平均接触平面施加。为恢复稳定的加速/制动俯仰，驱动力默认施加在质心下方 0.08 m 处，可通过 `Drive Force Application Depth` 调节：

- `0.0 m`：最稳定，无驱动俯仰。
- `0.05-0.08 m`：推荐范围，具有温和载荷转移。
- `0.10-0.15 m`：俯仰明显，更容易在陡坡或急刹时倾覆。

当前模型没有实现轮胎角动量、电机电流环和多体车轮碰撞。它优先服务于实时 ROS 导航、感知和控制算法仿真。

## 稳定性检查

- 只有根 `Collision` 组件模拟物理。
- 底盘和四个视觉轮网格保持 `NoCollision`。
- 地面必须阻挡 `Visibility` 通道。
- 旧蓝图若保存过组件参数，需要恢复 C++ 默认值或重新创建子类。
