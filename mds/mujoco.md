# mujoco的说明

## 关于两个变量的说明（mjModel* model, const mjData* data）
> 第一部分，关于mjModel* model

>一句话介绍他就是包含模拟的静态参数和模型定义： 

```cpp
// 模型维度信息
model->nq;      // 位置坐标数 (广义坐标)
model->nv;      // 速度坐标数 (广义速度)  
model->nu;      // 执行器数量
model->njnt;    // 关节数量
model->nbody;   // 刚体数量
model->nsensor; // 传感器数量

// 几何信息
model->geom_pos;    // 几何体位置 [3*ngeom]
model->geom_size;   // 几何体尺寸 [3*ngeom]
model->geom_type;   // 几何体类型 (球体、立方体、胶囊等)

// 动力学参数
model->body_mass;      // 刚体质量 [nbody]
model->body_inertia;   // 刚体惯性 [3*nbody]
model->jnt_stiffness;  // 关节刚度

// 执行器参数
model->actuator_trntype;  // 执行器传输类型
model->actuator_gear;     // 执行器齿轮比 [nu]

// 接触参数
model->opt;  // 物理选项 (重力、积分器类型等)
```

> 第二部分，关于const mjData* data
```cpp
// 状态变量
data->time;     // 当前模拟时间
data->qpos;     // 位置坐标 [nq]
data->qvel;     // 速度坐标 [nv]  
data->qacc;     // 加速度坐标 [nv]
data->act;      // 执行器激活状态 [na]

// 控制输入
data->ctrl;     // 控制信号 [nu] - 这是您设置控制指令的地方！

// 力和力矩
data->qfrc_actuator;  // 执行器力矩 [nv]
data->qfrc_passive;   // 被动力 (弹簧、阻尼等) [nv]
data->xfrc_applied;   // 施加的外部力 [6*nbody]

// 接触信息
data->ncon;        // 当前接触数量
data->contact;     // 接触信息数组

// 传感器数据
data->sensordata;  // 传感器读数 [nsensordata]

// 中间计算量
data->ten_wrapadr; // 肌腱包装地址
data->ten_wrapnum; // 肌腱包装数量
```

## 使用案例

> **读取状态**
void readRobotState(const mjModel* m, const mjData* d) {
    // 读取关节位置和速度
    for (int i = 0; i < m->njnt; i++) {
        int qpos_adr = m->jnt_qposadr[i];  // 位置地址
        int dof_adr = m->jnt_dofadr[i];    // 自由度地址
        
        double joint_position = d->qpos[qpos_adr];
        double joint_velocity = d->qvel[dof_adr];
        
        std::cout << "Joint " << i << ": pos=" << joint_position 
                  << ", vel=" << joint_velocity << std::endl;
    }
}```cpp

```

> **设置控制输入**

```cpp
void setJointControl(const mjModel* m, mjData* d, 
                    const std::vector<double>& target_positions) {
    for (int i = 0; i < m->nu && i < target_positions.size(); i++) {
        // 简单的PD控制
        int jnt_id = m->jnt_dofadr[i];  // 找到执行器对应的关节
        
        double current_pos = d->qpos[jnt_id];
        double current_vel = d->qvel[jnt_id];
        double target_pos = target_positions[i];
        
        // PD控制律
        double kp = 100.0, kd = 10.0;
        d->ctrl[i] = kp * (target_pos - current_pos) - kd * current_vel;
    }
}
```