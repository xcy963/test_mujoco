# Robot Description files for OpenArm

This package contains description files to generate OpenArm URDFs (Universal Robot Description Files). See [documentation](https://docs.openarm.dev/software/description) for details.

## Related links

- 📚 Read the [documentation](https://docs.openarm.dev/software/description)
- 💬 Join the community on [Discord](https://discord.gg/FsZaZ4z3We)
- 📬 Contact us through <openarm@enactic.ai>

## License

[Apache License 2.0](LICENSE.txt)

Copyright 2025 Enactic, Inc.

## Code of Conduct

All participation in the OpenArm project is governed by our
[Code of Conduct](CODE_OF_CONDUCT.md).


## 启动的命令

```bash
ros2 launch openarm_description display_openarm.launch.py   arm_type:=v10   ee_type:=openarm_hand   bimanual:=false
```

## 转化出urdf的命令

```bash
xacro src/openarm_description/urdf/robot/v10.urdf.xacro \
  arm_type:=v10 \
  bimanual:=false \
  ros2_control:=true \
  can_interface:=can0 \
  > openarm_single_control.urdf
```