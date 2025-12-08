import argparse
import pymeshlab as ml


import os
import pymeshlab as ml

def simplify_mesh(input_path: str, output_path: str, target_faces_rate: float):
    ms = ml.MeshSet()
    print(f"加载模型: {input_path}")
    ms.load_new_mesh(input_path)

    mesh = ms.current_mesh()
    print(f"原始面数: {mesh.face_number()}, 顶点数: {mesh.vertex_number()}")

    target_faces = int(mesh.face_number() * target_faces_rate)

    # 网格简化（降采样），并尽量保持外轮廓
    ms.meshing_decimation_quadric_edge_collapse(
        targetfacenum=target_faces,
        preservenormal=True,
        preserveboundary=True,
        preservetopology=True,
        optimalplacement=True,
        planarquadric=False,
        qualitythr=0.3
    )

    simplified = ms.current_mesh()
    print(f"简化后面数: {simplified.face_number()}, 顶点数: {simplified.vertex_number()}")

    ms.save_current_mesh(output_path)
    print(f"已保存简化模型到: {output_path}")


def simplify_mesh_folder(input_dir: str,
                         output_dir: str,
                         target_faces_rate: float,
                         exts=(".obj", ".ply", ".stl", ".off", ".3ds", ".fbx")):
    """
    对文件夹内所有指定后缀的模型做降采样
    """
    os.makedirs(output_dir, exist_ok=True)

    for filename in os.listdir(input_dir):
        # 只处理指定格式的文件
        if not filename.lower().endswith(exts):
            continue

        input_path = os.path.join(input_dir, filename)
        output_path = os.path.join(output_dir, filename)

        # 如果是文件才处理（排除子文件夹）
        if not os.path.isfile(input_path):
            continue

        print("=" * 60)
        simplify_mesh(input_path, output_path, target_faces_rate)


if __name__ == "__main__":
    # 示例：把 input_models 文件夹中的模型全部降采样，
    # 结果输出到 output_models 文件夹中

    parser = argparse.ArgumentParser(
        description="对三维模型降采样（简化面数），并尽量保持外轮廓不变"
    )
    parser.add_argument("input_path", help="输入模型路径，比如 Link1.STL")
    parser.add_argument("output_path", help="输出模型路径，比如 Link1_simplified.STL")

    parser.add_argument(
        "-f", "--faces",
        type=float,
        required=True,
        help="目标面数比例（例如 0.9）"
    )
    args = parser.parse_args()

    simplify_mesh_folder(
        input_dir=args.input_path,
        output_dir=args.output_path,
        target_faces_rate=args.faces  # 保留 30% 面数
    )


