import os
import subprocess
import trimesh
import tempfile
import argparse


##################################################################################
# 注意！！！ 需要将以下地址改为v-hacd的编译目录
vhcd_dir = "/home/mr-cheng/Documents/Alexbot/ros_ws/src/tools/STL_convert_tool/v-hacd/app/build/"
##################################################################################





def stl_to_obj(stl_path, obj_path):
    mesh = trimesh.load(stl_path)
    mesh.export(obj_path)
    print(f"Converted STL to OBJ: {obj_path}")

def run_vhacd(obj_path, build_dir):
    vhacd_cmd = [
        vhcd_dir + "TestVHACD",
        obj_path,
        "-o", "obj",       # 固定导出为 obj 格式，输出文件名固定为 decomp.obj
        "-e", "0.1",
        "-d", "25",
        "-r", "1000000",
        "-v", "256",
        "--min-volume-per-ch", "0.1"  # 更小凸包粒度
    ]
    print("Running VHACD...")
    subprocess.run(vhacd_cmd, check=True, cwd=build_dir)
    print("VHACD done.")

def split_obj_to_stls(vhacd_obj_path, output_dir):
    scene = trimesh.load(vhacd_obj_path, force='scene')
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    stl_files = []
    for i, geometry_name in enumerate(scene.geometry):
        geom = scene.geometry[geometry_name]
        stl_path = os.path.join(output_dir, f"{base_name}_{i}.stl")
        geom.export(stl_path)
        stl_files.append(stl_path)
        print(f"Exported: {stl_path}")
    return stl_files

def generate_mujoco_asset_file(stl_files, asset_file_path):
    # 确保目录存在
    asset_dir = os.path.dirname(asset_file_path)
    os.makedirs(asset_dir, exist_ok=True)

    with open(asset_file_path, 'w') as f:
        f.write('<mujocoinclude>\n  <asset>\n')
        for i, fpath in enumerate(stl_files):
            filename = os.path.basename(fpath)
            geom_name = f"{base_name}_{i}"
            f.write(f'    <mesh name="{geom_name}" file="{filename}"/>\n')
        f.write('  </asset>\n</mujocoinclude>\n')
        
    print(f"MuJoCo asset file generated: {asset_file_path}")
def generate_geom_lines(stl_files):
    """
    根据所有拆分后的STL文件数，生成对应的 <geom> 标签字符串，
    每个标签绑定对应的 mesh 名称，方便放入MuJoCo的<body>中。
    """
    lines = []
    for i in range(len(stl_files)):
        lines.append(f'<geom type="mesh" mesh="{base_name}_{i}" rgba="1 1 1 0"/>')
    return "\n".join(lines)
    
def main():
    global base_name
    parser = argparse.ArgumentParser(description="STL分解并生成MuJoCo asset xml文件")
    parser.add_argument("input_stl", help="需要处理的STL文件路径")
    args = parser.parse_args()

    input_stl = args.input_stl
    base_dir = os.path.dirname(input_stl)
    base_name = os.path.splitext(os.path.basename(input_stl))[0]

    output_dir = os.path.join(base_dir, f"{base_name}_parts")
    asset_xml_path = os.path.join(output_dir, "assets.xml")

    build_dir = os.path.abspath(vhcd_dir)
    if not os.path.exists(build_dir):
        raise RuntimeError(f"build目录不存在：{build_dir}")

    with tempfile.TemporaryDirectory() as tmpdir:
        tmp_obj = os.path.join(tmpdir, f"{base_name}.obj")

        # 1. STL -> OBJ
        stl_to_obj(input_stl, tmp_obj)

        # 2. 运行 VHACD（cwd切到build目录，输出文件固定decomp.obj）
        run_vhacd(tmp_obj, build_dir)

        vhacd_obj_path = os.path.join(build_dir, "decomp.obj")
        if not os.path.isfile(vhacd_obj_path):
            raise FileNotFoundError(f"VHACD输出文件不存在: {vhacd_obj_path}")

        # 3. 分割 VHACD 输出的OBJ文件，导出STL
        stl_files = split_obj_to_stls(vhacd_obj_path, output_dir)

        # 4. 清理build目录里的decomp.*文件
        for f in os.listdir(build_dir):
            if f.startswith("decomp."):
                os.remove(os.path.join(build_dir, f))

    # 5. 生成MuJoCo asset xml
    generate_mujoco_asset_file(stl_files, asset_xml_path)
    
    # 生成所有 <geom> 标签文本，方便粘贴到MuJoCo模型中
    geom_text = generate_geom_lines(stl_files)
    print("\n将以下文本替换调mujoco中原有的geom:")
    print(geom_text)
    
if __name__ == "__main__":
    main()

