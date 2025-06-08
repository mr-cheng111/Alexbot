import trimesh

mesh = trimesh.load('../decomp.obj', process=False)

# trimesh 会将带有多个group的obj解析成Scene
if isinstance(mesh, trimesh.Scene):
    # mesh.geometry 是一个 dict，每个key对应一个子网格
    for i, geom in enumerate(mesh.geometry.values()):
        geom.export(f'convex_part_{i}.stl')
else:
    # 只有一个网格，直接导出
    mesh.export('convex_part_0.stl')
