from IPython import embed
import xml.etree.ElementTree as ET
import rospkg
import os

ros_root = rospkg.get_ros_root()
r = rospkg.RosPack()
origin_path = r.get_path('franka_push_gazebo') + "/worlds/templates/"

# file_name, cog, size, exception index of cog
objects_info = {"quadrangle0.world" : [[0.06, 0], [0.12, 0.12], []],
                "quadrangle1.world" : [[0.06, 0], [0.12, 0.24], []],
                #"triangle0.world" : []
                #"triangle1.world" :,
                #"triangle1.world" :,
                #"circle0.world" :
}

for f in objects_info:
    cx, cy = objects_info[f][0]
    lx, ly = objects_info[f][1]
    exc = objects_info[f][2]

    file_path = origin_path + f
    object_name = f.split(".world")[0]

    tree = ET.parse(file_path)
    root = tree.getroot()

    link = root[0][0][0]

    for d_cx in range(2):
        for d_cy in range(-1, 2, 1):
            if [d_cx, d_cy] in exc:
                continue

            cog_x = -cx + (d_cx + 1) * lx / 3
            cog_y = -cy + d_cy * ly / 2

            # cog
            pose = link[0][0]
            pose.text = "{} {} 0 0 0 0".format(cog_x, cog_y)

            # collision shape
            collision_mesh_uri = link[1][0][0][0]
            collision_mesh_uri.text = "file://../models/" + object_name + "_{}_{} .stl".format(cog_x, cog_y)

            # visual shape
            visual_mesh_uri = link[2][0][0][0]
            visual_mesh_uri.text = "file://../models/" + object_name + "_{}_{} .stl".format(cog_x, cog_y)

            # output
            d = origin_path + "../" + object_name + "/"
            try:
                os.mkdir(d)
            except:
                pass
            output_file = d + object_name + "_{}_{} .world".format(cog_x, cog_y)
            tree.write(output_file)
