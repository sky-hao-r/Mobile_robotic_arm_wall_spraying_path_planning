import open3d as o3d
import numpy as np
import math
import matplotlib.pyplot as plt
import time
import copy

class planePointSet: #结构体
    def __init__(self):
        self.inlier_cloud = 0 # 平面包含的点
        self.inlier_cloudToPlane = o3d.geometry.PointCloud() # 存储投影到平面的点云
        self.cloudPointEdge = o3d.geometry.PointCloud() # 存储平面边缘点云
        self.planes_model_params = 0 # 平面参数 
        self.pointNum=0 # 平面包含的点云数量
        self.cosWithTargetVec=0
    def pointCnt(self):
        self.pointNum=len(self.inlier_cloud.points)
        return self.pointNum
    def pointProjectToPlane(self):
        projected_points = []
        for point in self.inlier_cloud.points:
            x, y, z = point
            distance = (self.planes_model_params[0] * x + self.planes_model_params[1] * y + self.planes_model_params[2] * z + self.planes_model_params[3]) / np.sqrt(self.planes_model_params[0]**2 + self.planes_model_params[1]**2 + self.planes_model_params[2]**2) 
            projected_point = [x - distance * self.planes_model_params[0], y - distance * self.planes_model_params[1], z - distance * self.planes_model_params[2]]
            projected_points.append(projected_point)
        # 创建投影后的点云
        self.inlier_cloudToPlane.points = o3d.utility.Vector3dVector(projected_points)
    def getEdgeCloudPoint(self):
        self.inlier_cloudToPlane.paint_uniform_color([0.5, 0.5, 0.5])
        pcd_tree = o3d.geometry.KDTreeFlann(self.inlier_cloudToPlane)

        cloudPointEdgeList = []
        for point in self.inlier_cloudToPlane.points:
            theta_deg=[]
            [k, idx, _] = pcd_tree.search_radius_vector_3d(point, 0.1) # k为邻近点个数 包含目标点
            # np.asarray(plane_selected.inlier_cloudToPlane.colors)[idx[1:], :] = [0, 1, 0]
            # o3d.visualization.draw_geometries([plane_selected.inlier_cloudToPlane], window_name="k近邻点")
            if k<3: # 目标点只有一个邻近点
                break
            else:
                vec_base=plane_selected.inlier_cloudToPlane.points[idx[1]]-plane_selected.inlier_cloudToPlane.points[idx[0]]
                vec_temp_cal_dir_refer=plane_selected.inlier_cloudToPlane.points[idx[2]]-plane_selected.inlier_cloudToPlane.points[idx[0]]
                dir_refer=np.cross(vec_base,vec_temp_cal_dir_refer)  # 用于控制向量夹角在0-2pi之间 将原点到第一个邻近点的向量转向原点到第二个邻近点的向量的方向设为基准方向
            
            # 原始程序-for循环计算
            # for i in idx[2:]:
            #     point_near=plane_selected.inlier_cloudToPlane.points[i] # 选择邻近点
            #     vec_temp=point_near-plane_selected.inlier_cloudToPlane.points[idx[0]] # 计算邻近点与原点向量
            #     theta_rad = calTheta_rad(vec_temp,vec_base) # 计算两向量之间的夹角
            #     # 将角度控制在0-2pi之间
            #     if k>3:
            #         dir_cur=np.cross(vec_base,vec_temp)
            #         if np.dot(dir_cur, dir_refer)<0:
            #             theta_rad+=math.pi
            #     theta_deg.append(theta_rad)
                # print(i,point_near)

            # 矩阵计算加速 13s变为1.3秒
            point_near=plane_selected.inlier_cloudToPlane.select_by_index(idx[2:])
            vec_temp=point_near.points-plane_selected.inlier_cloudToPlane.points[idx[0]]
            theta_rad = calTheta_rad(vec_temp,vec_base) # 计算两向量之间的夹角
            if k>3:
                dir_cur=np.cross(vec_base,vec_temp)
                dot_products = np.dot(dir_cur, dir_refer)
                positive_indices = np.where(dot_products < 0)
                theta_rad[positive_indices]+=+math.pi
                theta_deg=theta_rad
            theta_deg=theta_deg.tolist()

            # print(theta_deg)
            theta_deg.sort() # 将夹角按照从小到达排序
            # print(theta_deg)
            theta_deg.insert(0, 0) # 在列表最前端插入0
            theta_deg.append(2*math.pi) # 在列表最后端插入2*pi
            # print(theta_deg)
            
            # 原始程序
            theta_Difference=[]
            for theta_Difference_num in range(len(theta_deg)-1):
                theta_Difference.append(theta_deg[theta_Difference_num+1]-theta_deg[theta_Difference_num]) # 计算相邻两角差值，但是第一个是theta到0的差值，最后一个是2pi到最后一个theta的差值，所以第一个与最后一个角之间的差值应当为列表首尾之和
                      
            theta_Difference[0]=theta_Difference[0]+theta_Difference.pop() # 将列表最后一个值删除并加到第一个值上
            theta_Difference_max = max(theta_Difference)
            # print(theta_Difference_max)
            if theta_Difference_max>math.pi/3:
                pointToList=point.tolist()
                cloudPointEdgeList.append(pointToList)

        # print(cloudPointEdgeList)
        self.cloudPointEdge.points = o3d.utility.Vector3dVector(cloudPointEdgeList)

def pointCloudFrameTrans(pointCloud_input,initial_position,initial_direction,target_direction):
    # initial_position点云初始位置（重心替代））
    target_pos=np.array([0, 0, 0]) #先平移到原点
    pointCloud_input.translate(target_pos - initial_position)
    # o3d.visualization.draw_geometries([pcd_temp,plane_selected.inlier_cloud,mesh_frame], window_name="目标平面点")
    rotation_matrix = calRotationMatrix(initial_direction,target_direction)
    center_point = np.array([0, 0, 0])
    pointCloud_input.rotate(rotation_matrix, center=center_point)
    pointCloud_input.translate(initial_position-target_pos)
    return pointCloud_input

# 计算cos
def calCosine(vec1, vec2):
    dot_product = np.dot(vec1, vec2)
    if vec1.ndim>1:
        norm1 = np.linalg.norm(vec1, axis=1)
    else:
        norm1 = np.linalg.norm(vec1)
    norm2 = np.linalg.norm(vec2)
    return dot_product / (norm1 * norm2)

def calTheta_rad(vec1,vec2):
    cos_theta=calCosine(vec1, vec2)
    theta_rad = np.arccos(cos_theta)
    return theta_rad

def calRotationMatrix(initial_direction,target_direction):
    # 计算旋转轴
    rotation_axis = np.cross(initial_direction, target_direction)
    rotation_axis /= np.linalg.norm(rotation_axis)

    # 计算旋转角度（弧度）
    cos_theta = np.dot(initial_direction, target_direction)
    sin_theta = np.sqrt(1 - cos_theta**2)

    rotation_matrix = np.array([
    [cos_theta + rotation_axis[0]**2 * (1 - cos_theta), 
     rotation_axis[0] * rotation_axis[1] * (1 - cos_theta) - rotation_axis[2] * sin_theta,
     rotation_axis[0] * rotation_axis[2] * (1 - cos_theta) + rotation_axis[1] * sin_theta],
    
    [rotation_axis[1] * rotation_axis[0] * (1 - cos_theta) + rotation_axis[2] * sin_theta,
     cos_theta + rotation_axis[1]**2 * (1 - cos_theta),
     rotation_axis[1] * rotation_axis[2] * (1 - cos_theta) - rotation_axis[0] * sin_theta],
    
    [rotation_axis[2] * rotation_axis[0] * (1 - cos_theta) - rotation_axis[1] * sin_theta,
     rotation_axis[2] * rotation_axis[1] * (1 - cos_theta) + rotation_axis[0] * sin_theta,
     cos_theta + rotation_axis[2]**2 * (1 - cos_theta)]])
    return rotation_matrix
    
def create_arrow_normal_plane(plane_model,inlier_cloud):
    # 生成箭头 默认方向为[0,0,1]
    arrow = o3d.geometry.TriangleMesh.create_arrow(cylinder_radius=0.05,
                                                cone_radius=0.07,
                                                cylinder_height=0.5,
                                                cone_height=0.4,
                                                resolution=20,
                                                cylinder_split=4,
                                                cone_split=1)
    arrow.compute_vertex_normals()
    arrow.paint_uniform_color([1, 0, 0])
    # o3d.visualization.draw_geometries([arrow])
    # 箭头旋转至平面法相
    initial_direction=np.array([0,0,1])
    target_direction=np.array([plane_model[0],plane_model[1],plane_model[2]])
    rotation_matrix = calRotationMatrix(initial_direction,target_direction)
    start_point = np.array([0, 0, 0])
    arrow.rotate(rotation_matrix, center=start_point)
    # 箭头起点平移至平面重心
    points_tmp = np.asarray(inlier_cloud.points)
    center_of_mass = np.mean(points_tmp, axis=0)
    arrow.translate(center_of_mass - start_point)
    return arrow

# 冒泡排序
def bubble_sort(lst):
    n = len(lst)
    for j in range(0,n-1):
        count = 0
        for i in range(0, n - j - 1):
            if lst[i].pointNum < lst[i+1].pointNum:
                lst[i], lst[i+1] = lst[i+1], lst[i]
    return lst

# 选择位于顶层平面中的点
def plane_segment(point_cloud):
    # RANSAC算法参数设置
    distance_threshold = 0.045 # Max distance a point can be from the plane model 0.25 更好 0.3 更容易看出效果
    ransac_n = 3 # Number of initial points to be considered inliers in each iteration.
    num_iterations = 500
    plane_list=[] # 用于存储平面数据
    index_plane=0 # 用于记录平面数量
    # RANSAC算法依次拟合各个平面
    while len(point_cloud.points) > 0:
        plane_model, inliers = o3d.geometry.PointCloud.segment_plane(
            point_cloud,
            distance_threshold=distance_threshold,
            ransac_n=ransac_n,
            num_iterations=num_iterations
        )
        # plane_model：平面模型，即个平面方程系数（a,b,c,d），平面ax+by+cz+d=0
        inlier_cloud = point_cloud.select_by_index(inliers) # 将属于平面的点提取出来 
        if len(inlier_cloud.points)>2000:
            plane_list.append(planePointSet()) # 新建点云平面类
            plane_list[index_plane].inlier_cloud=inlier_cloud # 保存平面包含的点云
            plane_list[index_plane].planes_model_params=plane_model # 保存平面参数
            index_plane+=1
        point_cloud = point_cloud.select_by_index(inliers, invert=True) # 从原始点云中移除属于平面的点
        if len(point_cloud.points)<ransac_n: #当点数小于平面最小点时跳出循环
            break
    # 计算各个点云平面的平均距离,并选择平均距离最小者
    print("->共拟合了",index_plane,"个有效平面")
    # print(type(planes[0])) # <class 'open3d.cpu.pybind.geometry.PointCloud'>
    bubble_sort(plane_list) # 冒泡排序 按照包含点的数量从多到少排列
    point_cnt = [] # 用于存储平面点云中点的数量
    for i in range(index_plane):
        point_cnt.append(plane_list[i].pointCnt())
    print("->各平面中点数量为",point_cnt)
    return plane_list

def target_plane_select(reference_vec, plane_list):
    cos=[]
    for i in range(len(plane_list)):
        normal_vec=np.array([plane_list[i].planes_model_params[0],plane_list[i].planes_model_params[1],plane_list[i].planes_model_params[2]])
        plane_list[i].cosWithTargetVec=calCosine(reference_vec,normal_vec)
        cos.append(abs(plane_list[i].cosWithTargetVec))
    
    print("各平面与相机坐标系x轴间的余弦值为:",cos)
    max_value = max(cos)
    max_index = cos.index(max_value)
    return plane_list[max_index]

if __name__ == '__main__':
    
    print("-------------step 1:加载点云---------------")
    print("->正在加载点云... ")
    dataset="accumulated_cloud.pcd"
    point_cloud_pcd = o3d.io.read_point_cloud(dataset) #点云读取
    mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.3) # 显示坐标系
    o3d.visualization.draw_geometries([point_cloud_pcd,mesh_frame], window_name="原始点云数据")
    print("->点云加载成功:",point_cloud_pcd)
    points = np.asarray(point_cloud_pcd.points)
    print("->原始点云的个数为:",points.shape) 

    print("-------------step 2:点云直通滤波---------------")
    # print(np.shape(np.where(points[:, 0] > 5)))
    point_cloud_pcd = point_cloud_pcd.select_by_index(np.where(points[:, 0] < 6)[0])
    o3d.visualization.draw_geometries([point_cloud_pcd,mesh_frame], window_name="直通滤波后点云数据")
    points = np.asarray(point_cloud_pcd.points)
    print("->直通滤波后点云的个数为:",points.shape) 
    o3d.io.write_point_cloud("cloudPointEdge_zhitong.pcd", point_cloud_pcd)

    print("-------------step 3:使用5厘米的体素下采样点云---------------")
    downpcd = point_cloud_pcd.voxel_down_sample(voxel_size=0.025)
    o3d.visualization.draw_geometries([downpcd,mesh_frame], window_name="体素下采样后点云")
    points = np.asarray(downpcd.points)
    print("->体素下采样点云的个数为:",points.shape) 

    print("-------------step 4:统计滤波去除离群点（仿真不需要）---------------")
    # 统计滤波
    # k = 20  # K邻域点的个数
    # μ = 2.0  # 标准差乘数
    # sor_pcd, idx = downpcd.remove_statistical_outlier(k, μ)#当判断点的k近邻的平均距离大于【平均距离+μ*σ】，即判定为噪声点，一般取μ=2或3为极限误差
    # sor_pcd.paint_uniform_color([0, 0, 1])
    # # 提取噪声点云
    # sor_noise_pcd = downpcd.select_by_index(idx, invert=True)
    # sor_noise_pcd.paint_uniform_color([1, 0, 0])
    # o3d.visualization.draw_geometries([sor_pcd,sor_noise_pcd], window_name="SOR")

    print("-------------step 5:目标喷涂墙面选择-----------")
    # 平面分割
    plane_list=plane_segment(point_cloud=downpcd) 
    # for i in range(len(plane_list)):
    #     o3d.visualization.draw_geometries([plane_list[i].inlier_cloud], window_name="平面"+str(i))

    # 显示法向量
    cloud_show=[]
    for i in range(len(plane_list)):
        arrow=create_arrow_normal_plane(plane_list[i].planes_model_params,plane_list[i].inlier_cloud)
        cloud_show.append(mesh_frame)
        cloud_show.append(plane_list[i].inlier_cloud)
        cloud_show.append(arrow)
    # o3d.visualization.draw_geometries(cloud_show, window_name="平面"+str(i))
    # 目标平面选择
    reference_vec=np.array([1,0,0])
    plane_selected = target_plane_select(reference_vec, plane_list)
    o3d.visualization.draw_geometries([plane_selected.inlier_cloud], window_name="目标平面点")
   
    print("-------------step 6:将分割的点云投影到拟合平面--------------")
    # print(type(plane_selected))
    plane_selected.pointProjectToPlane()
    o3d.visualization.draw_geometries([plane_selected.inlier_cloudToPlane], window_name="目标平面点投影到平面")
    o3d.io.write_point_cloud("inlier_cloudToPlane.pcd", plane_selected.inlier_cloudToPlane)
    # print('坐标系矫正前')
    # print(np.asarray(plane_selected.inlier_cloudToPlane.points)[1:10])

    print("-------------step 6.5:将点云平面法方向旋转到x轴重合方向-----------")
    pcd_temp=copy.deepcopy(plane_selected.inlier_cloudToPlane) # 原始点云 用于比较变化
    wall_initial_position = np.mean(np.asarray(plane_selected.inlier_cloudToPlane.points), axis=0)
    wall_initial_direction=np.array([plane_selected.planes_model_params[0],plane_selected.planes_model_params[1],plane_selected.planes_model_params[2]])
    target_direction=np.array([1,0,0])
    plane_selected.inlier_cloudToPlane=pointCloudFrameTrans(plane_selected.inlier_cloudToPlane,wall_initial_position,wall_initial_direction,target_direction)
    o3d.visualization.draw_geometries([pcd_temp,plane_selected.inlier_cloudToPlane,mesh_frame], window_name="目标平面点")
    # print('坐标系矫正后')
    # print(np.asarray(plane_selected.inlier_cloudToPlane.points)[1:10])

    # # 将坐标系矫正后的平面还原到原始位置
    # wall_initial_direction=np.array([1,0,0])
    # target_direction=np.array([plane_selected.planes_model_params[0],plane_selected.planes_model_params[1],plane_selected.planes_model_params[2]])
    # plane_selected.inlier_cloudToPlane=pointCloudFrameTrans(plane_selected.inlier_cloudToPlane,wall_initial_position,wall_initial_direction,target_direction)
    # o3d.visualization.draw_geometries([pcd_temp,plane_selected.inlier_cloudToPlane,mesh_frame], window_name="目标平面点")
    # print('坐标系恢复后')
    # print(np.asarray(plane_selected.inlier_cloudToPlane.points)[1:10])

    print("-------------step 7:平面点云的边界特征提取--------------")
    time_start = start = time.perf_counter()
    plane_selected.getEdgeCloudPoint()
    time_end = start = time.perf_counter()
    run_time = time_end - time_start
    print("平面点云的边界特征提取运行时间：", run_time, "秒")
    o3d.visualization.draw_geometries([plane_selected.cloudPointEdge], window_name="目标平面点投影到平面")
    o3d.io.write_point_cloud("cloudPointEdge.pcd", plane_selected.cloudPointEdge)
    # print(np.asarray(plane_selected.cloudPointEdge.points)[1:10])

    # # print("-------------step 8:点云聚类--------------")
    # # # 执行DBSCAN聚类
    # # labels = np.array(plane_selected.cloudPointEdge.cluster_dbscan(eps=0.2, min_points=10))
    # # # 获取聚类的最大标签值
    # # max_label = labels.max()
    # # # 创建一个颜色映射以可视化每个聚类
    # # colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
    # # # 将颜色数据添加到点云中
    # # plane_selected.cloudPointEdge.colors = o3d.utility.Vector3dVector(colors[:, :3])
    # # # 可视化聚类结果
    # # o3d.visualization.draw_geometries([plane_selected.cloudPointEdge])
    


    # print("-------------step 8:点云图转深度图--------------")
    loadEdgePcd=1
    if loadEdgePcd==1:
        file_name="cloudPointEdge.pcd"
        PlaneEdge = o3d.io.read_point_cloud(file_name) #点云读取
        o3d.visualization.draw_geometries([PlaneEdge], window_name="点云平面边缘")
    else :
        PlaneEdge=plane_selected.cloudPointEdge
        o3d.visualization.draw_geometries([PlaneEdge], window_name="点云平面边缘")
    print(np.asarray(PlaneEdge.points)[1:10])


    

    # # 提取点云中的坐标数据
    # points = np.asarray(plane_selected.cloudPointEdge)

    # # 计算点云数据的边界框
    # min_bound = points.min(axis=0)
    # max_bound = points.max(axis=0)

    # # 设置相机参数
    # camera_params = o3d.camera.PinholeCameraIntrinsic()
    # camera_params.set_intrinsics(640, 480, 525.0, 525.0, 319.5, 239.5)

    # # 创建深度图像
    # depth_image = o3d.geometry.PointCloud.create_from_depth_map(points, camera_params, min_bound, max_bound)

    # # 将深度图像保存为文件
    # o3d.io.write_image("depth_image.png", depth_image)







# import open3d as o3d
# import numpy as np

# # 从文件加载点云数据
# pcd = o3d.io.read_point_cloud("your_point_cloud_file.pcd")

# # 转换点云为NumPy数组
# points = np.asarray(pcd.points)

# # 定义分割线段的阈值
# segmentation_threshold = 0.05  # 根据点之间的距离来分割线段

# # 初始化存储线段的列表
# line_segments = []

# while len(points) > 0:
#     # 分割线段
#     segment_indices = [0]  # 起始索引
#     for i in range(1, len(points)):
#         if np.linalg.norm(points[i] - points[i - 1]) > segmentation_threshold:
#             segment_indices.append(i)
    
#     # 如果线段长度大于某个阈值，就拟合线段
#     if len(segment_indices) > 2:
#         segment_points = points[segment_indices]
        
#         # 使用拟合算法（例如RANSAC）拟合线段
#         line_model = your_fitting_algorithm(segment_points)  # 请替换成适用的拟合算法
        
#         # 将线段添加到列表中
#         line_segments.append(line_model)
    
#     # 从点云中删除已处理的点
#     points = np.delete(points, segment_indices, axis=0)

# # 合并相似线段（根据需要）
# # 请根据您的应用场景和线段合并规则来实现合并操作

# # 可视化线段
# for line_model in line_segments:
#     line_set = o3d.geometry.LineSet()
#     line_set.lines = o3d.utility.Vector2iVector([[0, 1]])
#     line_set.points = o3d.utility.Vector3dVector(line_model)  # 这里假设您的线段模型返回点的坐标
#     o3d.visualization.draw_geometries([line_set])
