import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import cv2
from scipy.spatial import ConvexHull
import copy
import time

dataset="inlier_cloudToPlane.pcd"
point_cloud_pcd = o3d.io.read_point_cloud(dataset) #点云读取
# o3d.visualization.draw_geometries([point_cloud_pcd], window_name="目标平面点投影到平面")

print("-------------step 1:将平面点云转换为二维图像--------------")
# 3D点云数据，假设这是一个Nx3的NumPy数组，每一行代表一个点的XYZ坐标
point_cloud = np.asarray(point_cloud_pcd.points)
# print(point_cloud)

# 相机内参
fx = 500.0  # 焦距x
fy = 500.0  # 焦距y
cx = 320  # 主点x
cy = 240  # 主点y

# 创建深度图像大小
image_width = 640
image_height = 480
depth_map = np.zeros((image_height, image_width), dtype=np.uint8)

# 遍历每个点并将其投影到深度图上
for point in point_cloud:
    z, x, y = point
    if z > 0:
        u = image_width-int((fx * x / z) + cx)
        v = image_height-int((fy * y / z) + cy)

        # 检查点是否在图像范围内
        if 0 <= u < image_width and 0 <= v < image_height:
            # 将深度值存储在深度图中
            depth_map[v, u] = 255

# 将深度图缩放到0-255范围，以便保存为图像
# scaled_depth_map = ((depth_map - depth_map.min()) / (depth_map.max() - depth_map.min()) * 255).astype(np.uint8)

cv2.imshow("depth_map", depth_map)
cv2.waitKey(0)

kernel = np.ones(shape=[3,3],dtype=np.uint8)
depth_map_dilate = cv2.dilate(depth_map, kernel, iterations=2)
cv2.imshow("depth_map_dilate", depth_map_dilate)
cv2.waitKey(0)

kernel = np.ones(shape=[3,3],dtype=np.uint8)
depth_map_erode = cv2.erode(depth_map_dilate,kernel=kernel, iterations=1)  # 腐蚀操作
cv2.imshow("depth_map_erode", depth_map_erode)
cv2.waitKey(0)

print("-------------step 2:Graham算法凸包检测--------------")
points = np.column_stack(np.where(depth_map_erode == 255))
print(np.shape(points)) # 先纵坐标再横坐标

hull = ConvexHull(points)
# # # 绘制凸包
# plt.plot(image_height-points[:, 1], image_width-points[:, 0], 'o')
# for simplex in hull.vertices:
#     plt.plot(image_height-points[simplex, 1], image_width-points[simplex, 0], 'ro')
# for simplex in hull.simplices:
#     plt.plot(image_height-points[simplex, 1], image_width-points[simplex, 0], 'y-')
# plt.show()

img_rgb = np.zeros((image_height, image_width, 3), np.uint8) # 新建三通道彩色图片
for (x, y) in points:
    img_rgb[x,y]=[255,255,255]
cv2.imshow("Image_rgb", img_rgb)
cv2.waitKey(0)

# 显示边界点
for vertices in hull.vertices:
    img_rgb[points[vertices, 0],points[vertices, 1]]=[0,0,255]
cv2.imshow("Image_rgb_edge", img_rgb)
cv2.waitKey(0)

edge_points=points[hull.vertices]
edge_points_new = copy.deepcopy(edge_points)
edge_points_new[:,1] = edge_points[:,0]
edge_points_new[:,0] = edge_points[:,1]
'''
print(np.shape(edge_points))
cv2.polylines(img_rgb,[edge_points_new],isClosed=True,color=(0,0,255),thickness=3)
# for num in range(len(hull.vertices)-1):
#     cv2.line(img_rgb, (points[ hull.vertices[num], 1],points[hull.vertices[num], 0])
#              , (points[hull.vertices[num+1], 1],points[hull.vertices[num+1], 0]), [0,0,255],thickness=3)
# cv2.line(img_rgb, (points[ hull.vertices[-1], 1],points[hull.vertices[-1], 0])
#              , (points[hull.vertices[0], 1],points[hull.vertices[0], 0]), [0,0,255],thickness=3)
cv2.imshow("Image_rgb_edge", img_rgb)
cv2.waitKey(0)
'''

print("边界点",points[hull.vertices])

# 多边形填充 后期可以根据墙面凸起 凹陷（距离判断可以到达或不可到达点）
cv2.fillConvexPoly(img_rgb, edge_points_new, (255, 0, 0))
cv2.imshow("Image_rgb_edge", img_rgb)
cv2.waitKey(0)

# 创建掩码，其中颜色为[0, 0, 0]的像素为白色，其他为黑色
color_to_extract = [0, 0, 0]
lower_color = np.array(color_to_extract, dtype=np.uint8)
upper_color = np.array(color_to_extract, dtype=np.uint8)

color_mask = cv2.inRange(img_rgb, lower_color, upper_color)

# 查找白色像素的坐标
points_unreachable = np.column_stack(np.where(color_mask > 0))
for (x, y) in points_unreachable:
    img_rgb[x,y]=[0,255,0]
# cv2.imshow("Image_rgb_edge", img_rgb)
# cv2.waitKey(0)

# 由于凸包计算，会出现少量毛边并为被记录
img_rgb_final = np.zeros((image_height, image_width, 3), np.uint8) # 新建三通道彩色图片
for (x, y) in points_unreachable:
    img_rgb_final[x,y]=[0,255,0]
for (x, y) in points:
    img_rgb_final[x,y]=[0,0,255]
cv2.imshow("Image_rgb_edge", img_rgb_final) 
cv2.waitKey(0)

map_value = np.zeros((image_height, image_width), dtype=np.uint8)
map_value = map_value+127  # 可到达地区为127
for (x, y) in points_unreachable:
    map_value[x,y]=0 # 不可达为0
for (x, y) in points:
    map_value[x,y]=255 # 需要喷涂为255
cv2.imshow("Image_rgb_edge", map_value) 
cv2.waitKey(0)

# 确定栅格地图的大小和栅格大小
map_width = map_value.shape[1]
map_height = map_value.shape[0]
grid_width = 15  # 定义每个栅格的大小，根据需要调整
grid_hieght = 15
# 计算栅格地图的行数和列数
num_rows = map_height // grid_width # 向下取整
num_cols = map_width // grid_hieght 

# 创建一个空的二维栅格地图
grid_map = np.zeros((num_rows, num_cols), dtype=np.uint8)

# 分别计算每个栅格的平均灰度值
for row in range(num_rows):
    for col in range(num_cols):
        grid = map_value[row * grid_hieght:(row + 1) * grid_hieght, col * grid_width:(col + 1) * grid_width]
        # 要统计的元素
        target_element = 1
        # 使用条件索引来查找符合条件的元素
        count_unreachable = np.count_nonzero(grid == 0)
        count_needtopaint = np.count_nonzero(grid == 255)
        count_reachable = np.count_nonzero(grid == 127)
        if count_unreachable > grid_width * grid_hieght/3 :
            grid_map[row, col] = 0
        elif count_needtopaint>grid_width * grid_hieght/9: #grid_width * grid_hieght/6
            grid_map[row, col] = 255
        else:
            grid_map[row, col] = 127

        # average_value = np.mean(grid)
        # grid_map[row, col] = int(average_value)

# 现在，grid_map 包含了每个栅格的平均灰度值，您可以根据这些值进一步分析或可视化地图。

cv2.imshow("Image_rgb_edge", grid_map) 
cv2.waitKey(0)

# 用于展示的grid_map
grid_map_show = np.zeros((num_rows*grid_width, num_cols*grid_hieght,3), dtype=np.uint8)
# 分别计算每个栅格的平均灰度值
for row in range(num_rows):
    for col in range(num_cols):
        grid_map_show[row * grid_hieght:(row + 1) * grid_hieght, col * grid_width:(col + 1) * grid_width]=grid_map[row, col]*np.array([1,1,1])
for row in range(num_rows):
    cv2.line(grid_map_show, [0,row*grid_hieght], [num_cols*grid_hieght,row*grid_hieght], [100,100,100], 1)
for col in range(num_cols):
    cv2.line(grid_map_show, [col*grid_width,0], [col*grid_width,num_cols*grid_width], [100,100,100], 1)

cv2.imshow("Image_rgb_edge", grid_map_show) 
cv2.waitKey(0)

# 用于计算的栅格地图
E_val=100
grid_map_cal=np.zeros((num_rows, num_cols))
# 不可到达点
points_temp_unreachable = np.column_stack(np.where(grid_map == 0))
for (x, y) in points_temp_unreachable:
    grid_map_cal[x,y]=-E_val # 不可达为0
# 未遍历点
points_temp_needtopaint = np.column_stack(np.where(grid_map == 255))
for (x, y) in points_temp_needtopaint:
    grid_map_cal[x,y]=E_val 
# 可跨越点 或者 已经喷涂点
points_temp_finished = np.column_stack(np.where(grid_map == 127))
for (x, y) in points_temp_finished:
    grid_map_cal[x,y]=0 

# 修正量矩阵
# 优先横着走
# matrix_correct=np.array([[0,30,0],
#                          [70,0,90],
#                          [0,50,0]])
# 优先竖着走
matrix_correct=np.array([[0,70,0],
                         [30,0,50],
                         [0,90,0]])

position_last=0
position_now=0
position_next=0

# 选择起始点 左上角
startpoint=0
x_min = min(points_temp_needtopaint[0,:])
print(x_min)
points_x_min=[]
for (y, x) in points_temp_needtopaint:
    if x==x_min:
        points_x_min.append([y,x])
print(points_x_min)
y_max = min(points_temp_needtopaint[:,0])
for (y, x) in points_x_min:
    if y==y_max:
        startpoint=[y,x]
print(startpoint)

fill_grid_on=0
if fill_grid_on==1:
    grid_map_show[startpoint[0] * grid_hieght:(startpoint[0] + 1) * grid_hieght, startpoint[1] * grid_width:(startpoint[1] + 1) * grid_width]=np.array([0,0,255])
cv2.imshow("Image_rgb_edge", grid_map_show) 
cv2.waitKey(0)


# 开始路径规划
position_last=startpoint
position_now=startpoint
# grid_map_cal[startpoint[0],startpoint[1]]=0

point_wait_to_paint=np.column_stack(np.where(grid_map_cal == E_val))
print("初始形状为",np.shape(point_wait_to_paint))
print(np.shape(point_wait_to_paint)[0])

while(1):
    # for i in range(10):
    while(1):
        # 原始栅格值
        grid_near_val=np.array([[grid_map_cal[position_now[0]-1,position_now[1]-1],grid_map_cal[position_now[0]-1,position_now[1]],grid_map_cal[position_now[0]-1,position_now[1]+1]],
                    [grid_map_cal[position_now[0],position_now[1]-1],grid_map_cal[position_now[0],position_now[1]],grid_map_cal[position_now[0],position_now[1]+1]],
                    [grid_map_cal[position_now[0]+1,position_now[1]-1],grid_map_cal[position_now[0]+1,position_now[1]],grid_map_cal[position_now[0]+1,position_now[1]+1]]])
        # grid_near_val=np.array([[grid_map_cal[position_now[0]-1,position_now[1]-1],grid_map_cal[position_now[0],position_now[1]-1],grid_map_cal[position_now[0]+1,position_now[1]-1]],
        #                 [grid_map_cal[position_now[0]-1,position_now[1]],grid_map_cal[position_now[0],position_now[1]],grid_map_cal[position_now[0]+1,position_now[1]]],
        #                 [grid_map_cal[position_now[0]-1,position_now[1]+1],grid_map_cal[position_now[0],position_now[1]+1],grid_map_cal[position_now[0]+1,position_now[1]+1]]])
        # grid_near_val=np.array([[grid_map_cal[position_now[1]-1,position_now[0]-1],grid_map_cal[position_now[1],position_now[0]-1],grid_map_cal[position_now[1]+1,position_now[0]-1]],
        #                 [grid_map_cal[position_now[1]-1,position_now[0]],grid_map_cal[position_now[1],position_now[0]],grid_map_cal[position_now[1]+1,position_now[0]]],
        #                 [grid_map_cal[position_now[1]-1,position_now[0]+1],grid_map_cal[position_now[1],position_now[0]+1],grid_map_cal[position_now[1]+1,position_now[0]+1]]])
        print(grid_near_val)
        grid_near_val_cal=grid_near_val+matrix_correct # 活性修正后
        print(grid_near_val_cal)
        E_max = grid_near_val_cal.max()
        if E_max<E_val:
            grid_map_cal[position_now[0],position_now[1]]=0
            break
        print(E_max)
        target_best=np.column_stack(np.where(grid_near_val_cal == E_max))
        target_best=target_best.reshape(2)
        print(target_best)
        position_next=[position_now[0]+target_best[0]-1,position_now[1]+target_best[1]-1]
        print("position_next",position_next)
        if fill_grid_on==1:
            grid_map_show[position_now[0] * grid_hieght:(position_now[0] + 1) * grid_hieght, position_now[1] * grid_width:(position_now[1] + 1) * grid_width]=np.array([0,0,255])

        cv2.line(grid_map_show, [position_now[1] * grid_hieght+int(grid_hieght/2),position_now[0] * grid_width+int(grid_width/2)],  
                 [position_next[1] * grid_hieght+int(grid_hieght/2),position_next[0] * grid_width+int(grid_width/2)], [0,255,0], 3)

        cv2.imshow("Image_rgb_edge", grid_map_show)
        # # time.sleep(0.1)
        cv2.waitKey(0)
        
        position_last=position_now
        grid_map_cal[position_now[0],position_now[1]]=0
        position_now=position_next
        
        print("position_last",position_last)
        print("position_now",position_now)

    point_wait_to_paint=np.column_stack(np.where(grid_map_cal == E_val))
    print("最终形状为",np.shape(point_wait_to_paint))

    dis_list=[]
    if np.shape(point_wait_to_paint)[0]>0:
        for point_wait_to_paint_temp in point_wait_to_paint:
            distance = np.linalg.norm(point_wait_to_paint_temp - position_now)
            dis_list.append(distance)
        # 找到最大元素的值
        max_value = min(dis_list)
        # 找到最大元素的下标
        max_index = dis_list.index(max_value)
        cv2.line(grid_map_show, [position_now[1] * grid_hieght+int(grid_hieght/2),position_now[0] * grid_width+int(grid_width/2)],  
                 [point_wait_to_paint[max_index][1] * grid_hieght+int(grid_hieght/2),point_wait_to_paint[max_index][0] * grid_width+int(grid_width/2)], [0,0,255], 2)
        position_now=point_wait_to_paint[max_index]
        grid_map_cal[position_now[0],position_now[1]]=0
    else:
        print("路径规划完成")
        break

# points_x_min = np.column_stack(np.where(points_temp_needtopaint == 255))
# y_max = 
# print(min_x_point)

# 保存地图为图像
# cv2.imwrite('grid_map.png', grid_map)

# 打印点的坐标
# for point in points:
#     print("X:", point[1], "Y:", point[0])
# for (x, y) in points_unreachable:
#     img_rgb[x,y]=[0,255,0]
# cv2.imshow("Image_rgb_edge", img_rgb)
# cv2.waitKey(0)

# print(hull.vertices)
# print(np.shape(points))

# hull.vertices
# vertices：ndarray of ints, shape (nvertices,)
# 形成凸包的顶点的点的索引。对于二维凸包，顶点按逆时针顺序排列。对于其他尺寸，它们按输入顺序排列。
# hull.simplices
# simplices：ndarray of ints, shape (nfacet, ndim)
# 形成凸包的简单面的点的索引。

# print(hull.simplices)
# print(type(hull))

# filled_image = np.zeros_like(img3)
# img4=cv2.drawContours(filled_image, [hull.simplices], -1,255)  
# cv2.imshow("Image", img4)
# cv2.waitKey(0)

# # 找到最左下角的点作为起始点
# start_point = min(points, key=lambda point: (point[0], point[1]))

# # 定义一个比较函数，用于排序
# def compare(p1, p2):
#     # 计算极角
#     angle1 = np.arctan2(p1[1] - start_point[1], p1[0] - start_point[0])
#     angle2 = np.arctan2(p2[1] - start_point[1], p2[0] - start_point[0])
#     if angle1 < angle2:
#         return -1
#     elif angle1 > angle2:
#         return 1
#     else:
#         # 如果极角相同，选择距离较远的点
#         dist1 = np.linalg.norm(p1 - start_point)
#         dist2 = np.linalg.norm(p2 - start_point)
#         if dist1 < dist2:
#             return -1
#         else:
#             return 1

# # 对点进行排序
# sorted_points = sorted(points, key=lambda p: (p[0], p[1]))

# # 开始扫描
# hull = [start_point, sorted_points[0]]
# for point in sorted_points[1:]:
#     while len(hull) > 1 and np.cross(hull[-1] - hull[-2], point - hull[-1]) <= 0:
#         hull.pop()
#     hull.append(point)

# # 绘制凸包
# hull = np.array(hull)
# plt.plot(points[:, 0], points[:, 1], 'o')
# plt.plot(hull[:, 0], hull[:, 1], 'r-')
# plt.show()





# _, binary_image = cv2.threshold(img3, 0, 255, cv2.THRESH_BINARY)
# contours, _ = cv2.findContours(binary_image, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
# print(contours)
# filled_image = np.zeros_like(binary_image)
# img4=cv2.drawContours(filled_image, contours, -1,255)  
# cv2.imshow('image', img4)
# cv2.waitKey()


# filled_image = np.zeros_like(binary_image)
# for contour in contours:
#     cv2.fillPoly(filled_image, [contour], (0, 255, 0))  # 这里使用绿色进行填充
# cv2.imshow("Filled Connected Areas", filled_image)
# cv2.waitKey(0)
# cv2.destroyAllWindows()


# # 设置特定的像素值
# target_pixel_value = 255  # 这里以蓝色像素 [B, G, R] 为例

# # 查找匹配像素的坐标
# coords = np.column_stack(np.where(depth_map == target_pixel_value))
# cv2.fillConvexPoly(depth_map, coords,255)
# cv2.imshow('image', depth_map)
# cv2.waitKey()
# contours, _ = cv2.findContours(depth_map, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
# cv2.fillPoly(depth_map, contours, (0, 0, 255))  # 这里使用红色进行填充
# cv2.imshow('image', depth_map)
# cv2.waitKey()



