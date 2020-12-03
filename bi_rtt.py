import matplotlib.pyplot as plt
import numpy as np
import math
import random
from PIL import Image

class node: # 路径上的结点
    def __init__(self,pos,parent=None):
        self.pos = pos # 该点的位置
        self.parent = parent # 该点的上一个点

def openfile(path): # 将csv文件内容读入列表中
    f = open(path,'r')
    lines = f.readlines()
    f.close()
    maze = []
    for i in range(len(lines)):
        tmp = []
        for j in range(len(lines[i])):
            if lines[i][j] != '\n':
                tmp.append(lines[i][j])
            if lines[i][j] == 'S': # 记录出发点
                s = node([i,j])
            if lines[i][j] == 'E': # 记录终点
                e = node([i,j])
        maze.append(tmp)
    return maze,s,e

def draw_maze(maze):
    h = len(maze)
    l = len(maze[0])
    for i in range(h):
        for j in range(l):
            if maze[i][j]=='1':
                plt.scatter(i,j,c='k',marker='s',s=10)
            if maze[i][j]=='S':
                plt.scatter(i,j,marker='$S$')
            if maze[i][j]=='E':
                plt.scatter(i,j,marker='$E$')
    plt.axis('equal')

def get_rand_node(maze): # 获取一个随机点
    while(True):
        x = random.randint(0, len(maze)-1)
        y = random.randint(0, len(maze[0])-1)
        if check_obs(maze,x,y) == 0:
            break
    return [x,y]

def check_obs(maze,x,y): # 检查两点之间有无障碍物
    if maze[x][y] == '1':
        return 1
    return 0

def find_near(q_rand,explored): # 找到已探索的点中离随机点最近的点
    min_dis = 999999
    min_i = 0
    for i in range(len(explored)):
        dis = cal_dis(q_rand,explored[i])
        if dis < min_dis:
            min_dis = dis
            min_i = i
    return explored[min_i]

def cal_dis(n1,n2): # 计算两点之间距离
    dis = math.sqrt((n1.pos[0] - n2.pos[0]) ** 2 + (n1.pos[1] - n2.pos[1]) ** 2)
    return dis

def find_new(q_near,q_rand,delta_q): # 根据near和rand，得到一个new结点
    if cal_dis(q_near,q_rand)<delta_q: # 如果near与rand距离太近，就取它们的中点作为new结点
        x = (q_near.pos[0] + q_rand.pos[0])/2
        y = (q_near.pos[1] + q_rand.pos[1])/2
    else: # 否则从near向rand前进一步得到new
        theta = math.atan2(q_rand.pos[1]-q_near.pos[1],q_rand.pos[0]-q_near.pos[0])
        x = q_near.pos[0] + delta_q * math.cos(theta)
        y = q_near.pos[1] + delta_q * math.sin(theta)
    return node([round(x),round(y)])

def check_goal(q_new,e): # 检查是否达到终点
    if cal_dis(q_new,e)<threshold:
        return 1
    return 0

def get_path(e): # 由终点向前回溯得到路径
    cur = e
    path = []
    while(cur!=None):
        path.append(cur)
        cur = cur.parent
    return path

def draw_res(path,c): # 画出指定颜色的路径
    plt.ion()
    for i in range(len(path)-1):
        plt.plot([path[i].pos[0],path[i+1].pos[0]],[path[i].pos[1],path[i+1].pos[1]],c)
        plt.ioff()

def smooth_path(path): # 对路径进行平滑操作
    tmp = path[0]
    new_path = [tmp]
    i = 1
    while(True):
        obs_flag = 0
        for j in range(check_obs_k): # 判断tmp与path[i]之间是否有障碍物
            if check_obs(maze, round(tmp.pos[0] + (j + 1) * 0.1 * (path[i].pos[0] - tmp.pos[0])),
                         round(tmp.pos[1] + (j + 1) * 0.1 * (path[i].pos[1] - tmp.pos[1]))) == 1:
                obs_flag = 1
                break
        if obs_flag == 0: # 若tmp与path[i]之间没有障碍物，则将i+1，继续判断
            i += 1
            if i == len(path):
                new_path.append(path[i - 1])
                break
            continue
        new_path.append(path[i-1]) # 若tmp与path[i]之间有障碍物，则记录i-1，tmp与path[i-1]之间无障碍物，可以直接用直线连接，简化路径
        tmp = path[i-1] # 将tmp更新为path[i-1]
    return new_path # 返回的路径就是简化后的路径

def get_maze_from_pic(path):
    img = Image.open(path)
    img_np = np.array(img)
    lst_point_start = []
    lst_point_end = []
    for y in range(img_np.shape[0]):
        for x in range(img_np.shape[1]):
            # 找到是红色的所有点，它们的坐标平均值是起点中心
            if (img_np[y][x][:3] == color_start).all() == True:
                lst_point_start.append([y, x])
            # 找到是蓝色的所有点，它们的坐标平均值是终点中心
            elif (img_np[y][x][:3] == color_end).all() == True:
                lst_point_end.append([y, x])
    np_point_start = np.array(lst_point_start)
    np_point_end = np.array(lst_point_end)
    # 获取起点、终点、半径
    point_start = np_point_start.mean(axis=0).astype(int)
    point_end = np_point_end.mean(axis=0).astype(int)
    maze = []
    for i in range(len(img_np)):
        tmp = []
        for j in range(len(img_np[0])):
            if (img_np[i][j][:3] == color_ocp).all() == True:
            # if (img_np[i][j][:3] == color_ocp).all() == True:
                tmp.append('1')
            # elif (img_np[i][j][:3] == color_ocp).all() == True:
            else:
                tmp.append('0')
        maze.append(tmp)
    maze[point_start[0]][point_start[1]] = 'S'
    maze[point_end[0]][point_end[1]] = 'E'
    return maze, node(point_start.tolist()), node(point_end.tolist())


delta_q = 10 # 从near向rand前进的步伐大小
threshold = 1.5 # 判断是否达到终点的阈值
check_obs_k = 8 # 检查两点之间是否有障碍物，将两点间分为这么多份，判断中间的每个点是否在障碍物内
color_start = np.array([63, 72, 204])
color_end = np.array([236, 28, 36])
color_ocp = np.array([0,0,0])
color_free = np.array([255,255,255])

# maze, s, e = openfile('./simpleMaze.txt')
maze, s, e = get_maze_from_pic('./maze/test.png')
v1 = [s] # 第一棵树已探索的结点
v2 = [e] # 第二棵树已探索的结点
e1 = [] # 第一棵树已探索的路径
e2 = [] # 第二棵树已探索的路径
plt.figure()
draw_maze(maze)
plt.ion()
ch = 0
while(True):
    if len(v1)<=len(v2): # 哪棵树结点少就对哪棵树进行探索，保持两棵树结点数量平衡
        q_rand_1 = node(get_rand_node(maze)) # 获得rand1
        q_near_1 = find_near(q_rand_1,v1) # 获得near1
        q_new_1 = find_new(q_near_1,q_rand_1,delta_q) # 获得new1
        obs_flag = 0
        for i in range(check_obs_k): # 判断new1和near1之间有没有障碍物
            if check_obs(maze,round(q_near_1.pos[0]+(i+1)*0.1*(q_new_1.pos[0]-q_near_1.pos[0])),
                         round(q_near_1.pos[1]+(i+1)*0.1*(q_new_1.pos[1]-q_near_1.pos[1]))) == 1:
                obs_flag =1
                break
        if obs_flag == 0: # 若没有
            v1.append(q_new_1) # 将new1加入树1的结点中
            e1.append([q_new_1,q_near_1])
            q_new_1.parent = q_near_1
            plt.plot([q_new_1.pos[0],q_near_1.pos[0]],[q_new_1.pos[1],q_near_1.pos[1]],'r') # 画出新探索的路径
            plt.pause(0.01)
            plt.ioff()
            q_near_2 = find_near(q_new_1,v2) # 将new1作为树2的rand，对树2进行探索，获得near2
            q_new_2 = find_new(q_near_2,q_new_1,delta_q) # 获得new2
            for i in range(check_obs_k): # 检查near2与new2之间有没有障碍物
                if check_obs(maze, round(q_near_2.pos[0] + (i + 1) * 0.1 * (q_new_2.pos[0] - q_near_2.pos[0])),
                             round(q_near_2.pos[1] + (i + 1) * 0.1 * (q_new_2.pos[1] - q_near_2.pos[1]))) == 1:
                    obs_flag = 1
                    break
            if obs_flag == 0: # 若没有
                v2.append(q_new_2) # 将new2加入到树2的结点
                e2.append([q_new_2,q_near_2])
                q_new_2.parent = q_near_2
                plt.plot([q_new_2.pos[0], q_near_2.pos[0]], [q_new_2.pos[1], q_near_2.pos[1]], 'r') # 画出新路径
                plt.pause(0.01)
                plt.ioff()
                while check_goal(q_new_1,q_new_2) == 0: # 当两棵树没有相连时
                    q_new_2_ = find_new(q_new_2,q_new_1,delta_q) # 将new2作为near2，new1仍为rand2，获得new2_，相当于让树2在new1的方向上继续延伸
                    for i in range(check_obs_k): # 检查new2于new2_间有无障碍物
                        if check_obs(maze, round(q_new_2.pos[0] + (i + 1) * 0.1 * (q_new_2_.pos[0] - q_new_2.pos[0])),
                                     round(q_new_2.pos[1] + (i + 1) * 0.1 * (q_new_2_.pos[1] - q_new_2.pos[1]))) == 1:
                            obs_flag = 1
                            break
                    if obs_flag == 0: # 若无
                        v2.append(q_new_2_) # 将new2_加入到树2中
                        e2.append([q_new_2_,q_new_2])
                        q_new_2_.parent = q_new_2
                        plt.plot([q_new_2.pos[0], q_new_2_.pos[0]], [q_new_2.pos[1], q_new_2_.pos[1]], 'r') # 画新路径
                        plt.pause(0.01)
                        plt.ioff()
                        q_new_2 = q_new_2_ # new2_变为新的new2，重复上述步骤，让树2尽可能在这个方向上延伸，向树1靠近
                    else:
                        break
                if check_goal(q_new_1,q_new_2) == 1: # 检查两棵树是否相连
                    plt.plot([q_new_1.pos[0], q_new_2.pos[0]], [q_new_1.pos[1], q_new_2.pos[1]], 'r')
                    plt.pause(0.01)
                    plt.ioff()
                    print("reach goooooooaaaaaal!!!!!!!")
                    break
    else: # 若树1的结点更多，则将树1与树2交换，确保每次对结点少的树进行探索
        v_tmp = v1
        v1 = v2
        v2 = v_tmp
        e_tmp = e1
        e1 = e2
        e2 = e_tmp
        ch += 1
        # print("change!",ch)
path_1 = get_path(q_new_1) # 得到树1的路径
path_2 = get_path(q_new_2) # 得到树2的路径
# 将两条路径拼接，由于运行过程中两棵树有交换操作，为了保证拼接后结点的顺序，要考虑交换了多少次
if ch % 2 == 1:
    path_2.reverse()
    path = path_2 + path_1
else:
    path_1.reverse()
    path = path_1 + path_2
new_path = smooth_path(path) # 对路径进行平滑操作
draw_res(path,'g') # 用绿色画出起点到终点的初始路径
draw_res(new_path,'b') # 用蓝色画出起点到终点的简化路径
plt.show()

