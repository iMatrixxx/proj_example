'''
Author: aoi
Date: 2024-04-16 15:11:29
LastEditors: aoi
LastEditTime: 2024-04-28 22:25:02
Description: 
Copyright (c) Air by aoi, All Rights Reserved. 
'''
import numpy as np
import matplotlib.pyplot as plt
import math

    # 定义需要拟合的函数
def model_function(params, x):
    a, b, c = params
    return a * x**2 + b * x + c

# 定义残差函数
def residuals(params, x, y):
    return y - model_function(params, x)


# 读取文件中的数据
file_path = '/home/imatrix/test/distort_obs_new.txt'  # 假设文件名为data.txt

with open(file_path, 'r') as file:
    gt_x = []
    gt_y = []
    obs_x = []
    obs_y = []
    
    # for line in file:
    #     str_split=line.split('\t')
    #     if (abs(float(str_split[5]) * 221.77) > 2560) or (abs(float(str_split[6]) * 221.77) > 2048):
    #         continue
    #     gt_x.append(float(str_split[5]) * 221.77)
    #     gt_y.append(float(str_split[6]) * 221.77)
    #     obs_x.append(float(str_split[7]) * 221.77)
    #     obs_y.append(float(str_split[8]) * 221.77)
    
    
    obs_order = []
    for line in file:
        str_split=line.split(' ')
        # gt_x.append(float(str_split[5]) * 221.77)
        # gt_y.append(float(str_split[6]) * 221.77)
        x = float(str_split[1]) - 2560
        y = float(str_split[0].strip('\n')) - 2048
        
        angle = 0.055 * math.pi / 180.0
        cos_theta = math.cos(angle)
        sin_theta = math.sin(angle)
        
        
        obs_x.append(cos_theta *x - sin_theta *y)
        obs_y.append(sin_theta*x + cos_theta * y)
        xx = cos_theta *x - sin_theta *y
        yy = sin_theta*x + cos_theta * y
        obs_order.append((round(xx/132.741), round(yy/132.741) ,cos_theta *x - sin_theta *y, sin_theta*x + cos_theta * y))
        
    obs_order.sort()
    
    
    
        
    # for i in range(76):
    #     for (j) in range(61):
    #         gt_x.append(66.35 * (75 - i -37) -39.179)
    #         gt_y.append(66.35 * ( j -30) + 2.6807)
    
    
    
    for i in range(38):
        for (j) in range(30):
        
            gt_x.append(132.741 * (i -19) + 12.608)
            gt_y.append(132.741 * ( j - 15) + 50.858)

    plt.plot(gt_x, gt_y, "." ,color='blue')
    plt.plot(obs_x, obs_y, ".", color='red')
    plt.grid(True)
    plt.show()
    
    A = []
    b = []
    for i in range(len(gt_x)):
        obs = []
        r2 = np.square(gt_x[i]) + np.square(gt_x[i])
        obs.append(gt_x[i] * r2)
        obs.append(gt_x[i] * r2 * r2)
        obs.append(2 * gt_y[i] * gt_x[i])
        obs.append(r2 + 2*np.square(gt_x[i]))
        A.append(obs)
        b.append(- gt_x[i] + obs_order[i][2])
        
        obs = []
        obs.append(gt_y[i] * r2)
        obs.append(gt_y[i] * r2 * r2)
        obs.append(r2 + 2*np.square(gt_y[i]))
        obs.append(2 * gt_x[i] * gt_y[i])
        A.append(obs)
        b.append(-gt_y[i] + obs_order[i][3])

    # 使用最小二乘方法求解 Ax = b
    x, residuals, rank, s = np.linalg.lstsq(A, b, rcond=None)

    # 输出求解结果
    print("最小二乘解:", x)
    print("残差:", residuals)
    
    
    for i in range(len(gt_x)):
        r2 = np.square(obs_order[i][2]) + np.square(obs_order[i][3])
        x_cor = gt_x[i]*(1+x[0] * r2 + x[1] *r2 * r2 ) + x[2] * 2 * obs_order[i][2] * obs_order[i][3] + x[3] *(r2 + 2*np.square(obs_order[i][2]))
        y_cor = gt_y[i]*(1+x[0] * r2 + x[1] *r2 * r2 ) + x[3] * 2 * obs_order[i][2] * obs_order[i][3] + x[2] *(r2 + 2*np.square(obs_order[i][3]))
        
        print("残差：",gt_x[i] - obs_order[i][2], obs_order[i][2] - x_cor)
        print("残差：",gt_y[i] - obs_order[i][3], obs_order[i][3] - y_cor)
    
    
    
    print("最小二乘解:", x)
    print("残差:", residuals)
    
    
    ##

    # A = []
    # b = []
    # for i in range(len(gt_x)):
    #     obs = []
    #     r2 = np.square(obs_x[i]) + np.square(obs_y[i])
    #     obs.append(obs_x[i] * r2)
    #     obs.append(obs_x[i] * r2 * r2)
    #     obs.append(2 * obs_x[i] * obs_y[i])
    #     obs.append(r2 + 2*np.square(obs_x[i]))
    #     A.append(obs)
    #     b.append(gt_x[i] - obs_x[i])
        
    #     obs = []
    #     obs.append(obs_y[i] * r2)
    #     obs.append(obs_y[i] * r2 * r2)
    #     obs.append(r2 + 2*np.square(obs_y[i]))
    #     obs.append(2 * obs_x[i] * obs_y[i])
    #     A.append(obs)
    #     b.append(gt_y[i] - obs_y[i])

    # # 使用最小二乘方法求解 Ax = b
    # x, residuals, rank, s = np.linalg.lstsq(A, b, rcond=None)

    # # 输出求解结果
    # print("最小二乘解:", x)
    # print("残差:", residuals)
    
    
    # for i in range(len(gt_x)):
    #     r2 = np.square(obs_x[i]) + np.square(obs_y[i])
    #     x_cor = obs_x[i]*(1+x[0] * r2 + x[1] *r2 * r2 ) + x[2] * 2 * obs_x[i] * obs_y[i] + x[3] *(r2 + 2*np.square(obs_x[i]))
    #     y_cor = obs_y[i]*(1+x[0] * r2 + x[1] *r2 * r2 ) + x[3] * 2 * obs_x[i] * obs_y[i] + x[2] *(r2 + 2*np.square(obs_y[i]))
        
    #     print("残差：",gt_x[i] - obs_x[i], gt_x[i] - x_cor)
    #     print("残差：",gt_y[i] - obs_y[i], gt_y[i] - y_cor)
    
    
    
    # print("最小二乘解:", x)
    # print("残差:", residuals)
    
    
    plt.plot(gt_x, gt_y, "." ,color='blue')
    plt.plot(obs_x, obs_y, ".", color='red')
    plt.grid(True)
    plt.show()

    




with open(file_path, 'r') as file:
    gt_x = []
    gt_y = []
    obs_x = []
    obs_y = []
    
    for line in file:
        str_split=line.split('\t')
        gt_x.append(float(str_split[5]) * 221.77)
        gt_y.append(float(str_split[6]) * 221.77)
        obs_x.append(float(str_split[7]) * 221.77)
        obs_y.append(float(str_split[8]) * 221.77)

    A = []
    b = []
    for i in range(len(gt_x)):
        obs = []
        r2 = np.square(obs_x[i]) + np.square(obs_y[i])
        obs.append(obs_x[i] * r2)
        obs.append(obs_x[i] * r2 * r2)
        obs.append(2 * obs_x[i] * obs_y[i])
        obs.append(r2 + 2*np.square(obs_x[i]))
        A.append(obs)
        b.append(gt_x[i] - obs_x[i])
        
        obs = []
        obs.append(obs_y[i] * r2)
        obs.append(obs_y[i] * r2 * r2)
        obs.append(r2 + 2*np.square(obs_y[i]))
        obs.append(2 * obs_x[i] * obs_y[i])
        A.append(obs)
        b.append(gt_y[i] - obs_y[i])

    # 使用最小二乘方法求解 Ax = b
    x, residuals, rank, s = np.linalg.lstsq(A, b, rcond=None)

    # 输出求解结果
    print("最小二乘解:", x)
    print("残差:", residuals)
    
    
    for i in range(len(gt_x)):
        r2 = np.square(obs_x[i]) + np.square(obs_y[i])
        x_cor = obs_x[i]*(1+x[0] * r2 + x[1] *r2 * r2 ) + x[2] * 2 * obs_x[i] * obs_y[i] + x[3] *(r2 + 2*np.square(obs_x[i]))
        y_cor = obs_y[i]*(1+x[0] * r2 + x[1] *r2 * r2 ) + x[3] * 2 * obs_x[i] * obs_y[i] + x[2] *(r2 + 2*np.square(obs_y[i]))
        
        print("残差：",gt_x[i] - obs_x[i], gt_x[i] - x_cor)
        print("残差：",gt_y[i] - obs_y[i], gt_y[i] - y_cor)
    
    
    
    print("最小二乘解:", x)
    print("残差:", residuals)
    

    
    
    # cols 5120
    # rows 4096
    image_size = (5120, 4096)
    
    plt.plot(gt_x, gt_y, "." ,color='blue')
    plt.plot(obs_x, obs_y, ".", color='red')
    plt.show()

    

