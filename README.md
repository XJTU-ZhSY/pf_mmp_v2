# 粒子滤波SLAM建图的模拟

## 运行

run main.py

## particle.py

class particle

## robot.py

class robot

## radar.py

class slam

## outline

使用god视角和robot视角两个视角来实现模拟

其中，god视角记录robot的真实位置和真实地图，这些在robot进行雷达探测时使用

robot接受雷达探测的信号(接受的含噪声)并更新地图和进行移动，robot内部所见的都是robot认为自己所在的位置和认为探测到的地图

处理两个视角的细节：robot类激光雷达探测时，接受god视角的结果并加上噪声，god记录robot移动时，在robot移动的基础上(robot认为自己移动)加上噪声为真实移动，其中移动包括位移和旋转，雷达探测中也包括旋转角度的噪声处理。