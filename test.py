import sys
import pygame
import os
import random
import math
import numpy as np


#setting
SCREEN_WIDTH = 1280
SCREEN_HEIGHT = 720
SCREEN_CNETER=(SCREEN_WIDTH/2,SCREEN_HEIGHT/2)
FPS = 60
sys.setrecursionlimit(10000)
#color
BACKGROUND_COLOR = (25,25,25)

#初始化
pygame.init()
Screen = pygame.display.set_mode((SCREEN_WIDTH,SCREEN_HEIGHT))
pygame.display.set_caption('boids V0.0.0')
Timer = pygame.time.Clock()

#載入圖片、字體
Font = os.path.join("TaipeiSansTCBeta-Regular.ttf")

#全域變數
Running=True #遊戲是否運行
Bird_MAX_Speed = 500
Bird_MIN_Speed = 100
Bird_Color_Slow = pygame.math.Vector3(255,100,105)
Bird_Color_Fast = pygame.math.Vector3(63,255,50)
Bird_Color_ChangeRate = Bird_Color_Fast-Bird_Color_Slow
Bird_Perception_Radius = 20
Bird_Separation_Weight = 2
Bird_Alignment_Weight = 1.2
Bird_Cohesion_Weight = 1
Damping = 1-1e-3
Movement_accuracy = 100
DT=0

#物件定義
class Bird:
    def __init__(self,pos):
        self.position = pygame.math.Vector2(pos[0],pos[1]) #初始位置
        rad=random.uniform(0, 2*math.pi)
        self.direction = pygame.math.Vector2(math.cos(rad),math.sin(rad)) #初始方向
        self.speed = (Bird_MIN_Speed+Bird_MAX_Speed)/2 #初始速率
        self.velocity = self.direction*self.speed #初始速度
        self.color = (255, 255, 255) #顏色
        self.size = 5
    def draw(self, screen):
        right_vector = pygame.math.Vector2(-self.direction.y,self.direction.x) #垂直於 self.direction 的向量
        #三個頂點
        head = self.position + self.direction * self.size * 1
        left = self.position - self.direction * self.size + right_vector * self.size * 0.5
        right = self.position - self.direction * self.size - right_vector * self.size * 0.5
        #繪製三角形
        pygame.draw.polygon(screen, self.color, 
                            [(int(head.x), int(head.y)), 
                             (int(left.x), int(left.y)), 
                             (int(right.x), int(right.y))])
    def apply_force(self, boids):
        separation_force = pygame.math.Vector2(0,0) #分離推力
        alignment_force = pygame.math.Vector2(0,0) #對齊力
        cohesion_force = pygame.math.Vector2(0, 0) #聚集力
        center_of_mass = pygame.math.Vector2(0, 0) #聚集中心

        neighbor_count = 0 # 紀錄偵測到的近鄰數量

        # 遍歷所有其他的 Boid
        for i in np.random.randint(0,len(boids)-1,size=(Movement_accuracy,)):
            other = boids[i]
            # 確保不是自己
            if other is not self:
                # 計算兩個 Boid 之間的距離
                distance = self.position.distance_to(other.position)
                
                # 檢查距離是否在排斥範圍內
                if 0 < distance < Bird_Perception_Radius:
                    #計算推離力
                    separation_force += (self.position - other.position)/distance
                    #計算對齊力
                    alignment_force += other.velocity
                    #計算聚集中心
                    center_of_mass += other.position

                    neighbor_count+=1
        
        #總結
        if neighbor_count>0:
            # 計算推離力
            if separation_force.length()>0:
                separation_force/=neighbor_count
                if separation_force.length() > Bird_Separation_Weight:
                    separation_force.scale_to_length(Bird_Separation_Weight)
            
            #計算對齊力
            # 對齊力 = 理想速度 (平均速度*最大速度) - 目前速度
            if alignment_force.length()>0:
                alignment_force /= neighbor_count
                alignment_force = alignment_force.normalize() * Bird_MAX_Speed - self.velocity
                if alignment_force.length() > Bird_Alignment_Weight:
                    alignment_force.scale_to_length(Bird_Alignment_Weight)
            
            #計算聚集力
            #往質量中心移動
            center_of_mass /= neighbor_count
            cohesion_force = center_of_mass - self.position
            if cohesion_force.length()>0:
                cohesion_force = cohesion_force.normalize() * Bird_MAX_Speed - self.velocity
                if cohesion_force.length() > Bird_Cohesion_Weight:
                    cohesion_force.scale_to_length(Bird_Cohesion_Weight)

        return separation_force+alignment_force+cohesion_force
    
    def update(self,all_boids):
        separation_force = self.apply_force(all_boids) #計算作用力
        self.direction = (self.direction+separation_force).normalize() #調整方向
        #調整速率
        self.speed += separation_force.length()
        self.speed *= Damping
        self.speed = min(max(self.speed,Bird_MIN_Speed),Bird_MAX_Speed)

        self.color = Bird_Color_Slow+Bird_Color_ChangeRate*((self.speed-Bird_MIN_Speed)/(Bird_MAX_Speed-Bird_MIN_Speed))
        self.velocity = self.direction*self.speed*DT
            
        # 2. 更新位置
        self.position += self.velocity
        
        # 邊界處理
        if self.position.x > SCREEN_WIDTH:
            self.position.x = 0
        elif self.position.x < 0:
            self.position.x = SCREEN_WIDTH
        
        if self.position.y > SCREEN_HEIGHT:
            self.position.y = 0
        elif self.position.y < 0:
            self.position.y = SCREEN_HEIGHT

#main
#load
birds=[Bird((random.uniform(0,SCREEN_WIDTH),random.uniform(0,SCREEN_HEIGHT))) for _ in range(500)]
#tick
while Running:
    #取得動作
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            Running = False
            break
    
    DT = Timer.tick(FPS)/1000 #FPS
    
    #繪圖
    Screen.fill(BACKGROUND_COLOR)
    for bird in birds:
        bird.update(birds)
        bird.draw(Screen)

    pygame.display.flip()

#結束清理
pygame.quit()
sys.exit()