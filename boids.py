import sys
import pygame
import os
import numpy.random as random
import numpy as np


#setting
SCREEN_WIDTH = 1280
SCREEN_HEIGHT = 720
SCREEN_CNETER=(SCREEN_WIDTH/2,SCREEN_HEIGHT/2)
FPS = 60
#color
BACKGROUND_COLOR = (25,25,25)

#初始化
pygame.init()
Screen = pygame.display.set_mode((SCREEN_WIDTH,SCREEN_HEIGHT))
pygame.display.set_caption('boids V0.2.0')
Timer = pygame.time.Clock()

#載入圖片、字體
Font = os.path.join("TaipeiSansTCBeta-Regular.ttf")

#全域變數
Running=True #遊戲是否運行
Debug_AVG_FPS = {"total":0,"cnt":0} #效能檢查
Damping = 2e-3 #阻力

Bird_Number = 300 #bird 數量
Bird_Size = 8 #bird 大小
Bird_MAX_Speed = 200 #bird 最大速度
Bird_MIN_Speed = 20 #bird 最小速度
Bird_Color_Slow = pygame.math.Vector3(75,76,255) #bird 最慢速顏色
Bird_Color_Fast = pygame.math.Vector3(63,255,50) #bird 最快速顏色
Bird_Color_ChangeRate = Bird_Color_Fast-Bird_Color_Slow
Bird_Perception_Radius = 20 #bird 觀察範圍
Bird_Separation_Weight = 1 #bird 分離力最大值
Bird_Alignment_Weight = 1 #bird 對齊力最大值
Bird_Cohesion_Weight = 1 #bird 聚集力最大值
Bird_Flee_Weight = 5 #bird 逃跑力最大值
Bird_Alert_Radius = 40 #bird 警戒範圍

Predator_Number = 3 #Predator 數量
Predator_Size = 10 #Predator 大小
Predator_MIN_Speed = 40 #Predator 最小速度
Predator_MAX_Speed = 160 #Predator 最大速度
Predator_Perception_Radius = 60 #Predator 觀察範圍
Predator_Track_Weight = 1

Obstacle_Number = 4 # Obstacle 數量
Bounce_Damping = 0.8 # bird 碰撞時能量遞減

# 計算精度，若有 n 隻 bird ，則每隻 bird 需要與 n-1 隻 bird 互動，
# 為提升效能我這裡只讓 bird 與隨機 (n-1)*Movement_Accuracy 隻 bird 互動
# 另一種想法是讓每隻 bird 有 Movement_Accuracy 的機率"不合群"，違背自然法則，模擬自然的隨機性
Movement_Accuracy = 0.5

DT=0 #每楨之間時間間隔，確保不同楨率下動畫表現一致

#物件定義
class Animal:
    def __init__(self,pos,size,speed,color=(255, 255, 255)):
        self.position = pygame.math.Vector2(pos['x'],pos['y']) #初始位置
        self.direction=pygame.math.Vector2(0,0)
        self.direction.from_polar((1,random.uniform(1,360)))#初始方向
        self.speed = speed #初始速率
        self.velocity = self.direction*self.speed #初始速度
        self.color = color #顏色
        self.size = size
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
    def apply_bounce(self, obstacles):
        """
        迭代所有障礙物，並呼叫其 check_collision 方法來處理碰撞和反彈。
        """
        for obstacle in obstacles:
            # 將自己（Boid 實例）傳遞給障礙物物件
            if obstacle.check_collision(self):
                # 處理單一碰撞後立即退出，避免 Boid 被多個障礙物邊緣重複處理
                return
    def basis_update(self,MAX_SPEED,MIN_SPPED,obstacles):
        # 碰撞反彈
        self.apply_bounce(obstacles)

        #更改速度
        self.speed *= (1-Damping)
        self.speed = min(max(self.speed,MIN_SPPED),MAX_SPEED)
        self.velocity = self.direction*self.speed*DT
            
        # 更新位置
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

class Bird(Animal):
    def __init__(self,pos):
        super(Bird,self).__init__(pos,Bird_Size,(Bird_MIN_Speed+Bird_MAX_Speed)/2) 
    def apply_force(self, boids):
        separation_force = pygame.math.Vector2(0,0) #分離推力
        alignment_force = pygame.math.Vector2(0,0) #對齊力
        cohesion_force = pygame.math.Vector2(0, 0) #聚集力
        center_of_mass = pygame.math.Vector2(0, 0) #聚集中心

        neighbor_count = 0 # 紀錄偵測到的近鄰數量

        # 遍歷其他的 Boid
        for i in np.random.choice(np.arange(0,Bird_Number),
                                  size=(int(Bird_Number*Movement_Accuracy),), replace=False):
            other = boids[i]
            # 確保不是自己
            if other is not self:
                # 計算兩個 Boid 之間的距離
                distance = self.position.distance_to(other.position)
                
                # 檢查距離是否在排斥範圍內
                if 0 < distance < Bird_Perception_Radius:
                    #計算推離力

                    separation_force += (self.position - other.position).normalize()*(Bird_MAX_Speed/distance)
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
    def flee_predator(self, predators):
        flee_force = pygame.math.Vector2(0, 0)
        neighbor_count = 0
        
        for predator in predators:
            distance_vector = self.position - predator.position # 從掠食者指向 Boid 的向量
            distance = distance_vector.length()
            
            # 檢查是否在逃跑範圍內
            if distance < Bird_Alert_Radius and distance > 0:
                # 施加一個強大的推力
                flee_force += distance_vector.normalize()*Bird_MAX_Speed / distance
                neighbor_count+=1
        if neighbor_count>0:
            # 計算逃跑力
            if flee_force.length()>0:
                flee_force/=neighbor_count
                if flee_force.length() > Bird_Flee_Weight:
                    flee_force.scale_to_length(Bird_Flee_Weight)
        return flee_force   
    def update(self,all_boids,obstacles,predators):
        #調整速度
        force = self.apply_force(all_boids)+self.flee_predator(predators) #計算作用力
        self.direction = (self.direction+force).normalize() #調整方向
        self.speed += force.length() #調整速率

        #實際運動
        super(Bird,self).basis_update(Bird_MAX_Speed,Bird_MIN_Speed,obstacles)

        #更改顏色    
        self.color = Bird_Color_Slow+Bird_Color_ChangeRate*((self.speed-Bird_MIN_Speed)/(Bird_MAX_Speed-Bird_MIN_Speed))

class Predator(Animal):
    def __init__(self, pos):
        super(Predator,self).__init__(pos,Predator_Size,Predator_MAX_Speed,color=(255,30,45))
    def apply_track(self, all_boids):
        # 追蹤 bird
        track_force = pygame.math.Vector2(0, 0) #追蹤力
        if all_boids:
            track_radius = Predator_Perception_Radius**2
            neighbor_count = 0 # 紀錄偵測到的近鄰數量

            for bird in all_boids:
                forward_bird = bird.position-self.position
                if track_radius>forward_bird.length_squared():
                    track_force+=forward_bird
                    neighbor_count+=1
            
            if neighbor_count>0:
                track_force/=neighbor_count
                if track_force.length()>0:
                    track_force = track_force.normalize() * Predator_MAX_Speed - self.velocity
                    if track_force.length_squared() > Predator_Track_Weight**2:
                        track_force.scale_to_length(Predator_Track_Weight)

        return track_force
    def update(self, all_boids, obstacles):
        force = self.apply_track(all_boids) #計算作用力
        self.direction = (self.direction+force).normalize() #調整方向
        self.speed += force.length() #調整速率

        #實際運動
        super(Predator,self).basis_update(Predator_MAX_Speed,Predator_MIN_Speed,obstacles)
        
class Obstacle:
    def __init__(self, vertices, color=(150, 150, 150)):
        # 將頂點列表轉換為 Vector2 列表，方便運算
        self.vertices = [pygame.math.Vector2(v) for v in vertices]
        self.color = color
        
        # 計算出多邊形的中心點
        if self.vertices:
            self.center = sum(self.vertices, pygame.math.Vector2(0, 0)) / len(self.vertices)
        else:
            self.center = pygame.math.Vector2(0, 0)

        self.radius = 0
        for vertix in self.vertices:
            self.radius = max(self.radius,(vertix-self.center).length())
            
    def draw(self, screen):
        # 繪製實心多邊形
        int_vertices = [(int(v.x), int(v.y)) for v in self.vertices]
        if len(int_vertices) >= 3:
            pygame.draw.polygon(screen, self.color, int_vertices)

    def check_collision(self, boid) -> bool:
        """
        圓形碰撞
        極度簡化的碰撞邏輯，效能極高
        """ 
        # Boid 的碰撞半徑，bird 看到障礙且距離夠近時閃開
        COLLISION_RADIUS = Bird_Perception_Radius/2
        
        # 1. 計算 Boid 到圓心的向量和距離
        center_to_boid = boid.position - self.center
        distance_sq = center_to_boid.length_squared()
        
        # 總碰撞半徑 (兩者半徑之和) 的平方
        total_radius = self.radius + COLLISION_RADIUS
        total_radius_sq = total_radius ** 2
        
        # 2. 檢查是否發生碰撞 (使用平方比較，避免 math.sqrt)
        if distance_sq < total_radius_sq:
            # 距離
            distance = np.sqrt(distance_sq)
            
            # 法線 (從圓心指向 Boid 的向量)
            normal = center_to_boid
            
            if distance > 0: # 避免 Boid 剛好在圓心
                # 推回位置：解決穿透
                penetration_depth = total_radius - distance
                normal_norm = normal / distance # 這裡的 normal / distance 就是正規化
                
                # 將 Boid 沿著法線方向推開
                boid.position += normal_norm * penetration_depth
                
                # 使用法線進行反射
                boid.direction = boid.direction.reflect(normal_norm)
                boid.speed *= Bounce_Damping
                
                return True 
                
        return False
    @staticmethod
    def generate_random_polygon(center_x: float, center_y: float, min_radius: float, max_radius: float, num_vertices: int) -> list[tuple[float, float]]:
        """
        生成多邊形
        """
        center = pygame.math.Vector2(center_x, center_y)
        
        # 1. 隨機生成 num_vertices 個半徑 (使用 NumPy)
        rand_radii = np.random.uniform(min_radius, max_radius, size=num_vertices)
        
        # 2. 隨機生成角度
        angle_step = 2 * np.pi / num_vertices
        # 基準角度：讓頂點均勻分佈在圓周上
        base_angles = np.arange(num_vertices) * angle_step
        
        # 3. 隨機角度微調 (Jitter)：在 [-angle_step*0.2, angle_step*0.2] 範圍內
        angle_jitter = np.random.uniform(-angle_step * 0.2, angle_step * 0.2, size=num_vertices)
        final_angles = base_angles + angle_jitter
        
        # 4. 向量化計算所有頂點座標
        x_coords = center.x + rand_radii * np.cos(final_angles)
        y_coords = center.y + rand_radii * np.sin(final_angles)
        
        # 5. 組合成 [(x, y)] 列表
        vertices = []
        # 使用 zip 將 x, y 座標打包
        for x, y in zip(x_coords, y_coords):
            vertices.append((x, y))
            
        return vertices

#main
#load
birds = [Bird(random.choice([
    {'x':0,'y':random.randint(0,SCREEN_HEIGHT)},
    {'x':SCREEN_WIDTH,'y':random.randint(0,SCREEN_HEIGHT)},
    {'x':random.randint(0,SCREEN_WIDTH),'y':0},
    {'x':random.randint(0,SCREEN_WIDTH),'y':SCREEN_HEIGHT}
    ])) for _ in range(Bird_Number)] #在邊緣生成 bird

obstacles = [Obstacle(Obstacle.generate_random_polygon(
    random.randint(100,SCREEN_WIDTH-100),
    random.randint(100,SCREEN_HEIGHT-100),
    80,100,random.randint(4,20))) for _ in range(Obstacle_Number)]

predators = [Predator(random.choice([
    {'x':0,'y':random.randint(0,SCREEN_HEIGHT)},
    {'x':SCREEN_WIDTH,'y':random.randint(0,SCREEN_HEIGHT)},
    {'x':random.randint(0,SCREEN_WIDTH),'y':0},
    {'x':random.randint(0,SCREEN_WIDTH),'y':SCREEN_HEIGHT}
    ])) for _ in range(Predator_Number)]

#tick
while Running:
    #取得動作
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            Running = False
            break
    
    DT = Timer.tick(FPS)/1000 #FPS
    Debug_AVG_FPS["total"]+=DT
    Debug_AVG_FPS["cnt"]+=1
    if Debug_AVG_FPS["cnt"]>=FPS:
        print(f'{Debug_AVG_FPS["total"]*1000/Debug_AVG_FPS["cnt"]:.2f}')
        Debug_AVG_FPS={"total":0,"cnt":0}

    
    #繪圖
    Screen.fill(BACKGROUND_COLOR)
    for bird in birds:
        bird.update(birds,obstacles,predators)
        bird.draw(Screen)
    for obstacle in obstacles:
        obstacle.draw(Screen)
    for predator in predators:
        predator.update(birds,obstacles)
        predator.draw(Screen)

    pygame.display.flip()

#結束清理
pygame.quit()
sys.exit()