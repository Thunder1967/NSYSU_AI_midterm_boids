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
pygame.display.set_caption('boids V0.1.0')
Timer = pygame.time.Clock()

#載入圖片、字體
Font = os.path.join("TaipeiSansTCBeta-Regular.ttf")

#全域變數
Running=True #遊戲是否運行
Debug_AVG_FPS = {"total":0,"cnt":0} #效能檢查
Bird_Number = 300 #bird 數量
Bird_Size = 8 #bird 大小
Bird_MAX_Speed = 200 #bird 最大速度
Bird_MIN_Speed = 50 #bird 最小速度
Bird_Color_Slow = pygame.math.Vector3(75,76,255) #bird 最慢速顏色
Bird_Color_Fast = pygame.math.Vector3(63,255,50) #bird 最快速顏色
Bird_Color_ChangeRate = Bird_Color_Fast-Bird_Color_Slow
Bird_Perception_Radius = 20 #bird 觀察範圍
Bird_Separation_Weight = 1 #bird 分離力最大值
Bird_Alignment_Weight = 1 #bird 對齊力最大值
Bird_Cohesion_Weight = 1 #bird 聚集力最大值
Damping = 1e-3 #阻力

Obstacle_Number = 4 # Obstacle 數量

# 計算精度，若有 n 隻 bird ，則每隻 bird 需要與 n-1 隻 bird 互動，
# 為提升效能我這裡只讓 bird 與隨機 (n-1)*Movement_Accuracy 隻 bird 互動
# 另一種想法是讓每隻 bird 有 Movement_Accuracy 的機率"不合群"，違背自然法則，模擬自然的隨機性
Movement_Accuracy = 0.5

DT=0 #每楨之間時間間隔，確保不同楨率下動畫表現一致

#物件定義
class Bird:
    def __init__(self,x,y):
        self.position = pygame.math.Vector2(x,y) #初始位置
        rad=random.uniform(0, 2*np.pi)
        self.direction = pygame.math.Vector2(np.cos(rad),np.sin(rad)) #初始方向
        self.speed = (Bird_MIN_Speed+Bird_MAX_Speed)/2 #初始速率
        self.velocity = self.direction*self.speed #初始速度
        self.color = (255, 255, 255) #顏色
        self.size = Bird_Size
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
    def apply_bounce(self, obstacles):
        """
        迭代所有障礙物，並呼叫其 check_collision 方法來處理碰撞和反彈。
        """
        for obstacle in obstacles:
            # 將自己（Boid 實例）傳遞給障礙物物件
            if obstacle.check_collision(self):
                # 處理單一碰撞後立即退出，避免 Boid 被多個障礙物邊緣重複處理
                return
    def update(self,all_boids,obstacles):
        separation_force = self.apply_force(all_boids) #計算作用力
        self.direction = (self.direction+separation_force).normalize() #調整方向
        #調整速率
        self.speed += separation_force.length()
        self.speed *= (1-Damping)
        self.speed = min(max(self.speed,Bird_MIN_Speed),Bird_MAX_Speed)

        #更改顏色和速度    
        self.color = Bird_Color_Slow+Bird_Color_ChangeRate*((self.speed-Bird_MIN_Speed)/(Bird_MAX_Speed-Bird_MIN_Speed))
        self.velocity = self.direction*self.speed*DT
            
        # 2. 更新位置
        self.position += self.velocity

        self.apply_bounce(obstacles)
        
        # 邊界處理
        if self.position.x > SCREEN_WIDTH:
            self.position.x = 0
        elif self.position.x < 0:
            self.position.x = SCREEN_WIDTH
        
        if self.position.y > SCREEN_HEIGHT:
            self.position.y = 0
        elif self.position.y < 0:
            self.position.y = SCREEN_HEIGHT

class Obstacle:
    def __init__(self, vertices, color=(150, 150, 150)):
        # 將頂點列表轉換為 Vector2 列表，方便運算
        self.vertices = [pygame.math.Vector2(v) for v in vertices]
        self.color = color
        
        # 為了避障邏輯方便，計算出多邊形的中心點
        if self.vertices:
            self.center = sum(self.vertices, pygame.math.Vector2(0, 0)) / len(self.vertices)
        else:
            self.center = pygame.math.Vector2(0, 0)
            
    def draw(self, screen):
        # 繪製實心多邊形
        int_vertices = [(int(v.x), int(v.y)) for v in self.vertices]
        if len(int_vertices) >= 3:
            pygame.draw.polygon(screen, self.color, int_vertices)

    # 輔助函數：計算點到線段的最短距離 (私有方法，只供內部使用)
    def _closest_point_on_segment(self, p: pygame.math.Vector2, a: pygame.math.Vector2, b: pygame.math.Vector2) -> tuple[pygame.math.Vector2, float]:
        """
        相信 gemini
        計算點 p 到線段 ab 的最短距離和最近點。優化：使用長度的平方 (length_squared)
        """
        ab = b - a
        
        # 避免除以零：如果線段長度為零
        len_sq = ab.length_squared()
        if len_sq == 0.0:
            # 直接返回點 a 和 p 到 a 的距離平方
            return a, p.distance_squared_to(a)
        
        # 投影 t = (p - a) 向量在 ab 向量上的投影長度比例
        # 由於 dot() 和 length_squared() 都不涉及昂貴的平方根 (sqrt) 運算，這裡效率很高
        t = (p - a).dot(ab) / len_sq
        
        # 限制 t 在 [0, 1] 之間，確保最近點落在線段 ab 範圍內
        t = max(0.0, min(1.0, t)) 
        
        # 計算線段上的最近點
        closest = a + t * ab
        
        # 返回最近點和距離平方 (distance_sq)
        # 再次避免使用昂貴的 distance_to()，直接使用 distance_squared_to()
        distance_sq = p.distance_squared_to(closest) 
        
        return closest, distance_sq
    
    def check_collision(self, boid) -> bool:
        """
        相信 gemini
        檢查 Boid 是否與此障礙物發生碰撞，並解決碰撞。
        優化：嘗試使用 Bounding Box 快速剔除（這裡暫略），並優化迴圈內的計算。
        """
        COLLISION_RADIUS = boid.size
        # 使用半徑的平方來與 distance_sq 比較，避免 sqrt
        COLLISION_RADIUS_SQ = COLLISION_RADIUS ** 2 
        
        num_vertices = len(self.vertices)
        
        # 遍歷多邊形的每一條邊
        for i in range(num_vertices):
            v1 = self.vertices[i]
            v2 = self.vertices[(i + 1) % num_vertices]
            
            # 1. 計算 Boid 位置到線段的最近點 (使用整合的輔助方法)
            closest_pt, distance_sq = self._closest_point_on_segment(boid.position, v1, v2)
            
            # 2. 檢查是否發生碰撞
            if distance_sq < COLLISION_RADIUS_SQ:
                
                # --- 碰撞解決 (Resolution) ---
                
                # a. 找出法線 (Normal)：從最近點指向 Boid
                normal = boid.position - closest_pt
                
                # 再次優化：避免重複計算 length() 和 normalize()
                normal_length = normal.length()
                
                if normal_length > 0:
                    
                    # b. 推回位置：解決穿透
                    
                    # 避免在迴圈外每次都呼叫 math.sqrt(distance_sq)
                    distance = np.sqrt(distance_sq) 
                    penetration_depth = COLLISION_RADIUS - distance
                    
                    # 避免多次呼叫 normalize()，在這裡計算一次
                    normal_norm = normal / normal_length 
                    
                    # 沿著法線方向將 Boid 推開
                    boid.position += normal_norm * penetration_depth
                    
                    # c. 速度反射（Reflection）：改變方向
                    BOUNCE_DAMPING = 0.8 
                    
                    # Pygame 的 reflect 效率很高，直接使用
                    bounce_vector = boid.velocity.reflect(normal_norm)
                    boid.velocity = bounce_vector * BOUNCE_DAMPING
                    
                    # d. 更新 Boid 的方向/速率屬性
                    if boid.velocity.length() > 0:
                         # 由於 velocity 剛被修改，需要重新計算方向
                         boid.direction = boid.velocity.normalize()
                         boid.speed = boid.velocity.length()
                    
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
birds = [Bird(random.uniform(0,SCREEN_WIDTH),random.uniform(0,SCREEN_HEIGHT)) for _ in range(Bird_Number)]
obstacles = [Obstacle(Obstacle.generate_random_polygon(
    random.uniform(100,SCREEN_WIDTH-100),
    random.uniform(100,SCREEN_HEIGHT-100),
    80,120,random.randint(4,10))) for _ in range(Obstacle_Number)]
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
        bird.update(birds,obstacles)
        bird.draw(Screen)
    for obstacle in obstacles:
        obstacle.draw(Screen)

    pygame.display.flip()

#結束清理
pygame.quit()
sys.exit()