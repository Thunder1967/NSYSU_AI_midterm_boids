import json
def setGlobal():
    return {
        "verify": "Setting", # 讀檔驗證
        "Overall":{
            "FPS": 60,
            "Bounce_Damping": 0.8, # bird 碰撞時能量遞減
            "Damping" : 100, #阻力
            "Interval_Of_Record": 0 # 紀錄模擬狀態的時間間隔(s)，為 0 時不記錄
        },
        "Evolution":{
            "Init_Mutation_Rate": 0.5, # 初始個體差異
            "Mutation_Rate": 0.1, # 突變率
            "Reproduce_Weight": 2 # 繁衍權重，愈高的話生物 fitness 影響繁衍越劇烈
        },
        "Overall_Bird":{
            "Number": 100, # bird 數量
            "Color_Slow": (75,76,255), #bird 最慢速顏色
            "Color_Fast": (63,255,50), #bird 最快速顏色
            # 計算精度，若有 n 隻 bird ，則每隻 bird 需要與 n-1 隻 bird 互動，
            # 為提升效能我這裡只讓 bird 與隨機 (n-1)*Movement_Accuracy 隻 bird 互動
            # 另一種想法是讓每隻 bird 有 Movement_Accuracy 的機率"不合群"，違背自然法則，模擬自然的隨機性
            "Movement_Accuracy": 100, # bird 不合群率 (%)
            "Rotation_Weight": 10, # bird 的計算方向時，原方向的乘值，值越小越不容易轉向
            "Gen_Stamina": 80, # bird 回復體力的速度
            "MAX_Life_Time": 120, # bird 最大壽命
        },
        "Upper_Bound_Bird":{ # 血繼限界，突變的上限
            "Size": 100,
            "MAX_Speed": 200,
            "MIN_Speed": 100,
        },
        "Lower_Bound_Bird":{ # 血繼限界，突變的下限
            "Size": 2,
            "MAX_Speed": 20,
            "MIN_Speed": 10,
            "MAX_Stamina":30,
        },
        "Bird": {
            "Size": 8, # bird 大小
            "MIN_Speed": 20, # bird 最小速度
            "MAX_Speed": 140, # bird 最大速度
            "Perception_Radius": 40, # bird 觀察範圍
            "Separation_Weight": 10, #bird 分離力最大值
            "Alignment_Weight": 10, # bird 對齊力最大值
            "Cohesion_Weight": 10, # bird 聚集力最大值
            "Flee_Weight": 40, # bird 逃跑力最大值
            "Alert_Radius": 50, # bird 警戒範圍
            "MAX_Stamina": 60, # bird 體力上限
            "Fitness": 0, # bird 對環境的適應度
            "Survival_Time": 0, # bird 生存時間，活太久會老死
        },
        "Overall_Predator":{
            "Number": 4, # predator 數量
            "Consume_Time": 0.5, # 進食時間(s)
        },
        "Predator": {
            "Size": 10, # predator 大小
            "MIN_Speed": 20, # predator 最小速度
            "MAX_Speed": 180, # predator 最大速度
            "Perception_Radius": 100, # predator 觀察範圍
            "Separation_Weight": 10, # predator 分離力
            "Track_Weight": 20, # predator 追蹤力
            "Eat_Radius": 8, # predator 捕食範圍
            "Track_Mode": 4, # 追擊方式
        },
        "Particle": {
            "Size": 10, #粒子大小
            "Num": 3, #每次動物死亡，出現的粒子數
            "Lifetime": 25, #粒子的生命週期 (tick)
            "Radious": 1, #粒子半徑
            "MIN_Speed": 15, #粒子的最小速度
            "MAX_Speed": 25, #粒子的最大速度
            "offset_angle": 120, #粒子init後和輸入的angle最大差值
        },
        "Obstacle": {
            "Number": 0, # obstacle 數量
            "Size": 80, # obstacle 大小
        }
    }

def create_default(file_name = "setting_default.json"):
    Setting = setGlobal()
    with open(f"data/{file_name}", "w") as f:
        json.dump(Setting, f)
    return file_name

if __name__ == "__main__":
    create_default()