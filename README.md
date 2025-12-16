# 最低版本要求:
1. Python : 3.13.9
2. pygame : 2.6.1
3. numpy : 2.3.5
4. ttkbootstrap : 1.19.0

# 必備檔案/資料夾
1. data
2. data/record
3. initial.py
4. read_data.py
5. run_console.py
6. run_pygame.py
## 執行時請使用 run_console.py
如果跑檔案有輸入要讀取的檔案，可以在CMD執行python時中輸入，格式如下
```console
用預設設定跑
python run_console.py

用輸入的設定跑
python run_console.py setting.json

用輸入的設定和之前存檔跑
python run_console.py setting.json environment.json
```
如果是在UI中輸入要讀取的檔案，預設director位置和runconsole.py同，且須記得加上 .json

ex: data/setting_speed.json || data/data_speed1.json
## 檔案功用
1. 資料夾 data : 模擬的初始設定     ex: setting_speed.json 
   或模擬跑到一半的鳥群和掠食者參數  ex: data_speed1.json                   
2. 資料夾 data/record : 存模擬過程  ex: record_speed1.json
3. initial.py : 生成預設參數文件
4. read_data.py : 讀取資料
5. run_console.py : 打開主控台並執行模擬
6. run_pygame.py : 執行模擬
