# ==========================================
# SLAM型配送：人回避＋信号＋配送順序決定（完成版）
# （荷物重量ランダム反映・正解パターン）
# ==========================================
import random, heapq, time, copy
from itertools import permutations

# ===== マップ（全角） =====
RAW_MAP = [
"■■■Ｂ・・◆◆◆◆・・Ａ■■■",
"■■■■・・◆◆◆◆・・■■■■",
"■■■■・・◆◆◆◆・・■■■■",
"■■■■・②＃＃＃＃・・■■■■",
"■■■■・・＃＃＃＃②・■■■■",
"■■■■・・◆◆◆◆・・■■■■",
"・・・・・・◆◆◆◆・・・・・・",
"・・①・・・◆◆◆◆・・・・①・",
"◆＃＃◆◆◆◆◆◆◆◆◆◆＃＃◆",
"◆＃＃◆◆◆◆◆◆◆◆◆◆＃＃◆",
"◆＃＃◆◆◆◆◆◆◆◆◆◆＃＃◆",
"◆＃＃◆◆◆◆◆◆◆◆◆◆＃＃◆",
"・①・・・・◆◆◆◆・・・①・・",
"・・・・・・◆◆◆◆・・・・・・",
"■Ｃ■■・②＃＃＃＃・・受■■■",
"■■■■・・＃＃＃＃②・■■■■",
]

H, W = 16, 16

# ===== 方向 =====
DIRS = {
    0:(-1,0),  # 北
    1:(0,-1),  # 西
    2:(1,0),   # 南
    3:(0,1),   # 東
}
ARROW={0:"▲",1:"◀",2:"▼",3:"▶"}

# ===== util =====
def manhattan(a,b): return abs(a[0]-b[0])+abs(a[1]-b[1])

def find(ch):
    for r in range(H):
        for c in range(W):
            if grid[r][c]==ch:
                return (r,c)

# ===== 信号 =====
def signal_color(step, offset):
    t = (step + offset) % 50
    if t < 20: return 0
    if t < 30: return 1
    return 2

def signal_name(c):
    return ["青","黄","赤"][c]

# ===== 人配置 =====
def place_people(n=6):
    cand=[(r,c) for r in range(H) for c in range(W) if grid[r][c] in "・＃"]
    banned=set(find(x) for x in "ＡＢＣ受" if find(x))
    random.shuffle(cand)
    for r,c in cand:
        if any(manhattan((r,c),b)<=2 for b in banned):
            continue
        grid[r][c]="◯"
        banned.add((r,c))
        if sum(1 for r in range(H) for c in range(W) if grid[r][c]=="◯")>=n:
            break

# ===== A* =====
def astar(start,goal,map_):
    pq=[(0,start)]
    cost={start:0}
    prev={}
    while pq:
        _,cur=heapq.heappop(pq)
        if cur==goal: break
        for d in DIRS.values():
            nr,nc=cur[0]+d[0],cur[1]+d[1]
            if not(0<=nr<H and 0<=nc<W): continue
            if map_[nr][nc] in "■◆◎": continue
            ncst=cost[cur]+1
            if (nr,nc) not in cost or ncst<cost[(nr,nc)]:
                cost[(nr,nc)]=ncst
                heapq.heappush(pq,(ncst+manhattan((nr,nc),goal),(nr,nc)))
                prev[(nr,nc)]=cur
    if start==goal: return [start]
    if goal not in prev: return []
    path=[goal]
    while path[-1]!=start:
        path.append(prev[path[-1]])
    return path[::-1]

# =============================
# 初期化
# =============================
grid=[list(r) for r in RAW_MAP]
place_people()

agent=find("受")
dir=0
mode="ROTATE"
crossing=False

# =============================
# 配送順序決定ブロック（正解パターン）
# =============================

# --- 荷物マスタ（kg） ---
ITEMS = {
    "ダンベル": 20.0,
    "机": 5.0,
    "PC部品": 1.0,
    "ポケモンカード": 0.0001,  # 0.1g
}

# --- 各配送先にランダム割当 ---
WEIGHTS = {
    p: random.choice(list(ITEMS.values()))
    for p in ["Ａ","Ｂ","Ｃ"]
}

print("📦 各配送先の荷物重量(kg):", WEIGHTS)

POINTS = ["受","Ａ","Ｂ","Ｃ"]
POS = {p: find(p) for p in POINTS}

# 距離行列（事前計算）
DIST={}
for a in POINTS:
    for b in POINTS:
        if a==b: continue
        p=astar(POS[a],POS[b],grid)
        DIST[(a,b)] = len(p)-1

def total_cost(order):
    remain=sum(WEIGHTS[x] for x in order)
    cost=0
    cur="受"
    for nxt in order:
        cost += DIST[(cur,nxt)] * remain
        remain -= WEIGHTS[nxt]
        cur = nxt
    return cost

best_order=min(permutations(["Ａ","Ｂ","Ｃ"]), key=total_cost)
delivery_queue=list(best_order)
delivery_queue.append("受")

print("📦 配送順序決定:", " → ".join(delivery_queue))

goal = POS[delivery_queue[0]]

# =============================
# step loop
# =============================
for step in range(200):

    sig1 = signal_color(step, 0)
    sig2 = signal_color(step, 25)

    vis=copy.deepcopy(grid)
    vis[agent[0]][agent[1]]=ARROW[dir]

    print("\n"+"="*40)
    print(f"STEP {step}  目標:{delivery_queue[0]}")
    print(f"🚦 信号①:{signal_name(sig1)} 信号②:{signal_name(sig2)} 横断中:{crossing}")

    for r in vis:
        print("".join(r))

    if agent==goal:
        print(f"✅ 到達:{delivery_queue[0]}")
        delivery_queue.pop(0)
        if not delivery_queue:
            print("🎉 全配送完了")
            break
        goal=POS[delivery_queue[0]]
        continue

    path=astar(agent,goal,grid)
    if len(path)<2:
        print("❌ 経路なし")
        break

    nr,nc=path[1]
    dr,dc=nr-agent[0],nc-agent[1]
    next_dir=[k for k,v in DIRS.items() if v==(dr,dc)][0]

    if mode=="ROTATE":
        if dir!=next_dir:
            dir=next_dir
            print("🔄 回転")
            mode="MOVE"
            time.sleep(0.2)
            continue
        mode="MOVE"

    if mode=="MOVE":

        if grid[nr][nc]=="◯":
            grid[nr][nc]="◎"
            print("👀 人 → 停止")
            time.sleep(0.3)
            continue

        if grid[nr][nc]=="＃":
            sig = sig1 if nr < 8 else sig2
            if sig==2 or (sig==1 and not crossing):
                print("🚦 信号 → 停止")
                time.sleep(0.3)
                continue
            crossing=True

        if crossing and grid[agent[0]][agent[1]]=="＃" and grid[nr][nc]!="＃":
            crossing=False
            print("🚶 横断完了")

        agent=(nr,nc)
        print("➡ 前進")
        mode="ROTATE"
        time.sleep(0.2)