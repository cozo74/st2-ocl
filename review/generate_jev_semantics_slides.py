#!/usr/bin/env python3
"""Generate vector slide artwork for the Jev state/candidate presentation."""

from __future__ import annotations

import html
import json
import math
import textwrap
from pathlib import Path

ROOT = Path(__file__).resolve().parent
OUT = ROOT / "jev-semantics-slides"
DATA = json.loads((ROOT / "jev-input-example.json").read_text(encoding="utf-8"))

W, H = 1600, 900
BG = "#F6F3EA"
INK = "#16202A"
MUTED = "#5D6873"
TEAL = "#087E8B"
BLUE = "#2D5BFF"
CYAN = "#42C4D8"
ORANGE = "#F59E0B"
RED = "#E45756"
GREEN = "#21A179"
ROAD = "#59636C"
WHITE = "#FFFFFF"
FONT = "Noto Sans CJK SC"
MONO = "Noto Sans Mono CJK SC"


def esc(value):
    return html.escape(str(value), quote=True)


class Slide:
    def __init__(self, title: str, kicker: str = "JevPilot · Structured driving state"):
        self.items = [
            f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}">',
            "<defs>",
            '<filter id="shadow" x="-20%" y="-20%" width="140%" height="140%"><feDropShadow dx="0" dy="8" stdDeviation="10" flood-opacity=".10"/></filter>',
            '<marker id="arrow-blue" markerWidth="12" markerHeight="12" refX="10" refY="6" orient="auto"><path d="M0,0 L12,6 L0,12 z" fill="#2D5BFF"/></marker>',
            '<marker id="arrow-teal" markerWidth="12" markerHeight="12" refX="10" refY="6" orient="auto"><path d="M0,0 L12,6 L0,12 z" fill="#087E8B"/></marker>',
            '<marker id="arrow-orange" markerWidth="12" markerHeight="12" refX="10" refY="6" orient="auto"><path d="M0,0 L12,6 L0,12 z" fill="#F59E0B"/></marker>',
            "</defs>",
            f'<rect width="{W}" height="{H}" fill="{BG}"/>',
            f'<rect x="0" y="0" width="18" height="{H}" fill="{TEAL}"/>',
        ]
        self.text(70, 48, kicker.upper(), 18, TEAL, weight=700, spacing=2)
        self.text(70, 84, title, 40, INK, weight=800)
        self.line(70, 145, 1530, 145, "#D9D5CA", 2)

    def rect(self, x, y, w, h, fill=WHITE, stroke="none", sw=1, r=22, shadow=False, opacity=1):
        filt = ' filter="url(#shadow)"' if shadow else ""
        self.items.append(
            f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="{r}" fill="{fill}" fill-opacity="{opacity}" stroke="{stroke}" stroke-width="{sw}"{filt}/>'
        )

    def line(self, x1, y1, x2, y2, color=INK, sw=3, dash=None, arrow=None, opacity=1):
        attrs = f' stroke-dasharray="{dash}"' if dash else ""
        if arrow:
            attrs += f' marker-end="url(#arrow-{arrow})"'
        self.items.append(
            f'<line x1="{x1}" y1="{y1}" x2="{x2}" y2="{y2}" stroke="{color}" stroke-width="{sw}" stroke-linecap="round" opacity="{opacity}"{attrs}/>'
        )

    def path(self, d, fill="none", stroke=INK, sw=3, dash=None, arrow=None, opacity=1):
        attrs = f' stroke-dasharray="{dash}"' if dash else ""
        if arrow:
            attrs += f' marker-end="url(#arrow-{arrow})"'
        self.items.append(
            f'<path d="{d}" fill="{fill}" stroke="{stroke}" stroke-width="{sw}" stroke-linecap="round" stroke-linejoin="round" opacity="{opacity}"{attrs}/>'
        )

    def circle(self, cx, cy, r, fill=WHITE, stroke="none", sw=1, opacity=1):
        self.items.append(
            f'<circle cx="{cx}" cy="{cy}" r="{r}" fill="{fill}" fill-opacity="{opacity}" stroke="{stroke}" stroke-width="{sw}"/>'
        )

    def text(self, x, y, value, size=24, color=INK, weight=400, anchor="start", family=FONT, spacing=0):
        self.items.append(
            f'<text x="{x}" y="{y}" font-family="{family}" font-size="{size}" font-weight="{weight}" fill="{color}" text-anchor="{anchor}" letter-spacing="{spacing}">{esc(value)}</text>'
        )

    def multiline(self, x, y, value, size=23, color=INK, weight=400, width=34, leading=1.42, bullet=False):
        lines = []
        for paragraph in str(value).split("\n"):
            # Chinese prose commonly has no spaces, so wrapping must be allowed
            # inside a long token. SVG itself does not auto-wrap text elements.
            lines.extend(
                textwrap.wrap(
                    paragraph,
                    width=width,
                    break_long_words=True,
                    break_on_hyphens=False,
                )
                or [""]
            )
        for i, line in enumerate(lines):
            prefix = "• " if bullet and i == 0 else ""
            self.text(x, y + i * size * leading, prefix + line, size, color, weight)
        return y + len(lines) * size * leading

    def pill(self, x, y, value, fill, color=WHITE, width=None):
        width = width or max(90, len(value) * 16 + 34)
        self.rect(x, y, width, 42, fill=fill, r=21)
        self.text(x + width / 2, y + 28, value, 18, color, 700, anchor="middle")

    def car(self, cx, cy, angle=0, scale=1, fill=BLUE, label=None):
        # A compact top-view car; zero degrees points upward.
        g = [
            f'<g transform="translate({cx} {cy}) rotate({angle}) scale({scale})">',
            f'<rect x="-32" y="-60" width="64" height="120" rx="22" fill="{fill}" stroke="{WHITE}" stroke-width="4"/>',
            '<path d="M-23,-18 L-18,-42 L18,-42 L23,-18 Z" fill="#DCEEFF" opacity=".85"/>',
            '<path d="M-16,48 L0,60 L16,48" fill="#FFFFFF" opacity=".9"/>',
            '</g>',
        ]
        self.items.extend(g)
        if label:
            self.text(cx, cy + 88 * scale, label, 18, INK, 700, anchor="middle")

    def footer(self, page, source="Current source code + review/jev-input-example.json"):
        self.text(70, 866, source, 14, MUTED)
        self.text(1530, 866, f"{page:02d}", 16, MUTED, 700, anchor="end")

    def save(self, name):
        self.items.append("</svg>")
        (OUT / name).write_text("\n".join(self.items), encoding="utf-8")


def slide_01():
    s = Slide("从仿真坐标到 Jev 候选决策", "JevPilot · Technical walkthrough")
    s.text(76, 235, "位置 · 角度 · 道路边界 · 交通感知 · 候选轨迹", 34, INK, 700)
    s.text(76, 292, "把连续驾驶世界压缩成模型可选择、系统可校验的结构化状态", 24, MUTED)
    s.rect(76, 374, 1448, 300, fill="#102D35", r=34, shadow=True)
    labels = [("WORLD", "全局 x/z"), ("EGO", "right/ahead"), ("SENSE", "boundaries + traffic"), ("PLAN", "3 s rollout"), ("JEV", "选择 vN")]
    for i, (a, b) in enumerate(labels):
        x = 130 + i * 280
        s.circle(x + 70, 506, 64, fill=[TEAL, BLUE, CYAN, ORANGE, GREEN][i])
        s.text(x + 70, 500, a, 19, WHITE, 800, anchor="middle")
        s.text(x + 70, 535, b, 17, WHITE, 500, anchor="middle")
        if i < 4:
            s.line(x + 142, 506, x + 268, 506, CYAN, 5, arrow="teal")
    s.text(800, 741, "核心结论：Jev 不生成任意控制量，只从本地验证过的候选中选择。", 25, TEAL, 800, anchor="middle")
    s.footer(1)
    s.save("01-title.svg")


def slide_02():
    s = Slide("一张图看懂状态生成链路")
    stages = [
        ("① 世界真值", "x / z / heading\n道路图、车辆、行人", TEAL),
        ("② 自车坐标", "right / ahead\nrelative heading", BLUE),
        ("③ 感知筛选", "距离、视场、遮挡\n动态目标优先", CYAN),
        ("④ 候选 rollout", "局部采样 55 条\n预测未来 3 秒", ORANGE),
        ("⑤ Jev 输入", "表格压缩\n选择 v0…vN", GREEN),
    ]
    for i, (title, body, color) in enumerate(stages):
        x = 70 + i * 302
        s.rect(x, 235, 252, 320, fill=WHITE, stroke="#DED9CD", sw=2, r=24, shadow=True)
        s.rect(x, 235, 252, 70, fill=color, r=24)
        s.rect(x, 280, 252, 25, fill=color, r=0)
        s.text(x + 126, 280, title, 22, WHITE, 800, anchor="middle")
        s.multiline(x + 26, 360, body, 22, INK, 600, width=18, leading=1.65)
        if i < 4:
            s.line(x + 260, 395, x + 294, 395, BLUE, 4, arrow="blue")
    s.rect(280, 650, 1040, 100, fill="#E7F5F4", stroke="#B8DEDA", sw=2, r=22)
    s.text(800, 691, "三个坐标语义必须保持一致", 23, TEAL, 800, anchor="middle")
    s.text(800, 729, "世界坐标用于仿真 · 自车坐标用于模型理解 · route station 用于沿路线度量", 22, INK, 600, anchor="middle")
    s.footer(2)
    s.save("02-pipeline.svg")


def slide_03_state_layers():
    s = Slide("状态模型不是一个 JSON，而是三层派生视图")
    layers = [
        ("① 内部世界真值", "Simulation", "道路图、实体姿态、交通规则记忆、运行时状态", TEAL, 200, 670),
        ("② 规划与观察", "decisionState / observation", "局部道路、感知目标、导航、候选轨迹、遥测", BLUE, 340, 530),
        ("③ Jev 请求", "prepareJevRequest", "压缩表格、候选别名、需要模型回答的问题", ORANGE, 480, 390),
    ]
    for title, api, body, color, y, width in layers:
        x = 800 - width / 2
        s.rect(x, y, width, 125, fill=color, r=26, shadow=True)
        s.text(800, y + 38, title, 23, WHITE, 800, anchor="middle")
        s.text(800, y + 72, api, 19, "#E5FBFF", 700, anchor="middle", family=MONO)
        s.text(800, y + 104, body, 17, WHITE, 500, anchor="middle")
        if y < 480:
            s.line(800, y + 130, 800, y + 170, TEAL, 4, arrow="teal")
    s.rect(80, 690, 1440, 105, fill="#102D35", r=24)
    s.text(120, 732, "权威状态", 21, CYAN, 800)
    s.text(250, 732, "只存在于 Simulation；Three.js 场景和 Jev JSON 都是它的投影。", 22, WHITE, 650)
    s.text(120, 773, "设计意义", 21, CYAN, 800)
    s.text(250, 773, "渲染不能反向修改物理状态；模型也看不到完整世界真值。", 22, WHITE, 650)
    s.footer(3, "src/simulation.js Simulation / observation() / decisionState()")
    s.save("03-state-layers.svg")


def slide_04_world_model():
    s = Slide("World：道路拓扑 + 连续几何 + 静态对象")
    # Small graph
    s.rect(70, 190, 700, 600, fill="#EAF0EF", stroke="#CDD9D6", sw=2, r=26)
    nodes = [(190, 330, "j0"), (420, 265, "j1"), (625, 350, "j2"), (345, 565, "j3"), (610, 650, "j4")]
    edges = [(0,1),(1,2),(0,3),(1,3),(2,4),(3,4)]
    for a,b in edges:
        x1,y1,_=nodes[a]; x2,y2,_=nodes[b]
        s.line(x1,y1,x2,y2,ROAD,18)
        s.line(x1,y1,x2,y2,"#F7E6A6",2,dash="12 12")
    for x,y,label in nodes:
        s.circle(x,y,31,TEAL,WHITE,3)
        s.text(x,y+7,label,18,WHITE,800,anchor="middle",family=MONO)
    s.path("M190,330 C260,300 340,278 420,265 C500,270 565,305 625,350 C635,465 625,560 610,650", stroke=BLUE, sw=8, opacity=.9)
    s.text(115, 235, "nodes + edges：可达性与最短路径", 21, INK, 750)
    s.pill(118, 710, "route.points (x, z, s)", BLUE, width=300)
    s.text(440, 738, "把离散道路图变成车辆可跟随的连续曲线", 20, INK, 600)
    # right side cards
    cards = [
        ("nodes / byId", "路口坐标、邻接关系、stop/signal 控制", TEAL),
        ("edges", "宽度、限速、道路名称、方向、几何 path", BLUE),
        ("route", "ids、连续 points、crossings、高速 sections", ORANGE),
        ("objects", "building、tree、sign、light、overpass…", GREEN),
        ("metadata", "seed、type、theme、bounds、destination", RED),
    ]
    for i,(a,b,c) in enumerate(cards):
        y=190+i*122
        s.rect(830,y,700,100,fill=WHITE,stroke="#DDD7CC",sw=2,r=20,shadow=True)
        s.rect(830,y,12,100,fill=c,r=6)
        s.text(870,y+38,a,22,INK,800,family=MONO)
        s.text(870,y+75,b,19,MUTED,500)
    s.footer(4, "src/world.js generateWorld()/makeRoute() · src/highway.js")
    s.save("04-world-model.svg")


def slide_05_dynamic_model():
    s = Slide("动态状态：实体自身状态 + 跨实体交通关系")
    columns = [
        ("player", ["pose: x,z,heading", "motion: speed,steering,target", "navigation: route,s", "memory: stops,intersectionMemory"], BLUE),
        ("traffic[]", ["car / motorcycle", "pose + speed", "各自 route + s", "stops + waitingSince"], TEAL),
        ("pedestrians[]", ["pose + speed", "walkPath + progress", "crossing / walking", "nodeId + direction"], ORANGE),
    ]
    for i,(title,rows,color) in enumerate(columns):
        x=70+i*505
        s.rect(x,195,450,360,fill=WHITE,stroke="#DDD7CC",sw=2,r=24,shadow=True)
        s.rect(x,195,450,66,fill=color,r=24)
        s.rect(x,235,450,26,fill=color,r=0)
        s.text(x+225,238,title,25,WHITE,850,anchor="middle",family=MONO)
        for j,row in enumerate(rows):
            y=310+j*58
            s.circle(x+42,y-7,7,color)
            s.text(x+62,y,row,20,INK,550,family=MONO if ':' in row else FONT)
    s.text(70, 620, "跨实体关系", 25, TEAL, 850)
    relations = [
        ("locks", "路口占用：nodeId → vehicleId", TEAL),
        ("courtesy", "死锁释放：临时授予一辆车通行权", BLUE),
        ("perception", "当前可见对象的自车相对摘要", CYAN),
        ("lastPlan", "道路状态、vectors 与完整 projections", ORANGE),
        ("telemetry", "碰撞、违规、里程、事件与到达状态", RED),
    ]
    for i,(a,b,c) in enumerate(relations):
        x=70+(i%3)*505; y=660+(i//3)*82
        s.pill(x,y,a,c,width=145)
        s.text(x+165,y+28,b,18,INK,550)
    s.footer(5, "src/simulation.js reset()/step() · plain objects + Array/Map/Set")
    s.save("05-dynamic-model.svg")


def slide_06_state_views():
    s = Slide("同一世界状态，按使用者生成三种视图")
    s.rect(140, 180, 1320, 165, fill="#102D35", r=28, shadow=True)
    s.text(800, 226, "Simulation · 完整且可变的权威状态", 27, CYAN, 900, anchor="middle", family=MONO)
    core=["world","player","traffic[]","pedestrians[]","locks / courtesy","perception / lastPlan"]
    for i,item in enumerate(core):
        x=205+i*205
        s.pill(x,270,item,TEAL if i<4 else BLUE,width=178)
    views=[
        ("observation(full)", "Web / 调试", ["frame + ego", "sensor + telemetry", "可选完整 world"], TEAL),
        ("decisionState()", "本地规划", ["road + lane", "scene + traffic", "vectors + constraints"], BLUE),
        ("prepareJevRequest()", "模型输入", ["共享字段压缩", "候选匿名为 vN", "只保留选择题"], ORANGE),
    ]
    for i,(api,user,rows,color) in enumerate(views):
        x=70+i*510
        center=x+220
        s.line(800,345,center,390,color,4,arrow="teal" if color==TEAL else "blue" if color==BLUE else "orange")
        s.rect(x,400,440,330,fill=WHITE,stroke=color,sw=3,r=26,shadow=True)
        s.text(center,450,user,22,color,850,anchor="middle")
        s.multiline(x+35,500,api,19,INK,750,width=30,leading=1.25)
        for j,row in enumerate(rows):
            y=570+j*48
            s.circle(x+48,y-7,8,color)
            s.text(x+70,y,row,19,INK,550)
        s.text(center,700,["信息最全","服务于控制","最小必要信息"][i],18,MUTED,700,anchor="middle")
    s.rect(250,765,1100,60,fill="#E7F5F4",r=22)
    s.text(800,803,"派生方向是单向的：内部真值 → 视图；视图不能替代完整世界状态。",22,TEAL,800,anchor="middle")
    s.footer(6, "src/simulation.js observation()/decisionState() · src/jev-request.js")
    s.save("06-state-views.svg")


def slide_03():
    s = Slide("位置坐标规则：世界坐标系")
    # grid/map
    s.rect(70, 190, 850, 600, fill="#E9EFEA", stroke="#CDD7D0", sw=2, r=26)
    for x in range(130, 900, 85):
        s.line(x, 220, x, 755, "#D0D8D3", 1)
    for y in range(245, 760, 85):
        s.line(105, y, 890, y, "#D0D8D3", 1)
    ox, oy = 500, 500
    s.circle(ox, oy, 8, INK)
    s.line(ox, oy, 790, oy, BLUE, 5, arrow="blue")
    s.text(805, oy + 8, "+x 东 / East", 23, BLUE, 800)
    s.line(ox, oy, ox, 735, ORANGE, 5, arrow="orange")
    s.text(ox + 18, 748, "+z 南 / South", 23, ORANGE, 800)
    s.line(ox, oy, ox, 255, TEAL, 4, arrow="teal")
    s.text(ox + 18, 260, "heading = 0° 北", 22, TEAL, 800)
    s.car(ox, oy - 35, 0, .72, BLUE, "ego (x, z)")
    # explanation cards
    s.rect(970, 190, 560, 175, fill=WHITE, stroke="#DDD7CC", sw=2, r=24)
    s.text(1010, 238, "二维地面平面", 24, TEAL, 800)
    s.multiline(1010, 282, "车辆移动使用 x 和 z；Three.js 的 y 只表示高度。", 22, INK, 500, width=30)
    s.rect(970, 390, 560, 180, fill=WHITE, stroke="#DDD7CC", sw=2, r=24)
    s.text(1010, 438, "沿航向移动 d 米", 24, BLUE, 800)
    s.text(1010, 487, "x′ = x + sin(h) · d", 23, INK, 650, family=MONO)
    s.text(1010, 529, "z′ = z − cos(h) · d", 23, INK, 650, family=MONO)
    s.rect(970, 595, 560, 195, fill="#FFF4DF", stroke="#F2D39A", sw=2, r=24)
    s.text(1010, 643, "讲解提醒", 24, ORANGE, 800)
    s.multiline(1010, 687, "屏幕俯视图常把“向上”画成北，所以北向移动时 z 反而减小。", 22, INK, 500, width=30)
    s.footer(7, "src/math.js · src/simulation.js observation.frame.coordinates")
    s.save("07-world-coordinates.svg")


def slide_04():
    s = Slide("位置坐标规则：转换到自车坐标")
    s.rect(70, 190, 870, 610, fill="#EFF3F6", stroke="#D5DDE3", sw=2, r=26)
    cx, cy = 460, 550
    s.car(cx, cy, 35, .9, BLUE, "ego")
    # local axes approximately rotated: ahead NE/up, right SE/right
    s.line(cx, cy, 650, 320, TEAL, 5, arrow="teal")
    s.text(665, 310, "+ahead", 24, TEAL, 800)
    s.line(cx, cy, 690, 735, ORANGE, 5, arrow="orange")
    s.text(705, 755, "+right", 24, ORANGE, 800)
    tx, ty = 710, 380
    s.circle(tx, ty, 24, RED)
    s.text(tx, ty + 8, "T", 20, WHITE, 800, anchor="middle")
    s.line(cx, cy, tx, ty, RED, 3, dash="10 8")
    s.pill(110, 225, "输出永远是 [right, ahead]", TEAL, width=360)
    s.text(110, 302, "目标在左侧 → right < 0", 22, INK, 600)
    s.text(110, 341, "目标在后方 → ahead < 0", 22, INK, 600)
    s.rect(990, 190, 540, 285, fill=WHITE, stroke="#DDD7CC", sw=2, r=24)
    s.text(1030, 238, "世界差值", 24, TEAL, 800)
    s.text(1030, 282, "dx = target.x − ego.x", 22, INK, 600, family=MONO)
    s.text(1030, 320, "dz = target.z − ego.z", 22, INK, 600, family=MONO)
    s.text(1030, 375, "right = dx·cos(h) + dz·sin(h)", 20, BLUE, 700, family=MONO)
    s.text(1030, 415, "ahead = dx·sin(h) − dz·cos(h)", 20, BLUE, 700, family=MONO)
    s.rect(990, 505, 540, 295, fill="#E7F5F4", stroke="#B8DEDA", sw=2, r=24)
    s.text(1030, 553, "为什么发送局部坐标？", 24, TEAL, 800)
    points = ["与地图绝对位置无关", "车辆转向后语义仍一致", "直接回答：在左/右、前/后多远", "boundaries、traffic、candidate 终点统一"]
    for i, p in enumerate(points):
        s.circle(1045, 612 + i * 45, 7, TEAL)
        s.text(1065, 619 + i * 45, p, 21, INK, 550)
    s.footer(8, "src/road-geometry.js relativePoint() · src/jev-request.js destination conversion")
    s.save("08-ego-coordinates.svg")


def slide_05():
    s = Slide("角度规则：航向、相对航向与转向")
    cx, cy, r = 410, 485, 235
    s.circle(cx, cy, r, fill="#EDF4F3", stroke="#BFD8D5", sw=3)
    s.line(cx, cy, cx, cy - r + 20, TEAL, 4, arrow="teal")
    s.line(cx, cy, cx + r - 20, cy, BLUE, 4, arrow="blue")
    s.line(cx, cy, cx, cy + r - 20, RED, 4, arrow="orange")
    s.line(cx, cy, cx - r + 20, cy, ORANGE, 4)
    s.car(cx, cy, 0, .65, BLUE)
    s.text(cx, cy - r - 20, "0° 北", 23, TEAL, 800, anchor="middle")
    s.text(cx + r + 40, cy + 8, "+90° 东", 23, BLUE, 800)
    s.text(cx, cy + r + 42, "±180° 南", 23, RED, 800, anchor="middle")
    s.text(cx - r - 45, cy + 8, "−90° 西", 23, ORANGE, 800, anchor="end")
    s.rect(760, 190, 770, 175, fill=WHITE, stroke="#DDD7CC", sw=2, r=24)
    s.text(800, 238, "航向定义", 24, TEAL, 800)
    s.text(800, 286, "heading(a,b) = atan2(Δx, −Δz)", 23, INK, 700, family=MONO)
    s.text(800, 328, "angle(h) = atan2(sin h, cos h) → [−π, π]", 20, MUTED, 600, family=MONO)
    s.rect(760, 390, 770, 175, fill="#EFF3FF", stroke="#C7D2FF", sw=2, r=24)
    s.text(800, 438, "相对航向", 24, BLUE, 800)
    s.text(800, 485, "heading_relative = other.heading − ego.heading", 21, INK, 650, family=MONO)
    s.text(800, 528, "0° 同向 · ±90° 横穿 · ±180° 对向", 22, BLUE, 700)
    s.rect(760, 590, 770, 200, fill="#FFF4DF", stroke="#F2D39A", sw=2, r=24)
    s.text(800, 638, "转向符号", 24, ORANGE, 800)
    s.text(800, 686, "positive steering → 右转 → heading 增大", 22, INK, 650)
    s.text(800, 728, "负速度倒车时，同一方向盘输入会产生相反航向变化", 20, MUTED, 500)
    s.footer(9, "src/math.js heading()/angle() · src/planning.js integratePose()")
    s.save("09-angle-rules.svg")


def slide_06():
    s = Slide("boundaries：沿路线观察真实沥青边界")
    # curving road
    s.path("M160,745 C260,650 260,400 430,300 C580,210 760,310 850,190", stroke="#D1D4D1", sw=245)
    s.path("M160,745 C260,650 260,400 430,300 C580,210 760,310 850,190", stroke=ROAD, sw=185)
    s.path("M160,745 C260,650 260,400 430,300 C580,210 760,310 850,190", stroke="#F8E7A1", sw=4, dash="20 18")
    # stations and car
    for x1, y1, x2, y2 in [(125,685,300,750),(210,500,390,540),(385,280,500,410),(610,235,650,405),(760,180,880,290)]:
        s.line(x1, y1, x2, y2, CYAN, 3, opacity=.8)
    s.car(190, 690, -33, .55, BLUE, "ego")
    s.text(445, 250, "route center", 20, ORANGE, 800)
    s.text(95, 410, "actual asphalt left", 19, WHITE, 700)
    s.text(665, 430, "actual asphalt right", 19, WHITE, 700)
    s.rect(955, 190, 575, 330, fill=WHITE, stroke="#DDD7CC", sw=2, r=24)
    s.text(995, 238, "每个采样站包含 3 个点", 24, TEAL, 800)
    fields = [("center", "路线中心"), ("road_left", "横截面左侧沥青边缘"), ("road_right", "横截面右侧沥青边缘")]
    for i, (a, b) in enumerate(fields):
        y = 300 + i * 70
        s.pill(995, y - 30, a, [ORANGE, TEAL, BLUE][i], width=165)
        s.text(1180, y, b, 21, INK, 550)
    s.text(995, 480, "全部转换为 [right, ahead]", 21, MUTED, 650)
    s.rect(955, 550, 575, 240, fill="#E7F5F4", stroke="#B8DEDA", sw=2, r=24)
    s.text(995, 598, "不是车道线，也不是道路包围盒", 23, TEAL, 800)
    s.multiline(995, 644, "边界来自可行驶沥青多边形的横截面；路口处先合并相连路面，高速公路中央隔离带仍保持不可行驶。", 21, INK, 500, width=31)
    s.footer(10, "src/road-geometry.js roadCrossSection()/roadState()")
    s.save("10-boundaries-semantics.svg")


def slide_07():
    s = Slide("boundaries：从几何采样到紧凑 JSON")
    pipeline = [
        ("道路多边形", "localRoads", TEAL),
        ("路线采样站", "后方车长 + 当前 + 前方 12 点", BLUE),
        ("横截面求交", "选择中心所在/最近连通路面", CYAN),
        ("自车坐标", "center / left / right", ORANGE),
        ("折线简化", "误差上限 0.1 m", GREEN),
        ("table 压缩", "shared + columns + rows", RED),
    ]
    for i, (a, b, c) in enumerate(pipeline):
        x = 70 + (i % 3) * 500
        y = 205 + (i // 3) * 240
        s.rect(x, y, 430, 155, fill=WHITE, stroke="#DDD7CC", sw=2, r=22, shadow=True)
        s.circle(x + 55, y + 52, 28, c)
        s.text(x + 55, y + 60, str(i + 1), 22, WHITE, 800, anchor="middle")
        s.text(x + 100, y + 53, a, 23, INK, 800)
        s.text(x + 100, y + 96, b, 19, MUTED, 500)
        if i in (0, 1, 3, 4):
            s.line(x + 440, y + 78, x + 482, y + 78, BLUE, 4, arrow="blue")
        if i == 2:
            s.path(f"M{x+215},{y+165} L{x+215},{y+205} L{570},{y+205} L{570},{y+235}", stroke=BLUE, sw=4, arrow="blue")
    sample = DATA["state"]["road"]["boundaries"]
    s.rect(170, 700, 1260, 102, fill="#102D35", r=20)
    s.text(205, 741, "示例", 19, CYAN, 800)
    s.text(295, 741, "row 0", 18, WHITE, 700, family=MONO)
    s.text(390, 741, str(sample["rows"][0]), 18, WHITE, 500, family=MONO)
    s.text(295, 779, "row 1", 18, WHITE, 700, family=MONO)
    s.text(390, 779, str(sample["rows"][1]), 18, WHITE, 500, family=MONO)
    s.text(1180, 760, "直路只需端点", 19, CYAN, 700)
    s.footer(11, "src/road-geometry.js:291–387 · src/jev-request.js boundaryRows()/table()")
    s.save("11-boundaries-pipeline.svg")


def slide_08():
    s = Slide("traffic 感知：谁会进入 Jev 输入？")
    # funnel
    stages = [
        ("候选对象", "traffic + pedestrians + world objects", 210, TEAL),
        ("类型过滤", "车 / 摩托 / 行人 / 建筑 / 标志 / 信号灯", 370, BLUE),
        ("距离与视场", "range=max(80, |speed|×6)；动态目标 360°", 530, CYAN),
        ("建筑遮挡", "blockedByBuilding：视线穿过建筑则剔除", 690, ORANGE),
    ]
    widths = [810, 680, 550, 420]
    for i, (a, b, y, c) in enumerate(stages):
        w = widths[i]
        x = 530 - w / 2
        s.path(f"M{x},{y} L{x+w},{y} L{x+w-60},{y+115} L{x+60},{y+115} Z", fill=c, stroke="none")
        s.text(530, y + 43, a, 24, WHITE, 800, anchor="middle")
        s.text(530, y + 82, b, 18, WHITE, 500, anchor="middle")
    s.line(530, 808, 530, 838, TEAL, 4, arrow="teal")
    s.rect(970, 205, 560, 265, fill=WHITE, stroke="#DDD7CC", sw=2, r=24)
    s.text(1010, 253, "转换为自车相对状态", 24, TEAL, 800)
    keys = ["right_m / ahead_m", "speed_mps", "heading_relative_deg", "relative_velocity_ahead_mps"]
    for i, key in enumerate(keys):
        s.pill(1010, 292 + i * 44, key, [BLUE, TEAL, ORANGE, RED][i], width=360)
    s.rect(970, 500, 560, 300, fill="#EFF3FF", stroke="#C7D2FF", sw=2, r=24)
    s.text(1010, 548, "进入模型前再裁剪", 24, BLUE, 800)
    s.multiline(1010, 595, "仅动态目标进入 traffic 表；优先保留冲突目标、前车和后车，然后最多取 10 个。", 22, INK, 550, width=29)
    s.text(1010, 710, "静态控制设施", 20, MUTED, 700)
    s.text(1010, 748, "→ intersection / road 状态，而非 traffic 行", 20, MUTED, 500)
    s.footer(12, "src/simulation.js scanScene()/decisionState()")
    s.save("12-traffic-perception.svg")


def slide_09():
    s = Slide("traffic 示例：一行数据怎样读")
    s.rect(70, 190, 850, 610, fill="#E8EEF0", stroke="#CCD6DA", sw=2, r=26)
    # road and ego
    s.rect(325, 210, 340, 560, fill=ROAD, r=8)
    s.line(495, 220, 495, 760, "#F7E6A6", 4, dash="22 18")
    s.car(520, 675, 0, .62, BLUE, "ego")
    # pedestrian at left 10, ahead 44.7 (visual)
    s.circle(420, 350, 24, ORANGE)
    s.text(420, 358, "P", 20, WHITE, 800, anchor="middle")
    s.line(520, 630, 420, 350, ORANGE, 3, dash="10 8")
    s.text(190, 334, "pedestrian-9", 21, ORANGE, 800)
    s.text(155, 374, "left 10 m", 19, INK, 600)
    s.text(155, 406, "ahead 44.7 m", 19, INK, 600)
    s.path("M420,350 C365,350 365,430 420,430", stroke=RED, sw=4, arrow="orange")
    s.text(170, 460, "heading ≈ −180°", 19, RED, 700)
    s.rect(970, 190, 560, 210, fill="#102D35", r=24)
    s.text(1010, 238, "traffic row", 20, CYAN, 800)
    row = DATA["state"]["traffic"]["rows"][1]
    s.multiline(1010, 292, json.dumps(row, ensure_ascii=False), 21, WHITE, 550, width=34)
    s.rect(970, 430, 560, 185, fill=WHITE, stroke="#DDD7CC", sw=2, r=24)
    s.text(1010, 478, "shared 字段", 23, TEAL, 800)
    s.text(1010, 527, 'speed = 0', 21, INK, 650, family=MONO)
    s.text(1010, 566, 'relative_forward_speed = 0', 21, INK, 650, family=MONO)
    s.rect(970, 645, 560, 155, fill="#FFF4DF", stroke="#F2D39A", sw=2, r=24)
    s.text(1010, 693, "表格压缩不丢语义", 23, ORANGE, 800)
    s.multiline(1010, 736, "columns 决定数组位置；shared 对所有 rows 生效。", 20, INK, 500, width=31)
    s.footer(13, "review/jev-input-example.json · src/jev-request.js table()")
    s.save("13-traffic-example.svg")


def slide_10():
    s = Slide("candidate 生成：先广泛采样，再筛出可讲的 12 条")
    boxes = [
        ("① 环境分析", "道路占用、route、停车线、前车、限速", TEAL),
        ("② 生成池", "正常约 55 条；脱困 11 条", BLUE),
        ("③ 3 秒 rollout", "60 步积分；第 24 步作短期评价", CYAN),
        ("④ 全路径评价", "道路、车道、路线、碰撞、停止线", ORANGE),
        ("⑤ 排序与覆盖", "最优 3 + 横向 4 + 探索 4", GREEN),
        ("⑥ 加入停车", "最终最多 12 条候选", RED),
    ]
    for i, (a, b, c) in enumerate(boxes):
        x = 80 + (i % 3) * 505
        y = 200 + (i // 3) * 240
        s.rect(x, y, 450, 170, fill=WHITE, stroke="#DDD7CC", sw=2, r=24, shadow=True)
        s.rect(x, y, 14, 170, fill=c, r=7)
        s.text(x + 42, y + 57, a, 24, INK, 800)
        s.multiline(x + 42, y + 105, b, 20, MUTED, 500, width=28)
        if i in (0, 1, 3, 4):
            s.line(x + 456, y + 85, x + 494, y + 85, BLUE, 4, arrow="blue")
        if i == 2:
            s.path(f"M{x+225},{y+180} L{x+225},{y+218} L{300},{y+218} L{300},{y+235}", stroke=BLUE, sw=4, arrow="blue")
    s.rect(205, 700, 1190, 100, fill="#102D35", r=22)
    s.text(800, 741, "正常道路：围绕路线导向转角采样 + 少量宽范围探索", 22, WHITE, 700, anchor="middle")
    s.text(800, 779, "脱困模式：覆盖完整转向范围，并允许 ±2 m/s 前进/倒车", 20, CYAN, 650, anchor="middle")
    s.footer(14, "src/driving-plan.js createDrivingPlan() · src/planning.js projectVector()")
    s.save("14-candidate-generation.svg")


def slide_11():
    s = Slide("candidate 评价：整车、整条轨迹、动态障碍物")
    s.rect(70, 190, 920, 610, fill="#EAF0EF", stroke="#CDD9D6", sw=2, r=26)
    # candidate fan
    cx, cy = 500, 725
    s.car(cx, cy, 0, .58, BLUE, "ego")
    paths = [
        ("M500,675 C500,560 430,420 300,260", ORANGE, "off-road"),
        ("M500,675 C500,550 480,410 470,245", CYAN, "lane-left"),
        ("M500,675 C500,540 520,400 540,240", BLUE, "best"),
        ("M500,675 C510,550 620,430 760,295", TEAL, "lane-right"),
        ("M500,675 C535,565 735,535 860,475", RED, "collision"),
    ]
    for d, c, label in paths:
        s.path(d, stroke=c, sw=7, opacity=.92)
    s.circle(823, 490, 28, RED)
    s.text(823, 498, "X", 20, WHITE, 800, anchor="middle")
    s.text(165, 250, "离路惩罚", 19, ORANGE, 800)
    s.text(450, 225, "低误差", 19, BLUE, 800)
    s.text(765, 275, "横向覆盖", 19, TEAL, 800)
    s.text(835, 445, "预测碰撞", 19, RED, 800)
    s.rect(1040, 190, 490, 610, fill=WHITE, stroke="#DDD7CC", sw=2, r=26)
    s.text(1080, 240, "评价指标", 25, TEAL, 800)
    metrics = [
        ("道路", "整车 footprint 在沥青内", TEAL),
        ("车道", "offset / heading / lane excess", BLUE),
        ("路线", "progress 与 route error", CYAN),
        ("停止线", "crosses_line / line_after", ORANGE),
        ("碰撞", "swept OBB + 动态预测", RED),
        ("舒适性", "转向变化与速度上限", GREEN),
    ]
    for i, (a, b, c) in enumerate(metrics):
        y = 300 + i * 73
        s.circle(1092, y - 7, 10, c)
        s.text(1115, y, a, 21, INK, 800)
        s.text(1205, y, b, 18, MUTED, 500)
    s.text(1080, 760, "碰撞临近项具有最高惩罚权重", 19, RED, 700)
    s.footer(15, "src/driving-plan.js evaluate() · src/collisions.js firstCollision()")
    s.save("15-candidate-evaluation.svg")


def slide_12():
    s = Slide("candidate 示例：把 v3 讲成一条驾驶意图")
    v3 = DATA["state"]["candidates"]["rows"]["v3"]
    cols = DATA["state"]["candidates"]["columns"]
    values = dict(zip(cols, v3))
    s.rect(70, 190, 1460, 145, fill="#102D35", r=24)
    s.text(110, 240, "v3", 34, CYAN, 900, family=MONO)
    s.text(205, 240, str(v3), 21, WHITE, 550, family=MONO)
    s.text(110, 295, "columns 决定每个数字的语义；shared 还补充 crosses_line=false、stop_at_line=false", 19, "#C6D8DC", 500)
    cards = [
        ("目标速度", f'{values["speed"]} m/s', "动作意图", BLUE),
        ("3 秒终速", f'{values["end_speed"]} m/s', "预测结果", TEAL),
        ("路线进度", f'{values["progress"]} m', "向前推进", GREEN),
        ("路线 / 车道误差", f'{values["route_error"]} / {values["lane_error"]} m', "轻微偏离", ORANGE),
        ("航向误差", f'{values["heading_error"]}°', "基本对齐", CYAN),
        ("预测终点", f'right {values["end_right"]} m\nahead {values["end_ahead"]} m', "左前方", RED),
    ]
    for i, (a, b, c, color) in enumerate(cards):
        x = 70 + (i % 3) * 500
        y = 385 + (i // 3) * 205
        s.rect(x, y, 450, 165, fill=WHITE, stroke="#DDD7CC", sw=2, r=22, shadow=True)
        s.rect(x, y, 12, 165, fill=color, r=6)
        s.text(x + 40, y + 48, a, 21, MUTED, 700)
        s.multiline(x + 40, y + 92, b, 25, INK, 800, width=22, leading=1.25)
        s.pill(x + 310, y + 110, c, color, width=115)
    s.rect(280, 795, 1040, 48, fill="#E7F5F4", r=20)
    s.text(800, 827, "讲法：v3 是“一条略向左、保持高速、继续接近停车线”的可执行轨迹。", 21, TEAL, 800, anchor="middle")
    s.footer(16, "review/jev-input-example.json state.candidates.rows.v3")
    s.save("16-candidate-example.svg")


def slide_13():
    s = Slide("模型与本地系统的职责边界")
    s.rect(90, 205, 650, 500, fill="#E7F5F4", stroke="#B8DEDA", sw=3, r=30, shadow=True)
    s.text(415, 260, "本地确定性系统", 29, TEAL, 900, anchor="middle")
    local = ["世界坐标与相对坐标转换", "道路边界和车身占用计算", "交通感知与遮挡过滤", "候选生成、rollout 和碰撞预测", "候选合法性与响应一致性校验"]
    for i, item in enumerate(local):
        y = 330 + i * 67
        s.circle(145, y - 7, 9, TEAL)
        s.text(170, y, item, 22, INK, 600)
    s.rect(860, 205, 650, 500, fill="#EFF3FF", stroke="#C7D2FF", sw=3, r=30, shadow=True)
    s.text(1185, 260, "Jev 决策模型", 29, BLUE, 900, anchor="middle")
    model = ["理解紧凑结构化状态", "权衡速度、路线、车道与交通", "在 motion / vector / route 问题中选择", "输出每个选项的概率", "不直接生成未经验证的控制量"]
    for i, item in enumerate(model):
        y = 330 + i * 67
        s.circle(915, y - 7, 9, BLUE)
        s.text(940, y, item, 22, INK, 600)
    s.line(742, 455, 850, 455, ORANGE, 6, arrow="orange")
    s.text(796, 425, "candidates", 17, ORANGE, 800, anchor="middle")
    s.line(850, 520, 742, 520, GREEN, 6, arrow="teal")
    s.text(796, 566, "choice vN", 17, GREEN, 800, anchor="middle")
    s.rect(250, 755, 1100, 65, fill="#102D35", r=22)
    s.text(800, 797, "最终安全边界：模型负责“选哪条”，本地负责“哪些条能选、选后如何执行”。", 23, WHITE, 800, anchor="middle")
    s.footer(27, "src/jev-request.js · server/jev.js · src/planning.js decisionControls()")
    s.save("27-responsibility-boundary.svg")


def slide_17_candidate_sets():
    s = Slide("候选集合不是一次生成，而是逐层收缩")
    stages = [
        ("生成池", "正常约 55\n脱困 11", "采样与 3 s rollout", BLUE),
        ("展示 vectors", "最多 12", "可含探索性越线\n或未来风险", CYAN),
        ("eligible moving", "硬过滤", "非零速、可恢复\n无临近碰撞", TEAL),
        ("choice set", "场景约束", "上路、顺路线\n排队兼容", ORANGE),
        ("Jev aliases", "v0…vN", "只发送\n可选择项", GREEN),
        ("selected", "唯一一条", "控制由候选反推", RED),
    ]
    for i, (title, count, body, color) in enumerate(stages):
        x = 55 + i * 255
        w = 215
        y = 235 + (i % 2) * 68
        s.rect(x, y, w, 295, fill=WHITE, stroke=color, sw=3, r=24, shadow=True)
        s.rect(x, y, w, 66, fill=color, r=23)
        s.rect(x, y + 42, w, 24, fill=color, r=0)
        s.text(x + w / 2, y + 43, title, 21, WHITE, 850, anchor="middle")
        count_lines = count.split("\n")
        for line_index, count_line in enumerate(count_lines):
            s.text(x + w / 2, y + 105 + line_index * 32, count_line, 22, color, 900, anchor="middle", family=MONO)
        s.multiline(x + 24, y + 170, body, 19, INK, 560, width=14, leading=1.45)
        if i < len(stages) - 1:
            s.line(x + w + 6, y + 148, x + 249, 303 + ((i + 1) % 2) * 68, BLUE, 4, arrow="blue")
    s.rect(185, 690, 1230, 110, fill="#102D35", r=24)
    s.text(800, 733, "关键区分：展示出来 ≠ 允许 Jev 选择", 25, CYAN, 850, anchor="middle")
    s.text(800, 773, "探索轨迹帮助解释规划空间；真正的 choice set 由 candidateChoices() 再过滤。", 20, WHITE, 600, anchor="middle")
    s.footer(17, "src/driving-plan.js createDrivingPlan() · src/planning.js candidateChoices()")
    s.save("17-candidate-set-contraction.svg")


def slide_18_request_validation():
    s = Slide("进入 Jev 前：候选资格与请求结构双重校验")
    s.rect(70, 190, 690, 590, fill="#E7F5F4", stroke="#B8DEDA", sw=3, r=28, shadow=True)
    s.text(415, 240, "A · 本地候选资格", 27, TEAL, 900, anchor="middle")
    left = [
        ("运动", "速度非 0；脱困时不被阻挡"),
        ("碰撞", "collision_imminent 必须为 false"),
        ("道路", "正常模式要求 on_road"),
        ("方向", "前进阶段要求 follows_route_direction"),
        ("排队", "队列场景要求 queue_compatible"),
        ("停车", "仅在近阻挡/停车线/终点/无可行移动时加入"),
    ]
    for i, (a, b) in enumerate(left):
        y = 305 + i * 70
        s.pill(105, y - 31, a, TEAL if i < 5 else ORANGE, width=105)
        s.text(230, y, b, 19, INK, 560)
    s.rect(840, 190, 690, 590, fill="#EFF3FF", stroke="#C7D2FF", sw=3, r=28, shadow=True)
    s.text(1185, 240, "B · server.validState()", 27, BLUE, 900, anchor="middle", family=MONO)
    right = [
        ("批次", "batch_id = b数字；候选 ID 属于该批次"),
        ("数量", "candidate 数量 1…12"),
        ("数值", "字段必须有限；不能出现 NaN/Infinity"),
        ("物理范围", "steering ≤ 0.85；lane offset ≤ 1.4"),
        ("速度", "正常 ≥ 0；脱困 ≥ −2；均不超 ceiling"),
        ("停车项", "需要停车时必须存在固定停车 profile"),
    ]
    for i, (a, b) in enumerate(right):
        y = 305 + i * 70
        s.pill(875, y - 31, a, BLUE if i < 5 else RED, width=105)
        s.text(1000, y, b, 19, INK, 560)
    s.line(760, 485, 830, 485, ORANGE, 6, arrow="orange")
    s.text(795, 452, "HTTP", 17, ORANGE, 850, anchor="middle")
    s.footer(18, "src/planning.js candidateChoices() · server/jev.js validState()")
    s.save("18-request-validation.svg")


def slide_19_response_consistency():
    s = Slide("Jev 返回后：不是信任控制量，而是核对选择结果")
    steps = [
        ("① Jev 回答", "motion / vector / route\nchoice + probabilities", BLUE),
        ("② 服务端验答", "choice 属于 options\n概率逐项有限且在 [0,1]", TEAL),
        ("③ 展开别名", "vN → 内部 batch candidate ID", CYAN),
        ("④ 再查风险", "被选移动轨迹不能 collision_imminent", ORANGE),
        ("⑤ 反推控制", "speed / steering / maneuver\n来自候选本身", GREEN),
        ("⑥ 客户端复核", "batch、choice、概率、controls\n必须逐项一致", RED),
    ]
    for i, (title, body, color) in enumerate(steps):
        x = 70 + (i % 3) * 510
        y = 195 + (i // 3) * 270
        s.rect(x, y, 450, 190, fill=WHITE, stroke=color, sw=3, r=24, shadow=True)
        s.rect(x, y, 450, 62, fill=color, r=23)
        s.rect(x, y + 40, 450, 22, fill=color, r=0)
        s.text(x + 225, y + 41, title, 23, WHITE, 850, anchor="middle")
        s.multiline(x + 34, y + 112, body, 20, INK, 580, width=26, leading=1.35)
        if i not in (2, 5):
            s.line(x + 456, y + 96, x + 498, y + 96, BLUE, 4, arrow="blue")
    s.path("M1310,390 L1310,425 L295,425 L295,455", stroke=BLUE, sw=4, arrow="blue")
    s.rect(230, 735, 1140, 72, fill="#102D35", r=22)
    s.text(800, 779, "因此 Jev 只能选 ID；不能注入任意 steering、speed 或新轨迹。", 23, WHITE, 800, anchor="middle")
    s.footer(19, "server/jev.js validateAnswer()/buildResponse() · src/planning.js decisionControls()")
    s.save("19-response-consistency.svg")


def slide_20_runtime_guards():
    s = Slide("异步结果还要通过时效与运行时安全门")
    s.rect(70, 190, 980, 590, fill=WHITE, stroke="#DDD7CC", sw=2, r=28, shadow=True)
    gates = [
        ("generation token", "重置、切换模式或新请求后，旧结果作废", TEAL),
        ("routeVersion", "路线重算后，旧路线候选不能再应用", BLUE),
        ("1.8 s expiry", "响应太晚就丢弃，避免控制滞后", CYAN),
        ("context changed", "路口、信号、停车记忆等变化会使结果失效", ORANGE),
        ("step() envelope", "执行时重新限制速度/倒车，并处理停车线语义", GREEN),
        ("actual collision", "真实 swept collision 仍是最终物理终止条件", RED),
    ]
    for i, (a, b, c) in enumerate(gates):
        y = 245 + i * 82
        s.circle(125, y - 8, 20, c)
        s.text(125, y - 1, str(i + 1), 18, WHITE, 850, anchor="middle")
        s.text(170, y, a, 21, INK, 800, family=MONO if i < 2 else FONT)
        s.text(405, y, b, 19, MUTED, 540)
        if i < 5:
            s.line(125, y + 16, 125, y + 53, "#B9C7C7", 3)
    s.rect(1100, 190, 430, 350, fill="#FFF4DF", stroke="#F2D39A", sw=3, r=26, shadow=True)
    s.text(1315, 242, "检查强度", 25, ORANGE, 900, anchor="middle")
    levels = [("过滤", "不进入选项"), ("拒绝", "响应报错"), ("丢弃", "过期/上下文变化"), ("覆盖", "运行时限速/制动"), ("终止", "真实碰撞")]
    for i, (a, b) in enumerate(levels):
        y = 305 + i * 46
        s.text(1160, y, a, 19, ORANGE, 850)
        s.text(1245, y, b, 19, INK, 550)
    s.rect(1100, 575, 430, 205, fill="#FDECEC", stroke="#F3B7B5", sw=3, r=26)
    s.text(1315, 625, "已识别的校验空隙", 23, RED, 900, anchor="middle")
    s.multiline(1140, 677, "概率会逐项检查 [0,1]，但当前没有显式检查总和等于 1，也未明确拒绝额外概率键。", 19, INK, 580, width=24, leading=1.35)
    s.footer(20, "src/main.js async guards · src/simulation.js step() · src/planning.js response checks")
    s.save("20-runtime-safety-gates.svg")


def slide_21_dual_loops():
    s = Slide("自动驾驶由两个不同频率的闭环协同运行")
    # Decision loop
    s.rect(70, 190, 1460, 235, fill="#EFF3FF", stroke="#C7D2FF", sw=3, r=28, shadow=True)
    s.pill(105, 220, "异步决策环", BLUE, width=180)
    s.text(310, 250, "250 / 650 ms，自适应重规划", 21, BLUE, 800)
    decision = ["感知快照", "候选生成", "Jev 选择", "一致性校验", "安装 maneuver"]
    for i, label in enumerate(decision):
        x = 105 + i * 285
        s.rect(x, 305, 220, 72, fill=WHITE, stroke=BLUE, sw=2, r=18)
        s.text(x + 110, 350, label, 20, INK, 750, anchor="middle")
        if i < 4:
            s.line(x + 225, 341, x + 275, 341, BLUE, 4, arrow="blue")
    # Physics loop
    s.rect(70, 490, 1460, 250, fill="#E7F5F4", stroke="#B8DEDA", sw=3, r=28, shadow=True)
    s.pill(105, 520, "物理状态环", TEAL, width=180)
    s.text(310, 550, "每个渲染帧执行；必要时拆成多个 ≤ 25 ms 子步", 21, TEAL, 800)
    physics_steps = ["读取 maneuver", "速度/转向约束", "physics(dt)", "碰撞与规则", "新权威状态"]
    for i, label in enumerate(physics_steps):
        x = 105 + i * 285
        s.rect(x, 605, 220, 72, fill=WHITE, stroke=TEAL, sw=2, r=18)
        s.text(x + 110, 650, label, 20, INK, 750, anchor="middle")
        if i < 4:
            s.line(x + 225, 641, x + 275, 641, TEAL, 4, arrow="teal")
    s.path("M1390,377 C1510,420 1510,540 1390,605", stroke=ORANGE, sw=5, arrow="orange")
    s.text(1480, 490, "下发", 18, ORANGE, 850, anchor="middle")
    s.path("M135,605 C25,560 25,430 135,377", stroke=GREEN, sw=5, arrow="teal")
    s.text(54, 490, "反馈", 18, GREEN, 850, anchor="middle")
    s.rect(260, 780, 1080, 55, fill="#102D35", r=20)
    s.text(800, 816, "Jev 决定“接下来想怎么走”；step() 决定“这一小段时间实际发生什么”。", 21, WHITE, 800, anchor="middle")
    s.footer(21, "src/main.js decide()/animate() · src/simulation.js step()")
    s.save("21-autopilot-dual-loops.svg")


def slide_22_step_timing():
    s = Slide("物理 step：帧驱动的可变时间步，不是固定 60 Hz")
    s.rect(70, 190, 940, 600, fill=WHITE, stroke="#DDD7CC", sw=2, r=28, shadow=True)
    s.text(110, 242, "一帧如何转换为物理子步", 26, TEAL, 850)
    # Timeline examples
    examples = [
        ("60 FPS", "16 ms", 1, "1 × 16 ms", BLUE),
        ("30 FPS", "33 ms", 2, "2 × 16.5 ms", TEAL),
        ("卡顿帧", "80 ms", 4, "4 × 20 ms", ORANGE),
        ("上限帧", "≥ 200 ms", 8, "8 × 25 ms", RED),
    ]
    for i, (fps, frame, count, result, color) in enumerate(examples):
        y = 315 + i * 105
        s.text(115, y + 9, fps, 21, INK, 800)
        s.pill(245, y - 26, frame, color, width=120)
        s.line(380, y - 5, 460, y - 5, color, 4, arrow="blue" if color == BLUE else "teal" if color == TEAL else "orange")
        total_w = 310
        unit_w = total_w / count
        for j in range(count):
            s.rect(485 + j * unit_w, y - 30, unit_w - 6, 50, fill=color, r=10, opacity=.88)
        s.text(825, y + 8, result, 20, color, 850, family=MONO)
    s.rect(1060, 190, 470, 285, fill="#EFF3FF", stroke="#C7D2FF", sw=3, r=26)
    s.text(1295, 240, "时间计算", 25, BLUE, 900, anchor="middle")
    s.text(1100, 302, "frame dt = min(real dt, 0.2 s)", 19, INK, 700, family=MONO)
    s.text(1100, 355, "steps = ceil(dt / 0.025 s)", 19, INK, 700, family=MONO)
    s.text(1100, 408, "step dt = frame dt / steps", 19, INK, 700, family=MONO)
    s.rect(1060, 515, 470, 275, fill="#FFF4DF", stroke="#F2D39A", sw=3, r=26)
    s.text(1295, 565, "为什么高频子步？", 25, ORANGE, 900, anchor="middle")
    reasons = ["减少积分与转向误差", "避免跨过停车线", "改善接触时刻与位置", "让加速、制动更连续"]
    for i, reason in enumerate(reasons):
        y = 625 + i * 43
        s.circle(1100, y - 7, 7, ORANGE)
        s.text(1120, y, reason, 20, INK, 560)
    s.footer(22, "src/main.js animate(): dt cap + ≤25 ms substeps · src/simulation.js step(dt)")
    s.save("22-step-timing-model.svg")


def slide_23_step_state_update():
    s = Slide("每个 step() 把控制意图推进成新的权威世界状态")
    stages = [
        ("① 时间与路线", "time += dt\n偏航与重规划检查", BLUE),
        ("② 环境实体", "路口锁 / courtesy\n行人 / NPC", TEAL),
        ("③ 感知节流", "常规 scanScene\n约每 0.2 s", CYAN),
        ("④ 自车控制", "maneuver → speed/steering\n实时安全包络", ORANGE),
        ("⑤ 物理积分", "physics(dt)\n更新 pose 与 speed", GREEN),
        ("⑥ 结果判定", "swept collision\n进度 / 违规 / 到达", RED),
    ]
    for i, (title, body, color) in enumerate(stages):
        x = 55 + i * 255
        y = 225 + (i % 2) * 75
        s.rect(x, y, 215, 300, fill=WHITE, stroke=color, sw=3, r=24, shadow=True)
        s.rect(x, y, 215, 66, fill=color, r=23)
        s.rect(x, y + 42, 215, 24, fill=color, r=0)
        s.text(x + 108, y + 43, title, 20, WHITE, 850, anchor="middle")
        s.multiline(x + 25, y + 135, body, 19, INK, 620, width=14, leading=1.5)
        if i < 5:
            s.line(x + 220, y + 150, x + 249, 300 + ((i + 1) % 2) * 75, color, 4, arrow="blue" if i == 0 else "teal" if i < 4 else "orange")
    s.rect(120, 695, 1360, 120, fill="#102D35", r=24)
    s.text(175, 742, "权威状态输出", 23, CYAN, 850)
    s.text(380, 742, "x / z / heading / speed / route s / locks / stops / crash / violations / complete", 19, WHITE, 650, family=MONO)
    s.text(175, 782, "事件机制", 23, CYAN, 850)
    s.text(380, 782, "碰撞、越线和到达不是独立事件队列，而是在每个 step 后按状态条件同步触发。", 20, WHITE, 600)
    s.footer(23, "src/simulation.js step(): world update → physics → collision/rules/arrival")
    s.save("23-step-state-transition.svg")


def slide_24_constraint_map():
    s = Slide("约束检查按生命周期分层，但分布在多个模块")
    rows = [
        ("事实计算", "road-geometry / traffic-safety / collisions", "道路占用、间距、预测冲突、首次碰撞", TEAL),
        ("候选属性", "driving-plan.js", "on-road、路线方向、排队兼容、碰撞标记", BLUE),
        ("选择资格", "planning.js", "moving 硬过滤、停车可用性、choice set", CYAN),
        ("服务端边界", "server/jev.js", "状态结构、范围、答案与被选轨迹", ORANGE),
        ("异步一致性", "main.js + decisionControls()", "batch、routeVersion、过期、controls 一致", GREEN),
        ("执行期兜底", "simulation.js step()", "实时限速、脱困阻挡、真实碰撞与违规", RED),
    ]
    s.text(100, 195, "阶段", 20, MUTED, 800)
    s.text(365, 195, "主要位置", 20, MUTED, 800)
    s.text(865, 195, "约束效果", 20, MUTED, 800)
    for i, (stage, module, effect, color) in enumerate(rows):
        y = 220 + i * 88
        s.rect(70, y, 1460, 70, fill=WHITE, stroke="#DDD7CC", sw=1, r=15, shadow=i in (0, 5))
        s.rect(70, y, 14, 70, fill=color, r=7)
        s.pill(105, y + 14, stage, color, width=180)
        s.text(365, y + 44, module, 19, INK, 750, family=MONO)
        s.text(865, y + 44, effect, 19, INK, 540)
        if i < 5:
            s.line(310, y + 72, 310, y + 84, color, 3, arrow="teal" if i < 3 else "orange")
    s.rect(210, 765, 1180, 65, fill="#102D35", r=22)
    s.text(800, 806, "结论：不是随意散落，而是按阶段分层；但无法从单一入口看到约束全貌。", 22, WHITE, 800, anchor="middle")
    s.footer(24, "src/road-geometry.js · traffic-safety.js · driving-plan.js · planning.js · server/jev.js · main.js · simulation.js")
    s.save("24-constraint-distribution-map.svg")


def slide_25_concentration_tradeoff():
    s = Slide("哪些分散是必要防线，哪些会增加维护风险？")
    s.rect(70, 190, 700, 550, fill="#E7F5F4", stroke="#B8DEDA", sw=3, r=28, shadow=True)
    s.text(420, 245, "合理的分层防御", 28, TEAL, 900, anchor="middle")
    good = [
        ("预测阶段", "判断未来轨迹风险"),
        ("选择阶段", "决定哪些候选能交给 Jev"),
        ("服务端", "防止损坏或越权的输入/答案"),
        ("异步边界", "拒绝旧批次和过期结果"),
        ("执行阶段", "处理预测与现实之间的偏差"),
    ]
    for i, (a, b) in enumerate(good):
        y = 315 + i * 73
        s.circle(120, y - 7, 14, TEAL)
        s.text(120, y, "✓", 17, WHITE, 900, anchor="middle")
        s.text(155, y, a, 20, INK, 800)
        s.text(290, y, b, 19, MUTED, 540)
    s.rect(830, 190, 700, 550, fill="#FDECEC", stroke="#F3B7B5", sw=3, r=28, shadow=True)
    s.text(1180, 245, "当前维护风险", 28, RED, 900, anchor="middle")
    risks = [
        ("语义跨文件", "属性产生与消费位置分离"),
        ("规则重复", "碰撞与速度在多处重新表达"),
        ("契约隐式", "普通 JS 对象，无统一 schema"),
        ("原因不透明", "只能看到 false，看不到为何失败"),
        ("联动修改", "字段变化需同步 client/server/UI"),
    ]
    for i, (a, b) in enumerate(risks):
        y = 315 + i * 73
        s.circle(880, y - 7, 14, RED)
        s.text(880, y, "!", 17, WHITE, 900, anchor="middle")
        s.text(915, y, a, 20, INK, 800)
        s.text(1050, y, b, 19, MUTED, 540)
    s.rect(215, 775, 1170, 55, fill="#FFF4DF", stroke="#F2D39A", sw=2, r=20)
    s.text(800, 811, "应集中“规则定义与解释”，而不是把所有算法塞进同一个大文件。", 22, ORANGE, 850, anchor="middle")
    s.footer(25, "Architecture assessment: layered enforcement is useful; policy and contracts need consolidation")
    s.save("25-constraint-concentration-tradeoff.svg")


def slide_26_constraint_architecture():
    s = Slide("建议：统一约束策略与 Schema，保留各阶段独立兜底")
    # Facts layer
    s.rect(70, 205, 410, 500, fill="#E7F5F4", stroke="#B8DEDA", sw=3, r=28, shadow=True)
    s.text(275, 255, "① 事实计算层", 26, TEAL, 900, anchor="middle")
    facts = ["road-geometry", "traffic-safety", "collisions"]
    for i, item in enumerate(facts):
        s.rect(115, 315 + i * 90, 320, 60, fill=WHITE, stroke=TEAL, sw=2, r=16)
        s.text(275, 353 + i * 90, item, 20, INK, 750, anchor="middle", family=MONO)
    s.multiline(120, 615, "只计算事实：距离、占用、冲突、碰撞；不决定能否选择。", 19, MUTED, 560, width=24, leading=1.35)
    # Policy layer
    s.rect(595, 175, 410, 560, fill="#EFF3FF", stroke=BLUE, sw=4, r=28, shadow=True)
    s.text(800, 230, "② 统一约束策略层", 27, BLUE, 900, anchor="middle")
    s.text(800, 270, "constraint-policy.js", 20, BLUE, 800, anchor="middle", family=MONO)
    policy = ["共享常量与 candidate schema", "evaluateCandidate()", "硬失败 / 警告 / penalty", "稳定 reason codes", "stop / speed policy"]
    for i, item in enumerate(policy):
        y = 335 + i * 63
        s.circle(645, y - 7, 8, BLUE)
        s.text(670, y, item, 20, INK, 600)
    s.rect(640, 660, 320, 48, fill=BLUE, r=18)
    s.text(800, 691, "唯一可解释的策略结果", 19, WHITE, 800, anchor="middle")
    # Boundaries
    s.rect(1120, 205, 410, 500, fill="#FFF4DF", stroke="#F2D39A", sw=3, r=28, shadow=True)
    s.text(1325, 255, "③ 独立信任边界", 26, ORANGE, 900, anchor="middle")
    guards = ["server.validState", "decisionControls", "generation / expiry", "Simulation.step", "actual collision"]
    for i, item in enumerate(guards):
        s.rect(1165, 310 + i * 70, 320, 50, fill=WHITE, stroke=ORANGE, sw=2, r=15)
        s.text(1325, 342 + i * 70, item, 18, INK, 700, anchor="middle", family=MONO)
    s.line(485, 455, 585, 455, TEAL, 5, arrow="teal")
    s.line(1010, 455, 1110, 455, ORANGE, 5, arrow="orange")
    s.rect(200, 775, 1200, 58, fill="#102D35", r=20)
    s.text(800, 812, "目标：一处定义约束语义，多处执行纵深防御；每次拒绝都能给出统一原因。", 22, WHITE, 800, anchor="middle")
    s.footer(26, "Proposed design: shared schema + policy result + stage-specific guards")
    s.save("26-constraint-governance-architecture.svg")


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    for old in OUT.glob("*.svg"):
        old.unlink()
    for fn in [slide_01, slide_02, slide_03_state_layers, slide_04_world_model, slide_05_dynamic_model, slide_06_state_views, slide_03, slide_04, slide_05, slide_06, slide_07, slide_08, slide_09, slide_10, slide_11, slide_12, slide_17_candidate_sets, slide_18_request_validation, slide_19_response_consistency, slide_20_runtime_guards, slide_21_dual_loops, slide_22_step_timing, slide_23_step_state_update, slide_24_constraint_map, slide_25_concentration_tradeoff, slide_26_constraint_architecture, slide_13]:
        fn()
    print(f"Generated 27 SVG slides in {OUT}")


if __name__ == "__main__":
    main()
