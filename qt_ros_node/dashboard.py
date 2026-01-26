import sys
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QSlider, QLabel
from PyQt5.QtCore import Qt, QPoint, QRectF
from PyQt5.QtGui import QPainter, QColor, QPen, QFont, QPolygon, QPainterPath

# --- 配色方案 (参考您的UI图) ---
COLOR_BG = QColor(30, 30, 35)           # 仪表盘深色背景
COLOR_RING = QColor(60, 60, 70)         # 仪表盘外圈
COLOR_TEXT = QColor(220, 220, 220)      # 白色文字/刻度
COLOR_SKY = QColor(60, 120, 180)        # 天空蓝
COLOR_GROUND = QColor(160, 100, 50)     # 地面棕橙色
COLOR_ACCENT_ORANGE = QColor(255, 140, 0) # 亮橙色 (飞机标志)
COLOR_ACCENT_RED = QColor(220, 50, 50)    # 红色 (罗盘指针)
COLOR_ACCENT_BLUE = QColor(0, 180, 255)   # 亮蓝色 (Roll指示)

class CompassWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(180, 180)
        self.angle = 0  # 0-360度 (Yaw)

    def set_heading(self, angle):
        """设置航向角度"""
        self.angle = angle % 360
        self.update()  # 触发重绘

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        # 1. 基础尺寸计算
        w = self.width()
        h = self.height()
        cx, cy = w / 2, h / 2
        radius = min(w, h) / 2 - 10

        # 2. 绘制深色背景和外圈
        painter.setBrush(COLOR_BG)
        painter.setPen(QPen(COLOR_RING, 3))
        painter.drawEllipse(QPoint(int(cx), int(cy)), int(radius), int(radius))

        # 3. 绘制固定不动的刻度盘 (N/E/S/W)
        painter.translate(cx, cy)
        painter.save() 
        
        font = QFont("Arial", 10, QFont.Bold)
        painter.setFont(font)
        painter.setPen(QPen(COLOR_TEXT, 2))

        for i in range(0, 360, 30):
            painter.save()
            painter.rotate(i) 
            
            # 绘制刻度线
            is_major = (i % 90 == 0)
            tick_len = 12 if is_major else 8
            painter.drawLine(0, int(-radius + tick_len), 0, int(-radius + 3))
            
            # 绘制文字
            text = ""
            if i == 0: text = "N"
            elif i == 90: text = "E"
            elif i == 180: text = "S"
            elif i == 270: text = "W"
            
            if text:
                # 文字矫正，保持直立
                painter.translate(0, -radius + 28)
                painter.rotate(-i) 
                # N字用红色高亮 (可选，参考图中N似乎是白色，指针是红色，这里暂用白色)
                painter.setPen(COLOR_TEXT) 
                text_rect = QRectF(-20, -10, 40, 20)
                painter.drawText(text_rect, Qt.AlignCenter, text)
            
            painter.restore()
        painter.restore() 

        # 4. 绘制旋转的指针 (指向北方)
        painter.save()
        painter.rotate(self.angle)
        
        # 绘制红色北向指针箭头
        arrow_polygon = QPolygon([
            QPoint(0, -int(radius * 0.65)), # 针尖
            QPoint(-8, -int(radius * 0.1)), # 左翼
            QPoint(8, -int(radius * 0.1))   # 右翼
        ])
        painter.setBrush(COLOR_ACCENT_RED)
        painter.setPen(Qt.NoPen)
        painter.drawPolygon(arrow_polygon)
        
        # 绘制深色尾部
        tail_polygon = QPolygon([
            QPoint(0, int(radius * 0.2)),
            QPoint(-6, -int(radius * 0.1)),
            QPoint(6, -int(radius * 0.1))
        ])
        painter.setBrush(QColor(80, 80, 80)) # 深灰色尾巴
        painter.drawPolygon(tail_polygon)
        
        painter.restore()

        # 5. 中心装饰点
        painter.setBrush(QColor(200, 200, 200))
        painter.drawEllipse(QPoint(0, 0), 4, 4)


class ArtificialHorizonWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(180, 180) # 稍微加大尺寸以容纳外圈
        self.roll = 0   
        self.pitch = 0  

    def set_attitude(self, roll, pitch):
        self.roll = roll
        self.pitch = pitch
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        w = self.width()
        h = self.height()
        cx, cy = w / 2, h / 2
        
        # 定义半径
        # r_outer: 整个控件的半径（扣除边距）
        # r_inner: 内部显示的半径（留出30px给刻度圈）
        r_outer = min(w, h) / 2 - 5
        r_inner = r_outer - 11 

        # --- Layer 1: 绘制外圈底盘 (深色背景) ---
        painter.setBrush(QColor(20, 20, 25)) # 深灰底色
        painter.setPen(QPen(QColor(60, 60, 70), 2)) # 灰色边框
        painter.drawEllipse(QPoint(int(cx), int(cy)), int(r_outer), int(r_outer))

        # --- Layer 2: 绘制内部动态地平线 (使用Clip限制在内圆) ---
        # 2.1 设置圆形遮罩
        path = QPainterPath()
        path.addEllipse(cx - r_inner, cy - r_inner, r_inner * 2, r_inner * 2)
        painter.save() # 保存状态A
        painter.setClipPath(path)

        # 2.2 变换坐标系 (旋转 Roll, 平移 Pitch)
        painter.translate(cx, cy)
        painter.rotate(self.roll) 
        pitch_offset = self.pitch * 1.5 
        painter.translate(0, pitch_offset)

        # 2.3 绘制天空地面
        big_size = max(w, h) * 3
        # 天空
        painter.setBrush(COLOR_SKY) 
        painter.setPen(Qt.NoPen)
        painter.drawRect(int(-big_size/2), int(-big_size), int(big_size), int(big_size))
        # 地面
        painter.setBrush(COLOR_GROUND) 
        painter.drawRect(int(-big_size/2), 0, int(big_size), int(big_size))
        # 地平线
        painter.setPen(QPen(Qt.white, 2))
        painter.drawLine(int(-big_size/2), 0, int(big_size/2), 0)

        # 2.4 绘制 Pitch 梯子 (位于内部)
        font = QFont("Arial", 8, QFont.Bold)
        painter.setFont(font)
        for i in range(-120, 121, 10): 
            if i == 0: continue
            y_pos = -i * 1.5
            
            # 只有当线条在视野内时才绘制(优化性能)
            if abs(y_pos + pitch_offset) > r_inner: continue

            is_major = (i % 30 == 0)
            line_w = 25 if is_major else 12
            
            painter.setPen(QPen(QColor(255, 255, 255, 200), 1.5))
            painter.drawLine(-line_w, int(y_pos), line_w, int(y_pos))
            
            if is_major:
                painter.drawText(QRectF(-line_w - 30, int(y_pos) - 10, 25, 20), 
                               Qt.AlignRight | Qt.AlignVCenter, str(abs(i)))
                painter.drawText(QRectF(line_w + 5, int(y_pos) - 10, 25, 20), 
                               Qt.AlignLeft | Qt.AlignVCenter, str(abs(i)))

        painter.restore() # 恢复状态A (移除Clip，坐标系回到左上角)

        # --- Layer 3: 在外圈绘制 Roll 刻度 (静态固定) ---
        painter.translate(cx, cy)
        painter.save() # 保存中心原点状态

        # 我们只画上半圆的刻度 (-60 到 +60)
        # 这里的刻度是固定在外壳上的
        painter.setFont(QFont("Arial", 7, QFont.Bold))
        
        for r in range(-90, 91, 10):
            painter.save()
            painter.rotate(r) # 旋转对应角度
            
            # 计算刻度线位置 (从 r_inner 向外画)
            # 刻度线画在 r_inner 和 r_outer 之间
            
            is_major = (r % 30 == 0)
            tick_start = r_inner + 2
            tick_end = r_inner + (8 if is_major else 5)
            
            # 画刻度线 (白色)
            painter.setPen(QPen(QColor(220, 220, 220), 2 if is_major else 1))
            painter.drawLine(0, int(-tick_start), 0, int(-tick_end))
            
            painter.restore()
        
        painter.restore() # 恢复中心原点状态

        # --- Layer 4: 绘制指示当前 Roll 的指针 (随动) ---
        # 这个指针需要跟随 roll 旋转，指向外圈的刻度
        painter.save()
        painter.rotate(self.roll) # 旋转 roll 角度
        
        # 指针位置：内圆边缘
        pointer_y = -r_inner
        
        # 蓝色三角形指针
        painter.setBrush(COLOR_ACCENT_BLUE)
        painter.setPen(QPen(Qt.white, 1))
        
        arrow = QPolygon([
            QPoint(0, int(pointer_y)),        # 尖端指向上方(对应外圈刻度)
            QPoint(-6, int(pointer_y + 10)),  # 左下
            QPoint(6, int(pointer_y + 10))    # 右下
        ])
        painter.drawPolygon(arrow)
        painter.restore()

        # --- Layer 5: 中心固定飞机标志 ---
        painter.setPen(QPen(COLOR_ACCENT_ORANGE, 3, Qt.SolidLine, Qt.RoundCap))
        painter.setBrush(Qt.NoBrush)
        
        # 简单的 W 型翅膀
        wing_path = QPainterPath()
        wing_path.moveTo(-40, 0)
        wing_path.lineTo(-10, 0); wing_path.lineTo(-10, 10)
        
        wing_path.moveTo(40, 0)
        wing_path.lineTo(10, 0); wing_path.lineTo(10, 10)
        
        painter.drawPath(wing_path)
        
        # 中心点
        painter.setBrush(COLOR_ACCENT_ORANGE)
        painter.drawEllipse(QPoint(0, 0), 3, 3)


class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.resize(600, 400)
        # 设置背景色以匹配UI图的深色环境
        self.setStyleSheet("background-color: #222; color: white;")

        layout = QHBoxLayout(self)
        layout.setSpacing(20)

        # 左侧两个仪表
        gauges_layout = QVBoxLayout()
        self.horizon = ArtificialHorizonWidget()
        self.compass = CompassWidget()
        gauges_layout.addWidget(QLabel("Artificial Horizon"))
        gauges_layout.addWidget(self.horizon)
        gauges_layout.addWidget(QLabel("Compass"))
        gauges_layout.addWidget(self.compass)

        layout.addLayout(gauges_layout)

        # 右侧模拟控制器
        control_layout = QVBoxLayout()
        
        self.slider_yaw = QSlider(Qt.Horizontal)
        self.slider_yaw.setRange(0, 360)
        self.slider_yaw.valueChanged.connect(self.compass.set_heading)
        
        self.slider_roll = QSlider(Qt.Horizontal)
        self.slider_roll.setRange(-90, 90)
        self.slider_roll.valueChanged.connect(lambda v: self.update_attitude())

        self.slider_pitch = QSlider(Qt.Vertical)
        self.slider_pitch.setRange(-45, 45) # 水下机器人Pitch角度通常不用太大
        self.slider_pitch.valueChanged.connect(lambda v: self.update_attitude())

        control_layout.addWidget(QLabel("Test Heading (Yaw)"))
        control_layout.addWidget(self.slider_yaw)
        control_layout.addWidget(QLabel("Test Roll"))
        control_layout.addWidget(self.slider_roll)
        control_layout.addStretch()
        
        control_right_box = QHBoxLayout()
        control_right_box.addLayout(control_layout)
        control_right_box.addWidget(self.slider_pitch)
        control_right_box.addWidget(QLabel("Test Pitch"))
        
        layout.addLayout(control_right_box)

    def update_attitude(self):
        self.horizon.set_attitude(self.slider_roll.value(), self.slider_pitch.value())

if __name__ == "__main__":
    # 适配高DPI屏幕
    QApplication.setAttribute(Qt.AA_EnableHighDpiScaling)
    QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps)

    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
