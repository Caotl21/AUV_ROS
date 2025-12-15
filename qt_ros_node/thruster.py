import sys
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QSlider, QLabel, QHBoxLayout
from PyQt5.QtCore import Qt, QRectF, QPointF, QTimer
from PyQt5.QtGui import QPainter, QColor, QPen, QFont, QPixmap, QBrush, QLinearGradient

# --- 配置文件 ---
ROBOT_IMAGE_PATH = "/home/caotl21/work/PYQT/qt_ros/my_example/logo/robot_cv.png"  # 请确保图片在当前目录下

# 配色方案
COLOR_TEXT = QColor("#1E1E1E")
COLOR_SEGMENT_OFF = QColor("#333333")
COLOR_SEGMENT_POS = QColor("#00FF00")   # 正向绿
COLOR_SEGMENT_POS_DARK = QColor("#006400")  # 深绿色
COLOR_SEGMENT_NEG_DARK = QColor("#8B0000")  # 深红色
COLOR_SEGMENT_NEG = QColor("#FF8800")   # 反向橙
COLOR_BORDER = QColor("#AAAAAA")
COLOR_THRUSTER_BG = QColor("#A39C9C")
COLOR_BG_DARK = QColor("#ffffff")
COLOR_ROBOT_BG = QColor(245, 245, 245)   # 机器人HUD背景色"

class RobotHUDWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(300, 250)
        self.setStyleSheet(f"background-color: {COLOR_BG_DARK.name()};")

        # 1. 加载图片
        self.pixmap = QPixmap(ROBOT_IMAGE_PATH)
        if self.pixmap.isNull():
            print(f"Warning: Could not load {ROBOT_IMAGE_PATH}")
            # 生成一个测试用的灰色方块图代替
            self.pixmap = QPixmap(400, 600)
            self.pixmap.fill(QColor("#333"))
        
        # 原始图片尺寸
        self.img_w = self.pixmap.width()
        self.img_h = self.pixmap.height()

        # 2. 定义推进器位置配置 (关键部分！)
        # 坐标系：以图片中心为 (0, 0)。
        # x: 向右为正，向左为负 (单位：相对于图片宽度的比例)
        # y: 向下为正，向上为负 (单位：相对于图片高度的比例)
        # 例如：(-0.4, -0.3) 意味着在图片中心向左 40% 宽度，向上 30% 高度的位置
        self.thruster_positions = {
            # 左侧三个 (T1, T2, T3)
            0: (-0.35, -0.35),  # T1: 左上
            1: (-0.45,  0.0),   # T2: 左中 (稍微再靠外一点)
            2: (-0.35,  0.35),  # T3: 左下
            
            # 右侧三个 (T4, T5, T6)
            3: ( 0.35, -0.35),  # T4: 右上 (对应图中的 T6位置)
            4: ( 0.47,  0.0),   # T5: 右中
            5: ( 0.35,  0.35),  # T6: 右下
        }
        
        self.thrust_values = [0, 30, -30, 45, -45, 0] # 存储6个推力值

    def update_thrusts(self, values):
        """更新推力数据 [t1, t2, t3, t4, t5, t6]"""
        if len(values) == 6:
            self.thrust_values = values
            self.update() # 触发重绘

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        # A. 绘制背景色
        painter.fillRect(self.rect(), COLOR_BG_DARK)

        # B. 计算图片的缩放和居中位置
        w, h = self.width(), self.height()
        
        # 保持比例缩放 (KeepAspectRatio)
        scale_w = w / self.img_w
        scale_h = h / self.img_h
        scale = min(scale_w, scale_h) * 0.95 # 0.9 为了留一点边距
        
        draw_w = self.img_w * scale
        draw_h = self.img_h * scale
        
        # 图片左上角的屏幕坐标
        start_x = (w - draw_w) / 2
        start_y = (h - draw_h) / 2
        
        # 图片中心的屏幕坐标
        center_x = start_x + draw_w / 2
        center_y = start_y + draw_h / 2

        # C. 绘制底图 Robot.jpg
        target_rect = QRectF(start_x, start_y, draw_w, draw_h)
        painter.drawPixmap(target_rect.toRect(), self.pixmap)

        # D. 遍历并在对应位置绘制推力条
        # 定义推力条的显示尺寸 (随图片缩放而缩放，保持视觉比例)
        bar_w = 60 * scale # 原始宽度基准 30
        bar_h = 200 * scale # 原始高度基准 100
        
        labels = ["T1", "T2", "T3", "T4", "T5", "T6"]

        for i in range(6):
            # 1. 获取相对位置比例
            rx, ry = self.thruster_positions[i]
            
            # 2. 计算屏幕真实坐标 (中心点)
            # 真实位置 = 图片中心 + (图片半宽 * 比例)
            # 注意：这里乘的是 draw_w (缩放后的宽)，不是原图宽
            pos_x = center_x + (draw_w * rx)
            pos_y = center_y + (draw_h * ry)
            
            # 3. 绘制仪表
            # 我们将 (pos_x, pos_y) 作为仪表的中心点
            gauge_rect = QRectF(pos_x - bar_w/2, pos_y - bar_h/2, bar_w, bar_h)
            
            self.draw_segmented_bar(painter, gauge_rect, self.thrust_values[i], labels[i])

    def draw_segmented_bar(self, painter, rect, value, label):
        """
        在指定区域绘制连续的推力条 (带圆角 + 数值侧边显示)
        """
        painter.save()
        
        # 1. 绘制顶部标签 (T1, T2...)
        painter.setPen(COLOR_TEXT)
        # 根据缩放自适应字体大小
        font_size = max(10, int(rect.width() * 0.4)) 
        painter.setFont(QFont("Arial", font_size, QFont.Bold))
        
        # 标签依然画在顶部
        label_rect = QRectF(rect.x(), rect.y() - 20, rect.width(), 20)
        painter.drawText(label_rect, Qt.AlignCenter, label)
        
        # 2. 绘制半透明背景框 (带圆角)
        painter.setPen(QPen(COLOR_BORDER, 1))
        painter.setBrush(COLOR_THRUSTER_BG) # 半透明黑色背景
        # 这里统一给背景也加一点圆角，看起来更协调
        painter.drawRoundedRect(rect, 3, 3)
        
        # 3. 绘制连续的能量条
        value = max(-1.0, min(1.0, value))
        
        # 计算内部可用区域
        padding = 2
        bar_w = rect.width() - 2 * padding
        max_h = (rect.height() / 2) - padding 
        
        mid_y = rect.center().y()
        bar_x = rect.x() + padding
        
        # 只有数值足够大才绘制
        if abs(value) > 0.01:
            fill_h = abs(value) * max_h
            
            # 定义渐变和矩形位置
            if value > 0:
                # 正向：向上增长 (绿色)
                fill_rect = QRectF(bar_x, mid_y - fill_h, bar_w, fill_h)
                grad_top, grad_bot = COLOR_SEGMENT_POS.lighter(130), COLOR_SEGMENT_POS
            else:
                # 反向：向下增长 (橙色)
                fill_rect = QRectF(bar_x, mid_y, bar_w, fill_h)
                grad_top, grad_bot = COLOR_SEGMENT_NEG, COLOR_SEGMENT_NEG.lighter(130)

            # 创建渐变画刷
            grad = QLinearGradient(fill_rect.topLeft(), fill_rect.bottomLeft())
            grad.setColorAt(0, grad_top)
            grad.setColorAt(1, grad_bot)
            painter.setBrush(QBrush(grad))
            painter.setPen(Qt.NoPen)
            
            # 【关键点】绘制圆角矩形
            # xRadius=3, yRadius=3 实现圆角倒角
            painter.drawRoundedRect(fill_rect, 3, 3)

        # 4. 绘制中心线 (0位)
        painter.setPen(QPen(QColor(255, 255, 255, 200), 1))
        painter.drawLine(QPointF(rect.left(), mid_y), QPointF(rect.right(), mid_y))

        # 5. 绘制百分比数值 (侧边显示)
        percent_text = f"{int(value * 100)}%"
        
        # 设置数值颜色
        if value > 0.01: painter.setPen(COLOR_SEGMENT_POS_DARK)
        elif value < -0.01: painter.setPen(COLOR_SEGMENT_NEG_DARK)
        else: painter.setPen(QColor(150,150,150))
        
        # 设置字体
        font_size = max(12, int(rect.width() * 0.35))
        painter.setFont(QFont("Arial", font_size, QFont.Bold))
        
        # 【关键逻辑】判断是左侧推进器还是右侧推进器
        # 假设左侧为 T1, T2, T3；右侧为 T4, T5, T6
        is_left_thruster = label in ["T1", "T2", "T3"]
        
        text_w = 70 # 给文字预留的宽度区域
        text_h = 20
        # 文字垂直居中于仪表盘中心 (即 0 位线旁边)
        text_y = mid_y - (text_h / 2)

        if is_left_thruster:
            # 左侧推进器：文字显示在条形图的【右侧】
            # 坐标 = 矩形右边缘 + 间距
            text_x = rect.right() + 5 
            align_flag = Qt.AlignLeft | Qt.AlignVCenter
        else:
            # 右侧推进器：文字显示在条形图的【左侧】
            # 坐标 = 矩形左边缘 - 文字宽度 - 间距
            text_x = rect.left() - text_w - 5
            align_flag = Qt.AlignRight | Qt.AlignVCenter

        text_rect = QRectF(text_x, text_y, text_w, text_h)
        painter.drawText(text_rect, align_flag, percent_text)

        painter.restore()


# --- 测试窗口 ---
class TestWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ROV Overlay Thruster View")
        self.resize(300, 600)
        
        layout = QVBoxLayout(self)
        self.hud = RobotHUDWidget()
        layout.addWidget(self.hud)
        
        # 模拟滑块
        control_layout = QHBoxLayout()
        self.sliders = []
        for i in range(6):
            s = QSlider(Qt.Vertical)
            s.setRange(-100, 100)
            s.setValue(0)
            s.valueChanged.connect(self.update_hud)
            self.sliders.append(s)
            control_layout.addWidget(s)
            
        layout.addLayout(control_layout)

    def update_hud(self):
        vals = [s.value() / 100.0 for s in self.sliders]
        self.hud.update_thrusts(vals)

if __name__ == "__main__":
    QApplication.setAttribute(Qt.AA_EnableHighDpiScaling)
    app = QApplication(sys.argv)
    win = TestWindow()
    win.show()
    sys.exit(app.exec_())