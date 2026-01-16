# 写一个脚本 帮我将test.png的四个边缘变成圆角
from PIL import Image, ImageDraw
import cv2
import numpy as np

def resize_image(input_path, output_path, size=(640, 640)):
    #使用opencv调整图像大小
    img = cv2.imread(input_path)
    resized_img = cv2.resize(img, size, interpolation=cv2.INTER_AREA)
    cv2.imwrite(output_path, resized_img)

def add_rounded_corners(image_path, output_path, radius):
    # 打开图像
    img = Image.open(image_path).convert("RGBA")
    width, height = img.size

    # 创建一个与图像大小相同的透明蒙版
    mask = Image.new('L', (width, height), 0)
    draw = ImageDraw.Draw(mask)

    # 绘制四个圆角
    draw.rounded_rectangle([(0, 0), (width, height)], radius=radius, fill=255)

    # 应用蒙版到图像
    img.putalpha(mask)

    # 保存结果
    img.save(output_path)

# 方法1: 使用PIL将白色背景变透明
def white_to_transparent_pil(image_path, output_path, threshold=240):
    """
    将白色背景变成透明
    threshold: 白色阈值，像素值大于此值的会被视为白色
    """
    img = Image.open(image_path).convert("RGBA")
    datas = img.getdata()
    
    new_data = []
    for item in datas:
        # 如果RGB三个通道都接近白色，则将alpha通道设为0（透明）
        if item[0] > threshold and item[1] > threshold and item[2] > threshold:
            new_data.append((255, 255, 255, 0))
        else:
            new_data.append(item)
    
    img.putdata(new_data)
    img.save(output_path, "PNG")

# 方法2: 使用OpenCV将白色背景变透明
def white_to_transparent_cv(image_path, output_path, threshold=240):
    """
    使用OpenCV将白色背景变透明
    """
    img = cv2.imread(image_path)
    # 转换为RGBA
    img_rgba = cv2.cvtColor(img, cv2.COLOR_BGR2BGRA)
    
    # 创建mask：白色区域为True
    mask = np.all(img[:, :, :3] > threshold, axis=2)
    
    # 将白色区域的alpha通道设为0（透明）
    img_rgba[mask, 3] = 0
    
    cv2.imwrite(output_path, img_rgba)

# 使用函数
#add_rounded_corners('test.png', 'test_rounded.png', radius=120)
#resize_image('logo.png', 'logo/test.png', size=(1200, 560))
white_to_transparent_pil('robot.png', 'logo/robot_pil.png')
white_to_transparent_cv('robot.png', 'logo/robot_cv.png')