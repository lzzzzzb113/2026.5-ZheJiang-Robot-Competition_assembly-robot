
import numpy as np
import math

# ------------------------- 图像预处理 -------------------------
def preprocess_image(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blurred, 50, 150)
    return edges

def get_shape_approx(contour, epsilon_factor=0.02):
    peri = cv2.arcLength(contour, True)
    epsilon = epsilon_factor * peri
    approx = cv2.approxPolyDP(contour, epsilon, True)
    return len(approx), approx

def is_circle(contour, area_threshold_ratio=0.85):
    area = cv2.contourArea(contour)
    perimeter = cv2.arcLength(contour, True)
    if perimeter == 0:
        return False
    circularity = 4 * math.pi * area / (perimeter * perimeter)
    return circularity >= area_threshold_ratio

def get_angle_between_vectors(v1, v2):
    """返回向量夹角（度），范围0~180"""
    dot = v1[0]*v2[0] + v1[1]*v2[1]
    norm1 = math.hypot(v1[0], v1[1])
    norm2 = math.hypot(v2[0], v2[1])
    if norm1 == 0 or norm2 == 0:
        return 0.0
    cos = dot / (norm1 * norm2)
    cos = max(-1.0, min(1.0, cos))
    return math.degrees(math.acos(cos))

def get_trapezoid_angle(approx):
    """
    梯形：短边在上，方向向量 = 短边中点 → 长边中点（竖直向下为0°）
    返回与竖直向下方向的夹角（带符号）
    """
    pts = approx.reshape(4, 2)
    edges = []
    for i in range(4):
        p1 = pts[i]
        p2 = pts[(i+1)%4]
        vec = p2 - p1
        angle = math.degrees(math.atan2(vec[1], vec[0]))
        length = np.linalg.norm(vec)
        edges.append((angle, length, p1, p2))
    
    # 找平行边对
    parallel_pairs = []
    for i in range(2):
        diff = abs(edges[i][0] - edges[i+2][0])
        diff = min(diff, 180 - diff)
        if diff < 10:
            parallel_pairs.append((i, i+2))
    if len(parallel_pairs) != 1:
        return 0.0
    
    i1, i2 = parallel_pairs[0]
    len1, len2 = edges[i1][1], edges[i2][1]
    short_idx, long_idx = (i1, i2) if len1 < len2 else (i2, i1)
    
    p1_s, p2_s = edges[short_idx][2], edges[short_idx][3]
    mid_short = ((p1_s[0]+p2_s[0])/2, (p1_s[1]+p2_s[1])/2)
    p1_l, p2_l = edges[long_idx][2], edges[long_idx][3]
    mid_long = ((p1_l[0]+p2_l[0])/2, (p1_l[1]+p2_l[1])/2)
    
    direction = (mid_long[0]-mid_short[0], mid_long[1]-mid_short[1])
    vertical_down = (0, 1)
    angle = get_angle_between_vectors(direction, vertical_down)
    # 符号：叉积为正表示方向在竖直向下右侧（顺时针偏转）
    cross = direction[0]*vertical_down[1] - direction[1]*vertical_down[0]
    if cross < 0:
        angle = -angle
    return angle

def get_triangle_angle(approx):
    """
    三角形：以最长边为底，方向向量 = 底边中点 → 对角顶点
    初始位置：对角顶点竖直向上（方向向量 = 竖直向上 (0,-1)）
    返回当前方向与竖直向上方向的夹角（带符号，左负右正）
    """
    pts = approx.reshape(3, 2)
    # 计算三边长度
    edges = []
    for i in range(3):
        p1 = pts[i]
        p2 = pts[(i+1)%3]
        length = np.linalg.norm(p2 - p1)
        edges.append((length, i, p1, p2))
    longest = max(edges, key=lambda x: x[0])
    _, idx, p1, p2 = longest
    opposite_idx = (idx + 2) % 3
    opposite_pt = pts[opposite_idx]
    mid_base = ((p1[0]+p2[0])/2, (p1[1]+p2[1])/2)
    direction = (opposite_pt[0] - mid_base[0], opposite_pt[1] - mid_base[1])
    vertical_up = (0, -1)
    angle = get_angle_between_vectors(direction, vertical_up)
    # 符号：叉积 z = direction.x * (-1) - direction.y * 0 = -direction.x
    cross = direction[0]*vertical_up[1] - direction[1]*vertical_up[0]  # = -direction.x
    if cross < 0:   # direction.x > 0 时 cross<0，此时顶点偏右，角度取正
        angle = -angle
    return angle

def get_square_angle(contour):
    """正方形/长方形：最小外接矩形角度（垂直方向与竖直方向的偏转）"""
    rect = cv2.minAreaRect(contour)
    angle = rect[2]
    if angle > 90:
        angle -= 180
    elif angle < -90:
        angle += 180
    return angle

def is_trapezoid(approx, angle_threshold=10):
    if len(approx) != 4:
        return False
    pts = approx.reshape(4, 2)
    edges = []
    for i in range(4):
        p1 = pts[i]
        p2 = pts[(i+1)%4]
        vec = p2 - p1
        ang = math.degrees(math.atan2(vec[1], vec[0]))
        edges.append(ang)
    parallel_cnt = 0
    for i in range(2):
        diff = abs(edges[i] - edges[i+2])
        diff = min(diff, 180 - diff)
        if diff < angle_threshold:
            parallel_cnt += 1
    return parallel_cnt == 1

def detect_shapes(image, min_area=500):
    edges = preprocess_image(image)
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    results = []

    for cont in contours:
        area = cv2.contourArea(cont)
        if area < min_area:
            continue

        vertex_count, approx = get_shape_approx(cont, epsilon_factor=0.02)

        if vertex_count == 3:
            angle = get_triangle_angle(approx)
            results.append(('triangle', cont, approx, angle))
        elif vertex_count == 4:
            rect = cv2.minAreaRect(approx)
            w, h = rect[1]
            if w == 0 or h == 0:
                continue
            aspect = max(w, h) / min(w, h)
            if aspect < 1.1:   # 正方形
                angle = get_square_angle(cont)
                results.append(('square', cont, approx, angle))
            else:
                if is_trapezoid(approx):
                    angle = get_trapezoid_angle(approx)
                    results.append(('trapezoid', cont, approx, angle))
                else:
                    # 其他四边形（长方形）也输出角度（可选用）
                    angle = get_square_angle(cont)
                    results.append(('rectangle', cont, approx, angle))
        else:
            if is_circle(cont):
                results.append(('circle', cont, approx, 0.0))
    return results

def draw_results(image, results):
    output = image.copy()
    for shape_type, contour, approx, angle in results:
        cv2.drawContours(output, [contour], -1, (0, 255, 0), 2)
        M = cv2.moments(contour)
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
        else:
            cX, cY = contour[0][0]
        label = f"{shape_type} {angle:.1f}°"
        cv2.putText(output, label, (cX - 50, cY - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
    return output

def main():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("无法打开摄像头")
        return
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    print("实时检测中，按 'q' 退出")
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        results = detect_shapes(frame, min_area=500)
        display = draw_results(frame, results)

        # 在左上角打印所有形状的角度
        y = 30
        for typ, _, _, ang in results:
            cv2.putText(display, f"{typ}: {ang:.1f}°", (10, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            y += 25

        cv2.imshow("Shape Detection", display)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()