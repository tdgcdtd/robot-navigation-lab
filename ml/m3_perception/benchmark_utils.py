"""benchmark 和 shapes_check 共用的小工具。"""


def iou(a, b):
    """两个框 (x0,y0,x1,y1) 的重合度：交集面积 / 并集面积。"""
    ix = max(0, min(a[2], b[2]) - max(a[0], b[0])); iy = max(0, min(a[3], b[3]) - max(a[1], b[1]))
    inter = ix * iy
    return inter / ((a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter)
