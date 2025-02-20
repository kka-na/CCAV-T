import math


def upsample_path_1m(limited_path):
    """
    limited_path: (e, n) 좌표 튜플들의 리스트 (다운샘플된 경로)
    반환: 각 선분을 1m 간격으로 보간하여 생성된 경로
    """
    if not limited_path:
        return []
    
    upsampled = [limited_path[0]]
    
    for i in range(1, len(limited_path)):
        start = limited_path[i-1]
        end = limited_path[i]
        
        # 선분 길이 계산 (유클리드 거리)
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        segment_length = math.sqrt(dx**2 + dy**2)
        
        # 선분 길이가 1m보다 크다면, 1m 간격 점 추가 (선형 보간)
        num_points = int(segment_length)  # 정수 부분만큼 1m 간격 점이 들어갈 수 있음
        
        for j in range(1, num_points + 1):
            distance = j  # 시작점으로부터의 거리 (1, 2, 3, ... m)
            # segment_length보다 작은 거리까지만 보간 (endpoint는 마지막에 추가)
            if distance < segment_length:
                t = distance / segment_length
                new_point = (start[0] + dx * t, start[1] + dy * t)
                upsampled.append(new_point)
        
        # 선분의 끝 점을 추가 (중복되지 않도록)
        if upsampled[-1] != end:
            upsampled.append(end)
    
    return upsampled