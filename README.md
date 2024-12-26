# ros2_c2_ws

- 현재 완성된 것
    yolo로 인식하고 박스 그리기
    center 좌표 추출하기

- 실행 전 주의사항
    self.model = YOLO("/home/g1/ros2_c2_ws/src/object_detection/resource/best.pt")  -> 경로 수정

- 실행 방법
  
    ros2 run object_detection check_drone -> videocapture 해서 yolo로 판단한 이미지 전송
    ros2 run object_detection sub_compress_img -> compress_image sub해서 띄우는 코드
