# ros2_c2_ws

- 주의사항

    ros2_flask_sub_image_with_db_final.py 안에 있는
  
    conn = sqlite3.connect('{경로 설정}/log_car_captured_images.db') 변경

    sqlite3_save_fianl.py 안에 있는

    self.conn = sqlite3.connect('{경로 설정}/log_car_captured_images.db', check_same_thread=False) 변경
  
- 실행 방법
  
    ros2 run flask_ros_app ros2_flask_sub_image_with_db_final -> flask 실행 코드
    ros2 run flask_ros_app sqlite3_save_final -> sqlite database를 생성 및 저장하는 코드
