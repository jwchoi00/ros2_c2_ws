import sqlite3
import cv2
import time

# SQLite 데이터베이스 설정 및 테이블 생성
conn = sqlite3.connect('/home/g1/ros2_c2_ws/src/flask_ros_app/resource/captured_images.db')
cursor = conn.cursor()
cursor.execute('''
    CREATE TABLE IF NOT EXISTS CapturedImages (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        status TEXT NOT NULL,
        timestamp TEXT NOT NULL,
        image BLOB NOT NULL
    )
''')
conn.commit()

# 이미지를 BLOB 형식으로 저장하는 함수
def insert_captured_image(image):
    timestamp = time.strftime('%Y-%m-%d %H:%M:%S')
    time_sec = int(timestamp[-2:])
    if time_sec % 2 == 0:
        status = 'even'
    else:
        status = 'odd'
    _, buffer = cv2.imencode('.jpg', image)  # 이미지를 JPEG로 인코딩
    image_blob = buffer.tobytes()
    cursor.execute('INSERT INTO CapturedImages (status, timestamp, image) VALUES (?, ?, ?)', (status, timestamp, image_blob))
    conn.commit()

# SQLite에서 이미지를 가져와 파일로 저장하는 함수
def fetch_image(image_id, output_path):
    cursor.execute('SELECT image FROM CapturedImages WHERE id = ?', (image_id,))
    result = cursor.fetchone()
    if result:
        with open(output_path, 'wb') as file:
            file.write(result[0])
        print(f"Image saved to {output_path}")
    else:
        print(f"No image found with ID {image_id}")

# OpenCV로 이미지 캡처
cap = cv2.VideoCapture(0)  # 0번 카메라 (웹캠)

print("Press 'c' to capture and save an image, or 'q' to quit.")
while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to capture frame.")
        break

    cv2.imshow("Live Video", frame)

    key = cv2.waitKey(1)
    if key == ord('c'):  # 'c' 키를 눌러 이미지를 캡처
        insert_captured_image(frame)
        print("Image captured and saved to database.")
    elif key == ord('q'):  # 'q' 키를 눌러 종료
        break

# 리소스 정리
cap.release()
cv2.destroyAllWindows()
conn.close()
