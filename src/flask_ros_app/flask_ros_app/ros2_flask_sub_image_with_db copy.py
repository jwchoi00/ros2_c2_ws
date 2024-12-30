from flask import Flask, render_template, Response, redirect, url_for, flash, request, jsonify, send_file
import base64
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
import psutil
from flask_login import LoginManager, UserMixin, login_user, login_required, logout_user, current_user
import cv2
import numpy as np
import time
import sqlite3
import io

# Flask 앱 초기화 및 비밀 키 설정
app = Flask(__name__)
app.secret_key = "your_secret_key"  # 세션 관리를 위한 비밀 키

# Flask-Login 초기화
login_manager = LoginManager()
login_manager.init_app(app)

# 메인 함수
node = None

# 사용자 클래스를 정의 (Flask-Login과 연동)
class User(UserMixin):
    def __init__(self, id):
        self.id = id

# 하드코딩된 사용자 인증 데이터베이스 (간단한 예제용)
users = {"admin": {"password": "adminpassword"}}

# 세션에서 사용자 로드 함수 정의
@login_manager.user_loader
def load_user(user_id):
    return User(user_id)

# SQLite 데이터베이스 경로 설정
DB_PATH = '/home/g1/ros2_c2_ws/src/flask_ros_app/resource/test_car_captured_images.db'

# 메인 페이지
@app.route("/", methods=["GET", "POST"])
def index():
    # 로그인하지 않은 사용자는 로그인 페이지로 리디렉션
    if not current_user.is_authenticated:
        return redirect(url_for('login'))
    return render_template("index.html")  # 로그인된 사용자는 메인 페이지로 이동

# 시스템 상태 가져오기 함수
def get_system_stats():
    cpu_usage = psutil.cpu_percent(interval=1)  # CPU 사용률
    memory = psutil.virtual_memory()  # 메모리 상태
    memory_usage = memory.percent  # 메모리 사용률
    total_memory = memory.total / (1024 * 1024 * 1024)  # 메모리 총 용량 (GB 단위)
    disk = psutil.disk_usage('/')  # 디스크 상태
    disk_usage = disk.percent  # 디스크 사용률
    total_disk = disk.total / (1024 * 1024 * 1024)  # 디스크 총 용량 (GB 단위)
    
    stats = {
        "cpu_usage": cpu_usage,
        "memory_usage": memory_usage,
        "total_memory": total_memory,
        "disk_usage": disk_usage,
        "total_disk": total_disk
    }
    return stats

def get_db_connection():
    conn = sqlite3.connect('/home/g1/ros2_c2_ws/src/flask_ros_app/resource/test_car_captured_images.db')
    conn.row_factory = sqlite3.Row  # To access rows as dictionaries
    return conn

# log 페이지: 제품 정보와 이미지 목록 표시
@app.route('/log')
@login_required
def log():
    conn = get_db_connection()
    
    # Query the database
    sessions = conn.execute('SELECT * FROM TrackingSessions ORDER BY start_timestamp DESC').fetchall()
    conn.close()
    return render_template('index_db.html', sessions=sessions)

@app.route('/session/<int:tracking_session_id>')
def session_detail(tracking_session_id):
    conn = get_db_connection()
    session = conn.execute('SELECT * FROM TrackingSessions WHERE tracking_session_id = ?',
                           (tracking_session_id,)).fetchone()
    images = conn.execute('SELECT * FROM CapturedImages WHERE tracking_session_id = ? ORDER BY timestamp',
                          (tracking_session_id,)).fetchall()
    conn.close()

    # Convert rows to dictionaries and encode images to base64 for display
    images = [dict(image) for image in images]
    for image in images:
        image['image'] = base64.b64encode(image['image']).decode('utf-8')

    return render_template('session_detail.html', session=session, images=images)


# 특정 이미지 표시
# @app.route('/image/<int:image_id>')
# @login_required
# def get_image(image_id):
#     conn = sqlite3.connect(DB_PATH)
#     cursor = conn.cursor()
#     cursor.execute('SELECT image FROM CapturedImages WHERE id = ?', (image_id,))
#     result = cursor.fetchone()
#     conn.close()
#     if result:
#         # BLOB 데이터를 이미지로 반환
#         return send_file(io.BytesIO(result[0]), mimetype='image/jpeg')
#     else:
#         return "Image not found", 404

# 시스템 모니터링 페이지
@app.route("/sysmon", methods=["GET"])
@login_required
def sysmon():
    stats = get_system_stats()  # 시스템 상태 가져오기
    return render_template("sysmon.html", stats=stats)

# JSON 형식으로 시스템 상태 제공
@app.route("/sysmon_stats", methods=["GET"])
@login_required
def sysmon_stats():
    stats = get_system_stats()
    return jsonify(stats)

# 로그인 페이지
@app.route("/login", methods=["GET", "POST"])
def login():
    if current_user.is_authenticated:
        return redirect(url_for('index'))

    if request.method == "POST":
        username = request.form['username']
        password = request.form['password']
        
        if username in users and users[username]['password'] == password:
            user = User(username)
            login_user(user)
            return redirect(url_for('index'))
        else:
            flash("Invalid credentials")
            return redirect(url_for('login'))
    return render_template("login.html")

# 로그아웃 페이지
@app.route("/logout")
@login_required
def logout():
    logout_user()
    return redirect(url_for('login'))

# ROS 2 노드 정의: 로봇 액션을 퍼블리싱
class ActionPublisherNode(Node):
    def __init__(self):
        super().__init__('action_publisher_node')
        self.publisher = self.create_publisher(String, 'robot_actions', 10)
        self.get_logger().info("Flask Node initialized.")
        self.subscription = self.create_subscription(
            CompressedImage,
            'compressed_image',
            self.image_callback,
            10
        )
        self.current_image = ""

    def publish_action(self, action):
        msg = String()
        msg.data = action
        self.publisher.publish(msg)
        self.get_logger().info(f"Published action: {action}")

    def image_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is not None:
            _, encoded_image = cv2.imencode('.jpg', frame)
            self.current_image = encoded_image.tobytes()
            #self.get_logger().info("Image updated for Flask")
        else:
            self.get_logger().error('Failed to decode the image')

    def get_current_image(self):
        return self.current_image

# 비디오 스트리밍 엔드포인트
@app.route("/video_feed")
@login_required
def video_feed():
    def generate():
        while True:
            if node:
                image_data = node.get_current_image()
                if image_data:
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + image_data + b'\r\n')
            time.sleep(0.1)
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

# 로봇 액션 API 엔드포인트
@app.route("/action", methods=["POST"])
@login_required
def action():
    global node
    action = request.json.get('action')
    
    if action:
        node.publish_action(action)
        return jsonify({"status": "success", "action": action}), 200
    else:
        return jsonify({"status": "error", "message": "No action provided"}), 400

# Flask 서버 시작
def start_flask():
    app.run(host='0.0.0.0', port=5000)



def main(args=None):
    global node
    rclpy.init(args=args)
    node = ActionPublisherNode()
    
    # Flask 서버를 별도 스레드에서 실행
    flask_thread = threading.Thread(target=start_flask)
    flask_thread.start()
    
    rclpy.spin(node)
    
    flask_thread.join()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
