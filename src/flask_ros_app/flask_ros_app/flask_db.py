from flask import Flask, render_template, send_file
import sqlite3
import io

app = Flask(__name__)

# SQLite 데이터베이스 연결
DB_PATH = '/home/g1/ros2_c2_ws/src/flask_ros_app/resource/captured_images.db'

# 메인 페이지: 제품 정보와 이미지 목록 표시
@app.route('/')
def index():
    conn = sqlite3.connect(DB_PATH)
    cursor = conn.cursor()
    cursor.execute('SELECT id, status, timestamp FROM CapturedImages')
    products = cursor.fetchall()  # [(id, timestamp, name), ...]
    conn.close()
    return render_template('index_for_db.html', products=products)

# 특정 이미지 표시
@app.route('/image/<int:image_id>')
def get_image(image_id):
    conn = sqlite3.connect(DB_PATH)
    cursor = conn.cursor()
    cursor.execute('SELECT image FROM CapturedImages WHERE id = ?', (image_id,))
    result = cursor.fetchone()
    conn.close()
    if result:
        # BLOB 데이터를 이미지로 반환
        return send_file(io.BytesIO(result[0]), mimetype='image/jpeg')
    else:
        return "Image not found", 404

if __name__ == '__main__':
    app.run(debug=True)
