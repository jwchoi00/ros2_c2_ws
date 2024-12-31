from flask import Flask, render_template, jsonify
import sqlite3

app = Flask(__name__)

def get_db_connection():
    conn = sqlite3.connect('/home/g1/ros2_c2_ws/src/flask_ros_app/resource/test_car_captured_images.db')
    conn.row_factory = sqlite3.Row
    return conn

@app.route('/')
def index():
    conn = get_db_connection()
    sessions = conn.execute('SELECT * FROM TrackingSessions').fetchall()
    print(sessions)
    conn.close()
    return render_template('sessions.html', sessions=sessions)

@app.route('/session_details/<int:session_id>')
def session_details(session_id):
    conn = get_db_connection()
    details = conn.execute('SELECT * FROM CapturedImages WHERE tracking_session_id = ?', (session_id,)).fetchall()
    print(details)
    conn.close()
    
    # 'details'을 리스트로 변환 (이미 Row 객체 형태로 되어있음)
    details_list = [dict(row) for row in details]  # Row 객체를 딕셔너리로 변환
    return render_template('session_details.html', details=details_list)

@app.route('/image/<int:image_id>')
def serve_image(image_id):
    conn = get_db_connection()
    image_data = conn.execute('SELECT image FROM CapturedImages WHERE id = ?', (image_id,)).fetchone()
    conn.close()
    return (image_data['image'], {'Content-Type': 'image/jpeg'})

if __name__ == '__main__':
    app.run(debug=True)
