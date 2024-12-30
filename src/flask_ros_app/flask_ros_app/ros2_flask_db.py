import sqlite3
import base64
from flask import Flask, render_template, request, redirect, url_for
import time

app = Flask(__name__)

# Connect to SQLite Database
def get_db_connection():
    conn = sqlite3.connect('/home/g1/ros2_c2_ws/src/flask_ros_app/resource/test_car_captured_images.db')
    conn.row_factory = sqlite3.Row  # To access rows as dictionaries
    return conn

# Home page: Displays all tracking sessions
@app.route('/')
def index():
    conn = get_db_connection()
    sessions = conn.execute('SELECT * FROM TrackingSessions ORDER BY start_timestamp DESC').fetchall()
    conn.close()
    return render_template('index_db.html', sessions=sessions)

# Tracking session detail page: Displays images for the given session
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


# Start a new tracking session
@app.route('/start_tracking', methods=['POST'])
def start_tracking():
    conn = get_db_connection()
    timestamp = time.strftime('%Y-%m-%d %H:%M:%S')
    conn.execute('INSERT INTO TrackingSessions (start_timestamp, status) VALUES (?, ?)',
                 (timestamp, 'start_tracking'))
    conn.commit()
    conn.close()
    return redirect(url_for('index'))

# Stop tracking and update session status
@app.route('/end_tracking/<int:tracking_session_id>', methods=['POST'])
def end_tracking(tracking_session_id):
    action = request.form['action']
    timestamp = time.strftime('%Y-%m-%d %H:%M:%S')

    conn = get_db_connection()
    conn.execute('UPDATE TrackingSessions SET end_timestamp = ?, status = ? WHERE tracking_session_id = ?',
                 (timestamp, action, tracking_session_id))
    conn.commit()
    conn.close()
    return redirect(url_for('session_detail', tracking_session_id=tracking_session_id))

# Start the Flask app
if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0', port=5000)
