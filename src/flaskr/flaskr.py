from flask import Flask
from flask import request
from flask import render_template

web_gui = Flask(__name__)
@web_gui.route("/page", methods = ["GET","POST"])
def page_fnc():
    if request.method == "POST":
        return 'POST recevied'
    else:
        return render_template("page.html")
if __name__ == "__main__":
    web_gui.run(debug=True, port=9999)