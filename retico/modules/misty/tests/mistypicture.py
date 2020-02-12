import requests
from retico.modules.misty.mistyPy import Robot
#get http://10.10.0.7/api/cameras/rgb?base64=false&displayOnScreen=false&overwriteExisting=false
def takePicture(ip):
    requests.get('http://'+ip+'/api/cameras/rgb?base64=false&displayOnScreen=false&overwriteExisting=false')

ip = "10.10.0.7"
takePicture(ip)
#can see most recently taken picture at "http://10.10.0.7/api/cameras/rgb"
robot = Robot(ip)
robot.printImageList()