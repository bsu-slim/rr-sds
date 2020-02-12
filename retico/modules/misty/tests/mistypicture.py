import requests
import sys
#get http://10.10.0.7/api/cameras/rgb?base64=false&displayOnScreen=false&overwriteExisting=false
def takePicture(ip):
    resp = requests.get('http://'+ip+'/api/cameras/rgb?base64=true&displayOnScreen=false&overwriteExisting=false')
    resp = resp.json()
    return (resp['result'])

ip = "10.10.0.7"
response = takePicture(ip)
print(response)