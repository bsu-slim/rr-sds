import requests
from PIL import Image
from io import BytesIO
import base64
#get http://10.10.0.7/api/cameras/rgb
# misty's camera takes portrait oriented pictures, if width is greater 
# than height, size will be scaled to height given and proportionate 
# width for a 3:4 aspect ratio
# Returns a dictionary object containing the following:
    # base64 (string) - A string containing the Base64-encoded image data.
    # contentType (string) - The type and format of the image returned.
    # height (integer) - The height of the image in pixels.
    # name (string) - The name of the image.
    # width (integer) - The width of the image in pixels.
def takePicture(ip, width=320, height=427):
    
    resp = requests.get('http://'+ip+'/api/cameras/rgb?base64=true&width='+str(width)+'&height='+str(height))
    resp = resp.json()
    return (resp['result'])

ip = "10.10.0.7"
result = takePicture(ip)
im = Image.open(BytesIO(base64.b64decode(result.get('base64'))))
im.save("mistypic.jpg","JPEG")
print('image saved as mistypic.jpeg')