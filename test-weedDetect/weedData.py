from firebase import firebase
import json
firebase = firebase.FirebaseApplication("https://weeddetection-309c8-default-rtdb.firebaseio.com/",None)


import serial

serialPort = serial.Serial(port = "COM5", baudrate=9600,
                           bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE)
serialString = ""                           # Used to hold data coming over UART
# Get a database reference to our posts

serialData=""
spray=True
while(1):
    serialData = {
    "spray": spray
    
       
    
}
    result = firebase.get('kontrol','')

# Read the data at the posts reference (this is a blocking operation)
    print(result)

    res = json.dumps( serialData)
    serialPort.write(str(result).encode())
            
    # Wait until there is data waiting in the serial buffer
    if(serialPort.in_waiting > 0):

        # Read data out of the buffer until a carraige return / new line is found
        data = serialPort.readline().decode("utf-8")
        
       
        try:
            dict_json = json.loads(data)
            print(dict_json)
        except json.JSONDecodeError as e:
           print("JSON:", e)
        sent = json.dumps(serialString)
        result = firebase.patch('ot', dict_json)
serialPort.close()

print(result)