from subprocess import PIPE, Popen


networks={}
with Popen('nmcli dev wifi list', stdout=PIPE, stderr=None, shell=True) as process:
    output = process.communicate()[0].decode("utf-8")
    firstLine=True
    count=0
    for line in output.splitlines():
    	if firstLine:
    		firstLine=False
    		continue
    	arr=line.split(' ')
    	count+=1
    	ssid=line[27:43]
    	signalStrength=line[76:80].rstrip('_')
    	networks[str(count)]=ssid
    	print(str(count).ljust(3, ' ') + ' '+ ssid+' '+signalStrength)

while True:
    try:
    	networkNumber = int(input("Select network number: "))
    	if networkNumber>count or networkNumber<1:
    		print("That's not a valid option!")
    	else:
        	break
    except:
        print("That's not a valid option!")

print("You selected: " + networks[str(networkNumber)])